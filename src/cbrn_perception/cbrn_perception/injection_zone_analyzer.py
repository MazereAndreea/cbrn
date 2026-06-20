#!/usr/bin/env python3
"""
injection_zone_analyzer — ROS2 node (section 2.4 + 2.5 of the thesis)

Subscribes to /keypoints (KeypointArray) published by any detector node,
classifies the body position, identifies the injection zone (deltoid primary,
arm-midpoint fallback), estimates the robot–body distance, and computes the
Nav2Goal for the injection approach.

Works with both MediaPipe 33-keypoint format and COCO 17-keypoint format;
the model_name field in KeypointArray determines the index mapping.

Publishes
---------
/injection_zone   (cbrn_interfaces/InjectionZone)
"""

import math
import collections
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, CameraInfo
from nav_msgs.msg import Odometry
import csv
import time
import os
from datetime import datetime

from cbrn_interfaces.msg import KeypointArray, InjectionZone

# ── Keypoint index mappings ───────────────────────────────────────────────────

_COCO17 = dict(
    nose=0,
    l_shoulder=5,  r_shoulder=6,
    l_elbow=7,     r_elbow=8,
    l_wrist=9,     r_wrist=10,
    l_hip=11,      r_hip=12,
)

_MP33 = dict(
    nose=0,
    l_shoulder=11, r_shoulder=12,
    l_elbow=13,    r_elbow=14,
    l_wrist=15,    r_wrist=16,
    l_hip=23,      r_hip=24,
)

CONF_THRESHOLD = 0.3
WORK_OFFSET_M  = 0.5    # robot stops this far from the injection zone (m)
STABILITY_WINDOW = 5    # frames used for body-position majority vote


def _get_idx(model_name: str) -> dict:
    return _MP33 if 'mediapipe' in model_name.lower() else _COCO17


# ── Body-position classifier ──────────────────────────────────────────────────

def classify_body_position(kp_conf, idx: dict) -> str:
    """
    Determine lying orientation from keypoint confidences.

    Returns
    -------
    'face_up' | 'face_down' | 'lateral_left' | 'lateral_right' | 'unknown'

    Convention:
      lateral_right = person lying on their RIGHT side
                      → left arm is on top (accessible from above)
      lateral_left  = person lying on their LEFT side
                      → right arm is on top (accessible from above)
    """
    T  = CONF_THRESHOLD
    ls = kp_conf[idx['l_shoulder']] if idx['l_shoulder'] < len(kp_conf) else 0.0
    rs = kp_conf[idx['r_shoulder']] if idx['r_shoulder'] < len(kp_conf) else 0.0
    n  = kp_conf[idx['nose']]       if idx['nose']       < len(kp_conf) else 0.0

    both = ls > T and rs > T
    if both:
        return 'face_up' if n > T else 'face_down'

    diff = ls - rs
    if abs(diff) > 0.15:
        if ls > rs and ls > T:
            return 'lateral_right'
        if rs > ls and rs > T:
            return 'lateral_left'
    return 'unknown'


# ── Injection zone selector ───────────────────────────────────────────────────

def select_injection_zone(kp_x_norm, kp_y_norm, kp_conf, idx, body_pos,
                           frame_w, frame_h):
    """
    Returns (zone_type, pixel_x, pixel_y, confidence) or None.

    Primary: deltoid = 30 % along shoulder→elbow vector.
    Fallback: arm midpoint (elbow→wrist).
    """
    T = CONF_THRESHOLD

    def kpx(i): return int(kp_x_norm[i] * frame_w)
    def kpy(i): return int(kp_y_norm[i] * frame_h)

    def deltoid(sh_i, el_i):
        x = kpx(sh_i) + int(0.3 * (kpx(el_i) - kpx(sh_i)))
        y = kpy(sh_i) + int(0.3 * (kpy(el_i) - kpy(sh_i)))
        return x, y, float(min(kp_conf[sh_i], kp_conf[el_i]))

    def arm_mid(el_i, wr_i):
        return (
            (kpx(el_i) + kpx(wr_i)) // 2,
            (kpy(el_i) + kpy(wr_i)) // 2,
            float(min(kp_conf[el_i], kp_conf[wr_i])),
        )

    lsh = idx['l_shoulder']; rsh = idx['r_shoulder']
    lel = idx['l_elbow'];    rel = idx['r_elbow']
    lwr = idx['l_wrist'];    rwr = idx['r_wrist']

    primaries = {
        'face_up':       [('deltoid_right', lsh, lel), ('deltoid_left',  rsh, rel)],
        'face_down':     [('deltoid_left',  rsh, rel), ('deltoid_right', lsh, lel)],
        'lateral_right': [('deltoid_right', lsh, lel)],
        'lateral_left':  [('deltoid_left',  rsh, rel)],
        'unknown':       [('deltoid_right', lsh, lel), ('deltoid_left',  rsh, rel)],
    }
    for zone_type, sh_i, el_i in primaries.get(body_pos, []):
        if kp_conf[sh_i] > T and kp_conf[el_i] > T:
            dx, dy, c = deltoid(sh_i, el_i)
            return zone_type, dx, dy, c

    # Fallback: arm midpoint
    for zone_type, el_i, wr_i in [('arm_mid_left', lel, lwr),
                                   ('arm_mid_right', rel, rwr)]:
        if kp_conf[el_i] > T and kp_conf[wr_i] > T:
            mx, my, c = arm_mid(el_i, wr_i)
            return zone_type, mx, my, c

    return None


# ── Distance + Nav2Goal helpers ───────────────────────────────────────────────

def estimate_distance(bbox_w_px, bbox_h_px, focal_length,
                       lidar_dist=None, ref_m=0.45):
    """
    pinhole model D = (f * W_ref) / W_px using bbox larger dimension.
    """
    # Selectăm dimensiunea maximă a cutiei de încadrare (lățime sau înălțime) 
    # pentru a minimiza eroarea indusă de posturi asimetrice (ex. persoană întinsă vs. în picioare).
    ref_px = max(bbox_w_px, bbox_h_px)
    # Evităm împărțirea la zero și respingem detecțiile false/zgomotul (sub 10 pixeli).
    if ref_px > 10 and focal_length > 0:
        return (focal_length * ref_m) / ref_px
    #if lidar_dist is not None:
    #    return lidar_dist
    return 0.0


def compute_nav2_goal(zone_px, frame_w, focal_length, dist_m,
                       robot_x, robot_y, robot_yaw,
                       work_offset=WORK_OFFSET_M):
    """
    Map injection zone pixel → Nav2Goal in the map frame.
    Uses horizontal angle only (floor-plane 2-D navigation).
    Returns (goal_x, goal_y, approach_yaw).
    """
    # 1. Calculul unghiului orizontal față de centrul optic al camerei.
    # Se face conversia din diferența de pixeli (relativ la centrul cadrului) într-un unghi real (radiani).                       
    angle_h  = math.atan2(zone_px - frame_w / 2.0, focal_length)
                           
    # 2. Trecerea din coordonate polare (distanță, unghi) în coordonate carteziene locale (frame robot).
    # zone_r_x reprezintă distanța "în față", iar zone_r_y deviația "stânga/dreapta".                       
    zone_r_x = dist_m * math.cos(angle_h)
    zone_r_y = dist_m * math.sin(angle_h)
                           
    # Extragem componentele trigonometrice ale orientării curente a robotului.
    cos_y, sin_y = math.cos(robot_yaw), math.sin(robot_yaw)
                           
    # 3. Transformare de coordonate din frame-ul robotului în frame-ul global (hartă/odometrie).
    # Se aplică o matrice de rotație 2D și o translație cu poziția curentă a robotului.                       
    zone_m_x = robot_x + zone_r_x * cos_y - zone_r_y * sin_y
    zone_m_y = robot_y + zone_r_x * sin_y + zone_r_y * cos_y

    # 4. Calculul orientării ideale de apropiere (yaw).
    # Robotul trebuie să se orienteze cu fața către coordonatele globale ale zonei de injecție.                       
    approach_yaw = math.atan2(zone_m_y - robot_y, zone_m_x - robot_x)

    # 5. Aplicarea offset-ului de lucru (distanța de siguranță/operare).
    # Nu vrem ca baza robotului să se suprapună cu ținta, ci să se oprească la o distanță prescrisă (work_offset)
    # de-a lungul vectorului de apropiere. Scădem această distanță din coordonatele finale.                       
    goal_x = zone_m_x - work_offset * math.cos(approach_yaw)
    goal_y = zone_m_y - work_offset * math.sin(approach_yaw)
    return goal_x, goal_y, approach_yaw


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class InjectionZoneAnalyzer(Node):
    """
    Standalone analyzer node.

    Can run alongside any detector (mediapipe / yolo / movenet / vitpose)
    by subscribing to the shared /keypoints topic.
    Also used by the scenario_orchestrator to obtain the injection goal.
    """

    def __init__(self):
        super().__init__('injection_zone_analyzer')

        # Parameters
        self.declare_parameter('frame_width',  640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('work_offset',  WORK_OFFSET_M)
        self.declare_parameter('conf_threshold', CONF_THRESHOLD)

        self._fw      = self.get_parameter('frame_width').value
        self._fh      = self.get_parameter('frame_height').value
        self._offset  = self.get_parameter('work_offset').value
        self._conf_th = self.get_parameter('conf_threshold').value

        # Publisher
        self._inj_pub = self.create_publisher(InjectionZone, '/injection_zone', 10)

        # Subscribers
        self.create_subscription(KeypointArray, '/keypoints',
                                 self._kp_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self._info_cb, 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom',
                                 self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan',
                                 self._scan_cb, 10)

        # State
        self._focal_length   = 462.0
        self._robot_x        = 0.0
        self._robot_y        = 0.0
        self._robot_yaw      = 0.0
        self._lidar_min_dist = None

        # Stability: majority-vote buffer for body-position classification
        self._pos_buffer = collections.deque(maxlen=STABILITY_WINDOW)

        # CSV — section 3.4 injection zone test results
        log_dir = os.path.expanduser('~/bench_logs')
        os.makedirs(log_dir, exist_ok=True)
        ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(log_dir, f'injection_zone_{ts}.csv')
        self._csv_file = open(csv_path, mode='w', newline='')
        self._csv      = csv.writer(self._csv_file)
        self._csv.writerow([
            'Timestamp', 'Model', 'Detected',
            'Body_Position', 'Stable_Position',
            'Injection_Zone', 'Zone_Conf',
            'Distance_m', 'Goal_X', 'Goal_Y', 'Goal_Yaw', 'Goal_Valid',
        ])
        self.get_logger().info(f'InjectionZoneAnalyzer ready. CSV → {csv_path}')

    # ── Sensor callbacks ─────────────────────────────────────────────────────

    def _info_cb(self, msg):
        if msg.k[0] > 0:
            self._focal_length = msg.k[0]

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._robot_x = p.x
        self._robot_y = p.y
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)

    def _scan_cb(self, msg):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self._lidar_min_dist = min(valid) if valid else None

    # ── Main analysis callback ────────────────────────────────────────────────

    def _kp_cb(self, kp_msg: KeypointArray):
        inj_msg = InjectionZone()
        inj_msg.header        = kp_msg.header
        inj_msg.body_position = 'unknown'
        inj_msg.zone_type     = 'unknown'
        inj_msg.goal_valid    = False

        csv_body = 'unknown'
        csv_stable = 'unknown'
        csv_zone = 'none'
        csv_zconf = 0.0
        csv_dist = 0.0
        csv_gx = csv_gy = csv_gyaw = 0.0
        csv_gvalid = False

        if not kp_msg.is_detected or len(kp_msg.kp_conf) == 0:
            self._pos_buffer.append('unknown')
            self._inj_pub.publish(inj_msg)
            self._log_csv(kp_msg.model_name, False,
                          csv_body, csv_stable, csv_zone, csv_zconf,
                          csv_dist, csv_gx, csv_gy, csv_gyaw, csv_gvalid)
            return

        idx = _get_idx(kp_msg.model_name)
        kp_conf = list(kp_msg.kp_conf)

        # Body position (single frame)
        body_pos = classify_body_position(kp_conf, idx)
        self._pos_buffer.append(body_pos)
        csv_body = body_pos

        # Stable position via majority vote over recent frames
        counts = collections.Counter(self._pos_buffer)
        stable_pos = counts.most_common(1)[0][0]
        csv_stable = stable_pos
        inj_msg.body_position = stable_pos

        # Injection zone
        kp_x = list(kp_msg.kp_x)
        kp_y = list(kp_msg.kp_y)
        zone = select_injection_zone(
            kp_x, kp_y, kp_conf, idx, stable_pos, self._fw, self._fh)

        # Distance estimation
        dist_m = estimate_distance(
            kp_msg.bbox_width_px, kp_msg.bbox_height_px,
            self._focal_length, self._lidar_min_dist)
        csv_dist = dist_m
        inj_msg.distance_m = dist_m

        if zone:
            zone_type, zone_px, zone_py, zone_conf = zone
            csv_zone, csv_zconf = zone_type, zone_conf
            inj_msg.zone_type  = zone_type
            inj_msg.pixel_x    = float(zone_px)
            inj_msg.pixel_y    = float(zone_py)
            inj_msg.confidence = zone_conf

            if dist_m > 0.05:
                gx, gy, gyaw = compute_nav2_goal(
                    zone_px, self._fw, self._focal_length, dist_m,
                    self._robot_x, self._robot_y, self._robot_yaw,
                    work_offset=self._offset)
                inj_msg.nav_goal_x   = gx
                inj_msg.nav_goal_y   = gy
                inj_msg.nav_goal_yaw = gyaw
                inj_msg.goal_valid   = True
                csv_gx, csv_gy, csv_gyaw, csv_gvalid = gx, gy, gyaw, True

        self._inj_pub.publish(inj_msg)
        self._log_csv(kp_msg.model_name, True,
                      csv_body, csv_stable, csv_zone, csv_zconf,
                      csv_dist, csv_gx, csv_gy, csv_gyaw, csv_gvalid)

    def _log_csv(self, model, detected,
                 body, stable, zone, zconf,
                 dist, gx, gy, gyaw, gvalid):
        self._csv.writerow([
            time.time(), model, int(detected),
            body, stable, zone, f'{zconf:.3f}',
            f'{dist:.3f}',
            f'{gx:.3f}', f'{gy:.3f}', f'{gyaw:.4f}', int(gvalid),
        ])
        self._csv_file.flush()

    def destroy_node(self):
        self._csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = InjectionZoneAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
