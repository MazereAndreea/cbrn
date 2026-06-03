#!/usr/bin/env python3

# Protobuf compatibility shims — must run before any mediapipe import.
import sys as _sys, types as _types
import google.protobuf as _pb

if not hasattr(_pb, 'runtime_version'):
    _rv = _types.ModuleType('google.protobuf.runtime_version')
    _rv.ValidateProtobufGenlibrary = lambda *_: None
    _rv.ValidateProtobufRuntimeVersion = lambda *_: None
    _rv.OSS_MINIMUM_PROTOC_VERSION = (0, 0, 0)
    _rv.Domain = type('Domain', (), {'GOOGLE_INTERNAL': 0, 'PUBLIC': 1})()
    _sys.modules['google.protobuf.runtime_version'] = _rv
    _pb.runtime_version = _rv

import google.protobuf.message_factory as _proto_mf
if not hasattr(_proto_mf.MessageFactory, 'GetPrototype'):
    _proto_mf.MessageFactory.GetPrototype = lambda _, desc: _proto_mf.GetMessageClass(desc)

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import csv
import time
import os
from datetime import datetime

from cbrn_interfaces.msg import KeypointArray, InjectionZone

# ── MediaPipe 33-keypoint indices (critical for CBRN injection) ──────────────
NOSE       = 0
L_SHOULDER = 11; R_SHOULDER = 12
L_ELBOW    = 13; R_ELBOW    = 14
L_WRIST    = 15; R_WRIST    = 16
L_HIP      = 23; R_HIP      = 24

CRITICAL_IDX   = [L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW, L_WRIST, R_WRIST]
CRITICAL_NAMES = ['LShoulder', 'RShoulder', 'LElbow', 'RElbow', 'LWrist', 'RWrist']

CONF_THRESHOLD = 0.3
WORK_OFFSET_M  = 0.5   # robot stops this far from injection zone (m)


# ── Shared logic (also used by injection_zone_analyzer) ──────────────────────

def classify_body_position(vis):
    """
    Returns 'face_up' | 'face_down' | 'lateral_left' | 'lateral_right' | 'unknown'.
    MediaPipe visibility scores are used as confidence proxies.
    lateral_right = person lying on right side → only left shoulder visible from above.
    lateral_left  = person lying on left side  → only right shoulder visible from above.
    """
    ls   = vis[L_SHOULDER]
    rs   = vis[R_SHOULDER]
    nose = vis[NOSE]

    both = ls > CONF_THRESHOLD and rs > CONF_THRESHOLD
    if both:
        return 'face_up' if nose > CONF_THRESHOLD else 'face_down'

    diff = ls - rs
    if abs(diff) > 0.2:
        if ls > rs and ls > CONF_THRESHOLD:
            return 'lateral_right'
        if rs > ls and rs > CONF_THRESHOLD:
            return 'lateral_left'
    return 'unknown'


def select_injection_zone(x_norm, y_norm, vis, body_pos, frame_w, frame_h):
    """
    Primary: deltoid (30 % along shoulder→elbow vector).
    Fallback: arm midpoint (elbow→wrist midpoint).
    Returns (zone_type, px, py, confidence) or None.
    """
    def kpx(i): return int(x_norm[i] * frame_w)
    def kpy(i): return int(y_norm[i] * frame_h)

    def deltoid(sh_i, el_i):
        x = kpx(sh_i) + int(0.3 * (kpx(el_i) - kpx(sh_i)))
        y = kpy(sh_i) + int(0.3 * (kpy(el_i) - kpy(sh_i)))
        return x, y, float(min(vis[sh_i], vis[el_i]))

    def arm_mid(el_i, wr_i):
        return (kpx(el_i) + kpx(wr_i)) // 2, (kpy(el_i) + kpy(wr_i)) // 2, \
               float(min(vis[el_i], vis[wr_i]))

    T = CONF_THRESHOLD
    # Priority order per body position
    primaries = {
        'face_up':       [('deltoid_left',  L_SHOULDER, L_ELBOW),
                          ('deltoid_right', R_SHOULDER, R_ELBOW)],
        'face_down':     [('deltoid_right', R_SHOULDER, R_ELBOW),
                          ('deltoid_left',  L_SHOULDER, L_ELBOW)],
        'lateral_right': [('deltoid_left',  L_SHOULDER, L_ELBOW)],
        'lateral_left':  [('deltoid_right', R_SHOULDER, R_ELBOW)],
        'unknown':       [('deltoid_left',  L_SHOULDER, L_ELBOW),
                          ('deltoid_right', R_SHOULDER, R_ELBOW)],
    }
    for zone_type, sh_i, el_i in primaries.get(body_pos, []):
        if vis[sh_i] > T and vis[el_i] > T:
            dx, dy, c = deltoid(sh_i, el_i)
            return zone_type, dx, dy, c

    # Fallback: arm midpoint
    for zone_type, el_i, wr_i in [('arm_mid_left', L_ELBOW, L_WRIST),
                                   ('arm_mid_right', R_ELBOW, R_WRIST)]:
        if vis[el_i] > T and vis[wr_i] > T:
            mx, my, c = arm_mid(el_i, wr_i)
            return zone_type, mx, my, c
    return None


def estimate_distance_from_bbox(bbox_w_px, bbox_h_px, focal_length, ref_m=0.45):
    """
    Pinhole model: D = (f * W_real) / W_pixels.
    ref_m = shoulder width reference (0.45 m).
    Uses the larger of width/height as the reference dimension.
    """
    ref_px = max(bbox_w_px, bbox_h_px)
    if ref_px < 10 or focal_length <= 0:
        return None
    return (focal_length * ref_m) / ref_px


def compute_nav2_goal(zone_px, frame_w,
                       focal_length, dist_m,
                       robot_x, robot_y, robot_yaw,
                       work_offset=WORK_OFFSET_M):
    """
    Convert injection zone pixel → map-frame Nav2Goal.
    Only horizontal angle is used; the goal is computed in the 2-D floor plane.
    Returns (goal_x, goal_y, approach_yaw).
    """
    angle_h  = math.atan2(zone_px - frame_w / 2.0, focal_length)
    zone_r_x = dist_m * math.cos(angle_h)
    zone_r_y = dist_m * math.sin(angle_h)
    cos_y, sin_y = math.cos(robot_yaw), math.sin(robot_yaw)
    zone_m_x = robot_x + zone_r_x * cos_y - zone_r_y * sin_y
    zone_m_y = robot_y + zone_r_x * sin_y + zone_r_y * cos_y
    approach_yaw = math.atan2(zone_m_y - robot_y, zone_m_x - robot_x)
    goal_x = zone_m_x - work_offset * math.cos(approach_yaw)
    goal_y = zone_m_y - work_offset * math.sin(approach_yaw)
    return goal_x, goal_y, approach_yaw


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class MediaPipeDetector(Node):
    def __init__(self):
        super().__init__('mediapipe_detector')
        self.bridge = CvBridge()

        # Publishers
        self.target_pub = self.create_publisher(Point,         '/perception/target',            10)
        self.result_pub = self.create_publisher(Image,         '/pose_estimation/image_result', 10)
        self.kp_pub     = self.create_publisher(KeypointArray, '/keypoints',                    10)
        self.inj_pub    = self.create_publisher(InjectionZone, '/injection_zone',               10)

        # Subscribers
        self.create_subscription(Image,      '/camera',                     self._img_cb,   10)
        self.create_subscription(Image,      '/camera/depth',               self._depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info',         self._info_cb,  10)
        self.create_subscription(Odometry,   '/diff_drive_controller/odom', self._odom_cb,  10)
        self.create_subscription(LaserScan,  '/scan',                       self._scan_cb,  10)

        # MediaPipe
        self.mp_pose    = mp.solutions.pose
        self.pose       = self.mp_pose.Pose(min_detection_confidence=0.2,
                                            min_tracking_confidence=0.2)
        self.mp_drawing = mp.solutions.drawing_utils

        # State
        self.depth_image    = None
        self.focal_length   = 462.0   # px — updated from /camera/camera_info
        self.robot_x        = 0.0
        self.robot_y        = 0.0
        self.robot_yaw      = 0.0
        self.lidar_min_dist = None

        # CSV — section 1.6 evaluation metrics
        log_dir = os.path.expanduser('~/bench_logs')
        os.makedirs(log_dir, exist_ok=True)
        ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(log_dir, f'metrics_mediapipe_{ts}.csv')
        self._csv_file = open(csv_path, mode='w', newline='')
        self._csv      = csv.writer(self._csv_file)
        header = (
            ['Timestamp', 'Model', 'Detected', 'Inference_ms',
             'Distance_m', 'BBox_W_px', 'BBox_H_px']
            + [f'Conf_{n}' for n in CRITICAL_NAMES]
            + ['Kpts_Above_Thresh', 'Body_Position',
               'Injection_Zone', 'Zone_Conf',
               'Goal_X', 'Goal_Y', 'Goal_Yaw', 'Goal_Valid']
        )
        self._csv.writerow(header)
        self.get_logger().info(f'MediaPipe detector ready. Metrics CSV → {csv_path}')

    # ── Sensor callbacks ─────────────────────────────────────────────────────

    def _depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def _info_cb(self, msg):
        if msg.k[0] > 0:
            self.focal_length = msg.k[0]

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_x = p.x
        self.robot_y = p.y
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def _scan_cb(self, msg):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self.lidar_min_dist = min(valid) if valid else None

    # ── Main image callback ──────────────────────────────────────────────────

    def _img_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w  = frame.shape[:2]

        t0      = time.perf_counter()
        results = self.pose.process(rgb)
        inf_ms  = (time.perf_counter() - t0) * 1000.0

        # Default message stubs
        target_msg = Point()
        target_msg.z = 0.0

        kp_msg = KeypointArray()
        kp_msg.header           = msg.header
        kp_msg.model_name       = 'mediapipe'
        kp_msg.inference_time_ms = inf_ms
        kp_msg.is_detected      = False
        kp_msg.num_keypoints    = 33

        inj_msg = InjectionZone()
        inj_msg.header        = msg.header
        inj_msg.body_position = 'unknown'
        inj_msg.zone_type     = 'unknown'
        inj_msg.goal_valid    = False

        # CSV defaults
        csv_det  = False
        csv_dist, csv_bw, csv_bh = 0.0, 0.0, 0.0
        csv_crit  = [0.0] * len(CRITICAL_IDX)
        csv_above = 0
        csv_bpos  = 'unknown'
        csv_zone, csv_zconf = 'none', 0.0
        csv_gx = csv_gy = csv_gyaw = 0.0
        csv_gvalid = False

        if results.pose_landmarks:
            lms  = results.pose_landmarks.landmark
            x_n  = [lm.x        for lm in lms]
            y_n  = [lm.y        for lm in lms]
            vis  = [lm.visibility for lm in lms]

            # Pack KeypointArray
            kp_msg.is_detected = True
            kp_msg.kp_x    = x_n
            kp_msg.kp_y    = y_n
            kp_msg.kp_conf = vis

            px_all = [v * w for v in x_n]
            py_all = [v * h for v in y_n]
            bx1, by1 = min(px_all), min(py_all)
            bx2, by2 = max(px_all), max(py_all)
            bw = bx2 - bx1;  bh = by2 - by1
            kp_msg.bbox_x1, kp_msg.bbox_y1 = bx1, by1
            kp_msg.bbox_x2, kp_msg.bbox_y2 = bx2, by2
            kp_msg.bbox_width_px  = bw
            kp_msg.bbox_height_px = bh

            csv_det  = True
            csv_bw, csv_bh = bw, bh
            csv_crit  = [vis[i] for i in CRITICAL_IDX]
            csv_above = sum(1 for v in vis if v > CONF_THRESHOLD)

            # Distance estimation (pinhole primary; LiDAR fallback)
            dist_m = estimate_distance_from_bbox(bw, bh, self.focal_length)
            if dist_m is None and self.lidar_min_dist is not None:
                dist_m = self.lidar_min_dist
            if dist_m is None:
                dist_m = 0.0
            csv_dist = dist_m
            inj_msg.distance_m = dist_m

            # Body position classification
            body_pos = classify_body_position(vis)
            csv_bpos = body_pos
            inj_msg.body_position = body_pos

            # Injection zone identification
            zone = select_injection_zone(x_n, y_n, vis, body_pos, w, h)
            if zone:
                zone_type, zone_px, zone_py, zone_conf = zone
                csv_zone, csv_zconf = zone_type, zone_conf
                inj_msg.zone_type  = zone_type
                inj_msg.pixel_x    = float(zone_px)
                inj_msg.pixel_y    = float(zone_py)
                inj_msg.confidence = zone_conf

                cv2.circle(frame, (zone_px, zone_py), 12, (0, 0, 255), -1)
                cv2.circle(frame, (zone_px, zone_py), 14, (255, 255, 255), 2)
                cv2.putText(frame, zone_type, (zone_px + 15, zone_py),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Nav2Goal calculation
                if dist_m > 0.05:
                    gx, gy, gyaw = compute_nav2_goal(
                        zone_px, w, self.focal_length,
                        dist_m, self.robot_x, self.robot_y, self.robot_yaw)
                    inj_msg.nav_goal_x   = gx
                    inj_msg.nav_goal_y   = gy
                    inj_msg.nav_goal_yaw = gyaw
                    inj_msg.goal_valid   = True
                    csv_gx, csv_gy, csv_gyaw, csv_gvalid = gx, gy, gyaw, True

            # Skeleton overlay
            self.mp_drawing.draw_landmarks(
                frame, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

            # Centre target for legacy robot_controller
            cx = int(np.mean(px_all))
            cy = int(np.mean(py_all))
            cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
            target_msg.x = float((cx / w) - 0.5)
            target_msg.y = float(cy)
            target_msg.z = float(dist_m)

        cv2.putText(frame, f'MP | {inf_ms:.1f} ms | {csv_bpos}',
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)

        # Publish
        self.target_pub.publish(target_msg)
        self.kp_pub.publish(kp_msg)
        self.inj_pub.publish(inj_msg)
        try:
            out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out.header = msg.header
            self.result_pub.publish(out)
        except Exception as e:
            self.get_logger().error(f'Image publish error: {e}')

        # CSV row (section 1.6 metrics)
        self._csv.writerow(
            [time.time(), 'mediapipe', int(csv_det), f'{inf_ms:.2f}',
             f'{csv_dist:.3f}', f'{csv_bw:.1f}', f'{csv_bh:.1f}']
            + [f'{c:.3f}' for c in csv_crit]
            + [csv_above, csv_bpos, csv_zone, f'{csv_zconf:.3f}',
               f'{csv_gx:.3f}', f'{csv_gy:.3f}', f'{csv_gyaw:.4f}', int(csv_gvalid)]
        )
        self._csv_file.flush()

        cv2.imshow('MediaPipe View', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self._csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = MediaPipeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
