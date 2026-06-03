#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import csv
import time
import os
from datetime import datetime
import tensorflow as tf

from cbrn_interfaces.msg import KeypointArray, InjectionZone

tflite = tf.lite

# ── COCO 17-keypoint indices ─────────────────────────────────────────────────
NOSE       = 0
L_SHOULDER = 5;  R_SHOULDER = 6
L_ELBOW    = 7;  R_ELBOW    = 8
L_WRIST    = 9;  R_WRIST    = 10
L_HIP      = 11; R_HIP      = 12

CRITICAL_IDX   = [L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW, L_WRIST, R_WRIST]
CRITICAL_NAMES = ['LShoulder', 'RShoulder', 'LElbow', 'RElbow', 'LWrist', 'RWrist']

CONF_THRESHOLD = 0.1    # MoveNet works well at lower threshold
CRIT_THRESHOLD = 0.3
WORK_OFFSET_M  = 0.5

SKELETON = [
    (5, 7), (7, 9), (6, 8), (8, 10),
    (5, 6), (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16),
    (0, 1), (0, 2), (1, 3), (2, 4),
]

# ── Shared helpers (COCO-17) ─────────────────────────────────────────────────

def classify_body_position(kp_conf):
    T  = CRIT_THRESHOLD
    ls = kp_conf[L_SHOULDER] if L_SHOULDER < len(kp_conf) else 0.0
    rs = kp_conf[R_SHOULDER] if R_SHOULDER < len(kp_conf) else 0.0
    n  = kp_conf[NOSE]       if NOSE       < len(kp_conf) else 0.0
    both = ls > T and rs > T
    if both:
        return 'face_up' if n > T else 'face_down'
    diff = ls - rs
    if abs(diff) > 0.2:
        if ls > rs and ls > T:
            return 'lateral_right'
        if rs > ls and rs > T:
            return 'lateral_left'
    return 'unknown'


def select_injection_zone(kp_xy_norm, kp_conf, body_pos, frame_w, frame_h):
    """Returns (zone_type, px, py, confidence) or None. kp_xy_norm: (17,2) normalized."""
    T = CRIT_THRESHOLD

    def kpx(i): return int(kp_xy_norm[i][0] * frame_w)
    def kpy(i): return int(kp_xy_norm[i][1] * frame_h)

    def deltoid(sh_i, el_i):
        x = kpx(sh_i) + int(0.3 * (kpx(el_i) - kpx(sh_i)))
        y = kpy(sh_i) + int(0.3 * (kpy(el_i) - kpy(sh_i)))
        return x, y, float(min(kp_conf[sh_i], kp_conf[el_i]))

    def arm_mid(el_i, wr_i):
        return (kpx(el_i) + kpx(wr_i)) // 2, (kpy(el_i) + kpy(wr_i)) // 2, \
               float(min(kp_conf[el_i], kp_conf[wr_i]))

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
        if kp_conf[sh_i] > T and kp_conf[el_i] > T:
            dx, dy, c = deltoid(sh_i, el_i)
            return zone_type, dx, dy, c

    for zone_type, el_i, wr_i in [('arm_mid_left', L_ELBOW, L_WRIST),
                                   ('arm_mid_right', R_ELBOW, R_WRIST)]:
        if kp_conf[el_i] > T and kp_conf[wr_i] > T:
            mx, my, c = arm_mid(el_i, wr_i)
            return zone_type, mx, my, c
    return None


def estimate_distance_from_bbox(bbox_w_px, bbox_h_px, focal_length, ref_m=0.45):
    ref_px = max(bbox_w_px, bbox_h_px)
    if ref_px < 10 or focal_length <= 0:
        return None
    return (focal_length * ref_m) / ref_px


def compute_nav2_goal(zone_px, frame_w, focal_length, dist_m,
                       robot_x, robot_y, robot_yaw, work_offset=WORK_OFFSET_M):
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

class MoveNetDetector(Node):
    def __init__(self):
        super().__init__('movenet_detector')
        self.bridge = CvBridge()

        # Model path as ROS2 parameter so it can be overridden at launch
        self.declare_parameter('model_path',
                               os.path.expanduser('~/cbrn/movenet_thunder.tflite'))
        model_path = self.get_parameter('model_path').value

        if not os.path.exists(model_path):
            self.get_logger().error(
                f'MoveNet model not found at: {model_path}\n'
                'Download with: wget -O ~/cbrn/movenet_thunder.tflite '
                'https://tfhub.dev/google/lite-model/movenet/singlepose/thunder/tflite/float16/4?lite-format=tflite')
            raise FileNotFoundError(model_path)

        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details  = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.input_size = 256   # MoveNet Thunder uses 256×256

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

        # State
        self.depth_image    = None
        self.focal_length   = 462.0
        self.robot_x        = 0.0
        self.robot_y        = 0.0
        self.robot_yaw      = 0.0
        self.lidar_min_dist = None

        # CSV
        log_dir = os.path.expanduser('~/bench_logs')
        os.makedirs(log_dir, exist_ok=True)
        ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(log_dir, f'metrics_movenet_{ts}.csv')
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
        self.get_logger().info(f'MoveNet detector ready. Metrics CSV → {csv_path}')

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

    def _img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
        except Exception:
            return
        h, w = frame.shape[:2]

        # Preprocess
        rgb         = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(rgb, (self.input_size, self.input_size))
        input_data  = np.expand_dims(img_resized, axis=0).astype(np.float32)

        t0 = time.perf_counter()
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        inf_ms = (time.perf_counter() - t0) * 1000.0

        # Output: [1, 1, 17, 3]  →  each row is (y_norm, x_norm, score)
        raw        = self.interpreter.get_tensor(self.output_details[0]['index'])
        keypoints  = raw[0][0]   # shape (17, 3)

        # Reorder to (x_norm, y_norm) for consistency with other nodes
        kp_xy_norm = np.column_stack([keypoints[:, 1], keypoints[:, 0]])  # (17,2): x,y
        kp_conf    = keypoints[:, 2].tolist()

        # Default outputs
        target_msg = Point()
        target_msg.z = 0.0

        kp_msg = KeypointArray()
        kp_msg.header           = msg.header
        kp_msg.model_name       = 'movenet'
        kp_msg.inference_time_ms = inf_ms
        kp_msg.is_detected      = False
        kp_msg.num_keypoints    = 17

        inj_msg = InjectionZone()
        inj_msg.header        = msg.header
        inj_msg.body_position = 'unknown'
        inj_msg.zone_type     = 'unknown'
        inj_msg.goal_valid    = False

        csv_det  = False
        csv_dist, csv_bw, csv_bh = 0.0, 0.0, 0.0
        csv_crit  = [0.0] * len(CRITICAL_IDX)
        csv_above = 0
        csv_bpos  = 'unknown'
        csv_zone, csv_zconf = 'none', 0.0
        csv_gx = csv_gy = csv_gyaw = 0.0
        csv_gvalid = False

        # Consider detected if at least one critical keypoint is above threshold
        valid_mask = np.array(kp_conf) > CONF_THRESHOLD
        if valid_mask.any():
            csv_det = True
            kp_msg.is_detected = True
            kp_msg.kp_x        = kp_xy_norm[:, 0].tolist()
            kp_msg.kp_y        = kp_xy_norm[:, 1].tolist()
            kp_msg.kp_conf     = kp_conf

            # Bounding box from valid keypoints
            valid_x = kp_xy_norm[valid_mask, 0] * w
            valid_y = kp_xy_norm[valid_mask, 1] * h
            bx1, by1 = valid_x.min(), valid_y.min()
            bx2, by2 = valid_x.max(), valid_y.max()
            bw = bx2 - bx1;  bh = by2 - by1
            kp_msg.bbox_x1, kp_msg.bbox_y1 = float(bx1), float(by1)
            kp_msg.bbox_x2, kp_msg.bbox_y2 = float(bx2), float(by2)
            kp_msg.bbox_width_px  = float(bw)
            kp_msg.bbox_height_px = float(bh)
            csv_bw, csv_bh = bw, bh

            csv_crit  = [kp_conf[i] for i in CRITICAL_IDX]
            csv_above = int(valid_mask.sum())

            # Distance
            dist_m = estimate_distance_from_bbox(bw, bh, self.focal_length)
            if dist_m is None and self.lidar_min_dist is not None:
                dist_m = self.lidar_min_dist
            if dist_m is None:
                dist_m = 0.0
            csv_dist = dist_m
            inj_msg.distance_m = dist_m

            # Body position
            body_pos = classify_body_position(kp_conf)
            csv_bpos = body_pos
            inj_msg.body_position = body_pos

            # Injection zone
            zone = select_injection_zone(kp_xy_norm, kp_conf, body_pos, w, h)
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

                if dist_m > 0.05:
                    gx, gy, gyaw = compute_nav2_goal(
                        zone_px, w, self.focal_length,
                        dist_m, self.robot_x, self.robot_y, self.robot_yaw)
                    inj_msg.nav_goal_x   = gx
                    inj_msg.nav_goal_y   = gy
                    inj_msg.nav_goal_yaw = gyaw
                    inj_msg.goal_valid   = True
                    csv_gx, csv_gy, csv_gyaw, csv_gvalid = gx, gy, gyaw, True

            # Draw skeleton
            for i, (kx, ky) in enumerate(kp_xy_norm):
                if kp_conf[i] > CONF_THRESHOLD:
                    cv2.circle(frame, (int(kx * w), int(ky * h)), 5, (0, 255, 255), -1)
            for p1, p2 in SKELETON:
                if kp_conf[p1] > CRIT_THRESHOLD and kp_conf[p2] > CRIT_THRESHOLD:
                    pt1 = (int(kp_xy_norm[p1][0] * w), int(kp_xy_norm[p1][1] * h))
                    pt2 = (int(kp_xy_norm[p2][0] * w), int(kp_xy_norm[p2][1] * h))
                    cv2.line(frame, pt1, pt2, (0, 255, 255), 2)

            # Bounding box + centre target
            cv2.rectangle(frame, (int(bx1), int(by1)), (int(bx2), int(by2)), (255, 165, 0), 2)
            cx = int((bx1 + bx2) / 2)
            cy = int((by1 + by2) / 2)
            target_msg.x = float((cx / w) - 0.5)
            target_msg.y = float(cy)
            target_msg.z = float(dist_m)

        cv2.putText(frame, f'MoveNet | {inf_ms:.1f} ms | {csv_bpos}',
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)

        self.target_pub.publish(target_msg)
        self.kp_pub.publish(kp_msg)
        self.inj_pub.publish(inj_msg)
        try:
            out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out.header = msg.header
            self.result_pub.publish(out)
        except Exception as e:
            self.get_logger().error(f'Image publish error: {e}')

        self._csv.writerow(
            [time.time(), 'movenet', int(csv_det), f'{inf_ms:.2f}',
             f'{csv_dist:.3f}', f'{csv_bw:.1f}', f'{csv_bh:.1f}']
            + [f'{c:.3f}' for c in csv_crit]
            + [csv_above, csv_bpos, csv_zone, f'{csv_zconf:.3f}',
               f'{csv_gx:.3f}', f'{csv_gy:.3f}', f'{csv_gyaw:.4f}', int(csv_gvalid)]
        )
        self._csv_file.flush()

        cv2.imshow('MoveNet View', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self._csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = MoveNetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
