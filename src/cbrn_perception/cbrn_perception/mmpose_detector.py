#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import csv
import time
import os
from datetime import datetime

from cbrn_interfaces.msg import KeypointArray, InjectionZone, PerceptionMetrics

# ── MMPose registry fix (required for one-stage models such as RTMO) ──────────
from mmengine.registry import MODELS
try:
    import mmdet.models
    import mmpretrain.models
    MODELS.import_from_lib('mmdet')
except Exception as e:
    print(f'[WARN] Registry load error: {e}')

try:
    from mmpose.apis import MMPoseInferencer
except ImportError:
    print('MMPose is not installed.')
    raise

# ── COCO-17 keypoint indices ──────────────────────────────────────────────────
NOSE       = 0
L_SHOULDER = 5;  R_SHOULDER = 6
L_ELBOW    = 7;  R_ELBOW    = 8
L_WRIST    = 9;  R_WRIST    = 10
L_HIP      = 11; R_HIP      = 12

CRITICAL_IDX   = [L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW, L_WRIST, R_WRIST]
CRITICAL_NAMES = ['LShoulder', 'RShoulder', 'LElbow', 'RElbow', 'LWrist', 'RWrist']

CONF_THRESHOLD = 0.1
CRIT_THRESHOLD = 0.3
WORK_OFFSET_M  = 0.5
REAL_HEIGHT_M  = 1.70   # assumed standing person height for pinhole distance

SKELETON = [
    (5, 7), (7, 9), (6, 8), (8, 10),
    (5, 6), (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16),
    (0, 1), (0, 2), (1, 3), (2, 4),
]

# ── Injection-zone helpers (COCO-17) ─────────────────────────────────────────

def classify_body_position(kp_conf: list) -> str:
    T  = CRIT_THRESHOLD
    ls = kp_conf[L_SHOULDER] if L_SHOULDER < len(kp_conf) else 0.0
    rs = kp_conf[R_SHOULDER] if R_SHOULDER < len(kp_conf) else 0.0
    n  = kp_conf[NOSE]       if NOSE       < len(kp_conf) else 0.0
    if ls > T and rs > T:
        return 'face_up' if n > T else 'face_down'
    diff = ls - rs
    if abs(diff) > 0.2:
        if ls > rs and ls > T:
            return 'lateral_right'
        if rs > ls and rs > T:
            return 'lateral_left'
    return 'unknown'


def select_injection_zone(kp_xy_norm, kp_conf: list, body_pos: str,
                           frame_w: int, frame_h: int):
    """Returns (zone_type, px, py, confidence) or None.
    kp_xy_norm: (N, 2) array of normalized [x, y] coords."""
    T = CRIT_THRESHOLD

    def kpx(i): return int(kp_xy_norm[i][0] * frame_w)
    def kpy(i): return int(kp_xy_norm[i][1] * frame_h)

    def deltoid(sh_i, el_i):
        x = kpx(sh_i) + int(0.3 * (kpx(el_i) - kpx(sh_i)))
        y = kpy(sh_i) + int(0.3 * (kpy(el_i) - kpy(sh_i)))
        return x, y, float(min(kp_conf[sh_i], kp_conf[el_i]))

    def arm_mid(el_i, wr_i):
        return ((kpx(el_i) + kpx(wr_i)) // 2,
                (kpy(el_i) + kpy(wr_i)) // 2,
                float(min(kp_conf[el_i], kp_conf[wr_i])))

    primaries = {
        'face_up':       [('deltoid_right', L_SHOULDER, L_ELBOW),
                          ('deltoid_left',  R_SHOULDER, R_ELBOW)],
        'face_down':     [('deltoid_left',  R_SHOULDER, R_ELBOW),
                          ('deltoid_right', L_SHOULDER, L_ELBOW)],
        'lateral_right': [('deltoid_right', L_SHOULDER, L_ELBOW)],
        'lateral_left':  [('deltoid_left',  R_SHOULDER, R_ELBOW)],
        'unknown':       [('deltoid_right', L_SHOULDER, L_ELBOW),
                          ('deltoid_left',  R_SHOULDER, R_ELBOW)],
    }
    for zone_type, sh_i, el_i in primaries.get(body_pos, []):
        if kp_conf[sh_i] > T and kp_conf[el_i] > T:
            return (zone_type,) + deltoid(sh_i, el_i)

    for zone_type, el_i, wr_i in [('arm_mid_left', L_ELBOW, L_WRIST),
                                   ('arm_mid_right', R_ELBOW, R_WRIST)]:
        if kp_conf[el_i] > T and kp_conf[wr_i] > T:
            return (zone_type,) + arm_mid(el_i, wr_i)
    return None


def estimate_distance_from_bbox(bbox_h_px: float, focal_length: float,
                                 ref_m: float = REAL_HEIGHT_M) -> float | None:
    """Pinhole: D = (f * H_real) / H_px.  Returns None if inputs are invalid."""
    if bbox_h_px > 20 and focal_length > 0:
        return (focal_length * ref_m) / bbox_h_px
    return None


def compute_nav2_goal(zone_px: int, frame_w: int, focal_length: float,
                       dist_m: float, robot_x: float, robot_y: float,
                       robot_yaw: float, work_offset: float = WORK_OFFSET_M):
    angle_h  = math.atan2(zone_px - frame_w / 2.0, focal_length)
    zone_r_x = dist_m * math.cos(angle_h)
    zone_r_y = dist_m * math.sin(angle_h)
    cy, sy   = math.cos(robot_yaw), math.sin(robot_yaw)
    zone_m_x = robot_x + zone_r_x * cy - zone_r_y * sy
    zone_m_y = robot_y + zone_r_x * sy + zone_r_y * cy
    approach  = math.atan2(zone_m_y - robot_y, zone_m_x - robot_x)
    goal_x    = zone_m_x - work_offset * math.cos(approach)
    goal_y    = zone_m_y - work_offset * math.sin(approach)
    return goal_x, goal_y, approach


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class UniversalPoseDetector(Node):
    def __init__(self):
        super().__init__('universal_pose_detector')
        self.bridge = CvBridge()

        self.declare_parameter('model_config', 'human')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('csv_output', True)

        self.add_on_set_parameters_callback(self._param_cb)
        self._load_model()

        # State
        self.focal_length   = 800.0
        self.robot_x        = 0.0
        self.robot_y        = 0.0
        self.robot_yaw      = 0.0
        self.lidar_min_dist = None
        self.image_saved    = False

        # Publishers
        self.target_pub     = self.create_publisher(Point,            '/perception/target',            10)
        self.result_pub     = self.create_publisher(Image,            '/pose_estimation/image_result', 10)
        self.metrics_pub    = self.create_publisher(PerceptionMetrics, '/pose_estimation/metrics',     10)
        self.kp_pub         = self.create_publisher(KeypointArray,    '/keypoints',                    10)
        self.inj_pub        = self.create_publisher(InjectionZone,    '/injection_zone',               10)

        # Subscribers
        self.create_subscription(Image,      '/camera',                     self._img_cb,   10)
        self.create_subscription(CameraInfo, '/camera/camera_info',         self._info_cb,  10)
        self.create_subscription(Odometry,   '/diff_drive_controller/odom', self._odom_cb,  10)
        self.create_subscription(LaserScan,  '/scan',                       self._scan_cb,  10)

        # CSV
        log_dir = os.path.expanduser('~/bench_logs')
        os.makedirs(log_dir, exist_ok=True)
        ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(log_dir, f'metrics_mmpose_{ts}.csv')
        self._csv_file = open(csv_path, mode='w', newline='')
        self._csv      = csv.writer(self._csv_file)
        self._csv.writerow(
            ['Timestamp', 'Model', 'Detected', 'Inference_ms',
             'Distance_m', 'BBox_W_px', 'BBox_H_px']
            + [f'Conf_{n}' for n in CRITICAL_NAMES]
            + ['Kpts_Above_Thresh', 'Body_Position',
               'Injection_Zone', 'Zone_Conf',
               'Goal_X', 'Goal_Y', 'Goal_Yaw', 'Goal_Valid']
        )
        self.get_logger().info(f'MMPose detector ready. CSV → {csv_path}')

    # ── Model management ──────────────────────────────────────────────────────

    def _load_model(self):
        model_cfg = self.get_parameter('model_config').value
        device    = self.get_parameter('device').value
        self.get_logger().info(f'Loading MMPose model: {model_cfg} on {device}')
        try:
            self.inferencer = MMPoseInferencer(pose2d=model_cfg, device=device,
                                               scope='mmpose')
        except Exception as e:
            self.get_logger().error(f'Model load failed: {e}')
            raise

    def _param_cb(self, params):
        for p in params:
            if p.name == 'model_config':
                self.get_logger().warn(f'Reloading model: {p.value}')
                try:
                    self.inferencer = MMPoseInferencer(
                        pose2d=p.value,
                        device=self.get_parameter('device').value,
                        scope='mmpose')
                except Exception as e:
                    self.get_logger().error(f'Model reload failed: {e}')
                    return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    # ── Sensor callbacks ──────────────────────────────────────────────────────

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

    # ── Main image callback ───────────────────────────────────────────────────

    def _img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
        except Exception:
            return
        h, w = frame.shape[:2]

        t0 = time.perf_counter()
        result      = next(self.inferencer(frame, return_vis=False))
        inf_ms      = (time.perf_counter() - t0) * 1000.0
        predictions = result['predictions']

        model_cfg = self.get_parameter('model_config').value

        # ── Default empty messages ────────────────────────────────────────────
        target_msg = Point()

        kp_msg = KeypointArray()
        kp_msg.header          = msg.header
        kp_msg.model_name      = model_cfg
        kp_msg.inference_time_ms = inf_ms
        kp_msg.is_detected     = False
        kp_msg.num_keypoints   = 17

        inj_msg = InjectionZone()
        inj_msg.header        = msg.header
        inj_msg.body_position = 'unknown'
        inj_msg.zone_type     = 'unknown'
        inj_msg.goal_valid    = False

        metrics_msg = PerceptionMetrics()
        metrics_msg.model_name       = model_cfg
        metrics_msg.header           = msg.header
        metrics_msg.inference_time   = float(inf_ms)
        metrics_msg.is_detected      = False
        metrics_msg.confidence_score = 0.0
        metrics_msg.distance_estimate = 0.0

        csv_det = False
        csv_dist, csv_bw, csv_bh = 0.0, 0.0, 0.0
        csv_crit  = [0.0] * len(CRITICAL_IDX)
        csv_above = 0
        csv_bpos  = 'unknown'
        csv_zone, csv_zconf = 'none', 0.0
        csv_gx = csv_gy = csv_gyaw = 0.0
        csv_gvalid = False

        # ── Process first detected person ─────────────────────────────────────
        persons = predictions[0] if predictions and len(predictions[0]) > 0 else []
        if persons:
            # Pick the person with the tallest bounding box (closest / most visible)
            best = max(persons, key=lambda p: (
                max(kp[1] for kp in p['keypoints']) - min(kp[1] for kp in p['keypoints'])
                if p['keypoints'] else 0.0
            ))

            keypoints = np.array(best['keypoints'])   # (N, 2) pixel coords
            scores    = np.array(best['keypoint_scores'])  # (N,)
            kp_conf   = scores.tolist()

            # Normalize to [0, 1]
            kp_xy_norm = np.column_stack([keypoints[:, 0] / w, keypoints[:, 1] / h])

            valid_mask = scores > CONF_THRESHOLD
            if valid_mask.any():
                csv_det = True
                kp_msg.is_detected = True
                kp_msg.kp_x        = kp_xy_norm[:, 0].tolist()
                kp_msg.kp_y        = kp_xy_norm[:, 1].tolist()
                kp_msg.kp_conf     = kp_conf

                # Bounding box from valid keypoints (pixel coords)
                vx = keypoints[valid_mask, 0]
                vy = keypoints[valid_mask, 1]
                bx1, bx2 = vx.min(), vx.max()
                by1, by2 = vy.min(), vy.max()
                bw = bx2 - bx1;  bh = by2 - by1
                kp_msg.bbox_x1, kp_msg.bbox_y1 = float(bx1), float(by1)
                kp_msg.bbox_x2, kp_msg.bbox_y2 = float(bx2), float(by2)
                kp_msg.bbox_width_px  = float(bw)
                kp_msg.bbox_height_px = float(bh)
                csv_bw, csv_bh = bw, bh

                csv_crit  = [kp_conf[i] if i < len(kp_conf) else 0.0 for i in CRITICAL_IDX]
                csv_above = int(valid_mask.sum())

                metrics_msg.is_detected      = True
                metrics_msg.confidence_score = float(np.mean(scores[valid_mask]))

                # Distance: pinhole on bbox height, LiDAR fallback
                dist_m = estimate_distance_from_bbox(bh, self.focal_length)
                if dist_m is None and self.lidar_min_dist is not None:
                    dist_m = self.lidar_min_dist
                if dist_m is None:
                    dist_m = 0.0
                csv_dist = dist_m
                inj_msg.distance_m            = dist_m
                metrics_msg.distance_estimate = float(dist_m)

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

                # Skeleton + keypoints overlay
                for i, (kx, ky) in enumerate(kp_xy_norm):
                    if kp_conf[i] > CONF_THRESHOLD:
                        cv2.circle(frame, (int(kx * w), int(ky * h)), 5, (0, 255, 0), -1)
                for p1, p2 in SKELETON:
                    if (p1 < len(kp_conf) and p2 < len(kp_conf)
                            and kp_conf[p1] > CRIT_THRESHOLD
                            and kp_conf[p2] > CRIT_THRESHOLD):
                        pt1 = (int(kp_xy_norm[p1][0] * w), int(kp_xy_norm[p1][1] * h))
                        pt2 = (int(kp_xy_norm[p2][0] * w), int(kp_xy_norm[p2][1] * h))
                        cv2.line(frame, pt1, pt2, (255, 0, 0), 2)

                # Bounding box + centroid target
                cv2.rectangle(frame, (int(bx1), int(by1)), (int(bx2), int(by2)),
                              (255, 165, 0), 2)
                cx = (bx1 + bx2) / 2.0
                cy_px = (by1 + by2) / 2.0
                cv2.putText(frame, f'D:{dist_m:.2f}m',
                            (int(bx1), int(by1) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                target_msg.x = float(cx / w - 0.5)
                target_msg.y = float(cy_px)
                target_msg.z = float(dist_m)

        # ── Status overlay ────────────────────────────────────────────────────
        cv2.putText(frame,
                    f'MMPose {model_cfg} | {inf_ms:.1f} ms | {csv_bpos}',
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)

        # ── Publish ───────────────────────────────────────────────────────────
        self.target_pub.publish(target_msg)
        self.kp_pub.publish(kp_msg)
        self.inj_pub.publish(inj_msg)
        self.metrics_pub.publish(metrics_msg)
        try:
            out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out.header = msg.header
            self.result_pub.publish(out)
        except Exception as e:
            self.get_logger().error(f'Image publish error: {e}')

        # ── CSV ───────────────────────────────────────────────────────────────
        self._csv.writerow(
            [time.time(), model_cfg, int(csv_det), f'{inf_ms:.2f}',
             f'{csv_dist:.3f}', f'{csv_bw:.1f}', f'{csv_bh:.1f}']
            + [f'{c:.3f}' for c in csv_crit]
            + [csv_above, csv_bpos, csv_zone, f'{csv_zconf:.3f}',
               f'{csv_gx:.3f}', f'{csv_gy:.3f}', f'{csv_gyaw:.4f}', int(csv_gvalid)]
        )
        self._csv_file.flush()

        # ── Display ───────────────────────────────────────────────────────────
        cv2.imshow('MMPose Universal Monitor', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            save_dir = '/home/ai/cbrn/cbrn/src/models_images'
            os.makedirs(save_dir, exist_ok=True)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            cv2.imwrite(os.path.join(save_dir, f'{model_cfg}_{ts}.png'), frame)
            self.image_saved = True

    def destroy_node(self):
        self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UniversalPoseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()