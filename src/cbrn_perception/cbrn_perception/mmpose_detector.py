#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
import csv
import time
import os
from datetime import datetime
from cbrn_interfaces.msg import PerceptionMetrics

HAS_DISPLAY = bool(os.environ.get("DISPLAY"))

from mmengine.registry import MODELS
try:
    import mmdet.models
    import mmpretrain.models
    MODELS.import_from_lib('mmdet')
except Exception as e:
    print(f"[WARN] Registry import error: {e}")

try:
    from mmpose.apis import MMPoseInferencer
except ImportError:
    print("ERROR: MMPose not installed.")
    exit()

# COCO critical keypoint indices used for skeleton_completeness
CRITICAL_KP = [5, 6, 7, 8, 9, 10]  # shoulders, elbows, wrists
CONFIDENCE_THRESHOLD = 0.3


def _compute_completeness(scores, threshold=CONFIDENCE_THRESHOLD):
    if len(scores) < 17:
        return 0.0
    detected = sum(1 for i in CRITICAL_KP if scores[i] >= threshold)
    return detected / len(CRITICAL_KP)


class UniversalPoseDetector(Node):
    def __init__(self):
        super().__init__("universal_pose_detector")
        self.bridge = CvBridge()
        self.image_saved = False

        self.declare_parameter("model_config", "human")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("csv_output", True)

        self.add_on_set_parameters_callback(self.parameter_callback)
        self._load_model(
            self.get_parameter("model_config").value,
            self.get_parameter("device").value,
        )

        self.focal_length = 800.0
        self.robot_distance_gt = 0.0

        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.camera_info_cb, 10
        )
        self.image_sub = self.create_subscription(
            Image, "/camera", self.img_cb, 10
        )
        self.distance_gt_sub = self.create_subscription(
            Float64, "/bench/robot_distance_gt", self.distance_gt_cb, 10
        )

        self.target_pub = self.create_publisher(Point, "/perception/target", 10)
        self.result_image_pub = self.create_publisher(
            Image, "/pose_estimation/image_result", 10
        )
        self.metrics_pub = self.create_publisher(
            PerceptionMetrics, "/pose_estimation/metrics", 10
        )

        self.setup_csv()

    def _load_model(self, model_cfg, device):
        self.get_logger().info(f"Loading model: {model_cfg} on {device}...")
        try:
            self.inferencer = MMPoseInferencer(
                pose2d=model_cfg, device=device, scope="mmpose"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            exit()

    def parameter_callback(self, params):
        for param in params:
            if param.name == "model_config":
                self.get_logger().warn(f"Reloading model: {param.value}")
                try:
                    self.inferencer = MMPoseInferencer(
                        pose2d=param.value,
                        device=self.get_parameter("device").value,
                        scope="mmpose",
                    )
                    self.setup_csv()
                except Exception as e:
                    self.get_logger().error(f"Model reload failed: {e}")
                    return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    def camera_info_cb(self, msg):
        if msg.k[0] > 0:
            self.focal_length = msg.k[0]

    def distance_gt_cb(self, msg):
        self.robot_distance_gt = msg.data

    def setup_csv(self):
        timestamp = datetime.now().strftime("%H%M%S")
        directory = os.path.expanduser("~/cbrn_ws/csv")
        os.makedirs(directory, exist_ok=True)
        model_name = self.get_parameter("model_config").value.replace("/", "_")
        self.csv_filename = os.path.join(
            directory, f"landmarks_{model_name}_{timestamp}.csv"
        )
        self.csv_file = open(self.csv_filename, mode="w", newline="")
        self.writer = csv.writer(self.csv_file)
        header = (
            ["Timestamp", "Model", "Inference_ms", "Is_Detected",
             "Confidence_Avg", "Skeleton_Completeness", "Distance_Estimate_m",
             "Distance_GT_m"]
            + [f"KP_{i}_x" for i in range(17)]
            + [f"KP_{i}_y" for i in range(17)]
            + [f"KP_{i}_conf" for i in range(17)]
        )
        self.writer.writerow(header)

    def img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        h, w = frame.shape[:2]
        t_start = time.perf_counter()
        result_generator = self.inferencer(frame, return_vis=False)
        result = next(result_generator)
        inference_time = time.perf_counter() - t_start

        predictions = result["predictions"]

        metrics = PerceptionMetrics()
        metrics.header = msg.header
        metrics.model_name = self.get_parameter("model_config").value
        metrics.inference_time = float(inference_time)
        metrics.is_detected = False
        metrics.confidence_score = 0.0
        metrics.distance_estimate = 0.0
        metrics.skeleton_completeness = 0.0
        metrics.robot_distance_gt = self.robot_distance_gt
        metrics.keypoint_scores = [0.0] * 17
        metrics.keypoint_x = [0.0] * 17
        metrics.keypoint_y = [0.0] * 17

        if predictions and len(predictions[0]) > 0:
            metrics.is_detected = True
            person = predictions[0][0]
            keypoints = np.array(person["keypoints"])
            scores = np.array(person["keypoint_scores"])

            n = min(len(scores), 17)
            kp_scores = [0.0] * 17
            kp_x = [0.0] * 17
            kp_y = [0.0] * 17
            for i in range(n):
                kp_scores[i] = float(scores[i])
                kp_x[i] = float(keypoints[i][0] / w)
                kp_y[i] = float(keypoints[i][1] / h)

            metrics.keypoint_scores = kp_scores
            metrics.keypoint_x = kp_x
            metrics.keypoint_y = kp_y
            metrics.confidence_score = float(np.mean(scores[:n]))
            metrics.skeleton_completeness = _compute_completeness(kp_scores)

            # Pinhole distance estimate
            valid_y = [keypoints[i][1] for i in range(n) if scores[i] > 0.1]
            if len(valid_y) > 2:
                h_px = max(valid_y) - min(valid_y)
                if h_px > 20:
                    metrics.distance_estimate = float(
                        (self.focal_length * 1.70) / h_px
                    )

            self._draw_skeleton(frame, keypoints, scores, n)

            target = Point()
            target.x = float(keypoints[0][0] / w - 0.5)
            self.target_pub.publish(target)

            # CSV row
            self.writer.writerow(
                [time.time(), metrics.model_name,
                 round(inference_time * 1000, 2),
                 metrics.is_detected, round(metrics.confidence_score, 4),
                 round(metrics.skeleton_completeness, 4),
                 round(metrics.distance_estimate, 3),
                 round(self.robot_distance_gt, 3)]
                + kp_x + kp_y + kp_scores
            )
            self.csv_file.flush()

        self.metrics_pub.publish(metrics)

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.result_image_pub.publish(out_msg)

        if HAS_DISPLAY:
            key = cv2.waitKey(1) & 0xFF
            if key == ord("s"):
                save_dir = os.path.expanduser("~/cbrn_ws/snapshots")
                os.makedirs(save_dir, exist_ok=True)
                fname = os.path.join(
                    save_dir,
                    f"{metrics.model_name}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.png",
                )
                cv2.imwrite(fname, frame)
            cv2.imshow("MMPose Universal Monitor", frame)

    def _draw_skeleton(self, frame, keypoints, scores, n):
        SKELETON = [
            (5, 7), (7, 9), (6, 8), (8, 10),
            (5, 6), (5, 11), (6, 12), (11, 12),
            (11, 13), (13, 15), (12, 14), (14, 16),
            (0, 1), (0, 2), (1, 3), (2, 4),
        ]
        for p1, p2 in SKELETON:
            if p1 < n and p2 < n and scores[p1] > 0.1 and scores[p2] > 0.1:
                cv2.line(
                    frame,
                    (int(keypoints[p1][0]), int(keypoints[p1][1])),
                    (int(keypoints[p2][0]), int(keypoints[p2][1])),
                    (255, 0, 0), 2,
                )
        for i in range(n):
            if scores[i] > 0.1:
                cv2.circle(
                    frame, (int(keypoints[i][0]), int(keypoints[i][1])), 5,
                    (0, 255, 0), -1,
                )


def main(args=None):
    rclpy.init(args=args)
    node = UniversalPoseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
