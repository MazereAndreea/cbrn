#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from cbrn_interfaces.msg import PerceptionMetrics
from ultralytics import YOLO
import cv2
import numpy as np
import csv
import time
import os
from datetime import datetime

HAS_DISPLAY = bool(os.environ.get("DISPLAY"))
CONFIDENCE_SCORE = 0.15
CRITICAL_KP = [5, 6, 7, 8, 9, 10]
CONFIDENCE_THRESHOLD = 0.3


def _compute_completeness(scores, threshold=CONFIDENCE_THRESHOLD):
    detected = sum(1 for i in CRITICAL_KP if scores[i] >= threshold)
    return detected / len(CRITICAL_KP)


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")
        self.bridge = CvBridge()
        self.robot_distance_gt = 0.0
        self.focal_length = 800.0

        self.model = YOLO("yolo11n-pose.pt")

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

        directory = os.path.expanduser("~/cbrn_ws/csv")
        os.makedirs(directory, exist_ok=True)
        timestamp = datetime.now().strftime("%H%M%S")
        self.csv_filename = os.path.join(
            directory, f"landmarks_yolo_{timestamp}.csv"
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
        self.get_logger().info(f"YOLO detector started. CSV: {self.csv_filename}")

    def distance_gt_cb(self, msg):
        self.robot_distance_gt = msg.data

    def img_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w = frame.shape[:2]

        t_start = time.perf_counter()
        results = self.model(frame, verbose=False, conf=CONFIDENCE_SCORE)
        inference_time = time.perf_counter() - t_start

        kp_x = [0.0] * 17
        kp_y = [0.0] * 17
        kp_scores = [0.0] * 17

        metrics = PerceptionMetrics()
        metrics.header = msg.header
        metrics.model_name = "yolo11n-pose"
        metrics.inference_time = float(inference_time)
        metrics.is_detected = False
        metrics.confidence_score = 0.0
        metrics.distance_estimate = 0.0
        metrics.skeleton_completeness = 0.0
        metrics.robot_distance_gt = self.robot_distance_gt

        target_msg = Point()

        for r in results:
            if r.keypoints is None or not r.keypoints.has_visible:
                continue

            metrics.is_detected = True
            kpts_norm = r.keypoints.xyn.cpu().numpy()[0]  # (17, 2) normalised
            confs = (
                r.keypoints.conf.cpu().numpy()[0]
                if r.keypoints.conf is not None
                else np.zeros(17)
            )

            n = min(len(confs), 17)
            for i in range(n):
                kp_x[i] = float(kpts_norm[i][0])
                kp_y[i] = float(kpts_norm[i][1])
                kp_scores[i] = float(confs[i])

            metrics.keypoint_scores = kp_scores
            metrics.keypoint_x = kp_x
            metrics.keypoint_y = kp_y
            metrics.confidence_score = float(np.mean(confs[:n]))
            metrics.skeleton_completeness = _compute_completeness(kp_scores)

            # Pinhole distance from bounding box
            if r.boxes:
                x1, y1, x2, y2 = r.boxes.xyxy[0].cpu().numpy()
                box_h = y2 - y1
                if box_h > 20:
                    metrics.distance_estimate = float(
                        (self.focal_length * 1.70) / box_h
                    )
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                target_msg.x = float((cx / w) - 0.5)
                cv2.rectangle(
                    frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2
                )

            for i in range(n):
                if kp_scores[i] > CONFIDENCE_THRESHOLD:
                    cv2.circle(
                        frame,
                        (int(kp_x[i] * w), int(kp_y[i] * h)),
                        5, (0, 0, 255), -1,
                    )
            break  # first person only

        metrics.keypoint_scores = kp_scores
        metrics.keypoint_x = kp_x
        metrics.keypoint_y = kp_y

        self.target_pub.publish(target_msg)
        self.metrics_pub.publish(metrics)

        self.writer.writerow(
            [time.time(), "yolo11n-pose", round(inference_time * 1000, 2),
             metrics.is_detected, round(metrics.confidence_score, 4),
             round(metrics.skeleton_completeness, 4),
             round(metrics.distance_estimate, 3),
             round(self.robot_distance_gt, 3)]
            + kp_x + kp_y + kp_scores
        )
        self.csv_file.flush()

        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            out_msg.header = msg.header
            self.result_image_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Image publish error: {e}")

        if HAS_DISPLAY:
            cv2.imshow("YOLO View", frame)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloDetector()
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
