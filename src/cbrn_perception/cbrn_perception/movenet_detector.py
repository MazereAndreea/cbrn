#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from cbrn_interfaces.msg import PerceptionMetrics
import cv2
import numpy as np
import csv
import time
import os
from datetime import datetime
import tensorflow as tf

HAS_DISPLAY = bool(os.environ.get("DISPLAY"))
tflite = tf.lite

CRITICAL_KP = [5, 6, 7, 8, 9, 10]
CONFIDENCE_THRESHOLD = 0.3


def _compute_completeness(scores, threshold=CONFIDENCE_THRESHOLD):
    detected = sum(1 for i in CRITICAL_KP if scores[i] >= threshold)
    return detected / len(CRITICAL_KP)


class MoveNetDetector(Node):
    def __init__(self):
        super().__init__("movenet_detector")
        self.bridge = CvBridge()
        self.robot_distance_gt = 0.0
        self.focal_length = 800.0

        model_path = os.path.expanduser("~/cbrn_ws/movenet_thunder.tflite")
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model not found at: {model_path}")
            exit()

        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.input_size = 256  # MoveNet Thunder

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
            directory, f"landmarks_movenet_{timestamp}.csv"
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
        self.get_logger().info(f"MoveNet detector started. CSV: {self.csv_filename}")

    def distance_gt_cb(self, msg):
        self.robot_distance_gt = msg.data

    def img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
        except Exception:
            return

        h, w = frame.shape[:2]

        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb, (self.input_size, self.input_size))
        input_data = np.expand_dims(img_resized, axis=0).astype(np.float32)

        t_start = time.perf_counter()
        self.interpreter.set_tensor(self.input_details[0]["index"], input_data)
        self.interpreter.invoke()
        inference_time = time.perf_counter() - t_start

        # Output: [1, 1, 17, 3]  ->  (y_norm, x_norm, score)
        raw = self.interpreter.get_tensor(self.output_details[0]["index"])[0][0]

        kp_x = [0.0] * 17
        kp_y = [0.0] * 17
        kp_scores = [0.0] * 17
        for i, kp in enumerate(raw):
            ky, kx, conf = float(kp[0]), float(kp[1]), float(kp[2])
            kp_x[i] = kx
            kp_y[i] = ky
            kp_scores[i] = conf

        is_detected = any(s >= CONFIDENCE_THRESHOLD for s in kp_scores)

        metrics = PerceptionMetrics()
        metrics.header = msg.header
        metrics.model_name = "movenet_thunder"
        metrics.inference_time = float(inference_time)
        metrics.is_detected = is_detected
        metrics.confidence_score = float(np.mean(kp_scores))
        metrics.skeleton_completeness = _compute_completeness(kp_scores)
        metrics.robot_distance_gt = self.robot_distance_gt
        metrics.keypoint_scores = kp_scores
        metrics.keypoint_x = kp_x
        metrics.keypoint_y = kp_y

        # Pinhole distance from valid keypoint bounding box
        valid_y_px = [kp_y[i] * h for i in range(17) if kp_scores[i] > 0.1]
        if len(valid_y_px) > 2:
            h_px = max(valid_y_px) - min(valid_y_px)
            if h_px > 20:
                metrics.distance_estimate = float(
                    (self.focal_length * 1.70) / h_px
                )
        else:
            metrics.distance_estimate = 0.0

        target_msg = Point()
        if is_detected:
            valid_x = [kp_x[i] * w for i in range(17) if kp_scores[i] >= CONFIDENCE_THRESHOLD]
            valid_y = [kp_y[i] * h for i in range(17) if kp_scores[i] >= CONFIDENCE_THRESHOLD]
            if valid_x:
                cx = int(np.mean(valid_x))
                target_msg.x = float(cx / w - 0.5)
                # Draw skeleton
                for i in range(17):
                    if kp_scores[i] > CONFIDENCE_THRESHOLD:
                        cv2.circle(
                            frame,
                            (int(kp_x[i] * w), int(kp_y[i] * h)),
                            5, (0, 255, 255), -1,
                        )
                if valid_y:
                    cv2.rectangle(
                        frame,
                        (int(min(valid_x)), int(min(valid_y))),
                        (int(max(valid_x)), int(max(valid_y))),
                        (0, 255, 255), 2,
                    )

        self.target_pub.publish(target_msg)
        self.metrics_pub.publish(metrics)

        self.writer.writerow(
            [time.time(), "movenet_thunder", round(inference_time * 1000, 2),
             is_detected, round(metrics.confidence_score, 4),
             round(metrics.skeleton_completeness, 4),
             round(metrics.distance_estimate, 3),
             round(self.robot_distance_gt, 3)]
            + kp_x + kp_y + kp_scores
        )
        self.csv_file.flush()

        out_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        out_msg.header = msg.header
        self.result_image_pub.publish(out_msg)
        if HAS_DISPLAY:
            cv2.imshow("MoveNet View", frame)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = MoveNetDetector()
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
