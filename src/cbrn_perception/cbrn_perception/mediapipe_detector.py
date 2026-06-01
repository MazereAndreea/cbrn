#!/usr/bin/env python3
"""MediaPipe Pose Landmark detector using direct TFLite inference.

MediaPipe's Tasks API (PoseLandmarker) crashes in WSL2 due to its C++ TaskRunner
initialising GPU/XNNPACK delegates against unavailable hardware.  This implementation
uses tensorflow.lite.Interpreter directly — the same approach as movenet_detector —
bypassing the Tasks API entirely.

Two-stage pipeline (same as native MediaPipe):
  Stage 1 — pose_detection.tflite  : detect person ROI from full frame
  Stage 2 — pose_landmark_full.tflite : extract 33 BlazePose landmarks from ROI
"""
import os
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

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
tflite = tf.lite

HAS_DISPLAY = bool(os.environ.get("DISPLAY"))

# Model paths — bundled inside the mediapipe .local install
_MP_BASE = os.path.expanduser(
    "~/.local/lib/python3.12/site-packages/mediapipe/modules"
)
DETECTION_MODEL = os.path.join(_MP_BASE, "pose_detection", "pose_detection.tflite")
LANDMARK_MODEL  = os.path.join(_MP_BASE, "pose_landmark", "pose_landmark_full.tflite")

# BlazePose 33-landmark → COCO 17-keypoint mapping
MP_TO_COCO = {
    0: 0, 2: 1, 5: 2, 7: 3, 8: 4,
    11: 5, 12: 6, 13: 7, 14: 8, 15: 9, 16: 10,
    23: 11, 24: 12, 25: 13, 26: 14, 27: 15, 28: 16,
}
COCO_SKELETON = [
    (5,7),(7,9),(6,8),(8,10),(5,6),
    (5,11),(6,12),(11,12),(11,13),(13,15),(12,14),(14,16),
]
CRITICAL_KP = [5, 6, 7, 8, 9, 10]
CONF_THRESHOLD = 0.3


def _compute_completeness(scores):
    detected = sum(1 for i in CRITICAL_KP if scores[i] >= CONF_THRESHOLD)
    return detected / len(CRITICAL_KP)


def _load_interpreter(path):
    interp = tflite.Interpreter(model_path=path)
    interp.allocate_tensors()
    return interp, interp.get_input_details(), interp.get_output_details()


def _run(interp, in_details, out_details, data):
    interp.set_tensor(in_details[0]["index"], data)
    interp.invoke()
    return [interp.get_tensor(d["index"]) for d in out_details]


class MediaPipeDetector(Node):
    def __init__(self):
        super().__init__("mediapipe_detector")
        self.bridge = CvBridge()
        self.robot_distance_gt = 0.0
        self.focal_length = 800.0

        if not os.path.exists(DETECTION_MODEL) or not os.path.exists(LANDMARK_MODEL):
            self.get_logger().error(
                f"MediaPipe TFLite models not found at {_MP_BASE}. "
                "Install mediapipe 0.10.21:  pip install 'mediapipe==0.10.21'"
            )
            raise RuntimeError("Missing MediaPipe TFLite models")

        self.get_logger().info("Loading TFLite models...")
        self._det_interp, self._det_in, self._det_out = _load_interpreter(DETECTION_MODEL)
        self._lm_interp,  self._lm_in,  self._lm_out  = _load_interpreter(LANDMARK_MODEL)
        # Detection expects [1, 224, 224, 3] float32
        self._det_size = self._det_in[0]["shape"][1]   # 224
        # Landmark expects [1, 256, 256, 3] float32
        self._lm_size  = self._lm_in[0]["shape"][1]    # 256

        self.image_sub = self.create_subscription(Image, "/camera", self.img_cb, 10)
        self.distance_gt_sub = self.create_subscription(
            Float64, "/bench/robot_distance_gt", self.distance_gt_cb, 10)
        self.target_pub = self.create_publisher(Point, "/perception/target", 10)
        self.result_image_pub = self.create_publisher(Image, "/pose_estimation/image_result", 10)
        self.metrics_pub = self.create_publisher(PerceptionMetrics, "/pose_estimation/metrics", 10)

        directory = os.path.expanduser("~/cbrn_ws/csv")
        os.makedirs(directory, exist_ok=True)
        ts = datetime.now().strftime("%H%M%S")
        self.csv_filename = os.path.join(directory, f"landmarks_mediapipe_{ts}.csv")
        self.csv_file = open(self.csv_filename, mode="w", newline="")
        self.writer = csv.writer(self.csv_file)
        header = (
            ["Timestamp", "Model", "Inference_ms", "Is_Detected",
             "Confidence_Avg", "Skeleton_Completeness", "Distance_Estimate_m", "Distance_GT_m"]
            + [f"KP_{i}_x" for i in range(17)]
            + [f"KP_{i}_y" for i in range(17)]
            + [f"KP_{i}_conf" for i in range(17)]
        )
        self.writer.writerow(header)
        self.get_logger().info(f"MediaPipe (TFLite) detector ready. CSV: {self.csv_filename}")

    def distance_gt_cb(self, msg):
        self.robot_distance_gt = msg.data

    def _detect_roi(self, frame_rgb):
        """Run detection model; return (x1,y1,x2,y2) in pixel coords or None."""
        h, w = frame_rgb.shape[:2]
        det_img = cv2.resize(frame_rgb, (self._det_size, self._det_size))
        det_inp = np.expand_dims(det_img.astype(np.float32) / 255.0, axis=0)
        outs = _run(self._det_interp, self._det_in, self._det_out, det_inp)
        boxes, scores = outs[0][0], outs[1][0, :, 0]   # [N,12], [N]
        best = int(np.argmax(scores))
        if scores[best] < 0.5:
            return None
        # boxes format: [ymin, xmin, ymax, xmax] normalised (SSD-style)
        ymin, xmin, ymax, xmax = boxes[best, :4]
        pad = 0.15
        ymin = max(0.0, ymin - pad); xmin = max(0.0, xmin - pad)
        ymax = min(1.0, ymax + pad); xmax = min(1.0, xmax + pad)
        return (int(xmin*w), int(ymin*h), int(xmax*w), int(ymax*h))

    def _get_landmarks(self, roi_rgb):
        """Run landmark model on a person crop; return (kp_array[33,5], pose_flag)."""
        crop = cv2.resize(roi_rgb, (self._lm_size, self._lm_size))
        inp = np.expand_dims(crop.astype(np.float32) / 255.0, axis=0)
        outs = _run(self._lm_interp, self._lm_in, self._lm_out, inp)
        # Output[0]: [1, 195] → reshape to [39, 5]; first 33 rows are pose landmarks
        landmarks_flat = outs[0][0]   # [195]
        lms = landmarks_flat.reshape(39, 5)[:33]  # [33, 5]  (x,y,z,vis,presence) normalised
        pose_flag = float(outs[1][0, 0])
        return lms, pose_flag

    def img_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        t_start = time.perf_counter()

        roi = self._detect_roi(rgb)
        if roi is not None:
            x1, y1, x2, y2 = roi
            crop_rgb = rgb[max(0,y1):min(h,y2), max(0,x1):min(w,x2)]
            if crop_rgb.size == 0:
                roi = None

        if roi is None:
            x1, y1, x2, y2 = 0, 0, w, h
            crop_rgb = rgb

        lms, pose_flag = self._get_landmarks(crop_rgb)
        inference_time = time.perf_counter() - t_start

        roi_w = max(1, x2 - x1)
        roi_h = max(1, y2 - y1)

        kp_x = [0.0] * 17
        kp_y = [0.0] * 17
        kp_scores = [0.0] * 17

        is_detected = pose_flag > 0.5
        if is_detected:
            for mp_idx, coco_idx in MP_TO_COCO.items():
                # Convert from ROI-normalised to full-frame-normalised
                kp_x[coco_idx] = float((x1 + lms[mp_idx, 0] * roi_w) / w)
                kp_y[coco_idx] = float((y1 + lms[mp_idx, 1] * roi_h) / h)
                kp_scores[coco_idx] = float(lms[mp_idx, 3])  # visibility

            # Draw
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            for p1, p2 in COCO_SKELETON:
                if kp_scores[p1] > CONF_THRESHOLD and kp_scores[p2] > CONF_THRESHOLD:
                    cv2.line(frame,
                             (int(kp_x[p1]*w), int(kp_y[p1]*h)),
                             (int(kp_x[p2]*w), int(kp_y[p2]*h)),
                             (0, 255, 0), 2)
            for i in range(17):
                if kp_scores[i] > CONF_THRESHOLD:
                    cv2.circle(frame, (int(kp_x[i]*w), int(kp_y[i]*h)), 5, (0,255,0), -1)

        avg_conf = float(np.mean(kp_scores))
        completeness = _compute_completeness(kp_scores)

        # Pinhole distance from keypoint bounding box height
        dist_est = 0.0
        valid_y_px = [kp_y[i]*h for i in range(17) if kp_scores[i] > 0.1]
        if len(valid_y_px) > 2:
            h_px = max(valid_y_px) - min(valid_y_px)
            if h_px > 20:
                dist_est = float((self.focal_length * 1.70) / h_px)

        metrics = PerceptionMetrics()
        metrics.header = msg.header
        metrics.model_name = "mediapipe_tflite"
        metrics.inference_time = float(inference_time)
        metrics.is_detected = is_detected
        metrics.confidence_score = avg_conf
        metrics.skeleton_completeness = completeness
        metrics.distance_estimate = dist_est
        metrics.robot_distance_gt = self.robot_distance_gt
        metrics.keypoint_scores = kp_scores
        metrics.keypoint_x = kp_x
        metrics.keypoint_y = kp_y

        target = Point()
        if is_detected:
            target.x = float(kp_x[0] - 0.5)
        self.target_pub.publish(target)
        self.metrics_pub.publish(metrics)

        self.writer.writerow(
            [time.time(), "mediapipe_tflite", round(inference_time*1000, 2),
             is_detected, round(avg_conf, 4), round(completeness, 4),
             round(dist_est, 3), round(self.robot_distance_gt, 3)]
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
            cv2.imshow("MediaPipe TFLite View", frame)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = MediaPipeDetector()
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
