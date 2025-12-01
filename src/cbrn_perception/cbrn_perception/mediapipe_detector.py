#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import csv
import time
import os
from datetime import datetime

class MediaPipeDetector(Node):
    def __init__(self):
        super().__init__("mediapipe_detector")
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(Image, "/camera", self.img_cb, 10)
        self.depth_sub = self.create_subscription(Image, "/camera/depth", self.depth_cb, 10)
        # Publicăm ȚINTA către controller
        self.target_pub = self.create_publisher(Point, "/perception/target", 10)
        self.result_image_pub = self.create_publisher(Image, "/pose_estimation/image_result", 10)

        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.2, min_tracking_confidence=0.2)
        self.mp_drawing = mp.solutions.drawing_utils
        self.depth_image = None

        # --- SETUP CSV ---
        timestamp = datetime.now().strftime("%H%M%S")
        directory = os.path.expanduser("~/cbrn_ws/csv")
        self.csv_filename = os.path.join(directory, f"landmarks_mediapipe_{timestamp}.csv")
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)

    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def img_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb)
        
        target_msg = Point()
        target_msg.z = 0.0 # Semnal că nu vedem nimic

        if results.pose_landmarks:
            h, w, _ = frame.shape
            
            row = [time.time()]

            for lm in results.pose_landmarks.landmark:

                # MediaPipe dă coordonate normalizate (0.0 - 1.0)

                row.extend([lm.x, lm.y, lm.z, lm.visibility])

            self.writer.writerow(row)

            # Desenăm scheletul
            self.mp_drawing.draw_landmarks(frame, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
            
            # Calculăm centrul
            xs = [lm.x for lm in results.pose_landmarks.landmark]
            ys = [lm.y for lm in results.pose_landmarks.landmark]
            cx = int(np.mean(xs) * w)
            cy = int(np.mean(ys) * h)
            
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
            
            # Calculăm distanța
            dist = 5.0
            if self.depth_image is not None:
                try:
                    dist = float(self.depth_image[cy, cx])
                    if np.isnan(dist) or dist <= 0: dist = 5.0
                except: pass
            
            # Normalizăm X (-0.5 stânga, 0.5 dreapta)
            norm_x = (cx / w) - 0.5
            
            # Populăm mesajul
            target_msg.x = float(norm_x)
            target_msg.y = float(cy) # Opțional
            target_msg.z = float(dist)

        # Publicăm rezultatul către Controller
        self.target_pub.publish(target_msg)
        try:
            # Convertim înapoi din OpenCV (BGR) în ROS Message
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            out_msg.header = msg.header # Păstrăm timestamp-ul original pentru sincronizare
            self.result_image_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Eroare publicare imagine: {e}")
        cv2.imshow("MediaPipe View", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = MediaPipeDetector()
    rclpy.spin(node)
    rclpy.shutdown()