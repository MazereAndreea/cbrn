#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import csv
import time
import os
from datetime import datetime
import tensorflow as tf
tflite = tf.lite

class MoveNetDetector(Node):
    def __init__(self):
        super().__init__("movenet_detector")
        self.bridge = CvBridge()
        
        # 1. Încărcare Model MoveNet
        # Asigură-te că ai descărcat fișierul .tflite în folderul home sau specifică calea
        model_path = "/home/andreea/cbrn_ws/movenet_thunder.tflite"
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"Nu gasesc modelul la: {model_path}. Descarca-l cu wget!")
            exit()

        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.input_size = 256 # MoveNet Thunder folosește 256x256

        self.image_sub = self.create_subscription(Image, "/camera", self.img_cb, 10)
        self.depth_sub = self.create_subscription(Image, "/camera/depth", self.depth_cb, 10)
        self.target_pub = self.create_publisher(Point, "/perception/target", 10)
        self.result_image_pub = self.create_publisher(Image, "/pose_estimation/image_result", 10)
        
        self.depth_image = None

        # --- CSV SETUP (17 Keypoints - COCO Format) ---
        timestamp = datetime.now().strftime("%H%M%S")
        self.csv_filename = f"landmarks_movenet_{timestamp}.csv"
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        
        header = ["Timestamp"]
        for i in range(17):
            header.extend([f"KP_{i}_x", f"KP_{i}_y", f"KP_{i}_conf"])
        self.writer.writerow(header)
        
        self.get_logger().info("MoveNet Detector Pornit!")
        print("INPUT DETAILS:", self.input_details)


    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        h, w, _ = frame.shape
        
        # --- PREPROCESARE MOVENET ---
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img, (self.input_size, self.input_size))

        input_data = img_resized.astype(np.float32) / 255.0
        input_data = np.expand_dims(input_data, axis=0)

        # --- INFERENȚĂ ---
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        
        # Output shape: [1, 1, 17, 3] -> (y, x, score)
        keypoints_with_scores = self.interpreter.get_tensor(self.output_details[0]['index'])
        keypoints = keypoints_with_scores[0][0] # Scoatem array-ul

        # --- SALVARE CSV ---
        row = [time.time()]
        for kp in keypoints:
            ky, kx, conf = kp
            # MoveNet dă Y prima dată, apoi X!
            row.extend([kx, ky, conf])
        self.writer.writerow(row)

        # --- LOGICĂ CONTROL ---
        target_msg = Point()
        target_msg.z = 0.0
        
        # Calculăm centrul pe baza umerilor (5, 6) sau șoldurilor (11, 12)
        # Dacă scorul e bun (> 0.2)
        valid_points = []
        for kp in keypoints:
            if kp[2] > 0.2: # Threshold
                # Convertim din 0-1 în pixeli reali
                py = int(kp[0] * h)
                px = int(kp[1] * w)
                valid_points.append((px, py))
                cv2.circle(frame, (px, py), 5, (0, 255, 255), -1)

        if valid_points:
            # Calculăm centrul bounding box-ului punctelor valide
            xs = [p[0] for p in valid_points]
            ys = [p[1] for p in valid_points]
            
            if xs and ys:
                cx = int(np.mean(xs))
                cy = int(np.mean(ys))
                
                # Desenăm cutie
                cv2.rectangle(frame, (min(xs), min(ys)), (max(xs), max(ys)), (0,255,255), 2)
                
                # Distanță
                dist = 5.0
                if self.depth_image is not None:
                    try:
                        dist = float(self.depth_image[cy, cx])
                        if np.isnan(dist) or dist <= 0: dist = 5.0
                    except: pass
                
                target_msg.x = float((cx / w) - 0.5)
                target_msg.y = float(cy)
                target_msg.z = float(dist)

        self.target_pub.publish(target_msg)

        # Publicare imagine RViz
        out_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        out_msg.header = msg.header
        self.result_image_pub.publish(out_msg)
        
        cv2.imshow("MoveNet View", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = MoveNetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()