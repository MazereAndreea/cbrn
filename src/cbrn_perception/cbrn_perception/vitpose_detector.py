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

# --- IMPORTURI SPECIFICE MMPOSE ---
try:
    from mmpose.apis import MMPoseInferencer
except ImportError:
    print("❌ EROARE: MMPose nu este instalat. Rulează: mim install mmpose")
    exit()

CONFIDENCE_THRESHOLD = 0.3

class ViTPoseDetector(Node):
    def __init__(self):
        super().__init__("vitpose_detector")
        self.bridge = CvBridge()
        
        # 1. Încărcare Model ViTPose
        # 'pose2d' specifică modelul de postură (vitpose)
        # 'det_model' specifică cine găsește cutia (folosim yolox pentru viteză)
        self.get_logger().info("Se încarcă ViTPose (poate dura la prima rulare)...")
        
        try:
            # Folosim 'vitpose-b' (Base) pentru un balans bun viteză/precizie
            # Sau 'vitpose-h' (Huge) pentru precizie maximă dar lent
            self.inferencer = MMPoseInferencer(
                pose2d='vitpose-b', 
                det_model='yolox_l', 
                device='cuda' # Schimbă în 'cpu' dacă nu ai NVIDIA
            )
        except Exception as e:
            self.get_logger().error(f"Eroare la încărcarea modelului: {e}")
            exit()

        self.image_sub = self.create_subscription(Image, "/camera", self.img_cb, 10)
        self.depth_sub = self.create_subscription(Image, "/camera/depth", self.depth_cb, 10)
        self.target_pub = self.create_publisher(Point, "/perception/target", 10)
        self.result_image_pub = self.create_publisher(Image, "/pose_estimation/image_result", 10)
        
        self.depth_image = None

        # --- SETUP CSV ---
        timestamp = datetime.now().strftime("%H%M%S")
        directory = os.path.expanduser("~/cbrn_ws/csv")
        if not os.path.exists(directory):
            os.makedirs(directory)
            
        self.csv_filename = os.path.join(directory, f"landmarks_vitpose_{timestamp}.csv")
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        
        # Header standard COCO (17 puncte)
        header = ["Timestamp"]
        for i in range(17):
            header.extend([f"KP_{i}_x", f"KP_{i}_y", f"KP_{i}_conf"])
        self.writer.writerow(header)
        
        self.get_logger().info(f"ViTPose Pornit! Logare în: {self.csv_filename}")

    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Eroare convertire imagine: {e}")
            return

        h, w, _ = frame.shape

        # --- INFERENȚĂ VITPOSE ---
        # Inferencer returnează un generator, luăm primul rezultat
        # return_vis=False ca să desenăm noi manual
        result_generator = self.inferencer(frame, return_vis=False)
        result = next(result_generator)
        
        # Structura result: {'predictions': [{'keypoints': [[x,y],..], 'keypoint_scores': [s,..], 'bbox': ...}]}
        predictions = result['predictions']

        target_msg = Point()
        target_msg.z = 0.0

        if predictions:
            # Luăm prima persoană detectată
            person = predictions[0]
            
            # Extragem datele
            keypoints = np.array(person['keypoints']) # shape (17, 2)
            scores = np.array(person['keypoint_scores']) # shape (17,)
            bbox = person['bbox'][0] # [x1, y1, x2, y2]
            
            # --- 1. EXPORT CSV ---
            row = [time.time()]
            for i in range(17):
                # Normalizăm coordonatele pentru CSV (0-1) cum făcea YOLO, pentru consistență
                norm_x = keypoints[i][0] / w
                norm_y = keypoints[i][1] / h
                row.extend([norm_x, norm_y, scores[i]])
            self.writer.writerow(row)

            # --- 2. VIZUALIZARE ---
            # Desenăm puncte
            for i, (kp, score) in enumerate(zip(keypoints, scores)):
                if score > CONFIDENCE_THRESHOLD:
                    cx_kp, cy_kp = int(kp[0]), int(kp[1])
                    cv2.circle(frame, (cx_kp, cy_kp), 5, (0, 255, 0), -1)

            # Desenăm Bounding Box
            x1, y1, x2, y2 = map(int, bbox)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            
            # --- 3. CALCUL DISTANȚĂ ȘI TARGET ---
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            
            dist = 5.0
            if self.depth_image is not None:
                try:
                    cy_safe = min(max(cy, 0), h-1)
                    cx_safe = min(max(cx, 0), w-1)
                    dist = float(self.depth_image[cy_safe, cx_safe])
                    if np.isnan(dist) or dist <= 0: dist = 5.0
                except: pass

            target_msg.x = float((cx / w) - 0.5)
            target_msg.y = float(cy)
            target_msg.z = float(dist)
        
        self.target_pub.publish(target_msg)

        # Publicare imagine procesată
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out_msg.header = msg.header
        self.result_image_pub.publish(out_msg)

        cv2.imshow("ViTPose Detector", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ViTPoseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()