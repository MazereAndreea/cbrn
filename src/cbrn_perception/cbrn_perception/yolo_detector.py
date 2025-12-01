#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import csv
import time
import os
from datetime import datetime

class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")
        self.bridge = CvBridge()
        
        # Modelul Pose Standard
        self.model = YOLO("yolo11n-pose.pt") 
        
        self.image_sub = self.create_subscription(Image, "/camera", self.img_cb, 10)
        self.depth_sub = self.create_subscription(Image, "/camera/depth", self.depth_cb, 10)
        self.target_pub = self.create_publisher(Point, "/perception/target", 10)
        # Publisher pentru imaginea procesată (Către RViz)
        self.result_image_pub = self.create_publisher(Image, "/pose_estimation/image_result", 10)
        self.depth_image = None

        # --- SETUP CSV ---
        timestamp = datetime.now().strftime("%H%M%S")
        directory = os.path.expanduser("~/cbrn_ws/csv")
        self.csv_filename = os.path.join(directory, f"landmarks_mediapipe_{timestamp}.csv")
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        
        # YOLO are 17 puncte (Nas, Ochi, Umeri, Coate, Genunchi, etc.)
        header = ["Timestamp"]
        for i in range(17):
            header.extend([f"KP_{i}_x", f"KP_{i}_y", f"KP_{i}_conf"])
        self.writer.writerow(header)
        
        self.get_logger().info(f"Salvare date YOLO in: {self.csv_filename}")

    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def img_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = frame.shape
        
        # Inferenta Standard (Fără rotație)
        # Putem lăsa confidența mică (0.15) ca să prindă cât mai mult
        results = self.model(frame, verbose=False, conf=0.15)
        
        target_msg = Point()
        target_msg.z = 0.0 

        for r in results:
            # Verificăm dacă avem keypoints detectate
            if r.keypoints is not None and r.keypoints.has_visible:
                
                # Extragem datele brute (pe CPU, ca numpy array)
                # xyn = coordonate normalizate (0-1)
                kpts = r.keypoints.xyn.cpu().numpy()[0]
                confs = r.keypoints.conf.cpu().numpy()[0] if r.keypoints.conf is not None else [0]*17
                
                # --- 1. EXPORT CSV ---
                row = [time.time()]
                for i in range(17):
                    # Salvăm exact ce vede modelul
                    row.extend([kpts[i][0], kpts[i][1], confs[i]])
                self.writer.writerow(row)

                # --- 2. Vizualizare și Control ---
                # Desenăm punctele pe imagine
                for x, y in kpts:
                    if x > 0 and y > 0: 
                        cv2.circle(frame, (int(x*w), int(y*h)), 5, (0, 0, 255), -1)

                # Folosim bounding box-ul pentru a mișca robotul
                boxes = r.boxes
                if boxes:
                    x1, y1, x2, y2 = boxes.xyxy[0].cpu().numpy()
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255,0,0), 2)
                    
                    dist = 5.0
                    if self.depth_image is not None:
                        try:
                            cy_safe = min(max(cy, 0), h-1)
                            cx_safe = min(max(cx, 0), w-1)
                            dist = float(self.depth_image[int(cy_safe), int(cx_safe)])
                            if np.isnan(dist) or dist <= 0: dist = 5.0
                        except: pass

                    target_msg.x = float((cx / w) - 0.5)
                    target_msg.y = float(cy)
                    target_msg.z = float(dist)
                    
                    break # Luăm prima persoană

        self.target_pub.publish(target_msg)
        try:
            # Convertim înapoi din OpenCV (BGR) în ROS Message
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            out_msg.header = msg.header # Păstrăm timestamp-ul original pentru sincronizare
            self.result_image_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Eroare publicare imagine: {e}")
        cv2.imshow("YOLO Standard View", frame)
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