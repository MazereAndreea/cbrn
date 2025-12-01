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
from datetime import datetime
import torch
from torchvision.models.detection import keypointrcnn_resnet50_fpn
from torchvision.transforms import functional as F

class PyTorchDetector(Node):
    def __init__(self):
        super().__init__("pytorch_detector")
        self.bridge = CvBridge()
        
        self.get_logger().info("Incarc modelul PyTorch (Keypoint R-CNN)... Asteaptati...")
        # Folosim CPU (sau cuda daca ai)
        self.device = torch.device('cpu') 
        # Încărcăm modelul pre-antrenat
        self.model = keypointrcnn_resnet50_fpn(pretrained=True)
        self.model.eval().to(self.device)
        self.get_logger().info("Model Incarcat!")

        self.image_sub = self.create_subscription(Image, "/camera", self.img_cb, 10)
        self.depth_sub = self.create_subscription(Image, "/camera/depth", self.depth_cb, 10)
        self.target_pub = self.create_publisher(Point, "/perception/target", 10)
        self.result_image_pub = self.create_publisher(Image, "/pose_estimation/image_result", 10)
        
        self.depth_image = None

        # CSV Setup
        timestamp = datetime.now().strftime("%H%M%S")
        self.csv_filename = f"landmarks_pytorch_{timestamp}.csv"
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        
        header = ["Timestamp"]
        for i in range(17):
            header.extend([f"KP_{i}_x", f"KP_{i}_y", f"KP_{i}_conf"])
        self.writer.writerow(header)

    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return
        h, w, _ = frame.shape

        # Preprocesare PyTorch
        img_tensor = F.to_tensor(frame).to(self.device)
        input_list = [img_tensor]

        # Inferență
        with torch.no_grad():
            outputs = self.model(input_list)

        output = outputs[0]
        # output['keypoints'] are forma [N, 17, 3] -> (x, y, visible)
        
        target_msg = Point()
        target_msg.z = 0.0

        # Verificăm dacă avem detecții cu scor mare
        if len(output['scores']) > 0 and output['scores'][0] > 0.5:
            # Luăm prima persoană
            kpts = output['keypoints'][0].cpu().numpy()
            box = output['boxes'][0].cpu().numpy() # [x1, y1, x2, y2]

            # CSV Save (Normalizăm manual x/w, y/h pentru consistență)
            row = [time.time()]
            for kp in kpts:
                row.extend([kp[0]/w, kp[1]/h, kp[2]]) # x, y, vis
            self.writer.writerow(row)

            # Vizualizare
            for kp in kpts:
                if kp[2] > 0: # Dacă e vizibil
                    cv2.circle(frame, (int(kp[0]), int(kp[1])), 4, (255, 0, 0), -1)
            
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

            # Logică Robot
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            
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
        
        out_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        out_msg.header = msg.header
        self.result_image_pub.publish(out_msg)
        
        cv2.imshow("PyTorch R-CNN View", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = PyTorchDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()