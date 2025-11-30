import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")
        self.bridge = CvBridge()
        
        # Incarcam modelul YOLO (nano e cel mai rapid)
        self.model = YOLO("yolo11n.pt") 
        
        self.image_sub = self.create_subscription(Image, "/camera", self.img_cb, 10)
        self.depth_sub = self.create_subscription(Image, "/camera/depth", self.depth_cb, 10)
        self.target_pub = self.create_publisher(Point, "/perception/target", 10)
        
        self.depth_image = None

    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def img_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = frame.shape
        
        # Inferenta YOLO
        results = self.model(frame, verbose=False)
        
        target_msg = Point()
        target_msg.z = 0.0 # Default (nimic detectat)

        # Cautam clasa "person" (class id 0 in COCO)
        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls = int(box.cls[0])
                if cls == 0: # 0 este Person
                    # Extragem centrul cutiei
                    x1, y1, x2, y2 = box.xyxy[0]
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    
                    # Desenam pentru debug
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
                    
                    # Calculam distanta din Depth Image
                    dist = 5.0
                    if self.depth_image is not None:
                        try:
                            dist = float(self.depth_image[cy, cx])
                            if np.isnan(dist) or dist <= 0: dist = 5.0
                        except: pass
                    
                    # Normalizam X intre -0.5 (stanga) si 0.5 (dreapta) pentru controler
                    norm_x = (cx / w) - 0.5
                    
                    target_msg.x = float(norm_x)
                    target_msg.y = float(cy)
                    target_msg.z = float(dist)
                    
                    # Luam doar prima persoana gasita si ne oprim
                    break 

        self.target_pub.publish(target_msg)
        cv2.imshow("YOLO View", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YoloDetector()
    rclpy.spin(node)
    rclpy.shutdown()