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
    print("❌ EROARE: MMPose nu este instalat.")
    exit()



class PoseNetDetector(Node):
    def __init__(self):
        super().__init__("posenet_detector")
        self.bridge = CvBridge()
        
        self.get_logger().info("Se încarcă PoseNet (MobileNetV2)...")
        
        try:
            # Folosim MobileNetV2 ca echivalent PoseNet pentru viteză maximă pe CPU
            # Acesta este un model Top-Down, deci are nevoie de un detector (yolox_tiny)
            self.inferencer = MMPoseInferencer(
                pose2d='mobilenetv2_coco_256x192', 
                det_model='yolox_tiny', 
                device='cpu'
            )
        except Exception as e:
            self.get_logger().error(f"Eroare la încărcarea PoseNet: {e}")
            exit()

        # Topicuri
        self.image_sub = self.create_subscription(Image, "/camera", self.img_cb, 10)
        self.target_pub = self.create_publisher(Point, "/perception/target", 10)
        self.result_image_pub = self.create_publisher(Image, "/pose_estimation/posenet_result", 10)
        
        # Setup CSV
        self.setup_csv()
        self.get_logger().info("Nod PoseNet pornit pe CPU!")

    def setup_csv(self):
        timestamp = datetime.now().strftime("%H%M%S")
        directory = os.path.expanduser("~/cbrn_ws/csv")
        if not os.path.exists(directory): os.makedirs(directory)
        self.csv_filename = os.path.join(directory, f"landmarks_posenet_{timestamp}.csv")
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        # Header standard COCO
        header = ["Timestamp"]
        for i in range(17): header.extend([f"KP_{i}_x", f"KP_{i}_y", f"KP_{i}_conf"])
        self.writer.writerow(header)

    def img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        # Inferență PoseNet
        result_generator = self.inferencer(frame, return_vis=False)
        result = next(result_generator)
        predictions = result['predictions']

        if predictions:
            person = predictions[0]
            keypoints = np.array(person['keypoints'])
            scores = np.array(person['keypoint_scores'])
            
            # Desenăm punctele pe imagine
            for kp, score in zip(keypoints, scores):
                if score > 0.3:
                    cv2.circle(frame, (int(kp[0]), int(kp[1])), 4, (0, 0, 255), -1)

            # Trimitem nasul (punctul 0) ca referință target
            target_msg = Point()
            target_msg.x = float((keypoints[0][0] / frame.shape[1]) - 0.5)
            target_msg.y = float(keypoints[0][1])
            self.target_pub.publish(target_msg)

            # Logare CSV
            row = [time.time()]
            for i in range(17):
                row.extend([keypoints[i][0], keypoints[i][1], scores[i]])
            self.writer.writerow(row)

        # Publicăm rezultatul vizual
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.result_image_pub.publish(out_msg)
        cv2.imshow("PoseNet Monitor", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PoseNetDetector()
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