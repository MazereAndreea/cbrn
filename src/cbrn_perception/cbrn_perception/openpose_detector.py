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

# --- FIX REGISTRY PENTRU MODELE ONE-STAGE ---
from mmengine.registry import MODELS
try:
    import mmdet.models
    import mmpretrain.models
    # Permitem mmpose să vadă componentele de detecție necesare RTMO
    MODELS.import_from_lib('mmdet')
except Exception as e:
    print(f"[WARN] Eroare la încărcarea registrelor: {e}")

try:
    from mmpose.apis import MMPoseInferencer
except ImportError:
    print("❌ EROARE: MMPose nu este instalat.")
    exit()



class OpenPoseDetector(Node):
    def __init__(self):
        super().__init__("openpose_detector")
        self.bridge = CvBridge()
        
        self.get_logger().info("Se încarcă modelul stil OpenPose (RTMO)...")
        
        try:
            # RTMO-S (Small) este ideal pentru CPU. 
            # Fiind bottom-up, NU avem nevoie de det_model='yolox'
            self.inferencer = MMPoseInferencer(
                pose2d='rtmo-s', 
                device='cpu'
            )
        except Exception as e:
            self.get_logger().error(f"Eroare la încărcarea RTMO: {e}")
            exit()

        self.image_sub = self.create_subscription(Image, "/camera", self.img_cb, 10)
        self.target_pub = self.create_publisher(Point, "/perception/target", 10)
        self.result_image_pub = self.create_publisher(Image, "/pose_estimation/openpose_result", 10)
        
        # Setup CSV
        self.setup_csv()
        self.get_logger().info("Nod OpenPose (RTMO) activat!")

    def setup_csv(self):
        timestamp = datetime.now().strftime("%H%M%S")
        directory = os.path.expanduser("~/cbrn_ws/csv")
        if not os.path.exists(directory): os.makedirs(directory)
        self.csv_filename = os.path.join(directory, f"landmarks_openpose_{timestamp}.csv")
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        header = ["Timestamp"]
        for i in range(17): header.extend([f"KP_{i}_x", f"KP_{i}_y", f"KP_{i}_conf"])
        self.writer.writerow(header)

    def img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        # Inferență - RTMO procesează tot cadrul dintr-o singură trecere
        result_generator = self.inferencer(frame, return_vis=False)
        result = next(result_generator)
        predictions = result['predictions']

        if predictions:
            # Luăm prima persoană pentru target
            person = predictions[0]
            keypoints = np.array(person['keypoints'])
            scores = np.array(person['keypoint_scores'])
            
            # Desenăm rezultatele
            for i, (kp, score) in enumerate(zip(keypoints, scores)):
                if score > 0.3:
                    cv2.circle(frame, (int(kp[0]), int(kp[1])), 5, (255, 0, 0), -1)

            # Publicăm centrul corpului (Torso) ca target
            target_msg = Point()
            target_msg.x = float((keypoints[0][0] / frame.shape[1]) - 0.5) # Nasul ca referință
            self.target_pub.publish(target_msg)

        # Afișare
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.result_image_pub.publish(out_msg)
        cv2.imshow("OpenPose (RTMO) Monitor", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = OpenPoseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

# --- ACEST BLOC ESTE CRITIC PENTRU CA NODUL SĂ RULEZE ---
if __name__ == "__main__":
    main()