#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import csv
import time
import os
from datetime import datetime
from cbrn_interfaces.msg import PerceptionMetrics

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

class UniversalPoseDetector(Node):
    def __init__(self):
        self.image_saved = False
        super().__init__("universal_pose_detector")
        self.bridge = CvBridge()
        
        # DECLARE PARAMETERS 
        self.declare_parameter("model_config", "human") 
        self.declare_parameter("device", "cuda")                   
        self.declare_parameter("csv_output", True)

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.timer_callback()

        self.image_sub = self.create_subscription(Image, "/camera", self.img_cb, 10)
        self.target_pub = self.create_publisher(Point, "/perception/target", 10)
        self.result_image_pub = self.create_publisher(Image, "/pose_estimation/image_result", 10)
        
        # Setup CSV 
        self.metrics_pub = self.create_publisher(PerceptionMetrics, "/pose_estimation/metrics", 10)    
        self.setup_csv()
        
    def timer_callback(self):
        # Get parameter values
        model_cfg = self.get_parameter("model_config").get_parameter_value().string_value
        device = self.get_parameter("device").get_parameter_value().string_value
        
        self.get_logger().info(f"Loading model: {model_cfg} on {device}...")
        self.get_logger().info("Se încarcă modelul ...")
        
        try:
            self.inferencer = MMPoseInferencer(
                pose2d=model_cfg, 
                device=device,
                scope='mmpose'
            )
        except Exception as e:
            self.get_logger().error(f"Eroare la încărcarea RTMO: {e}")
            exit()

    def parameter_callback(self, params):
        for param in params:
            if param.name == "model_config":
                self.get_logger().warn(f"Reîncarc modelul cu: {param.value}")
                
                try:
                    self.inferencer = MMPoseInferencer(
                        pose2d=param.value,
                        device=self.get_parameter("device").value,
                        scope='mmpose'
                    )
                except Exception as e:
                    self.get_logger().error(f"Nu s-a putut schimba modelul: {e}")
                    return SetParametersResult(successful=False, reason=str(e))
                    
        return SetParametersResult(successful=True)

    def setup_csv(self):
        timestamp = datetime.now().strftime("%H%M%S")
        directory = os.path.expanduser("~/cbrn_ws/csv")
        if not os.path.exists(directory): os.makedirs(directory)
        self.csv_filename = os.path.join(directory, f"landmarks_pose_{timestamp}.csv")
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        header = ["Timestamp"]
        self.writer.writerow([header, "Inference_Time", "Confidence"])
        for i in range(17): header.extend([f"KP_{i}_x", f"KP_{i}_y", f"KP_{i}_conf"])
        self.writer.writerow(header)

    def img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        # --- MASURARE TIMP START ---
        t_start = time.perf_counter()


        # Inferență - RTMO procesează tot cadrul dintr-o singură trecere
        result_generator = self.inferencer(frame, return_vis=False)
        result = next(result_generator)
        predictions = result['predictions']

         # --- MASURARE TIMP FINAL ---
        t_end = time.perf_counter()
        inference_time = t_end - t_start

        # Pregătire mesaj Metrics
        metrics_msg = PerceptionMetrics()
        metrics_msg.header = msg.header
        metrics_msg.inference_time = float(inference_time)
        metrics_msg.is_detected = False
        metrics_msg.confidence_score = 0.0

        if predictions and len(predictions[0]) > 0:

            metrics_msg.is_detected = True
            # Handle list of lists structure often returned by MMPose
            persons = predictions[0] if isinstance(predictions[0], list) else predictions[0]
            for person in persons:
                keypoints = np.array(person['keypoints'])
                scores = np.array(person['keypoint_scores'])

                metrics_msg.confidence_score = float(np.mean(scores))

                self.draw_skeleton(frame, keypoints, scores)

                # Publish Target (Nose)
                target_msg = Point()
                target_msg.x = float((keypoints[0][0] / frame.shape[1]) - 0.5)
                self.target_pub.publish(target_msg)

        # Output
        self.metrics_pub.publish(metrics_msg)
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.result_image_pub.publish(out_msg)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            save_dir = "/home/ai/cbrn/cbrn/src/models_images"
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            model_cfg = self.get_parameter("model_config").value
            filename = os.path.join(save_dir, f"{model_cfg}_{timestamp}.png")
            cv2.imwrite(filename, frame)
            self.image_saved = True
        cv2.imshow("MMPose Universal Monitor", frame)
        cv2.waitKey(1)

    def draw_skeleton(self, frame, keypoints, scores):
        SKELETON = [
                    (5, 7), (7, 9),        # left arm
                    (6, 8), (8, 10),       # right arm
                    (5, 6),                # shoulders
                    (5, 11), (6, 12),      # torso upper
                    (11, 12),              # hips
                    (11, 13), (13, 15),    # left leg
                    (12, 14), (14, 16),    # right leg
                    (0, 1), (0, 2),        # face
                    (1, 3), (2, 4)
                ]
        for p1, p2 in SKELETON:
                # Verificăm dacă indicii există în modelul curent (important pentru WholeBody)
                if p1 < len(keypoints) and p2 < len(keypoints):
                    if scores[p1] > 0.1 and scores[p2] > 0.1:
                        pt1 = (int(keypoints[p1][0]), int(keypoints[p1][1]))
                        pt2 = (int(keypoints[p2][0]), int(keypoints[p2][1]))
                        cv2.line(frame, pt1, pt2, (255, 0, 0), 2) # Linie albastră, grosime 2
            
            # Draw results
        for kp, score in zip(keypoints, scores):
            if score > 0.1:
                cv2.circle(frame, (int(kp[0]), int(kp[1])), 5, (0, 255, 0), -1)

def main(args=None):
    rclpy.init(args=args)
    node = UniversalPoseDetector()
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