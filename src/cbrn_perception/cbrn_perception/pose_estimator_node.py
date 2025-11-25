import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')
        self.get_logger().info('Nodul de Pose Estimation (Perceptie) a pornit.')

        # 1. Inițializează CV_Bridge
        self.bridge = CvBridge()

        # 2. Inițializează MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

        # 3. Creează Subscriber-ul
        # Se abonează la topicul camerei din simulator
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10)

        # 4. Creează Publisher-ul
        # Va publica imaginea procesată (cu schelet) pe un topic nou
        self.publisher_ = self.create_publisher(Image, '/pose_estimation/image_result', 10)

    def image_callback(self, msg):
        # "msg" este imaginea brută de la simulator
        try:
            # Convertește imaginea ROS 2 -> imagine OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Eroare CvBridge la primire: {e}')
            return

        # --- AICI RULEAZĂ MODELUL AI ---

        # Convertește imaginea din BGR în RGB (MediaPipe folosește RGB)
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        cv_image_rgb.flags.writeable = False # Optimizare

        # Rulează detecția MediaPipe
        results = self.pose.process(cv_image_rgb)

        # --- SFÂRȘIT PROCESARE AI ---

        # Desenează scheletul înapoi pe imaginea BGR (pentru publicare)
        cv_image.flags.writeable = True
        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                cv_image,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS)

        # --- Publicarea rezultatului ---
        try:
            # Convertește imaginea OpenCV (cu schelet) -> mesaj ROS 2
            result_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            result_msg.header = msg.header # Păstrează același timp
            self.publisher_.publish(result_msg)
        except Exception as e:
            self.get_logger().error(f'Eroare CvBridge la publicare: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    rclpy.spin(node) # Ține nodul pornit
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()