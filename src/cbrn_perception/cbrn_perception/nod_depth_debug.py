#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DepthDebugNode(Node):
    def __init__(self):
        super().__init__('depth_debug_node')

        self.get_logger().info("Depth Debug Node pornit. Aștept /my_camera/depth ...")

        self.bridge = CvBridge()

        # Subscriber pentru depth image
        self.sub = self.create_subscription(
            Image,
            '/my_camera/depth',
            self.depth_callback,
            10
        )

    def depth_callback(self, msg):
        try:
            # Depth este float32, deci folosim encoding „passthrough”
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Eroare CvBridge: {e}")
            return

        # Depth este o matrice float32, deci putem verifica valori
        min_val = float(np.nanmin(depth_image))
        max_val = float(np.nanmax(depth_image))

        self.get_logger().info(f"Depth Frame -> Min: {min_val:.3f} m,  Max: {max_val:.3f} m")

def main(args=None):
    rclpy.init(args=args)
    node = DepthDebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
