#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np

mp_pose = mp.solutions.pose

class PersonFollower(Node):

    def __init__(self):
        super().__init__("person_follower")

        self.bridge = CvBridge()

        # USE YOUR CAMERA TOPIC
        self.image_sub = self.create_subscription(
            Image, "/camera", self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth", self.depth_callback, 10
        )
        

        # Publish commands to robot
        self.cmd_pub = self.create_publisher(
            TwistStamped, 
            "/diff_drive_controller/cmd_vel", 10
        )

        # MediaPipe Pose
        self.pose = mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.depth_image = None
        self.last_error = 0.0

        self.get_logger().info("PersonFollower node started.")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg, 
                desired_encoding="passthrough"
            )
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def image_callback(self, msg):

        # Convert ROS2 Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        result = self.pose.process(rgb)

        if result.pose_landmarks is None:
            self.stop_and_search()
            return

        # Compute centroid
        h, w, _ = frame.shape
        xs = [lm.x for lm in result.pose_landmarks.landmark]
        ys = [lm.y for lm in result.pose_landmarks.landmark]

        cx = int(np.mean(xs) * w)
        cy = int(np.mean(ys) * h)

        # Draw a red circle on the detected centre
        cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)

        if self.depth_image is None:
            self.stop_and_search()
            return

        try:
            depth_val = float(self.depth_image[cy, cx])
        except IndexError:
            depth_val = 5.0

        if depth_val <= 0 or np.isnan(depth_val):
            depth_val = 5.0  # assume far away

        self.follow_logic(cx, w, depth_val)
        # Optional debug display
        # cv2.circle(frame, (cx, cy), 6, (0,0,255), -1)
        # cv2.imshow("tracking", frame)
        # cv2.waitKey(1)


    def follow_logic(self, cx, width, depth):

        x_error = cx - width/2

        # Proportional turning
        Kp_turn = 0.0025
        turn = -Kp_turn * x_error

        # Forward control based on depth
        desired_distance = 1.0  # meters from human
        Kp_forward = 0.3

        forward_speed = Kp_forward * (depth - desired_distance)

        # Clamp speeds
        forward_speed = max(min(forward_speed, 0.4), -0.1)
        turn = max(min(turn, 1.0), -1.0)

        # If too close → stop
        if depth < 0.8:
            forward_speed = 0.0

        cmd = TwistStamped()
        cmd.twist.linear.x = forward_speed
        cmd.twist.angular.z = turn
        self.cmd_pub.publish(cmd)


    def stop_and_search(self):
        # Log to see if it enters this function
        self.get_logger().info("SEARCHING - NO HUMAN DETECTED")  
        cmd = TwistStamped()
        cmd.twist.angular.z = 1.0
        cmd.twist.angular.x = 0.0
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PersonFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
