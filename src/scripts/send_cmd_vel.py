#!/usr/bin/env python3
"""One-shot TwistStamped publisher for diff_drive_controller.

Usage:
    send_cmd_vel.py <linear_x> <angular_z> [duration_sec]

Publishes TwistStamped at 10 Hz to /diff_drive_controller/cmd_vel for
<duration_sec> seconds, then sends a zero-velocity stop.
"""
import sys
import time
import rclpy
import rclpy.parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <linear_x> <angular_z> [duration_sec]")
        sys.exit(1)

    linear_x = float(sys.argv[1])
    angular_z = float(sys.argv[2])
    duration = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.0

    rclpy.init()
    node = Node("send_cmd_vel", parameter_overrides=[
        rclpy.parameter.Parameter("use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)
    ])
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    pub = node.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", qos)

    def make_msg(lx, az):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = lx
        msg.twist.angular.z = az
        return msg

    def spin_until(end_monotonic):
        while time.monotonic() < end_monotonic:
            executor.spin_once(timeout_sec=0.02)

    # Spin 1 s so /clock syncs and the publisher discovers the controller.
    spin_until(time.monotonic() + 1.0)

    node.get_logger().info(
        f"Driving: linear.x={linear_x}, angular.z={angular_z} for {duration}s"
    )

    # Publish at 10 Hz for the full duration so cmd_vel_timeout never fires.
    interval = 0.1
    end = time.monotonic() + duration
    while time.monotonic() < end:
        pub.publish(make_msg(linear_x, angular_z))
        spin_until(time.monotonic() + interval)

    # Stop the robot.
    pub.publish(make_msg(0.0, 0.0))
    spin_until(time.monotonic() + 0.1)
    node.get_logger().info("Published stop (zero velocity)")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
