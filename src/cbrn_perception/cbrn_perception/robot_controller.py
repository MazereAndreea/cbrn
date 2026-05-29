#!/usr/bin/env python3
import csv
import time
import os
import rclpy
import rclpy.parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Point
from datetime import datetime


STOP_DISTANCE = 1.5   # metres — stop when person is this close
SEARCH_SPEED  = 0.3   # m/s forward when no detection yet
Kp_TURN       = 2.0   # proportional gain for heading correction
MAX_DRIVE_TIME = 60.0 # seconds — abort if person never detected


class RobotController(Node):
    def __init__(self):
        super().__init__(
            "robot_controller",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.parameter.Parameter.Type.BOOL,
                    True,
                )
            ],
        )

        self.declare_parameter("model_name", "unknown_model")
        model_name = self.get_parameter("model_name").value

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = os.path.expanduser("~/bench_logs")
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, f"log_{model_name}_{timestamp_str}.csv")

        self._csv_file = open(log_path, mode="w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(
            ["Timestamp", "Model", "Detected_Distance_m", "Horizontal_Error",
             "Linear_Vel", "Angular_Vel"]
        )
        self.get_logger().info(f"Logging to {log_path}")

        self._model_name = model_name
        self._stopped = False
        self._start_time = time.monotonic()

        self._cmd_pub = self.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10
        )

        self._target_sub = self.create_subscription(
            Point, "/pose_estimation/image_result", self._control_cb, 10
        )

        # Safety timer: stop if no detection after MAX_DRIVE_TIME seconds.
        self._timer = self.create_timer(0.1, self._tick)

    # ------------------------------------------------------------------

    def _publish(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    def _log(self, distance, horizontal_err, linear_v, angular_v):
        self._csv.writerow([
            time.time(),
            self._model_name,
            f"{distance:.3f}",
            f"{horizontal_err:.3f}",
            f"{linear_v:.3f}",
            f"{angular_v:.3f}",
        ])
        self._csv_file.flush()

    def _stop_and_exit(self, reason: str):
        if self._stopped:
            return
        self._stopped = True
        self._publish(0.0, 0.0)
        self.get_logger().info(reason)
        self._csv_file.close()
        rclpy.shutdown()

    # ------------------------------------------------------------------

    def _tick(self):
        if self._stopped:
            return
        elapsed = time.monotonic() - self._start_time
        if elapsed > MAX_DRIVE_TIME:
            self._stop_and_exit(
                f"Timeout ({MAX_DRIVE_TIME}s): person not detected — stopping."
            )

    def _control_cb(self, msg: Point):
        """
        msg.x = normalised horizontal deviation (-0.5 left … +0.5 right)
        msg.z = estimated distance to person in metres  (0 = no detection)
        """
        if self._stopped:
            return

        distance = msg.z
        horizontal_err = msg.x

        if distance <= 0.0:
            # No valid detection — drive forward to search.
            self._publish(SEARCH_SPEED, 0.0)
            self._log(0.0, 0.0, SEARCH_SPEED, 0.0)
            return

        if distance <= STOP_DISTANCE:
            self._log(distance, horizontal_err, 0.0, 0.0)
            self._stop_and_exit(
                f"Person detected at {distance:.2f} m — stopping."
            )
            return

        # Approach: scale speed by remaining distance, correct heading.
        forward = min(SEARCH_SPEED, SEARCH_SPEED * (distance - STOP_DISTANCE) / STOP_DISTANCE)
        turn = max(-1.0, min(1.0, -Kp_TURN * horizontal_err))

        self._publish(forward, turn)
        self._log(distance, horizontal_err, forward, turn)
        self.get_logger().info(
            f"Approaching: dist={distance:.2f}m  fwd={forward:.2f}  turn={turn:.2f}"
        )


def main():
    rclpy.init()
    node = RobotController()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if not node._stopped:
            node._publish(0.0, 0.0)
            node._csv_file.close()


if __name__ == "__main__":
    main()
