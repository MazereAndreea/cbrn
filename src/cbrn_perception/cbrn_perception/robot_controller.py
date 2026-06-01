#!/usr/bin/env python3
import csv
import time
import os
import rclpy
import rclpy.parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from cbrn_interfaces.msg import PerceptionMetrics
from datetime import datetime


SEARCH_SPEED   = 0.3   # m/s forward while searching
Kp_TURN        = 2.0   # proportional heading-correction gain
MAX_DRIVE_TIME = 60.0  # seconds — abort if person never detected

# ----- Detection quality thresholds ----------------------------------------
# skeleton_completeness: fraction of [shoulders 5,6 | elbows 7,8 | wrists 9,10]
# detected above their per-model confidence threshold (0.0 – 1.0).
#
#   0.50  → at least 3/6 critical keypoints visible
#           (minimum to locate the injection zone)
#   0.67  → at least 4/6 (both shoulders identified, enables L/R classification)
#
# confidence_score_min: mean score across all 17 keypoints — prevents stopping
# on a frame that is technically "complete" but overall noisy.
SKELETON_COMPLETENESS_THRESHOLD = 0.5   # ≥ 3 of 6 critical KPs detected
CONFIDENCE_SCORE_MIN             = 0.4  # mean keypoint confidence floor


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
             "Linear_Vel", "Angular_Vel", "Skeleton_Completeness", "Confidence_Avg"]
        )
        self.get_logger().info(f"Logging to {log_path}")

        self._model_name = model_name
        self._stopped = False
        self._start_time = time.monotonic()

        self._cmd_pub = self.create_publisher(
            Twist, "/diff_drive_controller/cmd_vel", 10
        )

        self._metrics_sub = self.create_subscription(
            PerceptionMetrics, "/pose_estimation/metrics", self._control_cb, 10
        )

        # Safety timer: stop if no detection after MAX_DRIVE_TIME seconds.
        self._timer = self.create_timer(0.1, self._tick)

    # ------------------------------------------------------------------

    def _publish(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    def _log(self, distance, horizontal_err, linear_v, angular_v,
             completeness=0.0, confidence=0.0):
        self._csv.writerow([
            time.time(),
            self._model_name,
            f"{distance:.3f}",
            f"{horizontal_err:.3f}",
            f"{linear_v:.3f}",
            f"{angular_v:.3f}",
            f"{completeness:.3f}",
            f"{confidence:.3f}",
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

    def _control_cb(self, msg: PerceptionMetrics):
        """Stop when the model reports a detection with sufficient quality.

        Stop condition (AND):
          1. msg.is_detected             — model found a person this frame
          2. skeleton_completeness >= 0.5 — ≥ 3/6 critical keypoints (shoulders,
                                            elbows, wrists) detected above threshold
          3. confidence_score >= 0.4     — mean keypoint confidence floor

        While the condition is not met the robot drives forward at SEARCH_SPEED
        and, if a partial detection exists, corrects its heading toward the person.
        """
        if self._stopped:
            return

        completeness = msg.skeleton_completeness
        confidence   = msg.confidence_score
        distance     = msg.distance_estimate
        # Horizontal error from nose keypoint (index 0), normalised −0.5…+0.5
        horizontal_err = (msg.keypoint_x[0] - 0.5) if msg.keypoint_x else 0.0

        detection_good = (
            msg.is_detected
            and completeness >= SKELETON_COMPLETENESS_THRESHOLD
            and confidence   >= CONFIDENCE_SCORE_MIN
        )

        if detection_good:
            self._log(distance, horizontal_err, 0.0, 0.0, completeness, confidence)
            self._stop_and_exit(
                f"Detection quality met — completeness={completeness:.2f} "
                f"confidence={confidence:.2f} dist≈{distance:.2f}m — stopping."
            )
            return

        # Not yet good enough — keep driving forward, correct heading if partial.
        turn = 0.0
        if msg.is_detected and horizontal_err != 0.0:
            turn = max(-1.0, min(1.0, -Kp_TURN * horizontal_err))

        self._publish(SEARCH_SPEED, turn)
        self._log(distance, horizontal_err, SEARCH_SPEED, turn, completeness, confidence)
        self.get_logger().info(
            f"Searching: completeness={completeness:.2f} "
            f"confidence={confidence:.2f} fwd={SEARCH_SPEED:.2f} turn={turn:.2f}"
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
