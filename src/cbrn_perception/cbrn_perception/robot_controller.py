#!/usr/bin/env python3
"""
robot_controller — two-phase autonomous positioning (sections 2.3 / 3.5)

Phase 1  APPROACHING  — proportional controller on /perception/target; robot
                        drives toward the detected person, correcting heading.

Phase 2  POSITIONING  — odometry-based go-to-goal; steers to the Nav2Goal
                        (nav_goal_x / nav_goal_y) computed by
                        injection_zone_analyzer in the map/odom frame.

Phase 3  ALIGNING     — rotates in place until the robot faces the injection
                        zone (nav_goal_yaw).

Phase 4  READY        — robot is at the injection position; stops and logs.

Transition APPROACHING → POSITIONING fires when /injection_zone carries a
valid goal with confidence ≥ MIN_CONF and a known body position.
"""

import csv
import math
import time
import os
import rclpy
import rclpy.parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from datetime import datetime

from cbrn_interfaces.msg import InjectionZone, PerceptionMetrics

# ── Tuning constants ─────────────────────────────────────────────────────────
SEARCH_SPEED      = 0.3    # m/s — forward speed while approaching
Kp_HEADING        = 2.0    # proportional gain: heading error → angular velocity
Kp_LINEAR         = 0.6    # proportional gain: distance error → forward speed
MAX_LINEAR        = 0.3    # m/s cap for positioning phase
MAX_ANGULAR       = 0.8    # rad/s cap

POSITION_TOL      = 0.12   # m  — goal reached when closer than this
YAW_TOL           = 0.06   # rad — aligned when yaw error smaller than this
ALIGN_ONLY_ANGLE  = 0.5    # rad — rotate in place when heading error > this

MIN_CONF          = 0.25   # minimum injection-zone confidence to trigger positioning
MIN_STABLE_FRAMES = 8      # valid-goal messages required before switching phase
APPROACH_STOP_M   = 0.6    # m — camera-based soft stop in APPROACHING
LIDAR_STOP_M      = 0.50   # m — LiDAR hard stop: obstacle closer than this → stop/skip phase
LIDAR_FWD_ANGLE   = math.pi / 3   # rad — half-angle of forward LiDAR arc used for collision check
MAX_APPROACH_TRAVEL_M = 2.5  # m — odom travel cap; robot starts ~3 m from person, stops 0.5 m before
POSITIONING_TIMEOUT = 20.0 # s  — force ALIGNING if stuck in POSITIONING

MAX_DRIVE_TIME    = 90.0   # s — global watchdog


class Phase:
    APPROACHING = 'APPROACHING'
    POSITIONING = 'POSITIONING'
    ALIGNING    = 'ALIGNING'
    READY       = 'READY'


def _normalize_angle(a: float) -> float:
    """Wrap angle to [-π, π]."""
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class RobotController(Node):
    def __init__(self):
        super().__init__(
            'robot_controller',
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    'use_sim_time',
                    rclpy.parameter.Parameter.Type.BOOL,
                    True,
                )
            ],
        )

        self.declare_parameter('model_name', 'unknown_model')
        model_name = self.get_parameter('model_name').value

        # CSV
        ts           = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_dir = os.path.expanduser('~/cbrn/cbrn/bench_logs')
        self._log_ts  = ts
        os.makedirs(self._log_dir, exist_ok=True)
        log_path = os.path.join(self._log_dir, f'log_{model_name}_{ts}.csv')
        self._log_path = log_path
        self._csv_file = open(log_path, mode='w', newline='')
        self._csv      = csv.writer(self._csv_file)
        self._csv.writerow([
            'Timestamp', 'Model', 'Phase',
            'Dist_to_Person_m', 'Dist_to_Goal_m',
            'Heading_Err_rad', 'Linear_Vel', 'Angular_Vel',
            'Body_Position', 'Injection_Zone', 'Zone_Conf', 'Goal_Valid',
            'Goal_X', 'Goal_Y', 'Goal_Yaw',
        ])
        self.get_logger().info(f'RobotController ready. Log → {log_path}')

        # State
        self._model_name    = model_name
        self._phase         = Phase.APPROACHING
        self._stopped       = False
        self._start_time    = time.monotonic()

        # Robot pose (updated from odometry)
        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._robot_yaw = 0.0
        self._start_x: float | None = None   # odom position when movement begins
        self._start_y: float | None = None

        # Latest messages
        self._last_target: Point | None          = None
        self._last_inj:    InjectionZone | None  = None
        self._valid_goal_count = 0   # consecutive valid-goal frames
        self._lidar_front_min: float | None = None  # min range in forward arc

        # Locked injection goal (frozen when we enter POSITIONING)
        self._goal_x   = 0.0
        self._goal_y   = 0.0
        self._goal_yaw = 0.0
        self._phase_start = time.monotonic()  # timestamp of last phase change

        # Publishers / subscribers
        self._cmd_pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)

        self.create_subscription(Point,             '/perception/target',          self._target_cb,  10)
        self.create_subscription(InjectionZone,    '/injection_zone',             self._inj_cb,     10)
        self.create_subscription(Odometry,         '/diff_drive_controller/odom', self._odom_cb,    10)
        self.create_subscription(LaserScan,        '/scan',                       self._scan_cb,    10)
        self.create_subscription(PerceptionMetrics, '/pose_estimation/metrics',   self._metrics_cb, 10)

        # Control loop at 10 Hz
        self.create_timer(0.1, self._control_loop)

    # ── Sensor callbacks ─────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        front = [
            r for i, r in enumerate(msg.ranges)
            if (msg.range_min < r < msg.range_max)
            and abs(msg.angle_min + i * msg.angle_increment) < LIDAR_FWD_ANGLE
        ]
        self._lidar_front_min = min(front) if front else None

    def _target_cb(self, msg: Point):
        self._last_target = msg

    def _metrics_cb(self, msg: PerceptionMetrics):
        if self._model_name != 'unknown_model' or not msg.model_name:
            return
        self._model_name = msg.model_name
        new_path = os.path.join(self._log_dir, f'log_{msg.model_name}_{self._log_ts}.csv')
        self._csv_file.flush()
        self._csv_file.close()
        os.rename(self._log_path, new_path)
        self._log_path = new_path
        self._csv_file = open(new_path, mode='a', newline='')
        self._csv      = csv.writer(self._csv_file)
        self.get_logger().info(f'Model auto-detected: {msg.model_name} → {new_path}')

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position # extrage poziția (x,y,z) a robotului
        q = msg.pose.pose.orientation # poziția + orientare (x,y,z,w)
        self._robot_x = p.x
        self._robot_y = p.y
        # Formule pentru extragerea yaw-ului (rotație in jurul axei verticale z)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        # Returnează unghi care reprezintă direcția in care priveste robotul
        self._robot_yaw = math.atan2(siny, cosy)
        if self._start_x is None:
            self._start_x = p.x
            self._start_y = p.y

    def _inj_cb(self, msg: InjectionZone):
        self._last_inj = msg

        # Count consecutive frames with a valid, confident injection goal.
        # body_position='unknown' is accepted — zone may still be correctly identified.
        if msg.goal_valid and msg.confidence >= MIN_CONF:
            self._valid_goal_count += 1
        else:
            self._valid_goal_count = 0

        # Transition: APPROACHING → POSITIONING
        if (self._phase == Phase.APPROACHING
                and self._valid_goal_count >= MIN_STABLE_FRAMES):
            self._goal_x      = msg.nav_goal_x
            self._goal_y      = msg.nav_goal_y
            self._goal_yaw    = msg.nav_goal_yaw
            self._phase       = Phase.POSITIONING
            self._phase_start = time.monotonic()
            self.get_logger().info(
                f'[PHASE] APPROACHING → POSITIONING  '
                f'goal=({self._goal_x:.2f}, {self._goal_y:.2f})  '
                f'yaw={math.degrees(self._goal_yaw):.1f}°  '
                f'zone={msg.zone_type}  body={msg.body_position}')

    # ── Main control loop ─────────────────────────────────────────────────────

    def _control_loop(self):
        if self._stopped:
            return

        # Global watchdog
        if time.monotonic() - self._start_time > MAX_DRIVE_TIME:
            self._stop_and_exit(f'Watchdog: {MAX_DRIVE_TIME:.0f} s elapsed.')
            return

        # LiDAR hard stop — camera distance estimates are unreliable; use actual range.
        lidar = self._lidar_front_min
        if lidar is not None and lidar < LIDAR_STOP_M:
            if self._phase == Phase.POSITIONING:
                # Can't reach the goal without hitting the person — skip straight to ALIGNING.
                self._publish(0.0, 0.0)
                self._phase       = Phase.ALIGNING
                self._phase_start = time.monotonic()
                self.get_logger().warn(
                    f'[LIDAR] {lidar:.2f} m — obstacle too close, skipping to ALIGNING')
            else:
                self._publish(0.0, 0.0)
                self.get_logger().warn(
                    f'[LIDAR] {lidar:.2f} m — holding position in {self._phase}')
            return

        # Odometry travel cap — distance estimates from the camera are unreliable for
        # a lying person (wrong ref_m), so we bound total travel from the starting
        # position instead of trusting the nav goal distance.
        if self._start_x is not None and self._phase in (Phase.APPROACHING, Phase.POSITIONING):
            traveled = math.sqrt((self._robot_x - self._start_x) ** 2
                                 + (self._robot_y - self._start_y) ** 2)
            if traveled >= MAX_APPROACH_TRAVEL_M:
                self._publish(0.0, 0.0)
                if self._phase == Phase.POSITIONING:
                    self._phase       = Phase.ALIGNING
                    self._phase_start = time.monotonic()
                    self.get_logger().warn(
                        f'[TRAVEL] {traveled:.2f} m — goal likely past person, forcing ALIGNING')
                else:
                    self.get_logger().warn(
                        f'[TRAVEL] {traveled:.2f} m — holding in APPROACHING')
                return

        if self._phase == Phase.APPROACHING:
            self._run_approaching()
        elif self._phase == Phase.POSITIONING:
            self._run_positioning()
        elif self._phase == Phase.ALIGNING:
            self._run_aligning()
        elif self._phase == Phase.READY:
            # Already stopped; just keep logging
            pass

    # ── Phase controllers ─────────────────────────────────────────────────────

    def _run_approaching(self):
        """
        Coarse approach using /perception/target.
        Drives forward, corrects heading, stops when person is close.
        """
        msg = self._last_target
        if msg is None or msg.z <= 0.0:
            # No detection yet — creep forward
            self._publish(SEARCH_SPEED * 0.5, 0.0)
            self._log_row(0.0, 0.0, 0.0, SEARCH_SPEED * 0.5, 0.0)
            return

        distance = msg.z
        h_err    = msg.x   # normalised [-0.5, +0.5]

        # Safety: person is very close but injection zone hasn't fired yet — stop.
        if distance < APPROACH_STOP_M:
            self._publish(0.0, 0.0)
            self._log_row(distance, 0.0, h_err, 0.0, 0.0)
            self.get_logger().warn(
                f'[APPROACHING] Person at {distance:.2f} m — waiting for injection zone.')
            return

        # Keep approaching; injection_zone_cb handles the phase switch
        forward = min(SEARCH_SPEED,
                      SEARCH_SPEED * max(0.0, distance - 0.5) / 1.0)
        turn    = max(-MAX_ANGULAR, min(MAX_ANGULAR, -Kp_HEADING * h_err))
        self._publish(forward, turn)
        self._log_row(distance, 0.0, h_err, forward, turn)

    def _run_positioning(self):
        """
        Go-to-goal controller in the odometry frame.
        1. If heading error is large → rotate in place.
        2. Else → drive forward with proportional heading correction.
        3. When within POSITION_TOL, or timeout elapsed → switch to ALIGNING.
        """
        dx   = self._goal_x - self._robot_x
        dy   = self._goal_y - self._robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        elapsed = time.monotonic() - self._phase_start

        # Force transition if close enough OR timeout exceeded
        if dist < POSITION_TOL or elapsed > POSITIONING_TIMEOUT:
            self._publish(0.0, 0.0)
            self._phase       = Phase.ALIGNING
            self._phase_start = time.monotonic()
            reason = 'timeout' if elapsed > POSITIONING_TIMEOUT else f'dist={dist:.3f} m'
            self.get_logger().info(
                f'[PHASE] POSITIONING → ALIGNING  ({reason})  '
                f'target_yaw={math.degrees(self._goal_yaw):.1f}°')
            return

        bearing = math.atan2(dy, dx)
        h_err   = _normalize_angle(bearing - self._robot_yaw)

        if abs(h_err) > ALIGN_ONLY_ANGLE:
            turn = max(-MAX_ANGULAR, min(MAX_ANGULAR, Kp_HEADING * h_err))
            self._publish(0.0, turn)
            self._log_row(0.0, dist, h_err, 0.0, turn)
        else:
            forward = min(MAX_LINEAR, Kp_LINEAR * dist)
            turn    = max(-MAX_ANGULAR, min(MAX_ANGULAR, Kp_HEADING * h_err))
            self._publish(forward, turn)
            self._log_row(0.0, dist, h_err, forward, turn)

        self.get_logger().info(
            f'[POS] dist={dist:.2f} m  h_err={math.degrees(h_err):.1f}°  '
            f'elapsed={elapsed:.1f} s')

    def _run_aligning(self):
        """
        Rotate in place to face the injection zone (goal_yaw).
        Switches to READY when yaw error < YAW_TOL.
        """
        yaw_err = _normalize_angle(self._goal_yaw - self._robot_yaw)

        if abs(yaw_err) < YAW_TOL:
            self._publish(0.0, 0.0)
            self._phase       = Phase.READY
            self._phase_start = time.monotonic()
            iz = self._last_inj
            self.get_logger().info(
                f'[PHASE] ALIGNING → READY  '
                f'zone={iz.zone_type if iz else "?"}  '
                f'body={iz.body_position if iz else "?"}')
            self._log_row(0.0, 0.0, yaw_err, 0.0, 0.0)
            self._announce_ready()
        else:
            turn = max(-MAX_ANGULAR, min(MAX_ANGULAR, Kp_HEADING * yaw_err))
            self._publish(0.0, turn)
            self._log_row(0.0, 0.0, yaw_err, 0.0, turn)

    def _announce_ready(self):
        iz = self._last_inj
        self.get_logger().info(
            '*** INJECTION POSITION REACHED ***  '
            f'Zone: {iz.zone_type if iz else "unknown"}  '
            f'Body: {iz.body_position if iz else "unknown"}  '
            f'Pose: ({self._robot_x:.2f}, {self._robot_y:.2f}, '
            f'{math.degrees(self._robot_yaw):.1f}°)'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _publish(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    def _log_row(self, dist_person, dist_goal, h_err, lin_v, ang_v):
        iz = self._last_inj
        self._csv.writerow([
            time.time(),
            self._model_name,
            self._phase,
            f'{dist_person:.3f}',
            f'{dist_goal:.3f}',
            f'{h_err:.4f}',
            f'{lin_v:.3f}',
            f'{ang_v:.3f}',
            iz.body_position      if iz else 'unknown',
            iz.zone_type          if iz else 'none',
            f'{iz.confidence:.3f}' if iz else '0.000',
            int(iz.goal_valid)    if iz else 0,
            f'{iz.nav_goal_x:.3f}'   if iz else '0.000',
            f'{iz.nav_goal_y:.3f}'   if iz else '0.000',
            f'{iz.nav_goal_yaw:.4f}' if iz else '0.0000',
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


if __name__ == '__main__':
    main()
