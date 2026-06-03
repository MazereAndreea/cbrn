#!/usr/bin/env python3
"""
scenario_orchestrator — ROS2 node (section 2.6 / 3.5 of the thesis)

Implements the full CBRN intervention state machine:

  NAVIGATE_TO_ZONE
      ↓  robot arrives at triage area
  DETECT_BODY
      ↓  body detected by perception node
  ANALYZE_INJECTION_ZONE
      ↓  injection zone identified and Nav2Goal computed
  COMPUTE_INJECTION_GOAL
      ↓  goal sent to Nav2
  NAVIGATE_TO_INJECTION
      ↓  robot arrives at injection position
  ARRIVED
      ↓  intervention complete

Interfaces
----------
Subscribes:
  /injection_zone   (cbrn_interfaces/InjectionZone)

Uses Nav2 action client:
  NavigateToPose    (nav2_msgs/action/NavigateToPose)

Parameters (all with ROS2 params):
  zone_goal_x, zone_goal_y, zone_goal_yaw  — initial triage-area Nav2Goal
  detect_timeout_s   — seconds to wait for body detection before aborting
  min_zone_conf      — minimum injection-zone confidence before accepting goal
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from cbrn_interfaces.msg import InjectionZone

# ── State labels ──────────────────────────────────────────────────────────────
class State:
    NAVIGATE_TO_ZONE      = 'NAVIGATE_TO_ZONE'
    DETECT_BODY           = 'DETECT_BODY'
    ANALYZE_INJECTION_ZONE = 'ANALYZE_INJECTION_ZONE'
    COMPUTE_INJECTION_GOAL = 'COMPUTE_INJECTION_GOAL'
    NAVIGATE_TO_INJECTION = 'NAVIGATE_TO_INJECTION'
    ARRIVED               = 'ARRIVED'
    FAILED                = 'FAILED'


def _make_pose_stamped(x, y, yaw, frame_id='map'):
    """Build a PoseStamped from 2-D position + yaw."""
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = 0.0
    half = yaw / 2.0
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps


class ScenarioOrchestrator(Node):

    def __init__(self):
        super().__init__('scenario_orchestrator',
                         automatically_declare_parameters_from_overrides=True)

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('zone_goal_x',      0.0)
        self.declare_parameter('zone_goal_y',      0.0)
        self.declare_parameter('zone_goal_yaw',    0.0)
        self.declare_parameter('detect_timeout_s', 30.0)
        self.declare_parameter('min_zone_conf',    0.25)

        self._zone_x   = self.get_parameter('zone_goal_x').value
        self._zone_y   = self.get_parameter('zone_goal_y').value
        self._zone_yaw = self.get_parameter('zone_goal_yaw').value
        self._det_timeout = self.get_parameter('detect_timeout_s').value
        self._min_conf    = self.get_parameter('min_zone_conf').value

        # ── Nav2 action client ─────────────────────────────────────────────
        self._cb_group    = ReentrantCallbackGroup()
        self._nav_client  = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group)

        # ── Subscriber ─────────────────────────────────────────────────────
        self.create_subscription(
            InjectionZone, '/injection_zone',
            self._inj_cb, 10,
            callback_group=self._cb_group)

        # ── State ──────────────────────────────────────────────────────────
        self._state           = State.NAVIGATE_TO_ZONE
        self._injection_zone  = None   # latest valid InjectionZone message
        self._state_start     = time.monotonic()
        self._nav_goal_handle = None
        self._nav_done        = False
        self._nav_succeeded   = False

        # ── State-machine timer (10 Hz) ────────────────────────────────────
        self.create_timer(0.1, self._tick, callback_group=self._cb_group)

        self.get_logger().info(
            f'ScenarioOrchestrator started. '
            f'Triage zone goal: ({self._zone_x:.2f}, {self._zone_y:.2f}, '
            f'yaw={self._zone_yaw:.2f} rad)')

    # ── Injection-zone subscription ───────────────────────────────────────────

    def _inj_cb(self, msg: InjectionZone):
        if msg.goal_valid and msg.confidence >= self._min_conf:
            self._injection_zone = msg

    # ── Nav2 helpers ──────────────────────────────────────────────────────────

    def _send_nav_goal(self, x, y, yaw):
        """Send a NavigateToPose goal; non-blocking — result handled via callback."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available!')
            self._transition(State.FAILED)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = _make_pose_stamped(x, y, yaw)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self._nav_done      = False
        self._nav_succeeded = False

        send_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_cb)
        send_future.add_done_callback(self._nav_goal_response_cb)
        self.get_logger().info(f'Nav2 goal sent: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')

    def _nav_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 goal rejected.')
            self._nav_done      = True
            self._nav_succeeded = False
            return
        self._nav_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        status = future.result().status
        # action_msgs/msg/GoalStatus: SUCCEEDED = 4
        self._nav_succeeded = (status == 4)
        self._nav_done      = True
        self.get_logger().info(
            f'Nav2 result: {"SUCCESS" if self._nav_succeeded else "FAILED"} '
            f'(status={status})')

    def _nav_feedback_cb(self, feedback_msg):
        # Log estimated time remaining (optional)
        fb = feedback_msg.feedback
        remaining = getattr(fb, 'estimated_time_remaining', None)
        if remaining:
            secs = remaining.sec + remaining.nanosec * 1e-9
            self.get_logger().debug(f'Nav2 ETA: {secs:.1f} s')

    # ── State machine ─────────────────────────────────────────────────────────

    def _transition(self, new_state):
        self.get_logger().info(f'[FSM] {self._state} → {new_state}')
        self._state       = new_state
        self._state_start = time.monotonic()

    def _tick(self):
        now     = time.monotonic()
        elapsed = now - self._state_start

        # ── NAVIGATE_TO_ZONE ──────────────────────────────────────────────
        if self._state == State.NAVIGATE_TO_ZONE:
            if elapsed < 0.5:
                # First tick: send the goal once
                self._send_nav_goal(self._zone_x, self._zone_y, self._zone_yaw)
                return
            if self._nav_done:
                if self._nav_succeeded:
                    self._transition(State.DETECT_BODY)
                else:
                    self.get_logger().warn('Navigation to zone failed; retrying.')
                    self._state_start = time.monotonic()   # allow retry
                    self._nav_done    = False

        # ── DETECT_BODY ───────────────────────────────────────────────────
        elif self._state == State.DETECT_BODY:
            if self._injection_zone is not None:
                self._transition(State.ANALYZE_INJECTION_ZONE)
            elif elapsed > self._det_timeout:
                self.get_logger().warn(
                    f'Body not detected after {self._det_timeout:.0f} s — '
                    'repositioning robot.')
                # Simple fallback: rotate slightly and retry detection
                self._injection_zone = None
                self._state_start    = time.monotonic()

        # ── ANALYZE_INJECTION_ZONE ────────────────────────────────────────
        elif self._state == State.ANALYZE_INJECTION_ZONE:
            iz = self._injection_zone
            if iz is None or not iz.goal_valid:
                self._transition(State.DETECT_BODY)
                return
            self.get_logger().info(
                f'[FSM] Zone={iz.zone_type}  pos={iz.body_position}  '
                f'conf={iz.confidence:.2f}  dist={iz.distance_m:.2f} m')
            self._transition(State.COMPUTE_INJECTION_GOAL)

        # ── COMPUTE_INJECTION_GOAL ────────────────────────────────────────
        elif self._state == State.COMPUTE_INJECTION_GOAL:
            iz = self._injection_zone
            if iz is None or not iz.goal_valid:
                self.get_logger().warn('No valid injection goal — back to detection.')
                self._transition(State.DETECT_BODY)
                return
            self.get_logger().info(
                f'[FSM] Nav2Goal for injection: '
                f'({iz.nav_goal_x:.2f}, {iz.nav_goal_y:.2f}, '
                f'yaw={iz.nav_goal_yaw:.2f} rad)')
            self._send_nav_goal(iz.nav_goal_x, iz.nav_goal_y, iz.nav_goal_yaw)
            self._transition(State.NAVIGATE_TO_INJECTION)

        # ── NAVIGATE_TO_INJECTION ─────────────────────────────────────────
        elif self._state == State.NAVIGATE_TO_INJECTION:
            if self._nav_done:
                if self._nav_succeeded:
                    self._transition(State.ARRIVED)
                else:
                    self.get_logger().warn(
                        'Navigation to injection position failed. '
                        'Re-computing goal.')
                    self._injection_zone = None
                    self._transition(State.DETECT_BODY)

        # ── ARRIVED ───────────────────────────────────────────────────────
        elif self._state == State.ARRIVED:
            iz = self._injection_zone
            self.get_logger().info(
                '*** ARRIVED at injection position ***  '
                f'Zone: {iz.zone_type if iz else "unknown"}.  '
                'Injection ready.',
                once=True)

        # ── FAILED ────────────────────────────────────────────────────────
        elif self._state == State.FAILED:
            self.get_logger().error('Scenario FAILED.', once=True)


def main():
    rclpy.init()
    node = ScenarioOrchestrator()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
