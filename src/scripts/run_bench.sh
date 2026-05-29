#!/usr/bin/env bash
# Benchmark script: positions the robot and a standing person in Gazebo,
# then drives the robot forward past the person to exercise perception models.
#
# Prerequisites: sim already running (ros2 launch sim_env start_sim.launch.py)
# Usage: ./run_bench.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEND_CMD_VEL="$SCRIPT_DIR/send_cmd_vel.py"

# ---------------------------------------------------------------------------
# 1. Place the robot at a known starting pose
# ---------------------------------------------------------------------------
echo "[bench] Setting robot pose..."
gz service -s /world/cbrn_world/set_pose \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req 'name: "cbrn_robot", position: {x: 0.0, y: 0.0, z: 0.05}'

# ---------------------------------------------------------------------------
# 2. Place the person model 3 m ahead of the robot
# ---------------------------------------------------------------------------
echo "[bench] Setting person pose..."
gz service -s /world/cbrn_world/set_pose \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req 'name: "person_standing", position: {x: 3.0, y: 0.0, z: 0.0}'

sleep 1

# ---------------------------------------------------------------------------
# Diagnostics
# ---------------------------------------------------------------------------
echo "[bench] Hardware components:"
timeout 5 ros2 control list_hardware_components 2>/dev/null || echo "  (service unavailable)"

echo "[bench] Controller status:"
timeout 5 ros2 control list_controllers 2>/dev/null || echo "  (ros2 control not available)"

# ---------------------------------------------------------------------------
# Activate hardware, then controllers
# ---------------------------------------------------------------------------
echo "[bench] Activating hardware component..."
timeout 5 ros2 control set_hardware_component_state GazeboSystem active 2>/dev/null || true
sleep 1

echo "[bench] Activating controllers..."
timeout 5 ros2 control set_controller_state joint_state_broadcaster configure 2>/dev/null || true
timeout 5 ros2 control set_controller_state joint_state_broadcaster active 2>/dev/null || true
timeout 5 ros2 control set_controller_state diff_drive_controller active 2>/dev/null || true
sleep 1

echo "[bench] Controller status after activation:"
timeout 5 ros2 control list_controllers 2>/dev/null || true

# ---------------------------------------------------------------------------
# 3. Drive forward for 5 s, then stop
#    Topic  : /diff_drive_controller/cmd_vel
#    Type   : geometry_msgs/TwistStamped  (use_stamped_vel: true)
# ---------------------------------------------------------------------------
echo "[bench] Driving forward (linear.x=0.3 m/s for 5 s)..."
python3 "$SEND_CMD_VEL" 0.3 0.0 5.0

echo "[bench] Done."
