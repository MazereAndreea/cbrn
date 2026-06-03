#!/usr/bin/env bash
# Benchmark script: resets the robot and runs it through all four phases
# (APPROACHING → POSITIONING → ALIGNING → READY).
#
# Start your detector separately before calling this script:
#   ros2 run cbrn_perception yolo_detector
#
# Prerequisites: sim already running (ros2 launch sim_env start_sim.launch.py)
#
# Usage:
#   ./run_bench.sh [--model <label>]
#     --model  optional label written into the CSV log (default: unknown)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash"
set -u

# ---------------------------------------------------------------------------
# Parse arguments
# ---------------------------------------------------------------------------
MODEL="unknown"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --model) MODEL="$2"; shift 2 ;;
        *) echo "[bench] Unknown argument: $1" >&2; exit 1 ;;
    esac
done

echo "[bench] Model label: $MODEL"

# ---------------------------------------------------------------------------
# 1. Wait for the robot to appear in Gazebo
# ---------------------------------------------------------------------------
echo "[bench] Waiting for cbrn_robot in Gazebo..."
until gz model --list 2>/dev/null | grep -q "cbrn_robot"; do sleep 1; done
echo "[bench] Robot found."

# ---------------------------------------------------------------------------
# 2. Wait for both drive controllers to be active
# ---------------------------------------------------------------------------
echo "[bench] Waiting for controllers to be active..."
for i in $(seq 1 30); do
    STATUS=$(timeout 3 ros2 control list_controllers 2>/dev/null || true)
    if echo "$STATUS" | grep -q "diff_drive_controller.*active" && \
       echo "$STATUS" | grep -q "joint_state_broadcaster.*active"; then
        echo "[bench] Controllers ready."
        break
    fi
    sleep 1
done

# ---------------------------------------------------------------------------
# 3. Reset robot to starting pose
# ---------------------------------------------------------------------------
echo "[bench] Resetting robot pose to (-3, 0) ..."
gz service -s /world/cbrn_world/set_pose \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req 'name: "cbrn_robot", position: {x: -3.0, y: 0.0, z: 0.05}'
sleep 1

# ---------------------------------------------------------------------------
# 4. Run robot_controller through all 4 phases
# ---------------------------------------------------------------------------
echo "[bench] Starting robot_controller ..."
ros2 run cbrn_perception robot_controller \
    --ros-args -p model_name:="$MODEL"

echo "[bench] Final robot position:"
gz model -m cbrn_robot -p 2>/dev/null || true
echo "[bench] Done."
