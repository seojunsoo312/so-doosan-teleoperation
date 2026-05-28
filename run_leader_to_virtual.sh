#!/usr/bin/env bash
# SO 리더 → virtual E0509 RViz (move_joint). virtual launch 가 먼저 떠 있어야 함.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh"
source "${HOME}/doosan_ws/install/setup.bash"

echo "[run_leader_to_virtual] SO /joint_states → move_joint + virtual gripper"
echo "  (터미널 1b: spawn_rh_p12_gripper_controller.sh 가 떠 있어야 Jaw→RViz 그리퍼 동작)"
exec python3 "${SCRIPT_DIR}/so_to_real_dsr_teleop.py" --ros-args \
  -p mapping_preset:=neutral \
  -p gripper_mode:=virtual \
  -p command_period:=0.05 \
  -p joint_tolerance_deg:=0.03 \
  -p leader_sample_tolerance_deg:=0.02 \
  -p move_vel:=40.0 \
  -p move_acc:=40.0 \
  -p sync_type:=1
