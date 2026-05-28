#!/usr/bin/env bash
# SO 리더 → 실기 move_joint + TCP 그리퍼 (gripper_service_node 가 먼저 ready 여야 함)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh"

echo "[run_leader_to_real_tcp] 팔: move_joint | 그리퍼: /gripper_service/set_position (TCP)"
exec python3 "${SCRIPT_DIR}/so_to_real_dsr_teleop.py" --ros-args \
  -p gripper_mode:=tcp \
  -p gripper_service_ns:=/gripper_service \
  -p gripper_position_max:=1150 \
  -p gripper_position_tolerance:=30 \
  -p gripper_move_timeout_sec:=0.0 \
  -p gripper_command_period:=0.12 \
  -p command_period:=0.05 \
  -p move_vel:=20.0 \
  -p move_acc:=20.0 \
  -p sync_type:=0
