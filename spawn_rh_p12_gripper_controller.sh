#!/usr/bin/env bash
# E0509 RViz launch (gripper:=rh_p12_rn) 후 그리퍼 ros2_control 스폰.
# 팔(dsr_controller2)과 별도 — 리더 Jaw → RViz 그리퍼.

set -e
ROBOT_ID="${ROBOT_ID:-dsr01}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARAM_FILE="${SCRIPT_DIR}/config/rh_p12_gripper_controller.yaml"
CM="/${ROBOT_ID}/controller_manager"

source /opt/ros/humble/setup.bash
source "${HOME}/doosan_ws/install/setup.bash" 2>/dev/null || true

echo "[spawn_rh_p12] Unload previous gripper controller (if any)..."
ros2 control unload_controller rh_p12_gripper_controller -c "${CM}" 2>/dev/null || true

echo "[spawn_rh_p12] Spawning rh_p12_gripper_controller on ${CM}"
exec ros2 run controller_manager spawner rh_p12_gripper_controller \
  -c "${CM}" \
  --param-file "${PARAM_FILE}" \
  --controller-manager-timeout 120
