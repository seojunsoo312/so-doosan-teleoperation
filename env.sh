#!/usr/bin/env bash
# SO–Doosan teleop: doosan_ws 우선 (cube_solver 구버전 dsr_description2는 gripper URDF 없음)

set -e

source /opt/ros/humble/setup.bash

# 다른 워크스페이스가 이미 source 된 터미널이면 cube_solver가 dsr_* 를 가로챌 수 있음
if [[ -n "${AMENT_PREFIX_PATH:-}" ]] && echo "${AMENT_PREFIX_PATH}" | grep -q cube_solver; then
  echo "[env.sh] warning: cube_solver_ver2_ws 가 AMENT_PREFIX_PATH 에 있습니다."
  echo "         gripper 가 RViz 에 안 보이면: 새 터미널에서 이 스크립트만 source 하세요."
fi

source "${HOME}/doosan_ws/install/setup.bash"

# dsr_description2 / dsr_bringup2 를 doosan_ws 로 고정
_doosan_prepend() {
  local pkg=$1
  local p="${HOME}/doosan_ws/install/${pkg}"
  if [[ -d "${p}" ]]; then
    AMENT_PREFIX_PATH="${p}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
    export AMENT_PREFIX_PATH
  fi
}

_doosan_prepend rh_p12_rn_description
_doosan_prepend dsr_description2
_doosan_prepend dsr_hardware2
_doosan_prepend dsr_controller2
_doosan_prepend dsr_msgs2
_doosan_prepend dsr_bringup2
_doosan_prepend dsr_gripper_tcp_interfaces
_doosan_prepend dsr_gripper_tcp

echo "[env.sh] dsr_description2 = $(ros2 pkg prefix dsr_description2 2>/dev/null || echo MISSING)"
echo "[env.sh] dsr_bringup2     = $(ros2 pkg prefix dsr_bringup2 2>/dev/null || echo MISSING)"
if ros2 pkg prefix dsr_gripper_tcp &>/dev/null; then
  echo "[env.sh] dsr_gripper_tcp  = $(ros2 pkg prefix dsr_gripper_tcp)"
else
  echo "[env.sh] dsr_gripper_tcp  = MISSING (실기 TCP 그리퍼: GRIPPER_TCP.md 참고)"
fi
