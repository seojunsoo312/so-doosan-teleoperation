#!/usr/bin/env bash
# SO 리더 USB → /joint_states (LeRobot 없이 joint_state_reader 사용)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE="${SCRIPT_DIR}/so-arm101-ros2-bridge"
USB_PORT="${USB_PORT:-/dev/ttyACM0}"

source /opt/ros/humble/setup.bash

# 다른 PC(bluephysi01)에서 빌드된 install 이면 재빌드 필요
if grep -q bluephysi01 "${BRIDGE}/install/setup.bash" 2>/dev/null; then
  echo "[run_leader_usb] bridge 재빌드 (이전 머신 경로 제거)..."
  cd "${BRIDGE}"
  rm -rf build install log
  colcon build --packages-select jointstatereader
fi

source "${BRIDGE}/install/setup.bash"

if ! python3 -c "import serial" 2>/dev/null; then
  echo "설치: pip3 install pyserial   (또는: sudo apt install python3-serial)"
  exit 1
fi

echo "[run_leader_usb] USB=${USB_PORT}  →  /joint_states (~30Hz)"
exec ros2 run jointstatereader joint_state_reader --ros-args \
  -p "usb_port:=${USB_PORT}" \
  -p publish_rate_hz:=30.0
