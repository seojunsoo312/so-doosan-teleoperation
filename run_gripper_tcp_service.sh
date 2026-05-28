#!/usr/bin/env bash
# Dakae TCP gripper bridge — 실기 E0509 + RH-P12-RN (텔레옵 전에 실행)
# https://github.com/Dakae/Doosan-E0509-ROBOTIS-RH-P12-RN-TCP-Bridge

set -e
CONTROLLER_HOST="${CONTROLLER_HOST:-110.120.1.56}"
ROBOT_NS="${ROBOT_NS:-dsr01}"
SERVICE_PREFIX="${SERVICE_PREFIX:-}"

source ~/so-doosan-teleop/env.sh

if ! ros2 pkg prefix dsr_gripper_tcp &>/dev/null; then
  echo "[run_gripper_tcp_service] dsr_gripper_tcp 패키지 없음."
  echo "  cd ~/doosan_ws/src"
  echo "  git clone https://github.com/Dakae/Doosan-E0509-ROBOTIS-RH-P12-RN-TCP-Bridge.git"
  echo "  cd ~/doosan_ws && colcon build --packages-select dsr_gripper_tcp_interfaces dsr_gripper_tcp"
  echo "  source install/setup.bash"
  exit 1
fi

pip3 install flask flask-socketio --user 2>/dev/null || true

echo "[run_gripper_tcp_service] controller_host=${CONTROLLER_HOST} namespace=${ROBOT_NS}"
if [[ -n "${SERVICE_PREFIX}" ]]; then
  echo "  service_prefix=${SERVICE_PREFIX}"
fi
echo "  INITIALIZE 실패 로그가 몇 번 나올 수 있음 → 'Gripper service node ready' 까지 대기"

LAUNCH_ARGS=(
  "controller_host:=${CONTROLLER_HOST}"
  "namespace:=${ROBOT_NS}"
  "position_max:=1150"
)
# 빈 service_prefix:= 는 ros2 launch 파싱 오류 → 인자 생략(기본값 "")
if [[ -n "${SERVICE_PREFIX}" ]]; then
  LAUNCH_ARGS+=("service_prefix:=${SERVICE_PREFIX}")
fi

exec ros2 launch dsr_gripper_tcp gripper_service_node.launch.py "${LAUNCH_ARGS[@]}"
