# RH-P12-RN TCP 그리퍼 (Dakae Bridge)

실기 E0509에서 리더 **Jaw** → ROBOTIS **RH-P12-RN** 제어는  
[Doosan-E0509-ROBOTIS-RH-P12-RN-TCP-Bridge](https://github.com/Dakae/Doosan-E0509-ROBOTIS-RH-P12-RN-TCP-Bridge) 를 권장합니다.

- DRL Modbus만 쓰면 **상태 피드백(read)** 이 불안정 → TCP 서버 + ROS 서비스로 해결
- `so_to_real_dsr_teleop.py` 의 `gripper_mode:=tcp` 가 `/gripper_service/set_position` 호출

| `gripper_mode` | 용도 |
|----------------|------|
| `virtual` | RViz mock (`rh_p12_gripper_controller` 스폰 필요) |
| `tcp` | **실기 권장** — `gripper_service_node` |
| `real` | legacy `drl_start` Modbus (move_web 동일, readback 약함) |

## Jaw → 그리퍼 pulse

| SO Jaw (rad) | 리더 | TCP `position` (max=1150) |
|--------------|------|---------------------------|
| 0.0 | 열림 | 1150 (두산 닫힘) |
| 2.2 | 닫힘 | 0 (두산 열림) |

`gripper_position_max` 파라미터로 max 맞추기 (`gripper_service_node` 의 `position_max` 와 동일 권장).

## 1. Bridge 빌드 (최초 1회)

```bash
cd ~/doosan_ws/src
git clone https://github.com/Dakae/Doosan-E0509-ROBOTIS-RH-P12-RN-TCP-Bridge.git
cd ~/doosan_ws
colcon build --packages-select dsr_gripper_tcp_interfaces dsr_gripper_tcp
source install/setup.bash
pip3 install flask flask-socketio --user
```

## 2. 실기 실행 순서

**터미널 A** — 두산 bringup (`mode:=real`, 로봇 IP)

```bash
source ~/so-doosan-teleoperation/env.sh
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real model:=e0509 host:=<로봇_IP> gripper:=rh_p12_rn gui:=true
```

**터미널 B** — TCP gripper service (`Gripper service node ready` 까지 대기)

```bash
CONTROLLER_HOST=<로봇_IP> ~/so-doosan-teleoperation/run_gripper_tcp_service.sh
```

**터미널 C** — SO 리더

```bash
USB_PORT=/dev/ttyACM0 ~/so-doosan-teleoperation/run_leader_usb.sh
```

**터미널 D** — 텔레옵 (팔 + 그리퍼)

```bash
~/so-doosan-teleoperation/run_leader_to_real_tcp.sh
```

## 3. RViz만 (virtual)

TCP 서비스 없이 RViz 그리퍼만:

1. `dsr_bringup2_rviz.launch.py` … `gripper:=rh_p12_rn`
2. `spawn_rh_p12_gripper_controller.sh`
3. `run_leader_to_virtual.sh` (`gripper_mode:=virtual`)

## 4. 수동 테스트

```bash
ros2 service call /gripper_service/set_position \
  dsr_gripper_tcp_interfaces/srv/SetPosition "{position: 0, timeout_sec: 5.0}"

ros2 topic echo /gripper_service/state
```

## 5. Safe grasp / 웹 UI (선택)

```bash
ros2 launch dsr_gripper_tcp web_dashboard_node.launch.py web_port:=5000
# http://localhost:5000
```

텔레옵 노드는 `SafeGrasp` action 을 쓰지 않습니다. 작업 로직에서 필요 시 별도 노드가 action 호출.
