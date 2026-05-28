# SO 리더 → 실기 E0509 + RH-P12 (복붙용)

**전제**
- RViz virtual(터미널 1·1b·3)은 **전부 종료** (실기와 동시에 쓰지 않음)
- 로봇 **E-Stop·작업 공간** 확인 후 진행
- `so_dsr_joint_mapper`, `move_web` **끄기**

`<로봇_IP>` 를 컨트롤러 IP로 바꾸세요 (예: `110.120.1.56`).

---

## 최초 1회 — TCP 그리퍼 Bridge

```bash
cd ~/doosan_ws/src
git clone https://github.com/Dakae/Doosan-E0509-ROBOTIS-RH-P12-RN-TCP-Bridge.git
cd ~/doosan_ws
colcon build --packages-select dsr_gripper_tcp_interfaces dsr_gripper_tcp
source install/setup.bash
pip3 install flask flask-socketio --user
```

---

## 터미널 A — 실기 bringup

```bash
export ROBOT_IP=<로봇_IP>
source ~/so-doosan-teleoperation/env.sh
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real model:=e0509 host:=${ROBOT_IP} port:=12345 \
  gripper:=rh_p12_rn gui:=true
```

`move_joint` 서비스 뜰 때까지 대기.

---

## 터미널 B — TCP 그리퍼 (`Gripper service node ready`)

```bash
export ROBOT_IP=<로봇_IP>
export CONTROLLER_HOST=${ROBOT_IP}
source ~/doosan_ws/install/setup.bash
~/so-doosan-teleoperation/run_gripper_tcp_service.sh
```

`INITIALIZE` 실패 로그가 몇 번 나올 수 있음 → **`Gripper service node ready`** 나올 때까지 대기.

---

## 터미널 C — SO 리더 USB

```bash
export USB_PORT=/dev/ttyACM0
~/so-doosan-teleoperation/run_leader_usb.sh
```

---

## 터미널 D — 텔레옵 (팔 + 그리퍼, Jaw 방향 반영됨)

```bash
~/so-doosan-teleoperation/run_leader_to_real_tcp.sh
```

기본: `move_vel=20%`, `sync_type=0`(SYNC), `gripper_mode=tcp`.

---

## 동작 확인 (선택)

```bash
ros2 service list | grep move_joint
ros2 service list | grep gripper_service
ros2 topic hz /joint_states
```

그리퍼만 테스트:

```bash
ros2 service call /gripper_service/set_position \
  dsr_gripper_tcp_interfaces/srv/SetPosition "{position: 1150, timeout_sec: 5.0}"
```

---

## 주의

| 하지 말 것 | 이유 |
|------------|------|
| `spawn_rh_p12_gripper_controller.sh` | virtual RViz 전용 |
| virtual launch 와 동시 실행 | `move_joint` 충돌 |
| 처음부터 `move_vel` 40% | 실기는 20% 권장 |

더 빠르게: 터미널 D에서 `-p move_vel:=25.0` (신중히).
