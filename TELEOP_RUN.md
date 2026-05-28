# SO 리더 → virtual E0509 텔레옵 (복붙용)

**전제:** `so_dsr_joint_mapper` / `move_web` 은 **끄기**.  
**순서:** 터미널 2 → 1 → **1b(그리퍼)** → 3 (virtual 먼저)

USB 포트 확인: `ls -l /dev/ttyACM*`

---

## 터미널 1 — E0509 virtual + RViz

```bash
source ~/so-doosan-teleoperation/env.sh
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=virtual model:=e0509 gui:=true gripper:=rh_p12_rn
```

---

## 터미널 1b — RH-P12 ros2_control (RViz 그리퍼용, **필수**)

`dsr_bringup2_rviz` 는 팔만 스폰합니다. 리더 **Jaw(6번)** → RViz 그리퍼를 쓰려면 별도 스폰:

```bash
source ~/doosan_ws/install/setup.bash
~/so-doosan-teleoperation/spawn_rh_p12_gripper_controller.sh
```

`Successfully loaded controller rh_p12_gripper_controller` 확인.

**`Failed to activate controller` 가 나오면:**

1. 터미널 1 **RViz launch 끄고** 아래 한 번 실행 (URDF mock 그리퍼 수정 반영):

```bash
cd ~/doosan_ws
colcon build --packages-select dsr_description2
source install/setup.bash
```

2. 터미널 1·1b 다시 실행 (`spawn_rh_p12_gripper_controller.sh` 는 갱신된 스크립트 사용 — unload 후 재스폰).

---

## 터미널 2 — SO 리더 → `/joint_states` (~30Hz)

```bash
source /opt/ros/humble/setup.bash
cd ~/so-doosan-teleoperation/so-arm101-ros2-bridge
source install/setup.bash
pip3 install pyserial --user

export USB_PORT=/dev/ttyACM0
ros2 run jointstatereader joint_state_reader --ros-args \
  -p usb_port:=${USB_PORT} \
  -p publish_rate_hz:=30.0
```

`Connected to SO leader on /dev/ttyACM0` 확인.

---

## 터미널 3 — 리더 → moveJ + 그리퍼 (저지연)

```bash
source ~/so-doosan-teleoperation/env.sh
source ~/doosan_ws/install/setup.bash

python3 ~/so-doosan-teleoperation/so_to_real_dsr_teleop.py --ros-args \
  -p gripper_mode:=virtual \
  -p command_period:=0.05 \
  -p joint_tolerance_deg:=0.03 \
  -p leader_sample_tolerance_deg:=0.02 \
  -p move_vel:=40.0 \
  -p move_acc:=40.0 \
  -p sync_type:=1 \
  -p blend_type:=1 \
  -p move_radius:=30.0 \
  -p gripper_command_period:=0.08 \
  -p gripper_tolerance_rad:=0.015
```

로그 예: `sync=ASYNC`, `gripper=virtual`, `Virtual RH-P12 gripper action server connected.`

| `gripper_mode` | 용도 |
|----------------|------|
| `none` | 팔만 (Jaw 무시) |
| `virtual` | RViz mock 그리퍼 (터미널 1b 필요) |
| `tcp` | **실기 권장** — [Dakae TCP Bridge](https://github.com/Dakae/Doosan-E0509-ROBOTIS-RH-P12-RN-TCP-Bridge) (`GRIPPER_TCP.md`) |
| `real` | legacy Modbus/DRL (`drl_start`, readback 약함) |

---

## (선택) 터미널 4 — 확인

```bash
ros2 topic hz /joint_states
ros2 service list | grep move_joint
```

---

## 스크립트로 실행 (동일 내용)

```bash
# 터미널 1: 위 launch
# 터미널 1b: spawn_rh_p12_gripper_controller.sh

# 터미널 2
USB_PORT=/dev/ttyACM0 ~/so-doosan-teleoperation/run_leader_usb.sh

# 터미널 3
~/so-doosan-teleoperation/run_leader_to_virtual.sh
```

`run_leader_usb.sh` 사용 전 bridge 한 번 빌드:

```bash
cd ~/so-doosan-teleoperation/so-arm101-ros2-bridge
colcon build --packages-select jointstatereader
```

---

## 더 빠르게 / 더 느리게

```bash
# 더 빠르게 (virtual만)
-p command_period:=0.03 -p move_vel:=50.0 -p move_acc:=50.0

# 더 부드럽게
-p command_period:=0.08 -p move_vel:=25.0 -p joint_tolerance_deg:=0.2
```

---

## 실기 (TCP 그리퍼 권장)

상세: **`GRIPPER_TCP.md`**

```bash
# A: bringup
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real model:=e0509 host:=<로봇_IP> gripper:=rh_p12_rn gui:=true

# B: TCP gripper service
CONTROLLER_HOST=<로봇_IP> ~/so-doosan-teleoperation/run_gripper_tcp_service.sh

# C: 리더 USB
USB_PORT=/dev/ttyACM0 ~/so-doosan-teleoperation/run_leader_usb.sh

# D: 텔레옵
~/so-doosan-teleoperation/run_leader_to_real_tcp.sh
```

RViz만 검증: `gripper_mode:=virtual` + 터미널 1b.
