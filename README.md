# so-doosan-teleoperation

SO-ARM101 **리더암** → Doosan **E0509** 텔레옵 (RViz virtual / 실기) + **RH-P12-RN** 그리퍼.

## 요구 사항

- Ubuntu 22.04, ROS 2 Humble
- [doosan-robot2](https://github.com/DoosanRobotics/doosan-robot2) (`~/doosan_ws`)
- SO 리더 USB (`pyserial`)
- (실기 그리퍼) [Dakae TCP Bridge](https://github.com/Dakae/Doosan-E0509-ROBOTIS-RH-P12-RN-TCP-Bridge) → `GRIPPER_TCP.md`

## 빠른 시작

```bash
# 1) SO 리더 bridge 빌드
cd ~/so-doosan-teleoperation/so-arm101-ros2-bridge
colcon build --packages-select jointstatereader
source install/setup.bash

# 2) 환경 (doosan_ws 먼저 빌드되어 있어야 함)
source ~/so-doosan-teleoperation/env.sh
```

| 목적 | 문서 |
|------|------|
| RViz virtual + 그리퍼 | [TELEOP_RUN.md](TELEOP_RUN.md) |
| 실기 E0509 + TCP 그리퍼 | [REAL_TELEOP_RUN.md](REAL_TELEOP_RUN.md) |
| TCP Bridge 상세 | [GRIPPER_TCP.md](GRIPPER_TCP.md) |

## 구조

```
├── so_to_real_dsr_teleop.py    # 리더 → move_joint + 그리퍼
├── doosan_rh_p12_gripper.py     # virtual / tcp / real 그리퍼
├── env.sh                        # doosan_ws 우선 source
├── run_leader_usb.sh             # 리더 → /joint_states
├── run_leader_to_virtual.sh      # RViz 텔레옵
├── run_leader_to_real_tcp.sh     # 실기 텔레옵
├── run_gripper_tcp_service.sh    # 실기 TCP 그리퍼 노드
├── spawn_rh_p12_gripper_controller.sh  # RViz 그리퍼 (virtual만)
└── so-arm101-ros2-bridge/        # joint_state_reader 패키지
```

## 매핑 요약

| SO 리더 | Doosan E0509 |
|---------|----------------|
| Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll | joint_1, 2, 3, 5, 6 |
| (없음) | joint_4 = 0 고정 |
| Jaw | RH-P12 그리퍼 (별도 경로) |

기본 `mapping_preset:=neutral` (실기·RViz 검증):

- joint_1 = −Rotation  
- joint_2 = +Pitch  
- joint_3 = +Elbow + 90°  
- joint_5 = +Wrist_Pitch  
- joint_6 = −Wrist_Roll  

선택: `offset_90` — rot·elbow 둘 다 −90° 오프셋 (레거시 teleop 호환).

리더 Jaw 열림 ↔ 두산 그리퍼 닫힘 (방향 반전 매핑 적용됨).

## 라이선스

`so-arm101-ros2-bridge` 는 upstream SO ARM bridge 기반 (Apache-2.0). 텔레옵 스크립트는 프로젝트 내부용.
