# SO-ARM101 ROS 2 bridge (jointstatereader)

SO 리더 USB → `/joint_states` (`Rotation` … `Jaw`).

```bash
cd ~/so-doosan-teleoperation/so-arm101-ros2-bridge
colcon build --packages-select jointstatereader
source install/setup.bash

export USB_PORT=/dev/ttyACM0
ros2 run jointstatereader joint_state_reader --ros-args \
  -p usb_port:=${USB_PORT} -p publish_rate_hz:=30.0
```

또는 루트의 `run_leader_usb.sh` 사용.
