"""SO Jaw → Doosan RH-P12-RN gripper.

Modes:
  - virtual: ros2_control rh_p12_gripper_controller (RViz)
  - real:    DrlStart + flange Modbus (move_web style, legacy)
  - tcp:     dsr_gripper_tcp gripper_service_node (recommended on real E0509)
             https://github.com/Dakae/Doosan-E0509-ROBOTIS-RH-P12-RN-TCP-Bridge
"""
from __future__ import annotations

import textwrap
from typing import List

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dsr_msgs2.srv import DrlStart

GRIPPER_JOINT_NAMES = ["rh_p12_rn", "rh_r2", "rh_l1", "rh_l2"]
GRIPPER_STROKE_MIN = 0
# move_web / legacy DRL examples
GRIPPER_STROKE_MAX_LEGACY = 700
# Dakae TCP bridge default (SetPosition pulse)
GRIPPER_POSITION_MAX_TCP = 1150

SO_JAW_OPEN_RAD = 0.0
SO_JAW_CLOSED_RAD = 2.2

GRIPPER_DRL_BASE_CODE = textwrap.dedent("""
g_slaveid = 0
flag = 0
def modbus_set_slaveid(slaveid):
    global g_slaveid
    g_slaveid = slaveid
def modbus_fc06(address, value):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, byteorder='big')
    data += (6).to_bytes(1, byteorder='big')
    data += (address).to_bytes(2, byteorder='big')
    data += (value).to_bytes(2, byteorder='big')
    return modbus_send_make(data)
def modbus_fc16(startaddress, cnt, valuelist):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, byteorder='big')
    data += (16).to_bytes(1, byteorder='big')
    data += (startaddress).to_bytes(2, byteorder='big')
    data += (cnt).to_bytes(2, byteorder='big')
    data += (2 * cnt).to_bytes(1, byteorder='big')
    for i in range(0, cnt):
        data += (valuelist[i]).to_bytes(2, byteorder='big')
    return modbus_send_make(data)
def recv_check():
    size, val = flange_serial_read(0.1)
    if size > 0:
        return True, val
    else:
        tp_log("CRC Check Fail")
        return False, val
def gripper_move(stroke):
    flange_serial_write(modbus_fc16(282, 2, [stroke, 0]))
    wait(1.0)

while True:
    flange_serial_open(
        baudrate=57600,
        bytesize=DR_EIGHTBITS,
        parity=DR_PARITY_NONE,
        stopbits=DR_STOPBITS_ONE,
    )

    modbus_set_slaveid(1)

    flange_serial_write(modbus_fc06(256, 1))
    flag, val = recv_check()

    flange_serial_write(modbus_fc06(275, 400))
    flag, val = recv_check()

    if flag is True:
        break

    flange_serial_close()
""").strip()

GRIPPER_INITIALIZE_SNIPPET = textwrap.dedent("""
flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
modbus_set_slaveid(1)
flange_serial_write(modbus_fc06(256, 1))
recv_check()
flange_serial_write(modbus_fc06(275, 400))
recv_check()
""").strip()


def _gripper_close_fraction(jaw_rad: float) -> float:
    """SO Jaw open(0)→Doosan close(1), SO Jaw closed(2.2)→Doosan open(0)."""
    jaw_clamped = max(SO_JAW_OPEN_RAD, min(SO_JAW_CLOSED_RAD, jaw_rad))
    return 1.0 - (jaw_clamped / SO_JAW_CLOSED_RAD)


def jaw_rad_to_rh_p12_positions(jaw_rad: float) -> List[float]:
    """SO Jaw → RH-P12-RN joint positions (rad), RViz mimic (direction matched to leader)."""
    main = _gripper_close_fraction(jaw_rad) * 1.1
    finger = min(1.0, main * (1.0 / 1.1))
    return [main, finger, main, finger]


def jaw_rad_to_position_pulse(
    jaw_rad: float,
    position_max: int = GRIPPER_POSITION_MAX_TCP,
    quantize_step: int = 1,
) -> int:
    """SO Jaw → RH-P12 pulse: 0=open, position_max=closed (TCP bridge / SetPosition)."""
    fraction = _gripper_close_fraction(jaw_rad)
    raw = fraction * float(position_max)
    if quantize_step > 1:
        raw = round(raw / quantize_step) * quantize_step
    return int(max(0, min(position_max, raw)))


def jaw_rad_to_stroke(jaw_rad: float, stroke_max: int = GRIPPER_STROKE_MAX_LEGACY) -> int:
    """Legacy DRL Modbus stroke (same open/close sense as TCP position)."""
    return jaw_rad_to_position_pulse(jaw_rad, position_max=stroke_max, quantize_step=100)


class VirtualRhP12Gripper:
    """Command mock ros2_control gripper via joint_trajectory_controller."""

    def __init__(self, node: Node, robot_id: str):
        self._robot_id = robot_id
        self._node = node
        self._logger = node.get_logger()
        action_name = f"/{robot_id}/rh_p12_gripper_controller/follow_joint_trajectory"
        self._client = ActionClient(node, FollowJointTrajectory, action_name)
        self._ready = False
        self._pending = False

    def ensure_ready(self) -> bool:
        if self._ready:
            return True
        if not self._client.wait_for_server(timeout_sec=0.0):
            if not self._client.wait_for_server(timeout_sec=2.0):
                self._logger.warn(
                    "rh_p12_gripper_controller action not available. "
                    "After RViz launch run: "
                    f"ros2 run controller_manager spawner rh_p12_gripper_controller "
                    f"-c /{self._robot_id}/controller_manager",
                    throttle_duration_sec=5.0,
                )
                return False
        self._ready = True
        self._logger.info("Virtual RH-P12 gripper action server connected.")
        return True

    def move_positions(self, positions: List[float]) -> None:
        if self._pending or not self.ensure_ready():
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(GRIPPER_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 80_000_000  # 80 ms
        goal.trajectory.points = [point]

        self._pending = True
        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        self._pending = False
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self._logger.warn(
                    "Virtual gripper trajectory rejected.",
                    throttle_duration_sec=2.0,
                )
        except Exception as exc:
            self._logger.error(
                f"Virtual gripper goal failed: {exc}",
                throttle_duration_sec=2.0,
            )


class RealRhP12Gripper:
    """Real RH-P12-RN on flange serial (DrlStart + Modbus), same as move_web."""

    def __init__(self, node: Node, robot_id: str, robot_system: int = 0):
        self._node = node
        self._logger = node.get_logger()
        self._robot_system = robot_system
        self._client = node.create_client(DrlStart, f"/{robot_id}/drl/drl_start")
        self._initialized = False
        self._busy = False

    def ensure_ready(self) -> bool:
        if not self._client.wait_for_service(timeout_sec=0.0):
            if not self._client.wait_for_service(timeout_sec=2.0):
                self._logger.warn(
                    "drl_start not available for real gripper.",
                    throttle_duration_sec=5.0,
                )
                return False
        return True

    def initialize(self) -> None:
        if self._initialized or self._busy or not self.ensure_ready():
            return
        script = f"{GRIPPER_DRL_BASE_CODE}\n{GRIPPER_INITIALIZE_SNIPPET}"
        self._logger.info("Initializing RH-P12 gripper (DRL/Modbus)...")
        self._send_script(script, on_success=self._mark_initialized)

    def move_stroke(self, stroke: int) -> None:
        if self._busy or not self.ensure_ready():
            return
        stroke_int = int(stroke)
        if stroke_int < GRIPPER_STROKE_MIN or stroke_int > GRIPPER_STROKE_MAX_LEGACY:
            return
        if not self._initialized:
            self.initialize()

        move_snippet = f"gripper_move({stroke_int})"
        script = f"{GRIPPER_DRL_BASE_CODE}\n{move_snippet}"
        self._send_script(script)

    def _send_script(self, code: str, on_success=None):
        req = DrlStart.Request()
        req.robot_system = self._robot_system
        req.code = code
        self._busy = True
        future = self._client.call_async(req)
        future.add_done_callback(
            lambda f, cb=on_success: self._on_drl_done(f, cb)
        )

    def _mark_initialized(self):
        self._initialized = True

    def _on_drl_done(self, future, on_success):
        self._busy = False
        try:
            result = future.result()
            if result is None or not result.success:
                self._logger.warn(
                    "Gripper DRL script failed.",
                    throttle_duration_sec=2.0,
                )
                return
            if on_success:
                on_success()
        except Exception as exc:
            self._logger.error(
                f"Gripper DrlStart exception: {exc}",
                throttle_duration_sec=2.0,
            )


class TcpRhP12Gripper:
    """Real gripper via Dakae dsr_gripper_tcp (gripper_service_node /set_position)."""

    def __init__(
        self,
        node: Node,
        gripper_service_ns: str = "/gripper_service",
        move_timeout_sec: float = 0.0,
    ):
        self._node = node
        self._logger = node.get_logger()
        self._service_ns = gripper_service_ns.rstrip("/")
        self._move_timeout_sec = float(move_timeout_sec)
        self._client = None
        self._busy = False

        try:
            from dsr_gripper_tcp_interfaces.srv import SetPosition
        except ImportError as exc:
            raise ImportError(
                "gripper_mode:=tcp requires dsr_gripper_tcp_interfaces. "
                "Clone https://github.com/Dakae/Doosan-E0509-ROBOTIS-RH-P12-RN-TCP-Bridge "
                "into ~/doosan_ws/src and: "
                "colcon build --packages-select dsr_gripper_tcp_interfaces dsr_gripper_tcp"
            ) from exc

        self._SetPosition = SetPosition
        svc = f"{self._service_ns}/set_position"
        self._client = node.create_client(SetPosition, svc)

    def ensure_ready(self) -> bool:
        if self._client is None:
            return False
        if self._client.service_is_ready():
            return True
        if not self._client.wait_for_service(timeout_sec=2.0):
            self._logger.warn(
                f"{self._service_ns}/set_position not available. "
                "Start: ros2 launch dsr_gripper_tcp gripper_service_node.launch.py "
                "controller_host:=<로봇IP>",
                throttle_duration_sec=5.0,
            )
            return False
        self._logger.info(f"TCP gripper service connected ({self._service_ns}).")
        return True

    def move_position(self, position: int) -> None:
        if self._busy or not self.ensure_ready():
            return

        req = self._SetPosition.Request()
        req.position = int(position)
        req.timeout_sec = float(self._move_timeout_sec)

        self._busy = True
        future = self._client.call_async(req)
        future.add_done_callback(self._on_set_position_done)

    def _on_set_position_done(self, future):
        self._busy = False
        try:
            result = future.result()
            if result is None or not result.success:
                msg = getattr(result, "message", "") if result else ""
                self._logger.warn(
                    f"TCP set_position failed: {msg}",
                    throttle_duration_sec=2.0,
                )
        except Exception as exc:
            self._logger.error(
                f"TCP set_position exception: {exc}",
                throttle_duration_sec=2.0,
            )
