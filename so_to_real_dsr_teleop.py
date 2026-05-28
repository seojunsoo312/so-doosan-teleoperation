#!/usr/bin/env python3
"""SO /joint_states → Doosan move_joint (virtual or real) + optional RH-P12 gripper."""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from dsr_msgs2.srv import MoveJoint

from doosan_rh_p12_gripper import (
    GRIPPER_POSITION_MAX_TCP,
    GRIPPER_STROKE_MAX_LEGACY,
    RealRhP12Gripper,
    TcpRhP12Gripper,
    VirtualRhP12Gripper,
    jaw_rad_to_position_pulse,
    jaw_rad_to_rh_p12_positions,
    jaw_rad_to_stroke,
)


class SoToRealDsrTeleop(Node):
    def __init__(self):
        super().__init__("so_to_real_dsr_teleop")

        self.robot_id = self.declare_parameter("robot_id", "dsr01").value
        self.input_joint_topic = self.declare_parameter("input_joint_topic", "/joint_states").value

        # none | virtual (RViz) | real (legacy DRL Modbus) | tcp (dsr_gripper_tcp, 실기 권장)
        self.gripper_mode = str(
            self.declare_parameter("gripper_mode", "none").value
        ).lower()
        if self.declare_parameter("use_gripper", False).value:
            if self.gripper_mode == "none":
                self.gripper_mode = "tcp"
            self.get_logger().warn(
                "use_gripper is deprecated; use gripper_mode:=virtual|tcp|real"
            )

        # --- latency tuning (ROS params) ---
        self.command_period = self.declare_parameter("command_period", 0.05).value
        self.stale_timeout = self.declare_parameter("stale_timeout", 0.5).value
        self.joint_tolerance_deg = self.declare_parameter("joint_tolerance_deg", 0.03).value
        self.leader_sample_tolerance_deg = self.declare_parameter(
            "leader_sample_tolerance_deg", 0.02
        ).value
        self.move_vel = float(self.declare_parameter("move_vel", 40.0).value)
        self.move_acc = float(self.declare_parameter("move_acc", 40.0).value)
        self.move_radius = float(self.declare_parameter("move_radius", 30.0).value)
        self.sync_type = int(self.declare_parameter("sync_type", 1).value)
        self.blend_type = int(self.declare_parameter("blend_type", 1).value)

        self.gripper_command_period = float(
            self.declare_parameter("gripper_command_period", 0.08).value
        )
        self.gripper_tolerance_rad = float(
            self.declare_parameter("gripper_tolerance_rad", 0.015).value
        )
        self.gripper_stroke_tolerance = int(
            self.declare_parameter("gripper_stroke_tolerance", 50).value
        )
        self.gripper_position_max = int(
            self.declare_parameter("gripper_position_max", GRIPPER_POSITION_MAX_TCP).value
        )
        self.gripper_position_tolerance = int(
            self.declare_parameter("gripper_position_tolerance", 30).value
        )
        self.gripper_service_ns = str(
            self.declare_parameter("gripper_service_ns", "/gripper_service").value
        ).rstrip("/")
        self.gripper_move_timeout_sec = float(
            self.declare_parameter("gripper_move_timeout_sec", 0.0).value
        )
        self.gripper_auto_init = bool(
            self.declare_parameter("gripper_auto_init", True).value
        )

        # SO leader → Doosan arm mapping (see _apply_arm_mapping_preset)
        self.mapping_preset = str(
            self.declare_parameter("mapping_preset", "neutral").value
        ).lower()
        self.rot_sign = float(self.declare_parameter("rot_sign", -1.0).value)
        self.pitch_sign = float(self.declare_parameter("pitch_sign", -1.0).value)
        self.elbow_sign = float(self.declare_parameter("elbow_sign", -1.0).value)
        self.wrist_pitch_sign = float(
            self.declare_parameter("wrist_pitch_sign", -1.0).value
        )
        self.wrist_roll_sign = float(
            self.declare_parameter("wrist_roll_sign", 1.0).value
        )
        self.rot_offset_rad = float(
            self.declare_parameter("rot_offset_rad", 0.0).value
        )
        self.elbow_offset_rad = float(
            self.declare_parameter("elbow_offset_rad", 0.0).value
        )
        self._apply_arm_mapping_preset()

        self._state_lock = threading.Lock()
        self._latest_joints_deg = None
        self._latest_jaw_rad = None
        self._latest_stamp = 0.0
        self._last_sent_joints = None
        self._last_move_time = 0.0
        self._prev_leader_joints_deg = None
        self._prev_leader_jaw_rad = None
        self._leader_dirty = False
        self._gripper_dirty = False
        self._last_sent_gripper_main_rad = None
        self._last_sent_gripper_stroke = None
        self._last_sent_gripper_position = None
        self._last_gripper_command_time = 0.0
        self._pending_moves = 0
        self._max_pending_moves = int(self.declare_parameter("max_pending_moves", 2).value)

        self._gripper = None
        if self.gripper_mode == "virtual":
            self._gripper = VirtualRhP12Gripper(self, self.robot_id)
        elif self.gripper_mode == "real":
            self._gripper = RealRhP12Gripper(self, self.robot_id, 0)
            if self.gripper_auto_init:
                self._gripper.initialize()
        elif self.gripper_mode == "tcp":
            try:
                self._gripper = TcpRhP12Gripper(
                    self,
                    gripper_service_ns=self.gripper_service_ns,
                    move_timeout_sec=self.gripper_move_timeout_sec,
                )
            except ImportError as exc:
                self.get_logger().error(str(exc))
                raise
        elif self.gripper_mode != "none":
            raise ValueError(
                f"Unknown gripper_mode={self.gripper_mode!r} "
                "(use none, virtual, tcp, real)"
            )

        self._command_timer = self.create_timer(self.command_period, self._command_timer_cb)

        self.client = self.create_client(MoveJoint, f"/{self.robot_id}/motion/move_joint")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for move_joint service...")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            JointState, self.input_joint_topic, self.cb, qos
        )

        self.get_logger().info(
            f"SO → Doosan teleop | period={self.command_period}s "
            f"tolerance={self.joint_tolerance_deg}° vel={self.move_vel}% "
            f"sync={'ASYNC' if self.sync_type else 'SYNC'} "
            f"gripper={self.gripper_mode} "
            f"mapping={self.mapping_preset}"
        )
        self.get_logger().info(
            f"  arm signs rot,pitch,elbow,w_pitch,w_roll="
            f"{self.rot_sign},{self.pitch_sign},{self.elbow_sign},"
            f"{self.wrist_pitch_sign},{self.wrist_roll_sign} "
            f"offsets rot,elbow(rad)={self.rot_offset_rad:.4f},{self.elbow_offset_rad:.4f}"
        )

    def _apply_arm_mapping_preset(self) -> None:
        """neutral: bluephysi01 부호만 (오프셋 0, 리더 0에서 90° 안 남). bluephysi01: -π/2 on rot/elbow."""
        if self.mapping_preset == "bluephysi01":
            self.rot_sign = -1.0
            self.pitch_sign = -1.0
            self.elbow_sign = -1.0
            self.wrist_pitch_sign = -1.0
            self.wrist_roll_sign = 1.0
            self.rot_offset_rad = -math.pi / 2
            self.elbow_offset_rad = -math.pi / 2
        elif self.mapping_preset not in ("", "neutral", "default"):
            raise ValueError(
                f"Unknown mapping_preset={self.mapping_preset!r} "
                "(use neutral or bluephysi01)"
            )

    def _leader_pos_to_doosan_arm_rad(self, pos: dict) -> list:
        """SO JointState dict → Doosan joint_1..6 rad (joint_4 fixed 0). bluephysi01 동일 부호."""
        rot = self.rot_sign * pos["Rotation"]
        pitch = self.pitch_sign * pos["Pitch"]
        elbow = self.elbow_sign * pos["Elbow"]
        w_pitch = self.wrist_pitch_sign * pos["Wrist_Pitch"]
        w_roll = self.wrist_roll_sign * pos["Wrist_Roll"]
        rot += self.rot_offset_rad
        elbow += self.elbow_offset_rad
        return [rot, pitch, elbow, 0.0, w_pitch, w_roll]

    def cb(self, msg):
        pos = {n: p for n, p in zip(msg.name, msg.position)}
        jaw = pos["Jaw"]

        joints_deg = [
            math.degrees(v) for v in self._leader_pos_to_doosan_arm_rad(pos)
        ]

        now = time.time()
        with self._state_lock:
            if self._prev_leader_joints_deg is None or self._joints_changed(
                joints_deg,
                self._prev_leader_joints_deg,
                self.leader_sample_tolerance_deg,
            ):
                self._leader_dirty = True
            self._prev_leader_joints_deg = list(joints_deg)

            if self._gripper is not None:
                if self._prev_leader_jaw_rad is None or abs(jaw - self._prev_leader_jaw_rad) >= (
                    self.gripper_tolerance_rad * 0.5
                ):
                    self._gripper_dirty = True
                self._prev_leader_jaw_rad = jaw
                self._latest_jaw_rad = jaw

            self._latest_joints_deg = joints_deg
            self._latest_stamp = now

    def _command_timer_cb(self):
        now = time.time()
        with self._state_lock:
            joints = None if self._latest_joints_deg is None else list(self._latest_joints_deg)
            jaw_rad = self._latest_jaw_rad
            stamp = self._latest_stamp
            gripper_dirty = self._gripper_dirty

        if (now - stamp) > self.stale_timeout:
            return

        if joints:
            should_send = self._leader_dirty or (
                not self._last_sent_joints
            ) or self._joints_changed(joints, self._last_sent_joints, self.joint_tolerance_deg)
            if should_send and (now - self._last_move_time) >= self.command_period:
                if self._pending_moves < self._max_pending_moves:
                    self._last_move_time = now
                    self._last_sent_joints = list(joints)
                    with self._state_lock:
                        self._leader_dirty = False
                    self.movej(joints)

        if self._gripper is not None and jaw_rad is not None and gripper_dirty:
            if (now - self._last_gripper_command_time) >= self.gripper_command_period:
                self._maybe_move_gripper(jaw_rad)
                self._last_gripper_command_time = now
                with self._state_lock:
                    self._gripper_dirty = False

    def _joints_changed(self, new_joints, prev_joints, tolerance_deg):
        if prev_joints is None:
            return True
        for prev, curr in zip(prev_joints, new_joints):
            if abs(curr - prev) > tolerance_deg:
                return True
        return False

    def _maybe_move_gripper(self, jaw_rad: float):
        if self.gripper_mode == "virtual":
            positions = jaw_rad_to_rh_p12_positions(jaw_rad)
            main = positions[0]
            if (
                self._last_sent_gripper_main_rad is not None
                and abs(main - self._last_sent_gripper_main_rad) < self.gripper_tolerance_rad
            ):
                return
            self._gripper.move_positions(positions)
            self._last_sent_gripper_main_rad = main
        elif self.gripper_mode == "tcp":
            position = jaw_rad_to_position_pulse(
                jaw_rad,
                position_max=self.gripper_position_max,
                quantize_step=1,
            )
            if (
                self._last_sent_gripper_position is not None
                and abs(position - self._last_sent_gripper_position)
                < self.gripper_position_tolerance
            ):
                return
            self._gripper.move_position(position)
            self._last_sent_gripper_position = position
        elif self.gripper_mode == "real":
            stroke = jaw_rad_to_stroke(
                jaw_rad, stroke_max=GRIPPER_STROKE_MAX_LEGACY
            )
            if (
                self._last_sent_gripper_stroke is not None
                and abs(stroke - self._last_sent_gripper_stroke) < self.gripper_stroke_tolerance
            ):
                return
            self._gripper.move_stroke(stroke)
            self._last_sent_gripper_stroke = stroke

    def movej(self, joint_values):
        req = MoveJoint.Request()
        req.pos = [float(v) for v in joint_values]
        req.vel = self.move_vel
        req.acc = self.move_acc
        req.time = 0.0
        req.radius = self.move_radius
        req.mode = 0
        req.blend_type = self.blend_type
        req.sync_type = self.sync_type

        self._pending_moves += 1
        future = self.client.call_async(req)
        future.add_done_callback(self._on_movej_done)

    def _on_movej_done(self, future):
        self._pending_moves = max(0, self._pending_moves - 1)
        try:
            result = future.result()
            if result is None or not result.success:
                self.get_logger().warn("MoveJoint request failed.", throttle_duration_sec=2.0)
        except Exception as exc:
            self.get_logger().error(f"MoveJoint exception: {exc}", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = SoToRealDsrTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
