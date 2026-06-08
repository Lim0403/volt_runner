#!/usr/bin/env python3

import math
import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import omni.kit.app
import omni.timeline
from omni.isaac.core.utils.prims import get_prim_at_path


# Isaac joint paths
JOINT_PATHS: Dict[str, str] = {
    "fl": "/World/volt_runner/joints/front_left_wheel_joint",
    "fr": "/World/volt_runner/joints/front_right_wheel_joint",
    "rl": "/World/volt_runner/joints/rear_left_wheel_joint",
    "rr": "/World/volt_runner/joints/rear_right_wheel_joint",
}

# Based on your earlier test:
# ++++ tended to go backward, so start with a global sign flip.
SIGNS: Dict[str, float] = {
    "fl": -1.0,
    "fr": -1.0,
    "rl": -1.0,
    "rr": -1.0,
}

# Joint drive defaults
JOINT_STIFFNESS = 0.0
JOINT_DAMPING = 5000.0
JOINT_MAX_FORCE = 10000.0


def rpm_to_rad_s(rpm: float) -> float:
    return rpm * 2.0 * math.pi / 60.0


def rad_s_to_rpm(rad_s: float) -> float:
    return rad_s * 60.0 / (2.0 * math.pi)


class IsaacWheelBridgeNode(Node):
    """
    ROS node that:
      - subscribes /wheel_target_rpm as [FL, FR, RL, RR]
      - stores latest wheel command
      - publishes /wheel_feedback_rpm as [FL, FR, RR, RL]
        so your existing fk_node can consume it unchanged
    """

    def __init__(self):
        super().__init__('isaac_wheel_bridge_node')

        self.latest_rpm_cmd: Optional[List[float]] = None
        self.latest_feedback_rpm: Optional[List[float]] = None

        self.sub = self.create_subscription(
            Float32MultiArray,
            '/wheel_target_rpm',
            self.cmd_callback,
            10
        )

        self.pub_feedback = self.create_publisher(
            Float32MultiArray,
            '/wheel_feedback_rpm',
            10
        )

        self.timer_feedback = self.create_timer(0.1, self.feedback_timer_callback)

        self.get_logger().info('isaac_wheel_bridge_node started')

    def cmd_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) != 4:
            self.get_logger().warn(f'/wheel_target_rpm length != 4: {data}')
            return

        # input order: [FL, FR, RL, RR]
        self.latest_rpm_cmd = data

    def feedback_timer_callback(self):
        if self.latest_feedback_rpm is None:
            return

        fl, fr, rl, rr = self.latest_feedback_rpm

        # IMPORTANT:
        # your fk_node expects raw order [FL, FR, RR, RL]
        fb = Float32MultiArray()
        fb.data = [fl, fr, rr, rl]
        self.pub_feedback.publish(fb)

    def destroy_node(self):
        self.get_logger().info('isaac_wheel_bridge_node stopping')
        super().destroy_node()


class IsaacWheelBridgeRunner:
    """
    Runs inside Isaac Sim.
    - spins ROS in background thread
    - on each Isaac update, applies latest wheel target to wheel joints
    - reads back joint velocity if available, otherwise echoes command
    """

    def __init__(self):
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
        except Exception:
            # harmless if already initialized in current Isaac session
            pass

        self.node = IsaacWheelBridgeNode()
        self._running = True

        self._ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self._ros_thread.start()

        self._update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._on_update,
            name="volt_runner_isaac_wheel_bridge_update"
        )

        print("[bridge] started")

    def _spin_ros(self):
        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _set_joint_velocity(self, joint_path: str, vel_rad_s: float):
        prim = get_prim_at_path(joint_path)
        if not prim.IsValid():
            print(f"[bridge][WARN] invalid prim: {joint_path}")
            return

        attr_vel = prim.GetAttribute("drive:angular:physics:targetVelocity")
        if attr_vel.IsValid():
            attr_vel.Set(float(vel_rad_s))
        else:
            print(f"[bridge][WARN] targetVelocity attr missing: {joint_path}")

        # velocity control tuning
        attr_stiff = prim.GetAttribute("drive:angular:physics:stiffness")
        if attr_stiff.IsValid():
            attr_stiff.Set(float(JOINT_STIFFNESS))

        attr_damp = prim.GetAttribute("drive:angular:physics:damping")
        if attr_damp.IsValid():
            attr_damp.Set(float(JOINT_DAMPING))

        attr_force = prim.GetAttribute("drive:angular:physics:maxForce")
        if attr_force.IsValid():
            attr_force.Set(float(JOINT_MAX_FORCE))

    def _read_joint_velocity_rpm(self, joint_path: str, sign: float, fallback_rpm: float) -> float:
        """
        Try to read actual joint velocity from USD.
        If not available, fall back to commanded rpm.
        """
        prim = get_prim_at_path(joint_path)
        if not prim.IsValid():
            return fallback_rpm

        # Try state velocity first
        state_attr = prim.GetAttribute("state:angular:physics:velocity")
        if state_attr.IsValid():
            val = state_attr.Get()
            if val is not None:
                return sign * rad_s_to_rpm(float(val))

        # Fall back to target velocity
        target_attr = prim.GetAttribute("drive:angular:physics:targetVelocity")
        if target_attr.IsValid():
            val = target_attr.Get()
            if val is not None:
                return sign * rad_s_to_rpm(float(val))

        return fallback_rpm

    def _on_update(self, _event):
        # apply commands only while timeline is playing
        if not omni.timeline.get_timeline_interface().is_playing():
            return

        if self.node.latest_rpm_cmd is None:
            return

        fl_rpm, fr_rpm, rl_rpm, rr_rpm = self.node.latest_rpm_cmd

        rpm_map = {
            "fl": fl_rpm,
            "fr": fr_rpm,
            "rl": rl_rpm,
            "rr": rr_rpm,
        }

        # apply to joints
        for key, joint_path in JOINT_PATHS.items():
            cmd_rad_s = SIGNS[key] * rpm_to_rad_s(rpm_map[key])
            self._set_joint_velocity(joint_path, cmd_rad_s)

        # build feedback (prefer measured, else echo command)
        feedback = []
        for key in ("fl", "fr", "rl", "rr"):
            fb_rpm = self._read_joint_velocity_rpm(
                JOINT_PATHS[key],
                SIGNS[key],
                rpm_map[key]
            )
            feedback.append(float(fb_rpm))

        self.node.latest_feedback_rpm = feedback

    def stop(self):
        self._running = False

        try:
            self._update_sub = None
        except Exception:
            pass

        try:
            self.node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        print("[bridge] stopped")


BRIDGE = None


def start_bridge():
    global BRIDGE
    try:
        if BRIDGE is not None:
            BRIDGE.stop()
    except Exception:
        pass
    BRIDGE = IsaacWheelBridgeRunner()
    return BRIDGE


def stop_bridge():
    global BRIDGE
    if BRIDGE is not None:
        BRIDGE.stop()
        BRIDGE = None