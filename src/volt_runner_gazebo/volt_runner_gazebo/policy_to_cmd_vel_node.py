#!/usr/bin/env python3

import numpy as np
import torch

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PolicyToCmdVelNode(Node):
    def __init__(self):
        super().__init__("policy_to_cmd_vel_node")

        self.declare_parameter(
            "model_path",
            "/home/lim/IsaacLab/logs/rsl_rl/exported/policy.pt",
        )

        self.declare_parameter("pt_topic", "/coil_efficiency")
        self.declare_parameter("odom_topic", "/odom")

        # RL model output
        self.declare_parameter("cmd_topic", "/cmd_vel_rl")

        # 실제 최종 적용된 command. manager가 발행하는 /cmd_vel.
        self.declare_parameter("applied_cmd_topic", "/cmd_vel")

        self.declare_parameter("control_hz", 10.0)

        # Isaac Sim 학습 action scale
        self.declare_parameter("max_vx", 0.30)
        self.declare_parameter("max_vy", 0.25)
        self.declare_parameter("max_wz", 0.45)

        # 실제 출력 안전 감속
        self.declare_parameter("output_speed_scale", 0.2)

        model_path = self.get_parameter("model_path").value
        pt_topic = self.get_parameter("pt_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value
        applied_cmd_topic = self.get_parameter("applied_cmd_topic").value

        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)
        self.output_speed_scale = float(self.get_parameter("output_speed_scale").value)

        self.latest_pt = None
        self.prev_pt = None
        self.pt_history = None

        # 실제 odom은 로그용
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_wz = 0.0

        # policy observation에 들어갈 실제 적용 command velocity
        self.applied_vx = 0.0
        self.applied_vy = 0.0
        self.applied_wz = 0.0

        # policy observation에 들어갈 previous action
        # 실제 적용 command를 action scale로 나눈 normalized action
        self.prev_action = np.zeros(3, dtype=np.float32)

        self.policy = torch.jit.load(model_path, map_location="cpu")
        self.policy.eval()

        self.sub_pt = self.create_subscription(
            Float32,
            pt_topic,
            self.pt_callback,
            10,
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )

        self.sub_applied_cmd = self.create_subscription(
            Twist,
            applied_cmd_topic,
            self.applied_cmd_callback,
            10,
        )

        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.pub_obs_debug = self.create_publisher(Float32MultiArray, "/rl_obs", 10)
        self.pub_raw_action_debug = self.create_publisher(Float32MultiArray, "/rl_raw_action", 10)

        period = 1.0 / float(self.get_parameter("control_hz").value)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(f"Loaded policy: {model_path}")
        self.get_logger().info(f"Subscribe Pt: {pt_topic}")
        self.get_logger().info(f"Subscribe odom: {odom_topic}")
        self.get_logger().info(f"Subscribe applied cmd: {applied_cmd_topic}")
        self.get_logger().info(f"Publish RL cmd: {cmd_topic}")
        self.get_logger().info(
            f"action_scale=({self.max_vx}, {self.max_vy}, {self.max_wz}), "
            f"output_speed_scale={self.output_speed_scale}"
        )

    def pt_callback(self, msg):
        pt = float(msg.data)

        if self.latest_pt is None:
            self.latest_pt = pt
            self.prev_pt = pt
            self.pt_history = np.array([pt, pt, pt, pt, pt], dtype=np.float32)
            return

        self.prev_pt = self.latest_pt
        self.latest_pt = pt

        self.pt_history[:-1] = self.pt_history[1:]
        self.pt_history[-1] = pt

    def odom_callback(self, msg):
        self.odom_vx = float(msg.twist.twist.linear.x)
        self.odom_vy = float(msg.twist.twist.linear.y)
        self.odom_wz = float(msg.twist.twist.angular.z)

    def applied_cmd_callback(self, msg):
        self.applied_vx = float(msg.linear.x)
        self.applied_vy = float(msg.linear.y)
        self.applied_wz = float(msg.angular.z)

        # 실제 적용된 cmd_vel을 학습 action scale 기준으로 normalized action으로 변환
        ax = self.applied_vx / self.max_vx if self.max_vx > 1e-6 else 0.0
        ay = self.applied_vy / self.max_vy if self.max_vy > 1e-6 else 0.0
        aw = self.applied_wz / self.max_wz if self.max_wz > 1e-6 else 0.0

        self.prev_action[:] = np.array([
            np.clip(ax, -1.0, 1.0),
            np.clip(ay, -1.0, 1.0),
            np.clip(aw, -1.0, 1.0),
        ], dtype=np.float32)

    def control_loop(self):
        if self.latest_pt is None or self.pt_history is None:
            return

        pt = float(self.latest_pt)
        delta_pt = float(self.latest_pt - self.prev_pt)

        obs_np = np.array([
            pt,
            delta_pt,
            self.pt_history[0],
            self.pt_history[1],
            self.pt_history[2],
            self.pt_history[3],
            self.pt_history[4],
            self.applied_vx,
            self.applied_vy,
            self.applied_wz,
            self.prev_action[0],
            self.prev_action[1],
            self.prev_action[2],
        ], dtype=np.float32)

        obs_debug = Float32MultiArray()
        obs_debug.data = obs_np.tolist()
        self.pub_obs_debug.publish(obs_debug)

        obs = torch.tensor(obs_np, dtype=torch.float32).unsqueeze(0)

        with torch.no_grad():
            action = self.policy(obs)

        if isinstance(action, (tuple, list)):
            action = action[0]

        action = action.detach().cpu().numpy().reshape(-1)
        action = np.nan_to_num(action, nan=0.0, posinf=0.0, neginf=0.0)

        raw_action_debug = Float32MultiArray()
        raw_action_debug.data = action.astype(np.float32).tolist()
        self.pub_raw_action_debug.publish(raw_action_debug)

        if action.shape[0] != 3:
            self.get_logger().warn(
                f"Unsupported policy output dim: {action.shape[0]}, expected 3"
            )
            return

        ax_raw = float(action[0])
        ay_raw = float(action[1])
        aw_raw = float(action[2])

        ax = float(np.clip(ax_raw, -1.0, 1.0))
        ay = float(np.clip(ay_raw, -1.0, 1.0))
        aw = float(np.clip(aw_raw, -1.0, 1.0))

        vx_policy = ax * self.max_vx
        vy_policy = ay * self.max_vy
        wz_policy = aw * self.max_wz

        vx_cmd = vx_policy * self.output_speed_scale
        vy_cmd = -vy_policy * self.output_speed_scale  # invert y-axis for ROS/virtual world
        wz_cmd = wz_policy * self.output_speed_scale

        cmd = Twist()
        cmd.linear.x = float(vx_cmd)
        cmd.linear.y = float(vy_cmd)
        cmd.angular.z = float(wz_cmd)
        self.pub_cmd.publish(cmd)

        self.get_logger().info(
            f"pt={pt:.3f}, dpt={delta_pt:.4f}, "
            f"raw=({ax_raw:.3f},{ay_raw:.3f},{aw_raw:.3f}), "
            f"clip=({ax:.3f},{ay:.3f},{aw:.3f}), "
            f"applied_vel=({self.applied_vx:.3f},{self.applied_vy:.3f},{self.applied_wz:.3f}), "
            f"prev_action=({self.prev_action[0]:.3f},{self.prev_action[1]:.3f},{self.prev_action[2]:.3f}), "
            f"cmd=({vx_cmd:.4f},{vy_cmd:.4f},{wz_cmd:.4f}), "
            f"odom=({self.odom_vx:.4f},{self.odom_vy:.4f},{self.odom_wz:.4f}), "
            f"speed_scale={self.output_speed_scale:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PolicyToCmdVelNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
