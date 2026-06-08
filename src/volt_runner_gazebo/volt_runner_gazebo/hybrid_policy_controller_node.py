#!/usr/bin/env python3

import numpy as np
import torch

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String, Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class HybridPolicyControllerNode(Node):
    def __init__(self):
        super().__init__("hybrid_policy_controller_node")

        self.declare_parameter(
            "model_path",
            "/home/lim/IsaacLab/logs/rsl_rl/exported/slow.pt",
        )

        self.declare_parameter("pt_topic", "/coil_efficiency")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("mode_topic", "/align_mode")

        # 학습 때 env step = 0.1s
        self.declare_parameter("control_hz", 10.0)

        # Isaac Sim / slow policy action scale
        self.declare_parameter("action_scale_vx", 0.06)
        self.declare_parameter("action_scale_vy", 0.05)
        self.declare_parameter("action_scale_wz", 0.05)

        # 실제 출력 감속 비율
        self.declare_parameter("output_speed_scale", 1.0)

        # ------------------------------------------------------------------
        # Odom-distance based raster search
        # ------------------------------------------------------------------
        # 기존 시간 기반 raster:
        #   lane_time 동안 y 방향 이동
        #   forward_time 동안 x 방향 이동
        #
        # 문제:
        #   바닥 마찰, 모터 응답, 시작 타이밍에 따라 실제 탐색 거리가 달라짐.
        #
        # 새 거리 기반 raster:
        #   odom 기준으로 차량 하부 영역을 직접 sweep.
        #   시작 위치를 (0,0)으로 보고,
        #   y=+0.5, y=-0.5 사이를 왕복하면서 x 방향으로 전진.
        #
        # 시각 센서 없이도 odom만 있으면 가능.
        self.declare_parameter("search_vx", 0.06)
        self.declare_parameter("search_vy", 0.05)

        # 차량 하부 탐색 폭: 중앙 기준 좌우 0.5m
        self.declare_parameter("raster_y_min", -0.50)
        self.declare_parameter("raster_y_max", 0.50)

        # 한 줄 sweep 후 앞으로 전진하는 거리
        self.declare_parameter("raster_x_step", 0.10)

        # 최대 전진 탐색 거리
        self.declare_parameter("raster_x_max", 0.40)

        # 목표 위치 도달 판정 허용 오차
        self.declare_parameter("raster_pos_tol", 0.02)

        # 처음에는 중앙에서 오른쪽(+y) 0.5m로 이동
        self.declare_parameter("raster_start_direction", 1.0)

        # Mode 기준
        self.declare_parameter("pt_switch_on", 0.15)
        self.declare_parameter("pt_success", 0.87)
        self.declare_parameter("success_hold_steps", 3)
        self.declare_parameter("lock_rl_once_detected", True)

        model_path = self.get_parameter("model_path").value
        pt_topic = self.get_parameter("pt_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value
        mode_topic = self.get_parameter("mode_topic").value

        self.control_hz = float(self.get_parameter("control_hz").value)

        self.action_scale_vx = float(self.get_parameter("action_scale_vx").value)
        self.action_scale_vy = float(self.get_parameter("action_scale_vy").value)
        self.action_scale_wz = float(self.get_parameter("action_scale_wz").value)
        self.output_speed_scale = float(self.get_parameter("output_speed_scale").value)

        self.search_vx = float(self.get_parameter("search_vx").value)
        self.search_vy = float(self.get_parameter("search_vy").value)

        self.raster_y_min = float(self.get_parameter("raster_y_min").value)
        self.raster_y_max = float(self.get_parameter("raster_y_max").value)
        self.raster_x_step = float(self.get_parameter("raster_x_step").value)
        self.raster_x_max = float(self.get_parameter("raster_x_max").value)
        self.raster_pos_tol = float(self.get_parameter("raster_pos_tol").value)
        self.raster_start_direction = float(self.get_parameter("raster_start_direction").value)

        self.pt_switch_on = float(self.get_parameter("pt_switch_on").value)
        self.pt_success = float(self.get_parameter("pt_success").value)
        self.success_hold_steps = int(self.get_parameter("success_hold_steps").value)
        self.lock_rl_once_detected = bool(self.get_parameter("lock_rl_once_detected").value)

        # Pt 상태
        self.latest_pt = None
        self.prev_pt = None
        self.delta_pt = 0.0
        self.pt_history = None

        # odom pose/velocity
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_wz = 0.0

        # 시작 odom을 기준점으로 삼기 위한 offset
        self.have_odom_origin = False
        self.odom_origin_x = 0.0
        self.odom_origin_y = 0.0

        # policy observation에 들어갈 값
        self.obs_vx = 0.0
        self.obs_vy = 0.0
        self.obs_wz = 0.0
        self.prev_action = np.zeros(3, dtype=np.float32)

        # mode
        self.mode = "RASTER"
        self.rl_locked = False
        self.success_count = 0

        # 거리 기반 raster 상태
        self.raster_started = False
        self.raster_phase = "sweep_y"  # sweep_y or step_x
        self.raster_direction = 1.0 if self.raster_start_direction >= 0.0 else -1.0
        self.raster_target_y = self.raster_y_max if self.raster_direction > 0.0 else self.raster_y_min
        self.raster_target_x = 0.0

        # model
        self.policy = torch.jit.load(model_path, map_location="cpu")
        self.policy.eval()

        # ROS I/O
        self.sub_pt = self.create_subscription(Float32, pt_topic, self.pt_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.pub_mode = self.create_publisher(String, mode_topic, 10)

        self.pub_rl_debug = self.create_publisher(Twist, "/cmd_vel_rl_debug", 10)
        self.pub_raster_debug = self.create_publisher(Twist, "/cmd_vel_raster_debug", 10)
        self.pub_obs_debug = self.create_publisher(Float32MultiArray, "/rl_obs", 10)
        self.pub_action_debug = self.create_publisher(Float32MultiArray, "/rl_raw_action", 10)

        period = 1.0 / self.control_hz
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info("Hybrid policy controller started")
        self.get_logger().info(f"Loaded policy: {model_path}")
        self.get_logger().info(
            f"action_scale=({self.action_scale_vx}, {self.action_scale_vy}, {self.action_scale_wz}), "
            f"output_speed_scale={self.output_speed_scale}"
        )
        self.get_logger().info(
            f"odom raster: y_min={self.raster_y_min}, y_max={self.raster_y_max}, "
            f"x_step={self.raster_x_step}, x_max={self.raster_x_max}, "
            f"pos_tol={self.raster_pos_tol}, start_dir={self.raster_direction}"
        )
        self.get_logger().info(
            f"mode: pt_switch_on={self.pt_switch_on}, pt_success={self.pt_success}, "
            f"success_hold_steps={self.success_hold_steps}"
        )

    def pt_callback(self, msg):
        self.latest_pt = float(msg.data)

        if self.prev_pt is None:
            self.prev_pt = self.latest_pt
            self.pt_history = np.array([self.latest_pt] * 5, dtype=np.float32)

    def odom_callback(self, msg):
        raw_x = float(msg.pose.pose.position.x)
        raw_y = float(msg.pose.pose.position.y)

        if not self.have_odom_origin:
            self.odom_origin_x = raw_x
            self.odom_origin_y = raw_y
            self.have_odom_origin = True
            self.get_logger().info(
                f"Odom origin set: x={self.odom_origin_x:.3f}, y={self.odom_origin_y:.3f}"
            )

        self.odom_x = raw_x - self.odom_origin_x
        self.odom_y = raw_y - self.odom_origin_y

        self.odom_vx = float(msg.twist.twist.linear.x)
        self.odom_vy = float(msg.twist.twist.linear.y)
        self.odom_wz = float(msg.twist.twist.angular.z)

    def compute_raster_cmd(self):
        cmd = Twist()

        if not self.have_odom_origin:
            return cmd

        if not self.raster_started:
            self.raster_started = True
            self.raster_phase = "sweep_y"
            self.raster_direction = 1.0 if self.raster_start_direction >= 0.0 else -1.0
            self.raster_target_y = self.raster_y_max if self.raster_direction > 0.0 else self.raster_y_min
            self.raster_target_x = 0.0

            self.get_logger().info(
                f"Raster started: phase={self.raster_phase}, "
                f"target_y={self.raster_target_y:.3f}, target_x={self.raster_target_x:.3f}"
            )

        if self.raster_phase == "sweep_y":
            err_y = self.raster_target_y - self.odom_y

            if abs(err_y) <= self.raster_pos_tol:
                self.raster_phase = "step_x"
                self.raster_target_x = min(self.odom_x + self.raster_x_step, self.raster_x_max)
                self.get_logger().info(
                    f"Raster switch to step_x: target_x={self.raster_target_x:.3f}, "
                    f"odom=({self.odom_x:.3f},{self.odom_y:.3f})"
                )
            else:
                cmd.linear.x = 0.0
                cmd.linear.y = self.search_vy if err_y > 0.0 else -self.search_vy
                cmd.angular.z = 0.0
                return cmd

        if self.raster_phase == "step_x":
            err_x = self.raster_target_x - self.odom_x

            if abs(err_x) <= self.raster_pos_tol or self.odom_x >= self.raster_x_max:
                self.raster_phase = "sweep_y"
                self.raster_direction *= -1.0
                self.raster_target_y = self.raster_y_max if self.raster_direction > 0.0 else self.raster_y_min
                self.get_logger().info(
                    f"Raster switch to sweep_y: target_y={self.raster_target_y:.3f}, "
                    f"odom=({self.odom_x:.3f},{self.odom_y:.3f})"
                )
            else:
                cmd.linear.x = self.search_vx if err_x > 0.0 else -self.search_vx
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                return cmd

        return cmd

    def twist_to_normalized_action(self, cmd: Twist):
        ax = cmd.linear.x / self.action_scale_vx if self.action_scale_vx > 1e-6 else 0.0
        ay = cmd.linear.y / self.action_scale_vy if self.action_scale_vy > 1e-6 else 0.0
        aw = cmd.angular.z / self.action_scale_wz if self.action_scale_wz > 1e-6 else 0.0

        return np.array([
            np.clip(ax, -1.0, 1.0),
            np.clip(ay, -1.0, 1.0),
            np.clip(aw, -1.0, 1.0),
        ], dtype=np.float32)

    def compute_rl_cmd(self):
        obs_np = np.array([
            self.latest_pt,
            self.delta_pt,
            self.pt_history[0],
            self.pt_history[1],
            self.pt_history[2],
            self.pt_history[3],
            self.pt_history[4],
            self.obs_vx,
            self.obs_vy,
            self.obs_wz,
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

        action_debug = Float32MultiArray()
        action_debug.data = action.astype(np.float32).tolist()
        self.pub_action_debug.publish(action_debug)

        if action.shape[0] != 3:
            self.get_logger().warn(f"Invalid action dim: {action.shape[0]}")
            return Twist(), np.zeros(3, dtype=np.float32)

        ax = float(np.clip(action[0], -1.0, 1.0))
        ay = float(np.clip(action[1], -1.0, 1.0))
        aw = float(np.clip(action[2], -1.0, 1.0))

        vx_policy = ax * self.action_scale_vx
        vy_policy = ay * self.action_scale_vy
        wz_policy = aw * self.action_scale_wz

        cmd = Twist()
        cmd.linear.x = vx_policy * self.output_speed_scale
        cmd.linear.y = vy_policy * self.output_speed_scale
        cmd.angular.z = wz_policy * self.output_speed_scale

        clipped_action = np.array([ax, ay, aw], dtype=np.float32)

        return cmd, clipped_action

    def update_pt_history_once_per_control_step(self):
        if self.latest_pt is None:
            return

        if self.prev_pt is None:
            self.prev_pt = self.latest_pt

        self.delta_pt = self.latest_pt - self.prev_pt

        if self.pt_history is None:
            self.pt_history = np.array([self.latest_pt] * 5, dtype=np.float32)
        else:
            self.pt_history[:-1] = self.pt_history[1:]
            self.pt_history[-1] = self.latest_pt

    def control_loop(self):
        if self.latest_pt is None:
            return

        self.update_pt_history_once_per_control_step()

        if self.latest_pt >= self.pt_success:
            self.success_count += 1
        else:
            self.success_count = 0

        # Mode decision:
        # Once Pt reaches switch threshold, lock into RL mode.
        # This preserves the previous working behavior while the raster itself
        # is now odom-distance based.
        if self.success_count >= self.success_hold_steps:
            self.mode = "STOP"
        else:
            if self.latest_pt >= self.pt_switch_on:
                self.rl_locked = True

            if self.rl_locked and self.lock_rl_once_detected:
                self.mode = "RL"
            else:
                self.mode = "RASTER"

        raster_cmd = self.compute_raster_cmd()
        rl_cmd, rl_action = self.compute_rl_cmd()

        self.pub_raster_debug.publish(raster_cmd)
        self.pub_rl_debug.publish(rl_cmd)

        if self.mode == "STOP":
            out_cmd = Twist()
            applied_action = self.twist_to_normalized_action(out_cmd)

        elif self.mode == "RASTER":
            out_cmd = raster_cmd
            applied_action = self.twist_to_normalized_action(out_cmd)

        elif self.mode == "RL":
            out_cmd = rl_cmd
            applied_action = rl_action

        else:
            out_cmd = Twist()
            applied_action = self.twist_to_normalized_action(out_cmd)

        self.pub_cmd.publish(out_cmd)

        mode_msg = String()
        mode_msg.data = self.mode
        self.pub_mode.publish(mode_msg)

        self.obs_vx = float(out_cmd.linear.x)
        self.obs_vy = float(out_cmd.linear.y)
        self.obs_wz = float(out_cmd.angular.z)
        self.prev_action[:] = applied_action

        self.get_logger().info(
            f"mode={self.mode}, "
            f"pt={self.latest_pt:.3f}, dpt={self.delta_pt:.4f}, "
            f"odom=({self.odom_x:.3f},{self.odom_y:.3f}), "
            f"raster_phase={self.raster_phase}, "
            f"cmd=({out_cmd.linear.x:.3f},{out_cmd.linear.y:.3f},{out_cmd.angular.z:.3f}), "
            f"prev_action=({self.prev_action[0]:.3f},{self.prev_action[1]:.3f},{self.prev_action[2]:.3f})"
        )

        self.prev_pt = self.latest_pt


def main(args=None):
    rclpy.init(args=args)
    node = HybridPolicyControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
