#!/usr/bin/env python3
# align_mode_manager_node.py

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Bool


class AlignModeManager(Node):
    """
    NAV2 → RL 자동 전환 & cmd_vel 포워딩 노드
    """

    def __init__(self):
        super().__init__("align_mode_manager")

        # ===== 파라미터 =====
        # 조금 널널하게 조정 (원래 0.040 / 0.060)
        self.switch_to_rl_dist_m = 0.04  # 50 mm 이하면 RL로
        self.switch_to_nav_dist_m = 0.06 # 70 mm 이상이면 NAV로 (히스테리시스)
        self.forward_hz = 20.0
        self.stale_timeout = 0.5
        self.stop_on_switch = True

        # ===== 내부 상태 =====
        self.mode = "NAV"
        self.last_nav2: Optional[Twist] = None
        self.last_rl: Optional[Twist] = None
        self.last_nav2_t = 0.0
        self.last_rl_t = 0.0

        # ===== 구독 / 퍼블리시 =====
        self.sub_pos = self.create_subscription(Point, "/coil_pos", self.cb_pos, 10)
        self.sub_nav = self.create_subscription(Twist, "/cmd_vel_nav2", self.cb_nav2, 10)
        self.sub_rl  = self.create_subscription(Twist, "/cmd_vel_rl",   self.cb_rl,   10)

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_mode = self.create_publisher(String, "/align_mode", 10)
        self.pub_is_rl = self.create_publisher(Bool, "/align_is_rl", 10)

        self.timer = self.create_timer(1.0/self.forward_hz, self.forward_loop)

        self.get_logger().info(
            f"align_mode_manager started. "
            f"toRL ≤ {self.switch_to_rl_dist_m*1000:.0f} mm, "
            f"toNAV ≥ {self.switch_to_nav_dist_m*1000:.0f} mm, "
            f"mode = {self.mode}"
        )

    # ---------- 콜백들 ----------
    def cb_nav2(self, msg: Twist):
        self.last_nav2 = msg
        self.last_nav2_t = time.time()

    def cb_rl(self, msg: Twist):
        self.last_rl = msg
        self.last_rl_t = time.time()

    def cb_pos(self, msg: Point):
        r = math.hypot(msg.x, msg.y)  # [m]
        r_mm = r * 1000.0

        # 디버그용 거리 로그
        self.get_logger().debug(
            f"/coil_pos: x={msg.x:.4f} m, y={msg.y:.4f} m, r={r_mm:.1f} mm, mode={self.mode}"
        )

        # 히스테리시스 스위치
        if self.mode == "NAV" and r <= self.switch_to_rl_dist_m:
            self.get_logger().info(
                f"r={r_mm:.1f} mm <= {self.switch_to_rl_dist_m*1000:.0f} mm → switch NAV→RL"
            )
            self.switch_mode("RL")
        elif self.mode == "RL" and r >= self.switch_to_nav_dist_m:
            self.get_logger().info(
                f"r={r_mm:.1f} mm >= {self.switch_to_nav_dist_m*1000:.0f} mm → switch RL→NAV"
            )
            self.switch_mode("NAV")

        self.publish_mode()

    # ---------- 모드 전환 ----------
    def switch_mode(self, new_mode: str):
        if new_mode == self.mode:
            return
        self.get_logger().info(f"Mode switch: {self.mode} → {new_mode}")
        self.mode = new_mode

        if self.stop_on_switch:
            self.send_zero_once()

        self.publish_mode()

    def send_zero_once(self):
        zero = Twist()
        self.pub_cmd.publish(zero)
        self.get_logger().info("Sent zero /cmd_vel on mode switch")

    def publish_mode(self):
        self.pub_mode.publish(String(data=self.mode))
        self.pub_is_rl.publish(Bool(data=(self.mode == "RL")))

    # ---------- 포워딩 루프 ----------
    def forward_loop(self):
        now = time.time()

        nav_fresh = (self.last_nav2 is not None) and (now - self.last_nav2_t <= self.stale_timeout)
        rl_fresh  = (self.last_rl  is not None) and (now - self.last_rl_t  <= self.stale_timeout)

        out = Twist()
        if self.mode == "NAV":
            if nav_fresh:
                out = self.last_nav2
            else:
                out = Twist()
        else:  # RL
            if rl_fresh:
                out = self.last_rl
            else:
                out = Twist()

        self.pub_cmd.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AlignModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
