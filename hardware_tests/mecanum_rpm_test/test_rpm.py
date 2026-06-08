#!/usr/bin/env python3

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class RPMTestNode(Node):
    """
    현재 실험 기준 매핑:
      target[0] = RR
      target[1] = RL
      target[2] = FR
      target[3] = FL

    현재 feedback 해석:
      RR = -feedback[0]
      RL =  feedback[1]
      FR = -feedback[2]
      FL =  feedback[3]
    """

    def __init__(self, args):
        super().__init__("test_rpm")

        self.args = args
        self.target_pub = self.create_publisher(
            Float32MultiArray,
            "/wheel_target_rpm",
            10,
        )

        self.feedback_sub = self.create_subscription(
            Float32MultiArray,
            "/wheel_feedback_rpm",
            self.feedback_callback,
            10,
        )

        self.latest_feedback = [math.nan, math.nan, math.nan, math.nan]
        self.rows = []

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.outdir = (
            Path(args.outdir).expanduser()
            / f"test_rpm_{stamp}_{args.mode}_max{args.max_rpm:g}"
        )
        self.outdir.mkdir(parents=True, exist_ok=True)

        self.start_time = time.monotonic()
        self.finished = False

        self.total_time = (
            args.pre_time
            + args.accel_time
            + args.hold_time
            + args.decel_time
            + args.post_time
        )

        self.timer = self.create_timer(1.0 / args.rate, self.timer_callback)

        self.get_logger().info("RPM test started")
        self.get_logger().info(f"Mode: {self.args.mode}")
        self.get_logger().info(f"Wheel order: [RR, RL, FR, FL]")
        self.get_logger().info(f"Output directory: {self.outdir}")

    def feedback_callback(self, msg):
        if len(msg.data) >= 4:
            self.latest_feedback = [
                float(msg.data[0]),
                float(msg.data[1]),
                float(msg.data[2]),
                float(msg.data[3]),
            ]

    def rpm_profile(self, t):
        args = self.args
        max_rpm = args.max_rpm * args.direction

        if t < args.pre_time:
            return 0.0

        t -= args.pre_time

        if t < args.accel_time:
            return max_rpm * (t / args.accel_time)

        t -= args.accel_time

        if t < args.hold_time:
            return max_rpm

        t -= args.hold_time

        if t < args.decel_time:
            return max_rpm * (1.0 - t / args.decel_time)

        return 0.0

    def publish_target_rpm(self, rpm):
        """
        Publish wheel target rpm using the confirmed robot wheel order:
          [RR, RL, FR, FL]

        mode x:
          forward/backward translation
          [+, +, +, +]

        mode y:
          lateral translation
          [-, +, +, -]

        mode z:
          in-place rotation
          [+, -, +, -]
        """
        msg = Float32MultiArray()

        if self.args.mode == "x":
            pattern = [1.0, 1.0, 1.0, 1.0]
        elif self.args.mode == "y":
            pattern = [-1.0, 1.0, 1.0, -1.0]
        elif self.args.mode == "z":
            pattern = [1.0, -1.0, 1.0, -1.0]
        else:
            raise ValueError(f"Unknown mode: {self.args.mode}")

        msg.data = [float(rpm * s) for s in pattern]
        self.target_pub.publish(msg)

    def target_pattern(self, rpm):
        """
        Return target rpm list in confirmed order:
          [RR, RL, FR, FL]
        """
        if self.args.mode == "x":
            pattern = [1.0, 1.0, 1.0, 1.0]
        elif self.args.mode == "y":
            pattern = [-1.0, 1.0, 1.0, -1.0]
        elif self.args.mode == "z":
            pattern = [1.0, -1.0, 1.0, -1.0]
        else:
            raise ValueError(f"Unknown mode: {self.args.mode}")

        return [float(rpm * s) for s in pattern]

    def publish_zero_rpm(self):
        self.publish_target_rpm(0.0)

    def timer_callback(self):
        t = time.monotonic() - self.start_time
        target_rpm = self.rpm_profile(t)

        self.publish_target_rpm(target_rpm)

        fb0, fb1, fb2, fb3 = self.latest_feedback

        row = {
            "time_s": t,

            # 현재 target topic index 기준: [RR, RL, FR, FL]
            "target_0_RR": self.target_pattern(target_rpm)[0],
            "target_1_RL": self.target_pattern(target_rpm)[1],
            "target_2_FR": self.target_pattern(target_rpm)[2],
            "target_3_FL": self.target_pattern(target_rpm)[3],

            # raw feedback
            "feedback_raw_0": fb0,
            "feedback_raw_1": fb1,
            "feedback_raw_2": fb2,
            "feedback_raw_3": fb3,

            # 현재 실험 기준으로 부호 보정한 physical feedback
            "feedback_RR": -fb0,
            "feedback_RL": fb1,
            "feedback_FR": -fb2,
            "feedback_FL": fb3,
        }

        self.rows.append(row)

        if t >= self.total_time:
            self.finished = True
            self.timer.cancel()
            self.publish_zero_rpm()
            self.get_logger().info("RPM test finished")

    def save_csv(self):
        csv_path = self.outdir / "test_rpm_log.csv"

        if not self.rows:
            self.get_logger().warn("No data rows to save")
            return None

        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)

        self.get_logger().info(f"CSV saved: {csv_path}")
        return csv_path

    def save_graphs(self):
        if not self.rows:
            self.get_logger().warn("No data rows to plot")
            return

        t = [r["time_s"] for r in self.rows]

        # 1. 부호 보정된 target vs feedback 그래프
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

        plot_items = [
            ("RR", "target_0_RR", "feedback_RR"),
            ("RL", "target_1_RL", "feedback_RL"),
            ("FR", "target_2_FR", "feedback_FR"),
            ("FL", "target_3_FL", "feedback_FL"),
        ]

        for ax, (wheel, target_col, feedback_col) in zip(axes, plot_items):
            target = [r[target_col] for r in self.rows]
            feedback = [r[feedback_col] for r in self.rows]

            ax.plot(t, target, label=f"{wheel} target")
            ax.plot(t, feedback, label=f"{wheel} feedback")
            ax.set_ylabel("RPM")
            ax.grid(True)
            ax.legend(loc="best")

        axes[-1].set_xlabel("Time [s]")
        fig.suptitle("RPM Target vs Feedback")
        fig.tight_layout()

        corrected_path = self.outdir / "rpm_target_vs_feedback.png"
        fig.savefig(corrected_path, dpi=150)
        plt.close(fig)

        # 2. raw feedback 그래프
        fig, ax = plt.subplots(figsize=(12, 6))

        for col, label in [
            ("target_0_RR", "target RR"),
            ("target_1_RL", "target RL"),
            ("target_2_FR", "target FR"),
            ("target_3_FL", "target FL"),
        ]:
            target = [r[col] for r in self.rows]
            ax.plot(t, target, linestyle="--", label=label)

        for i in range(4):
            raw_feedback = [r[f"feedback_raw_{i}"] for r in self.rows]
            ax.plot(t, raw_feedback, label=f"raw feedback[{i}]")

        ax.set_xlabel("Time [s]")
        ax.set_ylabel("RPM")
        ax.set_title("Raw RPM Feedback")
        ax.grid(True)
        ax.legend(loc="best")
        fig.tight_layout()

        raw_path = self.outdir / "raw_rpm_feedback.png"
        fig.savefig(raw_path, dpi=150)
        plt.close(fig)

        self.get_logger().info(f"Graph saved: {corrected_path}")
        self.get_logger().info(f"Graph saved: {raw_path}")


def parse_args():
    parser = argparse.ArgumentParser(description="RPM profile test logger")

    parser.add_argument(
        "--mode",
        type=str,
        default="x",
        choices=["x", "y", "z"],
        help="Motion mode based on confirmed wheel order [RR, RL, FR, FL]: x=[+,+,+,+], y=[-,+,+,-], z=[+,-,+,-]",
    )
    parser.add_argument("--max-rpm", type=float, default=25.0)
    parser.add_argument("--pre-time", type=float, default=2.0)
    parser.add_argument("--accel-time", type=float, default=3.0)
    parser.add_argument("--hold-time", type=float, default=3.0)
    parser.add_argument("--decel-time", type=float, default=3.0)
    parser.add_argument("--post-time", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--direction", type=float, default=1.0)
    parser.add_argument("--outdir", type=str, default="~/mecanum_test/results")

    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = RPMTestNode(args)

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)

        # 안전하게 zero RPM 몇 번 더 publish
        for _ in range(10):
            node.publish_zero_rpm()
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.05)

        node.save_csv()
        node.save_graphs()

    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted. Publishing zero RPM.")

        for _ in range(10):
            node.publish_zero_rpm()
            time.sleep(0.05)

        node.save_csv()
        node.save_graphs()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
