#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import time

from volt_runner_performance.msg import Perf2Command


class Perf2PCProbe(Node):
    """
    PC에서 perf2_publisher가 제대로 /perf2를 내보내는지 확인하는 용도.
    - /perf2 구독
    - PC 수신 시각(now) 찍음
    - msg.t_send_pc(보낸 시각)와 now를 ns로 변환해 버퍼링
    - 종료 시 CSV로 저장
    """
    def __init__(self):
        super().__init__('perf2_pc_probe')
        self.create_subscription(Perf2Command, '/perf2', self.cb, 10)

        self.buffer = []
        self.start_time_str = time.strftime("%Y%m%d-%H%M%S")

        self.get_logger().info("Perf2 PC Probe Started. Listening /perf2 ...")

    @staticmethod
    def time_to_ns(t):
        return (t.sec * 1_000_000_000) + t.nanosec

    def cb(self, msg: Perf2Command):
        now = self.get_clock().now().to_msg()

        t_send_pc_ns = self.time_to_ns(msg.t_send_pc)
        t_recv_pc_ns = self.time_to_ns(now)

        row = [
            msg.seq,
            msg.source,
            t_send_pc_ns,
            t_recv_pc_ns,
            msg.cmd_vel.linear.x,
            msg.cmd_vel.angular.z
        ]
        self.buffer.append(row)

    def save_csv(self):
        if not self.buffer:
            self.get_logger().warn("No records captured; nothing to save.")
            return

        filename = f'perf2_probe_{self.start_time_str}.csv'
        self.get_logger().info(f"Saving {len(self.buffer)} records to {filename} ...")
        try:
            with open(filename, mode='w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['Seq', 'Source', 'Tsend_pc_ns', 'Trecv_pc_ns', 'Linear_X', 'Angular_Z'])
                w.writerows(self.buffer)
            self.get_logger().info("Save Complete!")
        except Exception as e:
            self.get_logger().error(f"Failed to save CSV: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Perf2PCProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
