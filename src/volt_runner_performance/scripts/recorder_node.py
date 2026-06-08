#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import time
from volt_runner_performance.msg import PerfCommand

class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')
        self.create_subscription(PerfCommand, '/perf', self.listener_callback, 10)
        self.buffer = []
        self.start_time_str = time.strftime("%Y%m%d-%H%M%S")
        self.get_logger().info("Recorder Node Started (Full Data Mode). Waiting for /perf...")

    def listener_callback(self, msg):
        # 1. 도착 시간(Tr) 측정
        now = self.get_clock().now().to_msg()
        
        # 2. 시간 변환 (ns)
        ts_ns = (msg.time_sent.sec * 1_000_000_000) + msg.time_sent.nanosec
        tr_ns = (now.sec * 1_000_000_000) + now.nanosec
        
        # 3. 데이터 저장 (모든 축 데이터 포함!)
        row = [
            msg.source,
            ts_ns,  # 보낸 시간
            tr_ns,  # 받은 시간
            msg.cmd_vel.linear.x,  # 전후
            msg.cmd_vel.linear.y,  # 좌우 (메카넘 핵심)
            msg.cmd_vel.linear.z,  # (보통 0)
            msg.cmd_vel.angular.x, # (보통 0)
            msg.cmd_vel.angular.y, # (보통 0)
            msg.cmd_vel.angular.z  # 회전
        ]
        self.buffer.append(row)

    def save_csv(self):
        if not self.buffer: return
        filename = f'latency_log_{self.start_time_str}.csv'
        self.get_logger().info(f"Saving {len(self.buffer)} records to {filename}...")
        try:
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                # 헤더도 6축 전부 작성
                writer.writerow([
                    'Source', 'Ts_ns', 'Tr_ns', 
                    'Lin_X', 'Lin_Y', 'Lin_Z', 
                    'Ang_X', 'Ang_Y', 'Ang_Z'
                ])
                writer.writerows(self.buffer)
            self.get_logger().info("Save Complete!")
        except Exception as e:
            self.get_logger().error(f"Failed to save CSV: {e}")

    # 종료 에러 방지용 안전 종료 코드 추가됨
    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_csv()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
