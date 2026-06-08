#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import time
from volt_runner_performance.msg import Perf2Command


class Perf2Collector(Node):
    """
    PC에서 실행.
    - /perf2_echo 구독
    - 수신 즉시 t_recv_pc(Trecv2) 찍어서 msg에 채움
    - 4개 타임스탬프를 ns 정수(int)로 변환해 버퍼에 저장
    - Ctrl+C 종료 시 CSV로 한 번에 flush
    """
    def __init__(self):
        super().__init__('perf2_collector')
        self.create_subscription(Perf2Command, '/perf2_echo', self.cb, 10)

        self.buffer = []
        self.start_time_str = time.strftime("%Y%m%d-%H%M%S")
        self.get_logger().info("Perf2Collector Started. Listening /perf2_echo ...")

    @staticmethod
    def time_to_ns(t) -> int:
        # builtin_interfaces/Time -> int nanoseconds
        return int(t.sec) * 1_000_000_000 + int(t.nanosec)

    def cb(self, msg: Perf2Command):
        # 1) PC 최종 수신 시각(Trecv2) - 콜백 진입 즉시
        now = self.get_clock().now().to_msg()
        msg.t_recv_pc = now  # (선택) 메시지 필드도 채워둠

        # 2) ns 변환 (정수)
        t_send_pc_ns  = self.time_to_ns(msg.t_send_pc)
        t_recv_rpi_ns = self.time_to_ns(msg.t_recv_rpi)
        t_send_rpi_ns = self.time_to_ns(msg.t_send_rpi)
        t_recv_pc_ns  = self.time_to_ns(msg.t_recv_pc)

        # 3) 파생 지표(ms) (float)
        one_way_pc_to_rpi_ms = (t_recv_rpi_ns - t_send_pc_ns) / 1_000_000.0
        rpi_internal_ms      = (t_send_rpi_ns - t_recv_rpi_ns) / 1_000_000.0
        one_way_rpi_to_pc_ms = (t_recv_pc_ns - t_send_rpi_ns) / 1_000_000.0
        rtt_ms               = (t_recv_pc_ns - t_send_pc_ns) / 1_000_000.0

        # 4) 버퍼 저장 (ns는 정수 그대로 저장 -> CSV에 지수표기 방지)
        row = [
            int(msg.seq),
            msg.source,
            t_send_pc_ns,
            t_recv_rpi_ns,
            t_send_rpi_ns,
            t_recv_pc_ns,
            f"{one_way_pc_to_rpi_ms:.4f}",
            f"{rpi_internal_ms:.4f}",
            f"{one_way_rpi_to_pc_ms:.4f}",
            f"{rtt_ms:.4f}",
            f"{msg.cmd_vel.linear.x:.6f}",
            f"{msg.cmd_vel.angular.z:.6f}",
        ]
        self.buffer.append(row)

    def save_csv(self):
        if not self.buffer:
            # shutdown 타이밍에 rosout 경고 뜰 수 있어 print로도 충분
            try:
                self.get_logger().warn("No records captured; nothing to save.")
            except Exception:
                pass
            return

        filename = f'perf2_rtt_{self.start_time_str}.csv'
        try:
            self.get_logger().info(f"Saving {len(self.buffer)} records to {filename} ...")
        except Exception:
            pass

        with open(filename, mode='w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                'Seq', 'Source',
                'Tsend_pc_ns', 'Trecv_rpi_ns', 'Tsend2_rpi_ns', 'Trecv2_pc_ns',
                'OneWay_PC_to_RPI_ms', 'RPI_Internal_ms', 'OneWay_RPI_to_PC_ms', 'RTT_ms',
                'Linear_X', 'Angular_Z'
            ])
            w.writerows(self.buffer)

        try:
            self.get_logger().info("Save Complete!")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Perf2Collector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_csv()
        try:
            node.destroy_node()
        except Exception:
            pass
        # shutdown 중복/타이밍 에러 방지
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

