#!/usr/bin/env python3
import math
import sys
import inspect

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Float32

# RL 프로젝트 루트 경로 (지금 구조 기준)
RL_ROOT_DEFAULT = "/home/lim/RL"
EXCEL_DEFAULT = "/home/lim/RL/coil_measurements.xlsx"
BLOCK_IDX_DEFAULT = 1  # AIRGAP_0MM_BLOCK_INDEX 와 동일하게 사용


class CoilEfficiencyPublisher(Node):
    """
    /coil_pos(x, y [m]) -> 엑셀 기반 효율값 -> /coil_efficiency(Float32) 퍼블리셔 노드

    - /coil_pos : geometry_msgs/Point
        * x, y : 수신 코일 기준 송신 코일 위치 [m]
    - /coil_efficiency : std_msgs/Float32
        * data : eff_func 출력값 (예: 0.0~1.0 또는 0~100, load_eff_table 정의에 따름)
    """

    def __init__(self):
        super().__init__("coil_efficiency_publisher")

        # ===== 파라미터 =====
        # RL 프로젝트 경로 / 엑셀 경로 / block index
        self.declare_parameter("rl_root", RL_ROOT_DEFAULT)
        self.declare_parameter("excel_path", EXCEL_DEFAULT)
        self.declare_parameter("block_idx", BLOCK_IDX_DEFAULT)

        rl_root = self.get_parameter("rl_root").get_parameter_value().string_value
        excel_path = self.get_parameter("excel_path").get_parameter_value().string_value
        block_idx = self.get_parameter("block_idx").get_parameter_value().integer_value

        # ===== RL 프로젝트 모듈 import 준비 =====
        # /home/lim/RL 을 sys.path 에 추가해서 load_eff_table.py 를 import 가능하게
        if rl_root not in sys.path:
            sys.path.append(rl_root)

        try:
            from load_eff_table import make_eff_func
        except ImportError as e:
            self.get_logger().error(
                f"load_eff_table.py import 실패: {e}. "
                f"rl_root 파라미터({rl_root})와 파일 위치를 확인하세요."
            )
            raise

        self.get_logger().info(
            f"엑셀 효율 테이블 로드 중... excel_path={excel_path}, block_idx={block_idx}"
        )

        # 엑셀 기반 효율 함수 생성 (RL 학습 때와 동일하게)
        self.eff_func = make_eff_func(
            path=excel_path,
            block_idx=block_idx,
        )

        # eff_func 인자 개수 자동 판별 (r만 받는지, x,y 둘 다 받는지 모르니까)
        sig = inspect.signature(self.eff_func)
        self.eff_n_args = len(sig.parameters)
        self.get_logger().info(
            f"eff_func 시그니처 인자 개수 = {self.eff_n_args} "
            "(1개면 r_mm, 2개 이상이면 x_mm, y_mm 으로 호출합니다)"
        )

        # ===== ROS 통신 설정 =====
        # /coil_pos 구독
        self.sub_pos = self.create_subscription(
            Point,
            "/coil_pos",
            self.coil_pos_callback,
            10,
        )

        # /coil_efficiency 퍼블리시
        self.pub_eff = self.create_publisher(
            Float32,
            "/coil_efficiency",
            10,
        )

        self.get_logger().info("CoilEfficiencyPublisher 노드 시작됨!")

    # ---------- 콜백 ----------
    def coil_pos_callback(self, msg: Point):
        # [m] -> [mm]
        x_mm = msg.x * 1000.0
        y_mm = msg.y * 1000.0
        r_mm = math.hypot(x_mm, y_mm)

        # eff_func 인자 개수에 따라 호출 방식 다르게
        try:
            if self.eff_n_args == 1:
                # eff_func(r_mm)
                eff = float(self.eff_func(r_mm))
            elif self.eff_n_args >= 2:
                # eff_func(x_mm, y_mm)
                eff = float(self.eff_func(x_mm, y_mm))
            else:
                self.get_logger().warn(
                    "eff_func 인자 개수가 0개라서 효율 계산 불가 → NaN 퍼블리시"
                )
                eff = float("nan")
        except Exception as e:
            self.get_logger().warn(f"eff_func 계산 중 에러 발생: {e}")
            eff = float("nan")

        msg_eff = Float32()
        msg_eff.data = eff
        self.pub_eff.publish(msg_eff)

        # 디버그 용 로그 (원하면 rclpy logger 레벨 debug로)
        self.get_logger().debug(
            f"x={x_mm:.1f} mm, y={y_mm:.1f} mm, r={r_mm:.1f} mm -> eff={eff:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CoilEfficiencyPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
