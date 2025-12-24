#!/usr/bin/env python3
"""
rl_coil_controller_node.py

역할:
- ROS2에서 송신 코일의 (x, y) 좌표를 받아서
- 미리 학습된 PPO + VecNormalize 모델로부터 vx, vy를 계산하고
- /cmd_vel 로 Twist 메시지를 퍼블리시하는 브릿지 노드.

전제:
- /coil_pos: geometry_msgs/Point
    * x: 수신 코일 기준 송신 코일의 x 위치 [m]
    * y: 수신 코일 기준 송신 코일의 y 위치 [m]
    * z: 사용 안 함
- /cmd_vel: geometry_msgs/Twist
    * linear.x: 로봇 몸체 기준 x 방향 속도 [m/s]
    * linear.y: 로봇 몸체 기준 y 방향 속도 [m/s]
    * angular.z 등은 0 (yaw 제어 안 함)

- RL 학습 시:
    * 관측 obs = [x_mm, y_mm] (mm 단위)
    * VecNormalize 로 obs 정규화 사용
    * 액션 = [vx_norm, vy_norm] ∈ [-1, 1]
    * env에서 1 step에 최대 5 mm 이동 → v_max ≈ 0.05 m/s 정도라고 가정
"""

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecEnv, VecNormalize
from stable_baselines3.common.env_util import make_vec_env

from envs.env_coil import CoilAlignEnv       # RL 학습에 사용했던 환경
from load_eff_table import make_eff_func     # 엑셀 기반 효율 함수


# ===== 경로/설정 =====

# RL 프로젝트 경로 기준 절대 경로로 설정
EXCEL_PATH   = "/home/lim/RL/coil_measurements.xlsx"
MODEL_PATH   = "/home/lim/RL/models/coil_ppo/best_model.zip"
VECNORM_PATH = "/home/lim/RL/models/coil_ppo/best_vecnormalize.pkl"

# 엑셀에서 0mm 에어갭 데이터에 해당하는 block_idx
AIRGAP_0MM_BLOCK_INDEX = 1  # 필요하면 1 또는 2로 수정

# 제어 주기 [Hz]
CONTROL_HZ = 10.0       # 10 Hz → 0.1초마다 한 번씩 action

# 최대 선속도 [m/s] (학습 환경과 비슷하게)
V_MAX = 0.05            # 5 cm/s


def make_coil_env_for_vecnorm() -> CoilAlignEnv:
    """
    VecNormalize를 로드하기 위한 dummy 환경 생성.

    실제로 env의 reset/step은 사용하지 않고,
    obs 정규화만 VecNormalize에서 가져다 쓸 예정이므로
    학습 때와 대략 비슷하게 맞춰 둔다.
    """
    eff_func = make_eff_func(
        path=EXCEL_PATH,
        block_idx=AIRGAP_0MM_BLOCK_INDEX,
    )

    env = CoilAlignEnv(
        max_radius=40.0,   # mm
        init_radius=40.0,  # mm
        eff_func=eff_func,
    )
    return env


class RLCoilController(Node):
    def __init__(self):
        super().__init__("rl_coil_controller")

        # ===== 1) RL 모델 / VecNormalize 로드 =====
        self.get_logger().info("Loading VecNormalize statistics...")
        # VecNormalize를 로드하기 위해 dummy VecEnv 생성
        base_vec_env: VecEnv = make_vec_env(make_coil_env_for_vecnorm, n_envs=1)

        self.vecnorm: VecNormalize = VecNormalize.load(VECNORM_PATH, venv=base_vec_env)
        # 평가 모드: obs만 정규화, reward 정규화/업데이트는 끔
        self.vecnorm.training = False
        self.vecnorm.norm_reward = False

        self.get_logger().info("Loading PPO model...")
        # env=None 으로 로드해도 predict() 사용에는 문제 없음
        self.model: PPO = PPO.load(MODEL_PATH)
        self.get_logger().info(f"Loaded model from: {MODEL_PATH}")
        self.get_logger().info(f"Loaded VecNormalize stats from: {VECNORM_PATH}")

        # ===== 2) ROS2 통신 설정 =====

        # /coil_pos 구독 (송신 코일 좌표)
        #  - x: x [m]
        #  - y: y [m]
        self.coil_pos_sub = self.create_subscription(
            Point,
            "/coil_pos",
            self.coil_pos_callback,
            10,
        )

        # /cmd_vel 퍼블리셔 (로봇 속도 명령)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel_rl",
            10,
        )

        # 최신 좌표 저장용
        self.current_x_m: float | None = None
        self.current_y_m: float | None = None

        # 제어 루프 타이머
        timer_period = 1.0 / CONTROL_HZ
        self.control_timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f"RLCoilController node started. Control rate = {CONTROL_HZ} Hz"
        )

    # ---------- 콜백: 송신 코일 좌표 수신 ----------
    def coil_pos_callback(self, msg: Point):
        """
        /coil_pos 콜백

        msg.x, msg.y 는
        '수신 코일을 원점으로 한 송신 코일의 위치 [m]' 라고 가정.
        """
        self.current_x_m = msg.x
        self.current_y_m = msg.y

    # ---------- 제어 루프 ----------
    def control_loop(self):
        """
        CONTROL_HZ 주기로 호출되어,
        - 현재 (x, y) 좌표를 obs로 만들어
        - RL 정책으로부터 action = [vx_norm, vy_norm]을 얻고
        - 실제 속도 vx, vy [m/s]로 스케일링해서 /cmd_vel 로 퍼블리시
        """
        # 좌표를 아직 못 받았으면 아무것도 하지 않음
        if self.current_x_m is None or self.current_y_m is None:
            return

        # 1) ROS2 (m) → RL env 단위 (mm)
        x_mm = self.current_x_m * 1000.0
        y_mm = self.current_y_m * 1000.0

        # obs shape: (n_env, obs_dim) = (1, 2)
        obs = np.array([[x_mm, y_mm]], dtype=np.float32)

        # 2) VecNormalize 로 정규화
        obs_norm = self.vecnorm.normalize_obs(obs.copy())

        # 3) RL 정책에서 action 예측 (deterministic)
        action, _ = self.model.predict(obs_norm, deterministic=True)
        # action shape: (1, 2) 이라고 가정
        vx_norm, vy_norm = action[0]

        # 안정성을 위해 [-1, 1]로 한 번 더 클리핑
        vx_norm = float(np.clip(vx_norm, -1.0, 1.0))
        vy_norm = float(np.clip(vy_norm, -1.0, 1.0))

        # 4) 실제 속도 [m/s] 로 스케일링
        vx = vx_norm * V_MAX
        vy = vy_norm * V_MAX

        # 5) /cmd_vel 퍼블리시
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

        # 디버깅 로그 (필요하면 rclpy.logging에서 레벨 debug로 켜기)
        self.get_logger().debug(
            f"x={x_mm:.1f} mm, y={y_mm:.1f} mm -> "
            f"vx_norm={vx_norm:.3f}, vy_norm={vy_norm:.3f}, "
            f"vx={vx:.3f} m/s, vy={vy:.3f} m/s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RLCoilController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
