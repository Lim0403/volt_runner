#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Point
import tf2_ros


class CoilPosePublisher(Node):
    """
    역할:
      - TF 트리에서 receiver(수신 코일)와 sender(송신 코일) 사이의 상대 위치를 가져와서
      - 수신 코일을 원점(0,0)으로 두었을 때, 송신 코일의 (x, y) 위치를 /coil_pos로 퍼블리시.

    전제:
      - receiver frame:  "my_car/front_receiver_link"
      - sender frame:    "rear_sender_link"
      - 퍼블리시 토픽:   /coil_pos (geometry_msgs/Point)
        * x, y: [m] 단위 (나중에 RL에서 mm로 바꿔서 사용)
    """

    def __init__(self):
        super().__init__("coil_pose_publisher")

        # /coil_pos 퍼블리셔
        self.pos_pub = self.create_publisher(Point, "/coil_pos", 10)

        # TF Buffer + Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 10 Hz 타이머
        self.timer = self.create_timer(0.1, self.publish_pos)

        # frame 이름 (필요하면 여기만 수정)
        self.receiver_frame = "my_car/front_receiver_link"
        self.sender_frame = "rear_sender_link"

        self.get_logger().info(
            "coil_pose_publisher 시작됨! "
            f"(receiver='{self.receiver_frame}', sender='{self.sender_frame}')"
        )

    def publish_pos(self):
        try:
            # receiver 기준 sender의 위치를 가져옴
            #
            # lookup_transform(
            #   target_frame, source_frame, time
            # )
            # → source_frame을 target_frame 좌표계로 본 변환
            transform = self.tf_buffer.lookup_transform(
                self.receiver_frame,   # target_frame (원점)
                self.sender_frame,     # source_frame (위치 알고 싶은 frame)
                Time()                 # latest
            )

            # translation이 곧 receiver 좌표계에서 본 sender의 위치
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            # tz = transform.transform.translation.z  # 필요하면 사용

            msg = Point()
            msg.x = tx
            msg.y = ty
            msg.z = 0.0  # z는 일단 무시

            self.pos_pub.publish(msg)

        except Exception as e:
            # TF 관계가 아직 안 올라왔을 때 등 → debug로만 출력
            self.get_logger().debug(f"TF 변환 대기중... {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CoilPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
