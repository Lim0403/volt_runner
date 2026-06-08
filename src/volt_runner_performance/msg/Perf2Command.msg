geometry_msgs/Twist cmd_vel
string source              # "nav2" / "manual" / "rl"
uint64 seq                 # 메시지 매칭용 시퀀스 번호

builtin_interfaces/Time t_send_pc    # Tsend:   PC가 RPI로 publish 직전
builtin_interfaces/Time t_recv_rpi   # Trecv:   RPI가 메시지 수신 직후(콜백 진입 시점)
builtin_interfaces/Time t_send_rpi   # Tsend2:  RPI가 PC로 echo publish 직전
builtin_interfaces/Time t_recv_pc    # Trecv2:  PC가 echo 수신 직후(콜백 진입 시점)
