cmake_minimum_required(VERSION 3.8)
project(volt_runner_performance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# 1. 메시지 빌드
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PerfCommand.msg"
  "msg/Perf2Command.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces std_msgs
)

# 2. 파이썬 스크립트 설치
install(PROGRAMS
  scripts/perf_publisher.py
  scripts/recorder_node.py
  scripts/perf2_publisher.py
  scripts/recorder2_node.py
  scripts/perf2_collector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
