cmake_minimum_required(VERSION 3.8)
project(risk_nav)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 파이썬 모듈(빈 __init__.py 포함)
ament_python_install_package(${PROJECT_NAME})

# 실행 스크립트 설치
install(PROGRAMS
  scripts/llm_risk_node
  scripts/risk_layer_node
  scripts/cmd_vel_risk_filter
  DESTINATION lib/${PROJECT_NAME}
)

# 런치/파라미터 설치
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 