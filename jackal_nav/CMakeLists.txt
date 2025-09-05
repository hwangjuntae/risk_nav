cmake_minimum_required(VERSION 3.5)
project(jackal_nav)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_plugins REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/jackal_risk_navigation.py
  src/fake_scan_publisher.py
  src/run_isaacsim.py
  src/open_isaacsim_stage.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  rviz
  maps
  params
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python package data
install(DIRECTORY
  src/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py" PATTERN "*.pt"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 