cmake_minimum_required(VERSION 3.22)
project(motobat_gp4_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# Acceptance check (system python, rclpy):
#   ros2 run motobat_gp4_moveit_config gp4_check.py --station s4a|s5
install(PROGRAMS scripts/gp4_check.py DESTINATION lib/${PROJECT_NAME})
