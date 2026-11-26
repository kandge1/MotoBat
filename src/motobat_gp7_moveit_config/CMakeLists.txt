cmake_minimum_required(VERSION 3.22)
project(motobat_gp7_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
# Acceptance check and IK benchmark (system python, rclpy):
#   ros2 run motobat_gp7_moveit_config placer_check.py
#   ros2 run motobat_gp7_moveit_config ik_bench.py
install(PROGRAMS scripts/placer_check.py scripts/ik_bench.py DESTINATION lib/${PROJECT_NAME})
