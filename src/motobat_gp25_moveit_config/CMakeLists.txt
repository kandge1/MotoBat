cmake_minimum_required(VERSION 3.22)
project(motobat_gp25_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
# Acceptance check (system python, rclpy):
#   ros2 run motobat_gp25_moveit_config welder_check.py
install(PROGRAMS scripts/welder_check.py DESTINATION lib/${PROJECT_NAME})
