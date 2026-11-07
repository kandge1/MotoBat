cmake_minimum_required(VERSION 3.22)
project(motobat_hc10dt_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# IK check and solver comparison (system python, rclpy):
#   ros2 run motobat_hc10dt_moveit_config compare_ik.py
#   ros2 run motobat_hc10dt_moveit_config plan_exec_check.py   (acceptance: plan + execute)
install(PROGRAMS scripts/check_ik.py scripts/compare_ik.py scripts/plan_exec_check.py
        DESTINATION lib/${PROJECT_NAME})
