cmake_minimum_required(VERSION 3.8)
project(motobat_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz
        DESTINATION share/${PROJECT_NAME})

# Python scripts go in lib/<pkg> so `ros2 run` finds them.
install(PROGRAMS scripts/demo_line_motion.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
