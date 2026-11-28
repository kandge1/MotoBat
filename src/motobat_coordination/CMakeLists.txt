cmake_minimum_required(VERSION 3.22)
project(motobat_coordination)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/s1_station.py scripts/correction_node.py scripts/loop_controller.py scripts/loop_report.py
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_loop_controller test/test_loop_controller.py)
endif()

ament_package()
