cmake_minimum_required(VERSION 3.22)
project(motobat_safety)

find_package(ament_cmake REQUIRED)

#   ros2 run motobat_safety ssm_design.py --plot data/safety/ssm_design.png
#   ros2 run motobat_safety ssm_sim.py --plot data/safety/ssm_sim.png
install(PROGRAMS scripts/ssm.py scripts/ssm_design.py scripts/ssm_sim.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ssm test/test_ssm.py TIMEOUT 300)
endif()

ament_package()
