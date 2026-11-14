cmake_minimum_required(VERSION 3.8)
project(motobat_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KeyValue.msg"
  "msg/ScanResult.msg"
  "msg/CaptureInfo.msg"
  "msg/Finding.msg"
  "msg/Measurement.msg"
  "msg/Uncovered.msg"
  "msg/DefectReport.msg"
  "action/CorrectionCmd.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_report_roundtrip test/test_report_roundtrip.py)
endif()

ament_package()
