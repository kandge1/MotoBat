cmake_minimum_required(VERSION 3.16)
project(motobat_opw_kinematics_plugin LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)
# colcon passes no build type by default, which means no optimisation (IK ~90 us instead of a few).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(class_loader REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(random_numbers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)

add_library(${PROJECT_NAME} SHARED src/opw_kinematics_plugin.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# opw_kinematics headers (third_party/, copied unmodified; see third_party/opw_kinematics/VERSION)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/opw_kinematics/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  class_loader::class_loader
  moveit_core::moveit_kinematics_base
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
  random_numbers::random_numbers
  rclcpp::rclcpp
  tf2_eigen::tf2_eigen)

pluginlib_export_plugin_description_file(moveit_core motobat_opw_kinematics_plugin_description.xml)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ third_party/opw_kinematics/include/ DESTINATION include/${PROJECT_NAME})
install(FILES third_party/opw_kinematics/LICENSE DESTINATION share/${PROJECT_NAME}/third_party/opw_kinematics)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(srdfdom REQUIRED)
  find_package(urdf REQUIRED)
  # Expand each vendor xacro to a URDF at build time; the test reads them from here.
  set(TEST_URDF_DIR ${CMAKE_CURRENT_BINARY_DIR}/test_urdf)
  file(MAKE_DIRECTORY ${TEST_URDF_DIR})
  set(TEST_URDFS)
  foreach(arm gp4 gp7 gp25)
    find_package(motoman_${arm}_support REQUIRED)
    set(xacro_in ${motoman_${arm}_support_DIR}/../urdf/${arm}.xacro)
    add_custom_command(OUTPUT ${TEST_URDF_DIR}/${arm}.urdf
      COMMAND xacro ${xacro_in} -o ${TEST_URDF_DIR}/${arm}.urdf
      DEPENDS ${xacro_in}
      COMMENT "xacro ${arm}.xacro")
    list(APPEND TEST_URDFS ${TEST_URDF_DIR}/${arm}.urdf)
  endforeach()
  add_custom_target(${PROJECT_NAME}_test_urdfs ALL DEPENDS ${TEST_URDFS})

  ament_add_gtest(test_opw_plugin test/test_opw_plugin.cpp TIMEOUT 120)
  target_link_libraries(test_opw_plugin ${PROJECT_NAME} urdf::urdf srdfdom::srdfdom)
  target_compile_definitions(test_opw_plugin PRIVATE TEST_URDF_DIR="${TEST_URDF_DIR}")
  add_dependencies(test_opw_plugin ${PROJECT_NAME}_test_urdfs)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 class_loader moveit_core pluginlib random_numbers rclcpp tf2_eigen)
ament_package()
