cmake_minimum_required(VERSION 3.8)
project(motobat_bringup)

find_package(ament_cmake REQUIRED)

# Launch + YAML only (project_structure.md §3): nothing depends on this package.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
