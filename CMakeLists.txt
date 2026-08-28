cmake_minimum_required(VERSION 3.8)
project(mst110cr_loadsense)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FloatStamped.msg"
  DEPENDENCIES std_msgs
)

# Install the Python nodes alongside the generated Python message package.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(
  DIRECTORY mst110cr_loadsense/
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  FILES_MATCHING PATTERN "*.py"
)

install(
  PROGRAMS
    scripts/dump_bed_calibrator
    scripts/load_volume_estimator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch param model
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
