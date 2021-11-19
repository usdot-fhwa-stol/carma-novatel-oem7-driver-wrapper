cmake_minimum_required(VERSION 3.5)
project(carma_novatel_driver_wrapper)


find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(ament_cmake REQUIRED)
find_package(carma_cmake_common REQUIRED)
find_package(carma_ros2_utils REQUIRED)
find_package(carma_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_lifecycle_manager REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(carma_driver_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)

set(base_dependencies
  carma_cmake_common
  carma_ros2_utils
  carma_msgs
  rclcpp
  ros2_lifecycle_manager
  lifecycle_msgs
  sensor_msgs
  gps_msgs
  carma_driver_msgs
  novatel_oem7_msgs
)

set(base_lib_cpp src/carma_novatel_driver_wrapper.cpp)
carma_define_ament_library(carma_novatel_driver_wrapper "${base_dependencies}" "${base_lib_cpp}")

set(dependency_libs
  ${base_dependencies}
  carma_novatel_driver_wrapper
)

# carma_define_ament_executable(carma_novatel_driver_wrapper_worker "${dependency_libs}" src/main.cpp)
rclcpp_components_register_nodes(carma_novatel_driver_wrapper "carma_novatel_driver_wrapper")

# if(BUILD_TESTING)

#   add_subdirectory(test)

# endif()

install(DIRECTORY config/
  DESTINATION config/
)

ament_package()