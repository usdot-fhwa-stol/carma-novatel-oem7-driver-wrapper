cmake_minimum_required(VERSION 3.5)
project(carma_novatel_driver_wrapper)

carma_package()

find_package(ament_cmake_auto REQUIRED)
find_package(carma_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(uncertainty_tools REQUIRED)
find_package(carma_ros2_utils REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
  include
)

ament_auto_add_library(carma_novatel_driver_wrapper_lib SHARED
  src/carma_novatel_driver_wrapper.cpp
)

ament_auto_add_executable(carma_novatel_wrapper_node src/main.cpp)

target_link_libraries(carma_novatel_wrapper_node 
 carma_novatel_driver_wrapper_lib
)

if(BUILD_TESTING)  
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_carma_novatel_driver_wrapper test/carma_novatel_driver_wrapper_test.cpp)
  
  target_include_directories(test_carma_novatel_driver_wrapper
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_carma_novatel_driver_wrapper ${dependencies})
  
  target_link_libraries(test_carma_novatel_driver_wrapper carma_novatel_driver_wrapper_lib)

endif()

#Install
ament_auto_package(
  INSTALL_TO_SHARE config launch
)