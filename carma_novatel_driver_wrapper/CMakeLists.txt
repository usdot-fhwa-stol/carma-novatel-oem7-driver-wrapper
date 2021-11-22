cmake_minimum_required(VERSION 3.5)
project(carma_novatel_driver_wrapper)


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
)

ament_auto_add_library(carma_novatel_driver_wrapper_lib SHARED
  src/carma_novatel_driver_wrapper.cpp
)

ament_auto_add_executable(main src/main.cpp)

target_link_libraries(main 
 carma_novatel_driver_wrapper_lib
)

#Install
ament_auto_package(
  INSTALL_TO_SHARE config launch
)