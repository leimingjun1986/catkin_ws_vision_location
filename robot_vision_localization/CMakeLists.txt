cmake_minimum_required(VERSION 2.8.3)
project(robot_vision_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
# add_compile_options(-std=c++11)
# add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(OpenCV REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  eigen_conversions
  nav_msgs
  vision_msgs
  sensor_msgs
  cv_bridge
  image_transport
  serial
  std_msgs
  visualization_msgs
  nodelet
)

catkin_package(
# INCLUDE_DIRS include
#  LIBRARIES minicar_ctrl
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(ALL_TARGET_LIBRARIES "")

include(cmake/eigen.cmake)
include(cmake/YAML.cmake)
include(cmake/glog.cmake)
include(cmake/global_defination.cmake)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/modbus
  /usr/local/include/aruco
)



add_library(${PROJECT_NAME}   
  src/apps/vision_localization_nodelet.cpp
  src/vision_localization_process/vision_localization_process.cpp
  src/vision_localization_process/vision_location_interface.cpp
  src/tf_listener/tf_simple.cpp
  src/modbus/modbus_slave_rtu.cpp
  src/publisher/path_publisher.cpp
  src/publisher/pose2d_publisher.cpp
  src/publisher/pose2d_stamped_publisher.cpp
  src/publisher/camera_control_publisher.cpp
  src/publisher/poseCovStamped_publisher.cpp
  src/publisher/marker_publisher.cpp
  src/state_machine/Component.cc
  src/diagnostics/agv_diagnostics.cpp
  src/tools/file_manager.cpp
)    
                                        
                                                                       
target_link_libraries(${PROJECT_NAME}    
  ${catkin_LIBRARIES} 
  ${ALL_TARGET_LIBRARIES}
  /usr/local/lib/libmodbus.so
)

add_dependencies(${PROJECT_NAME}   vision_msgs_gencpp)


add_executable(  agv_vision_state_machine

   src/apps/agv_vision_state_machine_node.cpp

)

target_link_libraries(agv_vision_state_machine
${PROJECT_NAME}  )