cmake_minimum_required(VERSION 3.5)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  geometry_msgs
  sensor_msgs
  uav_utils
  nav_msgs
  tof_velocity_guard
)

catkin_package(
)

find_package(mavros_msgs QUIET CONFIG)
if(NOT mavros_msgs_FOUND)
  message(STATUS "mavros_msgs not found: skipping the optional px4ctrl executable")
  return()
endif()

find_package(Eigen3 REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)

add_dependencies(px4ctrl_node ${catkin_EXPORTED_TARGETS} ${mavros_msgs_EXPORTED_TARGETS})

target_link_libraries(px4ctrl_node
  ${catkin_LIBRARIES}
)

install(TARGETS px4ctrl_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
