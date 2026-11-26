cmake_minimum_required(VERSION 3.5)
project(tof_velocity_guard)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
find_package(quadrotor_msgs QUIET CONFIG)
catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(serial_range_node src/serial_range_node.cpp)
add_executable(velocity_limiter_node src/velocity_limiter_node.cpp)
foreach(node serial_range_node velocity_limiter_node)
  set_target_properties(${node} PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
  target_include_directories(${node} PRIVATE include ${catkin_INCLUDE_DIRS})
  target_link_libraries(${node} ${catkin_LIBRARIES})
  add_dependencies(${node} ${catkin_EXPORTED_TARGETS})
endforeach()

if(quadrotor_msgs_FOUND)
  target_compile_definitions(velocity_limiter_node PRIVATE TOF_HAS_POSITION_COMMAND)
  target_include_directories(velocity_limiter_node PRIVATE ${quadrotor_msgs_INCLUDE_DIRS})
  add_dependencies(velocity_limiter_node ${quadrotor_msgs_EXPORTED_TARGETS})
else()
  message(STATUS "quadrotor_msgs not found: velocity limiter will support TwistStamped input only")
endif()

install(TARGETS serial_range_node velocity_limiter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
