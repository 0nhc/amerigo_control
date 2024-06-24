cmake_minimum_required(VERSION 3.0.2)
project(amerigo_control)

find_package(catkin 
  REQUIRED
  COMPONENTS
  roscpp
  rospy
  sensor_msgs
  controller_manager
  amerigo_description
  amerigo_msgs
  dynamixel_sdk
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lk_ros
  CATKIN_DEPENDS 
  roscpp
  rospy
  sensor_msgs
  controller_manager
  amerigo_description
  amerigo_msgs
  dynamixel_sdk
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(amerigo_hardware_interface src/ros_interface/amerigo_hardware_interface.cpp)

target_link_libraries(amerigo_hardware_interface ${catkin_LIBRARIES})
add_executable(amerigo_hardware_interface_node src/ros_nodes/amerigo_hardware_interface_node.cpp)
target_link_libraries(amerigo_hardware_interface_node amerigo_hardware_interface ${catkin_LIBRARIES})

add_executable(dynamixel_node src/dynamixel_interface/dynamixel_node.cpp)
target_link_libraries(dynamixel_node ${catkin_LIBRARIES})

install(TARGETS 
  amerigo_hardware_interface
  amerigo_hardware_interface_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
