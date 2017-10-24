cmake_minimum_required(VERSION 2.8.3)
project(ibeo_lux)

add_definitions(-std=c++11 -Wall -Wextra -DBUILD_ROS)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  network_interface
  roscpp
  std_msgs
  pcl_ros
  geometry_msgs
  visualization_msgs
  ibeo_msgs
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS
  network_interface
  std_msgs
  pcl_ros
  geometry_msgs
  visualization_msgs
  ibeo_msgs
)

## Build executables
include_directories(
  include/${PROJECT_NAME}
  core_ibeo/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  core_ibeo/src/ibeo_core.cpp
  src/ros_ibeo_lux.cpp
  src/ibeo_lux_ros_msg_handler.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
                        
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
