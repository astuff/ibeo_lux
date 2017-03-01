cmake_minimum_required(VERSION 2.8.3)
project(ibeo_lux)

add_definitions(-std=c++11 -Wall -Wextra -DBUILD_ROS)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  network_interface
  roscpp
  message_generation
  std_msgs
  genmsg
  pcl_ros
  geometry_msgs
  visualization_msgs
)
## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES
  Point2D.msg
  Size2D.msg
  Float2D.msg
  Resolution.msg
  MountPosition.msg
  ScanPoint.msg
  LuxObject.msg
  LuxHeader.msg
  LuxScanData.msg
  LuxObjectData.msg
  LuxVehicleState.msg
  LuxErrorWarning.msg
  FusionScanPoint.msg
  FusionScanInfo2204.msg
  FusionScanData2204.msg
  FusionScanInfo2205.msg
  FusionScanData2205.msg
  FusionObject2225.msg
  FusionObjectData2225.msg
  FusionObjectData2280.msg
  FusionObject2280.msg
  FusionImage.msg
  FusionVehicleState2806.msg
  FusionVehicleState2807.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS
  network_interface
  message_runtime
  std_msgs
  pcl_ros
  geometry_msgs
  visualization_msgs
)

## Build executables
include_directories(
  core_ibeo_lux/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ibeo_lux
  core_ibeo_lux/src/core_ibeo_lux.cpp
  src/ros_ibeo_lux.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
                        
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

