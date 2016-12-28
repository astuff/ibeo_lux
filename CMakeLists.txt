cmake_minimum_required(VERSION 2.8.3)
project(ros_ibeo_lux)

add_definitions(-std=c++11 -Wall -Wextra -DBUILD_ROS)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        message_generation
        std_msgs
        genmsg
        as_generic_msg
        pcl_ros
        geometry_msgs
        sensor_msgs
        visualization_msgs
        )
## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES
        # Tcp/Ip Tx
        lux_scan_data.msg
        scan_point.msg
        lux_object_data.msg
        object_list.msg
        point2D.msg
        resolution.msg
        size2D.msg
        float2D.msg
        lux_vehicle_state.msg
        test_point.msg
        lux_header.msg
        lux_error_warning.msg
        lux_fusion_scan_data.msg
        lux_fusion_scan_info.msg
        lux_fusion_scan_point.msg
        lux_fusion_scan_info_2205.msg
        lux_fusion_scan_data_2205.msg
        lux_fusion_object_data_2225.msg
        object_list_2225.msg
        lux_fusion_object_data_2280.msg
        object_list_2280.msg
        lux_fusion_img_2403.msg
        mount_position.msg
        lux_fusion_vehicle_state.msg
        )

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS
        message_runtime
        std_msgs
        as_generic_msg
        pcl_ros
        geometry_msgs
        sensor_msgs
        visualization_msgs
        )

## Build executables
include_directories(
        core_ibeo_lux/include
        ${catkin_INCLUDE_DIRS}
)

add_executable(ros_ibeo_lux
        src/ros_ibeo_lux.cpp
        core_ibeo_lux/src/TCPMsg.cpp
        )


target_link_libraries(ros_ibeo_lux ${catkin_LIBRARIES})
add_dependencies(ros_ibeo_lux ros_ibeo_lux_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

#target_link_libraries(as_delphi_vehicle_pub ${catkin_LIBRARIES})
#add_dependencies(as_delphi_vehicle_pub ${catkin_EXPORTED_TARGETS})