/*
* AutonomouStuff, LLC ("COMPANY") CONFIDENTIAL
* Unpublished Copyright (c) 2009-2016 AutonomouStuff, LLC, All Rights Reserved.
*
* NOTICE:  All information contained herein is, and remains the property of COMPANY. The intellectual and technical concepts contained
* herein are proprietary to COMPANY and may be covered by U.S. and Foreign Patents, patents in process, and are protected by trade secret or copyright law.
* Dissemination of this information or reproduction of this material is strictly forbidden unless prior written permission is obtained
* from COMPANY.  Access to the source code contained herein is hereby forbidden to anyone except current COMPANY employees, managers or contractors who have executed
* Confidentiality and Non-disclosure agreements explicitly covering such access.
*
* The copyright notice above does not evidence any actual or intended publication or disclosure  of  this source code, which includes
* information that is confidential and/or proprietary, and is a trade secret, of  COMPANY.   ANY REPRODUCTION, MODIFICATION, DISTRIBUTION, PUBLIC  PERFORMANCE,
* OR PUBLIC DISPLAY OF OR THROUGH USE  OF THIS  SOURCE CODE  WITHOUT  THE EXPRESS WRITTEN CONSENT OF COMPANY IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE
* LAWS AND INTERNATIONAL TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS
* TO REPRODUCE, DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR SELL ANYTHING THAT IT  MAY DESCRIBE, IN WHOLE OR IN PART.
*/
// Sys
#include <std_msgs/String.h>
#include <iostream>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <string.h>
#include <boost/program_options.hpp>
#include <boost/asio.hpp>
#include <stdio.h>


#include <as_ibeo_lux.hpp>
//#include <TCPMsg.hpp>

//Ros
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


//Tx
#include <ros_ibeo_lux/lux_scan_data.h>
#include <ros_ibeo_lux/scan_point.h>
#include <ros_ibeo_lux/lux_object_data.h>
#include <ros_ibeo_lux/point2D.h>
#include <ros_ibeo_lux/size2D.h>
#include <ros_ibeo_lux/float2D.h>
#include <ros_ibeo_lux/lux_vehicle_state.h>
#include <ros_ibeo_lux/test_point.h>
#include <ros_ibeo_lux/lux_header.h>
#include <ros_ibeo_lux/lux_error_warning.h>
#include <ros_ibeo_lux/lux_fusion_scan_data.h>
#include <ros_ibeo_lux/lux_fusion_scan_info.h>
#include <ros_ibeo_lux/lux_fusion_scan_point.h>
#include <ros_ibeo_lux/lux_fusion_scan_data_2205.h>
#include <ros_ibeo_lux/lux_fusion_scan_info_2205.h>
#include <ros_ibeo_lux/resolution.h>
#include <ros_ibeo_lux/lux_fusion_object_data_2225.h>
#include <ros_ibeo_lux/object_list_2225.h>
#include <ros_ibeo_lux/lux_fusion_object_data_2280.h>
#include <ros_ibeo_lux/object_list_2280.h>
#include <ros_ibeo_lux/lux_fusion_img_2403.h>
#include <ros_ibeo_lux/mount_position.h>
#include <ros_ibeo_lux/lux_fusion_vehicle_state.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>


using namespace std;
using namespace boost;
using boost::asio::ip::tcp;
namespace po = boost::program_options;

// Main routine
int main(int argc, char **argv)
{
    //int c;
    string ip = "192.168.0.1";
    unsigned long port = 12002;
    string frame_id = "ibeo_lux";


    // ROS initialization
    ros::init(argc, argv, "ros_ibeo_lux");
    ros::NodeHandle n;
    ros::NodeHandle priv("~");
    ros::Rate loop_rate(1.0/0.01);

    // Advertise messages to send
    ros::Publisher scan_data_pub = n.advertise<ros_ibeo_lux::lux_scan_data>("lux_scan_data_2202", 1);
    ros::Publisher object_data_pub = n.advertise<ros_ibeo_lux::lux_object_data>("lux_object_data_2221", 1);
    ros::Publisher vehicle_state_pub = n.advertise<ros_ibeo_lux::lux_vehicle_state>("lux_vehicle_state_2805", 1);
    ros::Publisher error_warn_pub = n.advertise<ros_ibeo_lux::lux_error_warning>("lux_error_warning_2030", 1);
    ros::Publisher fusion_scan_2204_pub = n.advertise<ros_ibeo_lux::lux_fusion_scan_data>("lux_fusion_scan_2204", 1);
    ros::Publisher fusion_scan_2205_pub = n.advertise<ros_ibeo_lux::lux_fusion_scan_data_2205>("lux_fusion_scan_2205", 1);
    ros::Publisher fusion_object_2225_pub = n.advertise<ros_ibeo_lux::lux_fusion_object_data_2225>("lux_fusion_object_2225", 1);
    ros::Publisher fusion_object_2280_pub = n.advertise<ros_ibeo_lux::lux_fusion_object_data_2280>("lux_fusion_object_2280", 1);
    ros::Publisher fusion_img_2403_pub = n.advertise<ros_ibeo_lux::lux_fusion_img_2403>("lux_fusion_img_2403", 1);
    ros::Publisher fusion_vehicle_2806_pub = n.advertise<ros_ibeo_lux::lux_fusion_vehicle_state>("lux_fusion_vehicle_state_2806", 1);
    ros::Publisher fusion_vehicle_2807_pub = n.advertise<ros_ibeo_lux::lux_fusion_vehicle_state>("lux_fusion_vehicle_state_2807", 1);


    ros::Publisher scan_pointcloud2_pub = n.advertise<pcl::PointCloud <pcl::PointXYZ> >("lux_point_cloud2", 1);
    //ros::Publisher scan_pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("lux_point_cloud", 1);
    ros::Publisher object_visual_pub = n.advertise<visualization_msgs::MarkerArray>("object_markers", 1);
    //ros::Publisher point_test_pub = n.advertise<ros_ibeo_lux::test_point>("lux_test_points", 1);

    // Wait for time to be valid
    while (ros::Time::now().nsec == 0);

    ros_ibeo_lux::lux_header           lux_header_msg;
    //ros_ibeo_lux::test_point           test_point_list;

    //geometry_msgs::Point      test_point;
    //sensor_msgs::PointCloud   lux_point_cloud;
    geometry_msgs::Point32    point_cloud_data;


    TCPMsg    header_msg;
    header_msg.msgOffset = 0;
    header_msg.size = 4;
    header_msg.factor = 1;
    header_msg.valueOffset = 0;

    int    data_size;
    int    data_type;
    int    start_byte;
    int    device_id;
    char   data_type_hex[10];

    stringstream sPort;
    sPort << port;


    asio::io_service io;
    tcp::resolver res(io);
    tcp::resolver::query query(tcp::v4(), ip.c_str(), sPort.str());
    tcp::resolver::iterator it = res.resolve(query);
    tcp::socket socket(io);

    try {
        socket.connect(*it);
        ROS_INFO("LUX connected");
        // Loop as long as module should run
        while (ros::ok())
        {
            // Get current time
            ros::Time now = ros::Time::now();
            //double nowSec = now.toSec();

            std::array<unsigned char, LUX_PAYLOAD_SIZE> msgBuf;
            system::error_code error;
            unsigned long first_four_bytes;

            //memset(&msgBuf[0],0,LUX_PAYLOAD_SIZE);

            int rcvSize = socket.read_some(asio::buffer(msgBuf), error);
            
            bool package_rcvd = false;
            int i = 0;
            while (!package_rcvd)
            {
                header_msg.size = 4;
                header_msg.msgOffset = i;
                first_four_bytes = (uint32_t)read_big_endian(msgBuf, header_msg);
                if (first_four_bytes == magicWord)
                {
                    //ROS_INFO("Found the header msg");
                    header_msg.msgOffset = 8;
                    data_size = (uint32_t)read_big_endian(msgBuf, header_msg);
                    header_msg.msgOffset = 13;
                    device_id = (uint8_t)read_big_endian(msgBuf, header_msg);
                    header_msg.msgOffset = 14;
                    header_msg.size = 2;
                    data_type = (uint16_t)read_big_endian(msgBuf, header_msg);
                    start_byte = i;
                    package_rcvd = true;
                    sprintf(data_type_hex, "%x", data_type);

                    ROS_INFO("Found Data Type %x", data_type);
                    //ROS_INFO("start byte %f", i);
                    //ROS_INFO("package received %d", package_rcvd);
                }
                i = i+1;
            }

            if (package_rcvd)
            {
                //int count = 0;
                // ibeo LUX scan data
                start_byte = start_byte + LUX_MESSAGE_DATA_OFFSET;
                if (data_type == 0x2202)
                {
                    ROS_INFO("reading scan data 0x2202");
                    TCPMsg   scan_data;
                    ros_ibeo_lux::lux_scan_data        lux_scan_msg;
                    //publish the point cloud
                    //sensor_msgs::PointCloud2  point_cloud_msg;
                    pcl::PointCloud <pcl::PointXYZL> pcl_cloud;
                    pcl::PointXYZL cloud_point;

                    scan_data.size = 2;
                    scan_data.msgOffset = start_byte;
                    scan_data.factor = 1;
                    scan_data.valueOffset = 0;
                    lux_scan_msg.scan_number = (uint16_t)read_little_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 2;
                    lux_scan_msg.scan_status = (uint16_t)read_little_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte  + 4;
                    lux_scan_msg.sync_phase_offset = (uint16_t)read_little_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte  + 6;
                    scan_data.size = 8;
                    lux_scan_msg.scan_start_time = (uint64_t)read_little_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte  + 14;
                    lux_scan_msg.scan_end_time = (uint64_t)read_little_endian(msgBuf, scan_data);
                    scan_data.size = 2;
                    scan_data.msgOffset = start_byte  + 22;
                    lux_scan_msg.angle_ticks = (uint16_t)read_little_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 24;
                    lux_scan_msg.start_angle = (int16_t)read_little_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 26;
                    lux_scan_msg.end_angle = (int16_t)read_little_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 28;
                    lux_scan_msg.num_scan_pts = (uint16_t)read_little_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 30;
                    lux_scan_msg.mounting_yaw_angle = (int16_t)read_little_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 32;
                    lux_scan_msg.mounting_pitch_angle = (int16_t)read_little_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 34;
                    lux_scan_msg.mounting_roll_angle = (int16_t)read_little_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 36;
                    lux_scan_msg.mounting_position_x = (int16_t)read_little_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 38;
                    lux_scan_msg.mounting_position_y = (int16_t)read_little_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 40;
                    lux_scan_msg.mounting_position_z = (int16_t)read_little_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 42;
                    lux_scan_msg.scan_flags = (int16_t)read_little_endian(msgBuf, scan_data);

                    pcl_cloud.reserve(lux_scan_msg.num_scan_pts);

                    ros_ibeo_lux::scan_point scan_point_data;
                    //scan_data.msgOffset = start_byte + 44;
                    int    pt_start_byte = start_byte + 44;
                    //scan_data.msgOffset = pt_start_byte;
                    //scan_data.size = 1;
                    for(int k = 0; k < lux_scan_msg.num_scan_pts; k++)
                    {
                        //scan_data.msgOffset = scan_data.msgOffset + 10*k;
                        scan_point_data.layer = (uint8_t)(msgBuf[pt_start_byte] & 0x0F);
                        scan_point_data.echo = (uint8_t)((msgBuf[pt_start_byte] >> 4) & 0x0F);
                        scan_data.msgOffset = pt_start_byte + 1;
                        scan_point_data.flags = (uint8_t)msgBuf[scan_data.msgOffset];
                        scan_data.msgOffset = pt_start_byte + 2;
                        scan_data.size = 2;
                        scan_point_data.horizontal_angle = (int16_t)read_little_endian(msgBuf, scan_data);
                        scan_data.msgOffset = pt_start_byte + 4;
                        scan_point_data.radial_distance = (uint16_t)read_little_endian(msgBuf, scan_data);
                        scan_data.msgOffset = pt_start_byte + 6;
                        scan_point_data.echo_pulse_width = (uint16_t)read_little_endian(msgBuf, scan_data);
                        pt_start_byte = pt_start_byte + 10;
                        //scan_data.msgOffset = pt_start_byte;
                        lux_scan_msg.scan_points.push_back(scan_point_data);
                        //point cloud
                        cloud_point.label = scan_point_data.flags;
                        double phi;
                        switch (scan_point_data.layer)
                        {
                            case 0: phi = -1.6 * PI / 180.0; break;
                            case 1: phi = -0.8 * PI / 180.0; break;
                            case 2: phi =  0.8 * PI / 180.0; break;
                            case 3: phi =  1.6 * PI / 180.0; break;
                            default: phi = 0.0; break;
                        }

                        cloud_point.x = 0.01*(double)scan_point_data.radial_distance*cos(convertAngle(scan_point_data.horizontal_angle,lux_scan_msg.angle_ticks))*cos(phi);
                        cloud_point.y = 0.01*(double)scan_point_data.radial_distance*sin(convertAngle(scan_point_data.horizontal_angle,lux_scan_msg.angle_ticks))*cos(phi);
                        cloud_point.z = 0.01*(double)scan_point_data.radial_distance*sin(phi);

                        pcl_cloud.points.push_back(cloud_point);

                        //test_point.x =  (float)cloud_point.x;
                        //test_point.y =  (float)cloud_point.y;
                        //test_point.z =  (float)cloud_point.z;

                        //test_point_list.point.push_back(test_point);

                        //point_cloud_data.x = (float)cloud_point.x;
                        //point_cloud_data.y = (float)cloud_point.y;
                        //point_cloud_data.z = (float)cloud_point.z;
                        //lux_point_cloud.points.push_back(point_cloud_data);
                    }

                    lux_header_msg.header.stamp = now;
                    lux_header_msg.header.frame_id = frame_id;
                    lux_header_msg.message_size = data_size;
                    lux_header_msg.device_id = device_id;
                    lux_header_msg.data_type = data_type_hex;

                    lux_scan_msg.header = lux_header_msg;
                    scan_data_pub.publish(lux_scan_msg);
                    lux_scan_msg.scan_points.clear();

                    //pcl_cloud.header.stamp = now;
                    pcl_cloud.header.frame_id = frame_id;
                    scan_pointcloud2_pub.publish(pcl_cloud);

                    //point_test_pub.publish(test_point_list);
                    //lux_point_cloud.header.stamp = now;
                    //lux_point_cloud.header.frame_id = frame_id;
                    //scan_pointcloud_pub.publish(lux_point_cloud);
                }


                // ibeo LUX object data
                if (data_type == 0x2221)
                {
                    ROS_INFO("reading object data 0x2221");
                    TCPMsg   object_data;
                    ros_ibeo_lux::lux_object_data      lux_object_msg;
                    visualization_msgs::MarkerArray   object_markers;
                    ros::Duration lifetime(0.1);

                    object_data.size = 8;
                    object_data.msgOffset = start_byte;
                    object_data.factor = 1;
                    object_data.valueOffset = 0;
                    lux_object_msg.scan_start_time = (uint64_t)read_little_endian(msgBuf, object_data);
                    object_data.msgOffset = start_byte + 8;
                    object_data.size = 2;
                    lux_object_msg.num_of_objects = (uint16_t)read_little_endian(msgBuf, object_data);

                    ros_ibeo_lux::object_list scan_object;
                    ros_ibeo_lux::point2D     object_point;
                    int    object_start_byte = start_byte + 10;
                    object_data.msgOffset = object_start_byte;

                    for(int k = 0; k < lux_object_msg.num_of_objects; k++)
                    {
                        scan_object.ID = (uint16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 2;
                        scan_object.age = (uint16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 4;
                        scan_object.prediction_age = (uint16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 6;
                        scan_object.relative_timestamp = (uint16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 8;
                        scan_object.reference_point.x = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 10;
                        scan_object.reference_point.y = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 12;
                        scan_object.reference_point_sigma.x = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 14;
                        scan_object.reference_point_sigma.y = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 16;
                        scan_object.closest_point.x = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 18;
                        scan_object.closest_point.y = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 20;
                        scan_object.bounding_box_center.x = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 22;
                        scan_object.bounding_box_center.y = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 24;
                        scan_object.bounding_box_width = (uint16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 26;
                        scan_object.bounding_box_length = (uint16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 28;
                        scan_object.object_box_center.x = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 30;
                        scan_object.object_box_center.y = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 32;
                        scan_object.object_box_size.x = (uint16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 34;
                        scan_object.object_box_size.y = (uint16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 36;
                        scan_object.object_box_orientation = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 38;
                        scan_object.absolute_velocity.x = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 40;
                        scan_object.absolute_velocity.y = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 42;
                        scan_object.absolute_velocity_sigma.x = (uint16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 44;
                        scan_object.absolute_velocity_sigma.y = (uint16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 46;
                        scan_object.relative_velocity.x = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 48;
                        scan_object.relative_velocity.y = (int16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 50;
                        scan_object.classification = (uint16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 52;
                        scan_object.classification_age = (uint16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 54;
                        scan_object.classification_certaiinty = (uint16_t)read_little_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 56;
                        scan_object.number_of_contour_points = (uint16_t)read_little_endian(msgBuf, object_data);

                        visualization_msgs::Marker   object_marker;
                        object_marker.header.frame_id = frame_id;
                        object_marker.header.stamp = now;
                        object_marker.id  = scan_object.ID;
                        object_marker.ns = "object_contour";
                        object_marker.type = visualization_msgs::Marker::LINE_STRIP;
                        object_marker.action = 0;
                        object_marker.scale.x = object_marker.scale.y = object_marker.scale.z = 1;
                        object_marker.lifetime = lifetime;
                        object_marker.color.a = 0.5;
                        object_marker.color.r = object_marker.color.g = object_marker.color.b = 1.0;
                        std::string   label;
                        switch (scan_object.classification)
                        {
                            case 0:
                                label = "Unclassified";
                                // Unclassified - white
                                break;
                            case 1:
                                label = "Unknown Small";
                                // Unknown small - blue
                                object_marker.color.r = object_marker.color.g = 0;
                                break;
                            case 2:
                                label = "Unknown Big";
                                // Unknown big - dark blue
                                object_marker.color.r = object_marker.color.g = 0;
                                object_marker.color.b = 0.5;
                                break;
                            case 3:
                                label = "Pedestrian";
                                // Pedestrian - red
                                object_marker.color.g = object_marker.color.b = 0;
                                break;
                            case 4:
                                label = "Bike";
                                // Bike - dark red
                                object_marker.color.g = object_marker.color.b = 0;
                                object_marker.color.r = 0.5;
                                break;
                            case 5:
                                label = "Car";
                                // Car - green
                                object_marker.color.b = object_marker.color.r = 0;
                                break;
                            case 6:
                                label = "Truck";
                                // Truck - dark gree
                                object_marker.color.b = object_marker.color.r = 0;
                                object_marker.color.g = 0.5;
                                break;
                            case 12:
                                label = "Under drivable";
                                // Under drivable - grey
                                object_marker.color.r = object_marker.color.b = object_marker.color.g = 0.7;
                                break;
                            case 13:
                                label = "Over drivable";
                                // Over drivable - dark grey
                                object_marker.color.r = object_marker.color.b = object_marker.color.g = 0.4;
                                break;
                            default:
                                label = "Unknown";
                                object_marker.color.r = object_marker.color.b = object_marker.color.g = 0.0;
                                break;
                        }

                        int    pt_start_byte = object_start_byte + 58;
                        object_data.msgOffset = pt_start_byte;

                        for(int j =0; j< scan_object.number_of_contour_points; j++)
                        {
                            geometry_msgs::Point    dis_object_point;
                            object_point.x = (int16_t)read_little_endian(msgBuf, object_data);
                            object_data.msgOffset = pt_start_byte + 2;
                            object_point.y = (int16_t)read_little_endian(msgBuf, object_data);
                            scan_object.list_of_contour_points.push_back(object_point);
                            object_data.msgOffset = pt_start_byte + 4;

                            dis_object_point.x = object_point.x;
                            dis_object_point.y = object_point.y;
                            object_marker.points.push_back(dis_object_point);
                        }
                        object_start_byte = pt_start_byte + 4*scan_object.number_of_contour_points;
                        object_data.msgOffset = object_start_byte;

                        object_markers.markers.push_back(object_marker);

                        /*visualization_msgs::Marker   text_label;

                        text_label.ns = "classification_label";
                        text_label.header.frame_id = frame_id;
                        text_label.header.stamp = object_marker.header.stamp;
                        */

                        lux_object_msg.object.push_back(scan_object);
                        scan_object.list_of_contour_points.clear();
                    }

                    lux_header_msg.header.stamp = now;
                    lux_header_msg.header.frame_id = frame_id;
                    lux_header_msg.message_size = data_size;
                    lux_header_msg.device_id = device_id;
                    lux_header_msg.data_type = data_type_hex;

                    lux_object_msg.header = lux_header_msg;
                    object_data_pub.publish(lux_object_msg);
                    lux_object_msg.object.clear();

                    object_visual_pub.publish(object_markers);
                }

                //vehicle state 2805
                if (data_type == 0x2805)
                {
                    ROS_INFO("reading vehicle state data 0x2805");
                    TCPMsg   vehicle_state_data;
                    ros_ibeo_lux::lux_vehicle_state    lux_vehicle_state_msg;
                    vehicle_state_data.size = 8;
                    vehicle_state_data.msgOffset = start_byte;
                    vehicle_state_data.factor = 1;
                    vehicle_state_data.valueOffset = 0;
                    lux_vehicle_state_msg.timestamp = (uint64_t)read_little_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.size = 2;
                    vehicle_state_data.msgOffset = start_byte + 8;
                    lux_vehicle_state_msg.scan_number = (uint16_t)read_little_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 10;
                    lux_vehicle_state_msg.error_flags = (uint16_t)read_little_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 12;
                    lux_vehicle_state_msg.longitudinal_velocity = (int16_t)read_little_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 14;
                    lux_vehicle_state_msg.steering_wheel_angle = (int16_t)read_little_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 16;
                    lux_vehicle_state_msg.front_wheel_angle = (int16_t)read_little_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 20;
                    vehicle_state_data.size = 4;
                    lux_vehicle_state_msg.vehicle_x = (int32_t)read_little_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 24;
                    lux_vehicle_state_msg.vehicle_y = (int32_t)read_little_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 28;
                    vehicle_state_data.size = 2;
                    lux_vehicle_state_msg.course_angle = (int16_t)read_little_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 30;
                    lux_vehicle_state_msg.time_difference = (uint16_t)read_little_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 32;
                    lux_vehicle_state_msg.diff_in_x = (int16_t)read_little_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 34;
                    lux_vehicle_state_msg.diff_in_y = (int16_t)read_little_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 36;
                    lux_vehicle_state_msg.diff_in_heading = (int16_t)read_little_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 40;
                    lux_vehicle_state_msg.current_yaw_rate = (int16_t)read_little_endian(msgBuf, vehicle_state_data);

                    lux_header_msg.header.stamp = now;
                    lux_header_msg.header.frame_id = frame_id;
                    lux_header_msg.message_size = data_size;
                    lux_header_msg.device_id = device_id;

                    lux_header_msg.data_type = data_type_hex;

                    lux_vehicle_state_msg.header  = lux_header_msg;
                    vehicle_state_pub.publish(lux_vehicle_state_msg);

                }
                // error and warning
                if (data_type == 0x2030)
                {
                    ROS_INFO("reading lux errors and warnings data 0x2030");
                    TCPMsg   error_warn_data;
                    ros_ibeo_lux::lux_error_warning    lux_error_warning_msg;
                    char     temp_value_string[10];
                    error_warn_data.size = 2;
                    error_warn_data.msgOffset = start_byte;
                    error_warn_data.factor = 1;
                    error_warn_data.valueOffset = 0;
                    sprintf(temp_value_string,"%x",(uint16_t)read_little_endian(msgBuf, error_warn_data));
                    lux_error_warning_msg.error_register1 = temp_value_string;
                    error_warn_data.msgOffset = start_byte + 2;
                    sprintf(temp_value_string,"%x",(uint16_t)read_little_endian(msgBuf, error_warn_data));
                    lux_error_warning_msg.error_register2 = temp_value_string;
                    error_warn_data.msgOffset = start_byte + 4;
                    sprintf(temp_value_string,"%x",(uint16_t)read_little_endian(msgBuf, error_warn_data));
                    lux_error_warning_msg.warning_register1 = temp_value_string;
                    error_warn_data.msgOffset = start_byte + 6;
                    sprintf(temp_value_string,"%x",(uint16_t)read_little_endian(msgBuf, error_warn_data));
                    lux_error_warning_msg.warning_register2 = temp_value_string;



                    lux_header_msg.header.stamp = now;
                    lux_header_msg.header.frame_id = frame_id;
                    lux_header_msg.message_size = data_size;
                    lux_header_msg.device_id = device_id;
                    lux_header_msg.data_type = data_type_hex;

                    lux_error_warning_msg.header  = lux_header_msg;
                    error_warn_pub.publish(lux_error_warning_msg);
                }

                // Fusion scan data 2204
                if (data_type == 0x2204)
                {
                    ROS_INFO("reading FUSION SYSTEM/ECU scan data 0x2204");
                    TCPMsg   scan_data;
                    ros_ibeo_lux::lux_fusion_scan_data           lux_fusion_scan_2204;
                    //publish the point cloud
                    //sensor_msgs::PointCloud2  point_cloud_msg;
                    pcl::PointCloud <pcl::PointXYZL> pcl_cloud;
                    pcl::PointXYZL cloud_point;

                    scan_data.size = 8;
                    scan_data.msgOffset = start_byte;
                    scan_data.factor = 1;
                    scan_data.valueOffset = 0;

                    lux_fusion_scan_2204.scan_start_time = (uint64_t)read_big_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 8;
                    scan_data.size = 4;
                    lux_fusion_scan_2204.scan_end_time_offset = (uint32_t)read_big_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte  + 12;
                    lux_fusion_scan_2204.flags = (uint32_t)read_big_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte  + 16;
                    scan_data.size = 2;
                    lux_fusion_scan_2204.scan_number = (uint16_t)read_big_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte  + 18;
                    lux_fusion_scan_2204.num_scan_pts = (uint16_t)read_big_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte  + 20;
                    scan_data.size = 1;
                    lux_fusion_scan_2204.num_scan_info = (uint8_t)read_big_endian(msgBuf, scan_data);

                    ros_ibeo_lux::lux_fusion_scan_info   scan_info_2204;
                    ros_ibeo_lux::lux_fusion_scan_point  scan_point_2204;
                    //scan_data.msgOffset = start_byte + 44;
                    int    info_start_byte = start_byte + 24;
                    //scan_data.msgOffset = pt_start_byte;

                    for(int k = 0; k < lux_fusion_scan_2204.num_scan_info; k++)
                    {
                        //scan_data.msgOffset = scan_data.msgOffset + 10*k;
                        scan_info_2204.device_id = (uint8_t)msgBuf[info_start_byte];
                        scan_info_2204.type = (uint8_t)msgBuf[info_start_byte + 1];
                        scan_data.msgOffset = info_start_byte + 2;
                        scan_data.size = 2;
                        scan_info_2204.number = (uint16_t)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 8;
                        scan_data.size = 4;
                        scan_info_2204.start_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 12;
                        scan_info_2204.end_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 16;
                        scan_info_2204.yaw_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 20;
                        scan_info_2204.pitch_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 24;
                        scan_info_2204.roll_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 28;
                        scan_info_2204.mount_offset_x = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 32;
                        scan_info_2204.mount_offset_y = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 36;
                        scan_info_2204.mount_offset_z = (float)read_big_endian(msgBuf, scan_data);
                        info_start_byte = info_start_byte + 40;

                        lux_fusion_scan_2204.scan_info_list.push_back(scan_info_2204);
                    }

                    int    pt_start_byte = start_byte + 24 + lux_fusion_scan_2204.num_scan_info *40;
                    scan_data.msgOffset = pt_start_byte;
                    scan_data.size = 4;
                    for(int k = 0; k < lux_fusion_scan_2204.num_scan_pts; k++)
                    {
                        //scan_data.msgOffset = scan_data.msgOffset + 10*k;
                        scan_point_2204.x_position = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = pt_start_byte + 4;
                        scan_point_2204.y_position = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = pt_start_byte + 8;
                        scan_point_2204.z_position = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = pt_start_byte + 12;
                        scan_point_2204.echo_width = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = pt_start_byte + 16;
                        scan_point_2204.device_id = (uint8_t)msgBuf[scan_data.msgOffset];
                        scan_data.msgOffset = pt_start_byte + 17;
                        scan_point_2204.layer = (uint8_t)msgBuf[scan_data.msgOffset];
                        scan_data.msgOffset = pt_start_byte + 18;
                        scan_point_2204.echo = (uint8_t)msgBuf[scan_data.msgOffset];
                        scan_data.msgOffset = pt_start_byte + 20;
                        scan_data.size = 2;
                        scan_point_2204.time_stamp = (uint32_t)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = pt_start_byte + 24;
                        scan_data.size = 2;
                        scan_point_2204.flags = (uint16_t)read_big_endian(msgBuf, scan_data);


                        pt_start_byte = pt_start_byte + 28;
                        lux_fusion_scan_2204.scan_point_list.push_back(scan_point_2204);
                    }

                    lux_header_msg.header.stamp = now;
                    lux_header_msg.header.frame_id = frame_id;
                    lux_header_msg.message_size = data_size;
                    lux_header_msg.device_id = device_id;
                    lux_header_msg.data_type = data_type_hex;

                    lux_fusion_scan_2204.header = lux_header_msg;
                    fusion_scan_2204_pub.publish(lux_fusion_scan_2204);
                }
                // Fusion scan data 2205
                if (data_type == 0x2205)
                {
                    ROS_INFO("reading FUSION SYSTEM/ECU scan data 0x2205");
                    TCPMsg   scan_data;
                    ros_ibeo_lux::lux_fusion_scan_data_2205      lux_fusion_scan_2205;

                    scan_data.size = 8;
                    scan_data.msgOffset = start_byte;
                    scan_data.factor = 1;
                    scan_data.valueOffset = 0;

                    lux_fusion_scan_2205.scan_start_time = (uint64_t)read_big_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 8;
                    scan_data.size = 4;
                    lux_fusion_scan_2205.scan_end_time_offset = (uint32_t)read_big_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte  + 12;
                    lux_fusion_scan_2205.flags = (uint32_t)read_big_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte  + 16;
                    scan_data.size = 2;
                    lux_fusion_scan_2205.scan_number = (uint16_t)read_big_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte  + 18;
                    lux_fusion_scan_2205.num_scan_pts = (uint16_t)read_big_endian(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte  + 20;
                    scan_data.size = 1;
                    lux_fusion_scan_2205.num_scan_info = (uint8_t)read_big_endian(msgBuf, scan_data);

                    ros_ibeo_lux::lux_fusion_scan_info_2205   scan_info_2205;
                    ros_ibeo_lux::lux_fusion_scan_point  scan_point_2205;
                    //scan_data.msgOffset = start_byte + 44;
                    int    info_start_byte = start_byte + 24;
                    //scan_data.msgOffset = pt_start_byte;

                    for(int k = 0; k < lux_fusion_scan_2205.num_scan_info; k++)
                    {
                        //scan_data.msgOffset = scan_data.msgOffset + 10*k;
                        scan_info_2205.device_id = (uint8_t)msgBuf[info_start_byte];
                        scan_info_2205.type = (uint8_t)msgBuf[info_start_byte + 1];
                        scan_data.msgOffset = info_start_byte + 2;
                        scan_data.size = 2;
                        scan_info_2205.number = (uint16_t)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 8;
                        scan_data.size = 4;
                        scan_info_2205.start_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 12;
                        scan_info_2205.end_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 16;
                        scan_data.size = 8;
                        scan_info_2205.scan_start_time = (uint64_t)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 24;
                        scan_info_2205.scan_end_time = (uint64_t)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 32;
                        scan_info_2205.device_start_time = (uint64_t)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 40;
                        scan_info_2205.device_end_time = (uint64_t)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 48;
                        scan_data.size = 4;
                        scan_info_2205.scan_frequency = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 52;
                        scan_info_2205.beam_tilt = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 56;
                        scan_info_2205.scan_flags = (uint32_t)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 60;
                        scan_info_2205.yaw_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 64;
                        scan_info_2205.pitch_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 68;
                        scan_info_2205.roll_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 72;
                        scan_info_2205.offset_x = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 76;
                        scan_info_2205.offset_y = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 80;
                        scan_info_2205.offset_z = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 84;
                        scan_info_2205.resolution1.resolution_start_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 88;
                        scan_info_2205.resolution1.resolution = (float)read_big_endian(msgBuf, scan_data);

                        scan_data.msgOffset = info_start_byte + 92;
                        scan_info_2205.resolution2.resolution_start_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 96;
                        scan_info_2205.resolution2.resolution = (float)read_big_endian(msgBuf, scan_data);

                        scan_data.msgOffset = info_start_byte + 100;
                        scan_info_2205.resolution3.resolution_start_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 104;
                        scan_info_2205.resolution3.resolution = (float)read_big_endian(msgBuf, scan_data);

                        scan_data.msgOffset = info_start_byte + 108;
                        scan_info_2205.resolution4.resolution_start_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 112;
                        scan_info_2205.resolution4.resolution = (float)read_big_endian(msgBuf, scan_data);

                        scan_data.msgOffset = info_start_byte + 116;
                        scan_info_2205.resolution5.resolution_start_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 120;
                        scan_info_2205.resolution5.resolution = (float)read_big_endian(msgBuf, scan_data);

                        scan_data.msgOffset = info_start_byte + 124;
                        scan_info_2205.resolution6.resolution_start_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 128;
                        scan_info_2205.resolution6.resolution = (float)read_big_endian(msgBuf, scan_data);

                        scan_data.msgOffset = info_start_byte + 132;
                        scan_info_2205.resolution7.resolution_start_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 136;
                        scan_info_2205.resolution7.resolution = (float)read_big_endian(msgBuf, scan_data);

                        scan_data.msgOffset = info_start_byte + 140;
                        scan_info_2205.resolution8.resolution_start_angle = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = info_start_byte + 144;
                        scan_info_2205.resolution8.resolution = (float)read_big_endian(msgBuf, scan_data);


                        info_start_byte = info_start_byte + 148;

                        lux_fusion_scan_2205.scan_info_list.push_back(scan_info_2205);
                    }

                    int    pt_start_byte = start_byte + 24 + lux_fusion_scan_2205.num_scan_info *148;
                    scan_data.msgOffset = pt_start_byte;
                    scan_data.size = 4;
                    for(int k = 0; k < lux_fusion_scan_2205.num_scan_pts; k++)
                    {
                        //scan_data.msgOffset = scan_data.msgOffset + 10*k;
                        scan_point_2205.x_position = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = pt_start_byte + 4;
                        scan_point_2205.y_position = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = pt_start_byte + 8;
                        scan_point_2205.z_position = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = pt_start_byte + 12;
                        scan_point_2205.echo_width = (float)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = pt_start_byte + 16;
                        scan_point_2205.device_id = (uint8_t)msgBuf[scan_data.msgOffset];
                        scan_data.msgOffset = pt_start_byte + 17;
                        scan_point_2205.layer = (uint8_t)msgBuf[scan_data.msgOffset];
                        scan_data.msgOffset = pt_start_byte + 18;
                        scan_point_2205.echo = (uint8_t)msgBuf[scan_data.msgOffset];
                        scan_data.msgOffset = pt_start_byte + 20;
                        scan_data.size = 2;
                        scan_point_2205.time_stamp = (uint32_t)read_big_endian(msgBuf, scan_data);
                        scan_data.msgOffset = pt_start_byte + 24;
                        scan_data.size = 2;
                        scan_point_2205.flags = (uint16_t)read_big_endian(msgBuf, scan_data);


                        pt_start_byte = pt_start_byte + 28;
                        lux_fusion_scan_2205.scan_point_list.push_back(scan_point_2205);
                    }

                    lux_header_msg.header.stamp = now;
                    lux_header_msg.header.frame_id = frame_id;
                    lux_header_msg.message_size = data_size;
                    lux_header_msg.device_id = device_id;
                    lux_header_msg.data_type = data_type_hex;

                    lux_fusion_scan_2205.header = lux_header_msg;
                    fusion_scan_2205_pub.publish(lux_fusion_scan_2205);
                }
                // Fusion object data 2225
                if (data_type == 0x2225)
                {
                    ROS_INFO("reading Fusion object data 0x2225");
                    TCPMsg   object_data;
                    ros_ibeo_lux::lux_fusion_object_data_2225    lux_fusion_object_2225;

                    object_data.size = 8;
                    object_data.msgOffset = start_byte;
                    object_data.factor = 1;
                    object_data.valueOffset = 0;
                    lux_fusion_object_2225.mid_scan_timestamp = (uint64_t)read_big_endian(msgBuf, object_data);
                    object_data.msgOffset = start_byte + 8;
                    object_data.size = 2;
                    lux_fusion_object_2225.num_of_objects = (uint16_t)read_big_endian(msgBuf, object_data);

                    ros_ibeo_lux::object_list_2225 scan_object;
                    ros_ibeo_lux::float2D     object_point;
                    int    object_start_byte = start_byte + 10;
                    object_data.msgOffset = object_start_byte;

                    for(int k = 0; k < lux_fusion_object_2225.num_of_objects; k++)
                    {
                        scan_object.ID = (uint16_t)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 4;
                        object_data.size = 4;
                        scan_object.object_age = (uint32_t)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 8;
                        object_data.size = 8;
                        scan_object.time_stamp = (uint64_t)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 16;
                        object_data.size = 2;
                        scan_object.object_hidden_age = (uint16_t)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 18;
                        scan_object.classification = (uint8_t)msgBuf[object_data.msgOffset];
                        object_data.msgOffset = object_start_byte + 19;
                        scan_object.classification_certainty = (uint8_t)msgBuf[object_data.msgOffset];
                        object_data.msgOffset = object_start_byte + 20;
                        object_data.size = 4;
                        scan_object.calssifciation_age = (uint32_t)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 24;
                        scan_object.bounding_box_center.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 28;
                        scan_object.bounding_box_center.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 32;
                        scan_object.bounding_box_size.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 36;
                        scan_object.bounding_box_size.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 40;
                        scan_object.object_box_center.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 44;
                        scan_object.object_box_center.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 48;
                        scan_object.object_box_center_sigma.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 52;
                        scan_object.object_box_center_sigma.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 56;
                        scan_object.object_box_size.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 60;
                        scan_object.object_box_size.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 72;
                        scan_object.yaw_angle = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 80;
                        scan_object.relative_velocity.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 84;
                        scan_object.relative_velocity.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 88;
                        scan_object.relative_velocity_sigma.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 92;
                        scan_object.relative_velocity_sigma.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 96;
                        scan_object.absolute_velocity.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 100;
                        scan_object.absolute_velocity.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 104;
                        scan_object.absolute_velocity_sigma.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 108;
                        scan_object.absolute_velocity_sigma.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 130;
                        scan_object.number_of_contour_points = (uint8_t)msgBuf[object_data.msgOffset];
                        object_data.msgOffset = object_start_byte + 131;
                        scan_object.closest_point_index = (uint8_t)msgBuf[object_data.msgOffset];

                        int    pt_start_byte = object_start_byte + 132;
                        object_data.msgOffset = pt_start_byte;
                        object_data.size = 4;

                        for(int j =0; j< scan_object.number_of_contour_points; j++)
                        {
                            object_point.x = (float)read_big_endian(msgBuf, object_data);
                            object_data.msgOffset = pt_start_byte + 4;
                            object_point.y = (float)read_big_endian(msgBuf, object_data);
                            scan_object.list_of_contour_points.push_back(object_point);
                            object_data.msgOffset = pt_start_byte + 4;
                        }
                        object_start_byte = pt_start_byte + 8*scan_object.number_of_contour_points;
                        object_data.msgOffset = object_start_byte;

                        lux_fusion_object_2225.object.push_back(scan_object);
                    }

                    lux_header_msg.header.stamp = now;
                    lux_header_msg.header.frame_id = frame_id;
                    lux_header_msg.message_size = data_size;
                    lux_header_msg.device_id = device_id;
                    lux_header_msg.data_type = data_type_hex;

                    lux_fusion_object_2225.header = lux_header_msg;
                    fusion_object_2225_pub.publish(lux_fusion_object_2225);
                }

                // Fusion object data 2280
                if (data_type == 0x2280)
                {
                    ROS_INFO("reading Fusion object data 0x2280");
                    TCPMsg   object_data;
                    ros_ibeo_lux::lux_fusion_object_data_2280    lux_fusion_object_2280;

                    object_data.size = 8;
                    object_data.msgOffset = start_byte;
                    object_data.factor = 1;
                    object_data.valueOffset = 0;
                    lux_fusion_object_2280.mid_scan_timestamp = (uint64_t)read_big_endian(msgBuf, object_data);
                    object_data.msgOffset = start_byte + 8;
                    object_data.size = 2;
                    lux_fusion_object_2280.num_of_objects = (uint16_t)read_big_endian(msgBuf, object_data);

                    ros_ibeo_lux::object_list_2280 scan_object;
                    ros_ibeo_lux::float2D     object_point;
                    int    object_start_byte = start_byte + 10;
                    object_data.msgOffset = object_start_byte;

                    for(int k = 0; k < lux_fusion_object_2280.num_of_objects; k++)
                    {
                        scan_object.ID = (uint16_t)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 2;
                        scan_object.flags = (uint16_t)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 4;
                        object_data.size = 4;
                        scan_object.object_age = (uint32_t)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 8;
                        object_data.size = 8;
                        scan_object.time_stamp = (uint64_t)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 16;
                        object_data.size = 2;
                        scan_object.object_prediction_age = (uint16_t)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 18;
                        scan_object.classification = (uint8_t)msgBuf[object_data.msgOffset];
                        object_data.msgOffset = object_start_byte + 19;
                        scan_object.classification_quality = (uint8_t)msgBuf[object_data.msgOffset];
                        object_data.msgOffset = object_start_byte + 20;
                        object_data.size = 4;
                        scan_object.calssifciation_age = (uint32_t)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 40;
                        scan_object.object_box_center.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 44;
                        scan_object.object_box_center.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 56;
                        scan_object.object_box_size.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 60;
                        scan_object.object_box_size.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 72;
                        scan_object.object_course_angle = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 76;
                        scan_object.object_course_angle_sigma = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 80;
                        scan_object.relative_velocity.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 84;
                        scan_object.relative_velocity.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 88;
                        scan_object.relative_velocity_sigma.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 92;
                        scan_object.relative_velocity_sigma.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 96;
                        scan_object.absolute_velocity.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 100;
                        scan_object.absolute_velocity.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 104;
                        scan_object.absolute_velocity_sigma.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 108;
                        scan_object.absolute_velocity_sigma.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 130;
                        scan_object.number_of_contour_points = (uint8_t)msgBuf[object_data.msgOffset];
                        object_data.msgOffset = object_start_byte + 131;
                        scan_object.closest_point_index = (uint8_t)msgBuf[object_data.msgOffset];
                        object_data.msgOffset = object_start_byte + 132;
                        object_data.size = 2;
                        scan_object.reference_point_location = (uint16_t)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 134;
                        object_data.size = 4;
                        scan_object.reference_point_coordinate.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 138;
                        scan_object.reference_point_coordinate.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 142;
                        scan_object.reference_point_coordinate_sigma.x = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 146;
                        scan_object.reference_point_coordinate_sigma.y = (float)read_big_endian(msgBuf, object_data);
                        object_data.msgOffset = object_start_byte + 162;
                        object_data.size = 2;
                        scan_object.object_priority = (uint16_t)read_big_endian(msgBuf, object_data);

                        int    pt_start_byte = object_start_byte + 132;
                        object_data.msgOffset = pt_start_byte;
                        object_data.size = 4;

                        for(int j =0; j< scan_object.number_of_contour_points; j++)
                        {
                            object_point.x = (float)read_big_endian(msgBuf, object_data);
                            object_data.msgOffset = pt_start_byte + 4;
                            object_point.y = (float)read_big_endian(msgBuf, object_data);
                            scan_object.list_of_contour_points.push_back(object_point);
                            object_data.msgOffset = pt_start_byte + 4;
                        }
                        object_start_byte = pt_start_byte + 8*scan_object.number_of_contour_points;
                        object_data.msgOffset = object_start_byte;

                        lux_fusion_object_2280.object.push_back(scan_object);
                    }

                    lux_header_msg.header.stamp = now;
                    lux_header_msg.header.frame_id = frame_id;
                    lux_header_msg.message_size = data_size;
                    lux_header_msg.device_id = device_id;
                    lux_header_msg.data_type = data_type_hex;

                    lux_fusion_object_2280.header = lux_header_msg;
                    fusion_object_2280_pub.publish(lux_fusion_object_2280);
                }

                // Fusion image data 2403
                if (data_type == 0x2403)
                {
                    ROS_INFO("reading FUSION SYSTEM/ECU image 2403");
                    TCPMsg   image_data;
                    ros_ibeo_lux::lux_fusion_img_2403           lux_fusion_image_2403;

                    image_data.size = 2;
                    image_data.msgOffset = start_byte;
                    image_data.factor = 1;
                    image_data.valueOffset = 0;

                    lux_fusion_image_2403.image_format = (uint16_t)read_big_endian(msgBuf, image_data);
                    image_data.msgOffset = start_byte + 2;
                    image_data.size = 4;
                    lux_fusion_image_2403.time_stamp = (uint32_t)read_big_endian(msgBuf, image_data);
                    image_data.msgOffset = start_byte  + 6;
                    image_data.size = 8;
                    lux_fusion_image_2403.time_stamp_ntp = (uint64_t)read_big_endian(msgBuf, image_data);
                    image_data.msgOffset = start_byte  + 14;
                    lux_fusion_image_2403.ID =  (uint8_t)msgBuf[image_data.msgOffset];
                    image_data.msgOffset = start_byte  + 15;
                    image_data.size = 4;
                    lux_fusion_image_2403.mouting_position.yaw_angle = (float)read_big_endian(msgBuf, image_data);
                    image_data.msgOffset = start_byte  + 19;
                    lux_fusion_image_2403.mouting_position.pitch_angle = (float)read_big_endian(msgBuf, image_data);
                    image_data.msgOffset = start_byte  + 23;
                    lux_fusion_image_2403.mouting_position.roll_angle = (float)read_big_endian(msgBuf, image_data);
                    image_data.msgOffset = start_byte  + 27;
                    lux_fusion_image_2403.mouting_position.offset_x = (float)read_big_endian(msgBuf, image_data);
                    image_data.msgOffset = start_byte  + 31;
                    lux_fusion_image_2403.mouting_position.offset_y = (float)read_big_endian(msgBuf, image_data);
                    image_data.msgOffset = start_byte  + 35;
                    lux_fusion_image_2403.mouting_position.offset_z = (float)read_big_endian(msgBuf, image_data);
                    image_data.msgOffset = start_byte  + 39;
                    image_data.size = 8;
                    lux_fusion_image_2403.horizontal_opening_angle = read_big_endian(msgBuf, image_data);
                    image_data.msgOffset = start_byte  + 47;
                    lux_fusion_image_2403.vertical_opening_angle = read_big_endian(msgBuf, image_data);
                    image_data.msgOffset = start_byte  + 55;
                    image_data.size = 2;
                    lux_fusion_image_2403.image_width = (uint16_t)read_big_endian(msgBuf, image_data);
                    image_data.msgOffset = start_byte  + 57;
                    lux_fusion_image_2403.image_height = (uint16_t)read_big_endian(msgBuf, image_data);
                    image_data.msgOffset = start_byte  + 59;
                    image_data.size = 4;
                    lux_fusion_image_2403.compress_size = (uint32_t)read_big_endian(msgBuf, image_data);

                    //unsigned char   image_byte;
                    int    image_start_byte = start_byte + 63;

                    for(unsigned int k = 0; k < lux_fusion_image_2403.compress_size; k++)
                    {

                        lux_fusion_image_2403.image_bytes.push_back((uint8_t)msgBuf[image_start_byte + k]);
                    }

                    lux_header_msg.header.stamp = now;
                    lux_header_msg.header.frame_id = frame_id;
                    lux_header_msg.message_size = data_size;
                    lux_header_msg.device_id = device_id;
                    lux_header_msg.data_type = data_type_hex;

                    lux_fusion_image_2403.header = lux_header_msg;
                    fusion_img_2403_pub.publish(lux_fusion_image_2403);
                }


                //Fusion vehicle state 2806
                if (data_type == 0x2806)
                {
                    ROS_INFO("reading Fusion vehicle state data 0x2806");
                    TCPMsg   vehicle_state_data;
                    ros_ibeo_lux::lux_fusion_vehicle_state    lux_vehicle_state_msg;
                    vehicle_state_data.size = 8;
                    vehicle_state_data.msgOffset = start_byte + 4;
                    vehicle_state_data.factor = 1;
                    vehicle_state_data.valueOffset = 0;
                    lux_vehicle_state_msg.time_stamp = (uint64_t)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 12;
                    vehicle_state_data.size = 4;
                    lux_vehicle_state_msg.distance_x = (int32_t)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 16;
                    lux_vehicle_state_msg.distance_y = (int32_t)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 20;
                    lux_vehicle_state_msg.course_angle = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 24;
                    lux_vehicle_state_msg.longitudinal_velocity = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 28;
                    lux_vehicle_state_msg.yaw_rate = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 32;
                    lux_vehicle_state_msg.steering_wheel_angle = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 40;
                    lux_vehicle_state_msg.front_wheel_angle = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 46;
                    lux_vehicle_state_msg.vehicle_width = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 54;
                    lux_vehicle_state_msg.front_axle_to_front_axle = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 58;
                    lux_vehicle_state_msg.rear_axle_to_front_axle = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 62;
                    lux_vehicle_state_msg.rear_axle_to_vehicle_rear = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 70;
                    lux_vehicle_state_msg.steer_ratio_poly0 = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 74;
                    lux_vehicle_state_msg.steer_ratio_poly1 = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 78;
                    lux_vehicle_state_msg.steer_ratio_poly2 = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 82;
                    lux_vehicle_state_msg.steer_ratio_poly3 = (float)read_big_endian(msgBuf, vehicle_state_data);

                    lux_header_msg.header.stamp = now;
                    lux_header_msg.header.frame_id = frame_id;
                    lux_header_msg.message_size = data_size;
                    lux_header_msg.device_id = device_id;

                    lux_header_msg.data_type = data_type_hex;

                    lux_vehicle_state_msg.header  = lux_header_msg;
                    vehicle_state_pub.publish(lux_vehicle_state_msg);
                }
                //Fusion vehicle state 2807
                if (data_type == 0x2807)
                {
                    ROS_INFO("reading Fusion vehicle state data 0x2807");
                    TCPMsg   vehicle_state_data;
                    ros_ibeo_lux::lux_fusion_vehicle_state    lux_vehicle_state_msg;
                    vehicle_state_data.size = 8;
                    vehicle_state_data.msgOffset = start_byte + 4;
                    vehicle_state_data.factor = 1;
                    vehicle_state_data.valueOffset = 0;
                    lux_vehicle_state_msg.time_stamp = (uint64_t)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 12;
                    vehicle_state_data.size = 4;
                    lux_vehicle_state_msg.distance_x = (int32_t)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 16;
                    lux_vehicle_state_msg.distance_y = (int32_t)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 20;
                    lux_vehicle_state_msg.course_angle = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 24;
                    lux_vehicle_state_msg.longitudinal_velocity = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 28;
                    lux_vehicle_state_msg.yaw_rate = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 32;
                    lux_vehicle_state_msg.steering_wheel_angle = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 40;
                    lux_vehicle_state_msg.front_wheel_angle = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 46;
                    lux_vehicle_state_msg.vehicle_width = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 54;
                    lux_vehicle_state_msg.front_axle_to_front_axle = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 58;
                    lux_vehicle_state_msg.rear_axle_to_front_axle = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 62;
                    lux_vehicle_state_msg.rear_axle_to_vehicle_rear = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 70;
                    lux_vehicle_state_msg.steer_ratio_poly0 = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 74;
                    lux_vehicle_state_msg.steer_ratio_poly1 = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 78;
                    lux_vehicle_state_msg.steer_ratio_poly2 = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 82;
                    lux_vehicle_state_msg.steer_ratio_poly3 = (float)read_big_endian(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 86;
                    lux_vehicle_state_msg.lobitudinal_accelration = (float)read_big_endian(msgBuf, vehicle_state_data);

                    lux_header_msg.header.stamp = now;
                    lux_header_msg.header.frame_id = frame_id;
                    lux_header_msg.message_size = data_size;
                    lux_header_msg.device_id = device_id;

                    lux_header_msg.data_type = data_type_hex;

                    lux_vehicle_state_msg.header  = lux_header_msg;
                    vehicle_state_pub.publish(lux_vehicle_state_msg);
                }
            }
            // Wait for next loop
            loop_rate.sleep();
            ros::spinOnce();
        }
    } catch (std::exception& e) {
        cerr << e.what() << endl;
        socket.close();
        return ERROR_UNHANDLED_EXCEPTION;
    }

    socket.close();

    return 0;
}

