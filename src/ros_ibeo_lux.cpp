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
//#include <as_ibeo_lux.hpp>
#include <TCPMsg.hpp>

//Ros
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


//Tx
#include <ros_ibeo_lux/lux_scan_data.h>
#include <ros_ibeo_lux/scan_point.h>
#include <ros_ibeo_lux/lux_object_data.h>
#include <geometry_msgs/Point32.h>
#include <ros_ibeo_lux/point2D.h>
#include <ros_ibeo_lux/size2D.h>
#include <ros_ibeo_lux/lux_vehicle_state.h>



using namespace std;
using namespace boost;
using boost::asio::ip::tcp;
namespace po = boost::program_options;


const size_t ERROR_UNHANDLED_EXCEPTION = 2;
//const unsigned int ESR_XCP_PAYLOAD_SIZE =8568;
const uint8_t  LUX_MESSAGE_DATA_OFFSET = 24;
const   unsigned int LUX_PAYLOAD_SIZE = 100000;
size_t magicWord = 0xAFFEC0C2;


// little endian
double read_value(std::array<unsigned char, LUX_PAYLOAD_SIZE> &bufArray, TCPMsg msg) {
    unsigned long rcvData = 0;

    for (unsigned int i = msg.size; i > 0; i--) {
        rcvData <<= 8;
        //Need to use -1 because array is 0-based
        //and offset is not.
        rcvData |= bufArray[(msg.msgOffset - 1) + i];
    }

    double retVal = ((double)rcvData * msg.factor) - msg.valueOffset;

    return retVal;
}

// big endian
double read_header(std::array<unsigned char, LUX_PAYLOAD_SIZE> &bufArray, TCPMsg msg) {
    unsigned long rcvData = 0;

    for (unsigned int i = 0; i <  msg.size; i++) {
        rcvData <<= 8;

        rcvData |= bufArray[(msg.msgOffset) + i];
    }

    double retVal = ((double)rcvData * msg.factor) - msg.valueOffset;

    return retVal;
}

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
    ros::Publisher scan_data_pub = n.advertise<ros_ibeo_lux::lux_scan_data>("lux_scan_data", 1);
    ros::Publisher object_data_pub = n.advertise<ros_ibeo_lux::lux_object_data>("lux_object_data", 1);
    ros::Publisher vehicle_state_pub = n.advertise<ros_ibeo_lux::lux_vehicle_state>("lux_vehicle_state", 1);

  /*  ros::Publisher radarstatus1_pub = n.advertise<as_delphi_esr::radar_status1>("radarstatus1", 1);
    ros::Publisher radarstatus2_pub = n.advertise<as_delphi_esr::radar_status2>("radarstatus2", 1);
    ros::Publisher radarstatus3_pub = n.advertise<as_delphi_esr::radar_status3>("radarstatus3", 1);
    ros::Publisher radarstatus4_pub = n.advertise<as_delphi_esr::radar_status4>("radarstatus4", 1);
    ros::Publisher radarstatus5_pub = n.advertise<as_delphi_esr::radar_status5>("radarstatus5", 1);
    ros::Publisher radarstatus6_pub = n.advertise<as_delphi_esr::radar_status6>("radarstatus6", 1);
    ros::Publisher radarstatus7_pub = n.advertise<as_delphi_esr::radar_status7>("radarstatus7", 1);
    ros::Publisher radarstatus8_pub = n.advertise<as_delphi_esr::radar_status8>("radarstatus8", 1);
    ros::Publisher radarstatus9_pub = n.advertise<as_delphi_esr::radar_status9>("radarstatus9", 1);
    ros::Publisher radarvalid1_pub = n.advertise<as_delphi_esr::radar_valid1>("radarvalid1", 1);
    ros::Publisher radarvalid2_pub = n.advertise<as_delphi_esr::radar_valid2>("radarvalid2", 1);

    ros::Publisher radarstatus_pub = n.advertise<radar_msg::radar_status>("radar_status", 1);
    ros::Publisher radarerrorstatus_pub = n.advertise<radar_msg::radar_error_status>("radar_error_status", 1);
     // Subscribe messages (Rx)
    ros::Subscriber vehicle1_sub = n.subscribe("vehicle1_msgs", 1, vehicle1MsgCallback);
    ros::Subscriber vehicle2_sub = n.subscribe("vehicle2_msgs", 1, vehicle2MsgCallback);
    ros::Subscriber vehicle3_sub = n.subscribe("vehicle3_msgs", 1, vehicle3MsgCallback);
    ros::Subscriber vehicle4_sub = n.subscribe("vehicle4_msgs", 1, vehicle4MsgCallback);    
    ros::Subscriber vehicle5_sub = n.subscribe("vehicle5_msgs", 1, vehicle5MsgCallback);

    // tf::TransformListener listener;
*/
    // Wait for time to be valid
    while (ros::Time::now().nsec == 0);


   /*if (priv.getParam("lux_ip", ip))
    {
        ROS_INFO("Got LUX ip address: %s", ip.c_str());
    }

    if (priv.getParam("lux_port", port))
    {
        ROS_INFO("Got LUX port: %d", port);
    }
    if (priv.getParam("lux_frame_id",frame_id))
    {
        ROS_INFO("Got LUX frame_id: %s", frame_id.c_str());
    }
*/
    ros_ibeo_lux::lux_scan_data lux_scan_msg;
    ros_ibeo_lux::lux_object_data lux_object_msg;
    ros_ibeo_lux::lux_vehicle_state lux_vehicle_state_msg;

    TCPMsg    header_msg;
    header_msg.msgOffset = 0;
    header_msg.size = 4;
    header_msg.factor = 1;
    header_msg.valueOffset = 0;

    //std::array<unsigned char, LUX_PAYLOAD_SIZE> xcpMsgBuf;

    //unsigned int packetCounter = 0;
    int    data_size;
    int    data_type;
    int    start_byte;


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
            size_t rcvSize;
            system::error_code error;
            unsigned long first_four_bytes;
            rcvSize = socket.read_some(asio::buffer(msgBuf), error);
            bool package_rcvd = false;
            int i = 0;
            while (!package_rcvd)
            {
                header_msg.size = 4;
                header_msg.msgOffset = i;
                first_four_bytes = (uint32_t)read_header(msgBuf, header_msg);
                if (first_four_bytes == magicWord)
                {
                    ROS_INFO("Found the header msg");
                    header_msg.msgOffset = 8;
                    data_size = (uint32_t)read_header(msgBuf, header_msg);
                    header_msg.msgOffset = 14;
                    header_msg.size = 2;
                    data_type = (uint16_t)read_header(msgBuf, header_msg);
                    start_byte = i;
                    package_rcvd = true;
                    ROS_INFO("Found Data Type %x", data_type);
                    ROS_INFO("start byte %f", i);
                    ROS_INFO("package received %d", package_rcvd);
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
                    ROS_INFO("reading scan data");
                    TCPMsg   scan_data;
                    scan_data.size = 2;
                    scan_data.msgOffset = start_byte;
                    scan_data.factor = 1;
                    scan_data.valueOffset = 0;
                    lux_scan_msg.scan_number = (uint16_t)read_value(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 2;
                    lux_scan_msg.scan_status = (uint16_t)read_value(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte  + 4;
                    lux_scan_msg.sync_phase_offset = (uint16_t)read_value(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte  + 6;
                    scan_data.size = 8;
                    lux_scan_msg.scan_start_time = (uint64_t)read_value(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte  + 14;
                    lux_scan_msg.scan_end_time = (uint64_t)read_value(msgBuf, scan_data);
                    scan_data.size = 2;
                    scan_data.msgOffset = start_byte  + 22;
                    lux_scan_msg.angle_ticks = (uint16_t)read_value(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 24;
                    lux_scan_msg.start_angle = (int16_t)read_value(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 26;
                    lux_scan_msg.end_angle = (int16_t)read_value(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 28;
                    lux_scan_msg.num_scan_pts = (uint16_t)read_value(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 30;
                    lux_scan_msg.mounting_yaw_angle = (int16_t)read_value(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 32;
                    lux_scan_msg.mounting_pitch_angle = (int16_t)read_value(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 34;
                    lux_scan_msg.mounting_roll_angle = (int16_t)read_value(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 36;
                    lux_scan_msg.mounting_position_x = (int16_t)read_value(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 38;
                    lux_scan_msg.mounting_position_y = (int16_t)read_value(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 40;
                    lux_scan_msg.mounting_position_z = (int16_t)read_value(msgBuf, scan_data);
                    scan_data.msgOffset = start_byte + 42;
                    lux_scan_msg.scan_flags = (int16_t)read_value(msgBuf, scan_data);

                    ros_ibeo_lux::scan_point scan_point_data;
                    scan_data.msgOffset = start_byte + 44;
                    scan_data.size = 1;
                    for(int k = 0; k < lux_scan_msg.num_scan_pts; k++)
                    {

                        scan_point_data.layer = (uint8_t)(msgBuf[scan_data.msgOffset] & 0x0F);
                        scan_point_data.echo = (uint8_t)((msgBuf[scan_data.msgOffset] >> 4) & 0x0F);
                        scan_data.msgOffset = scan_data.msgOffset + 1;
                        scan_point_data.flags = (uint8_t)read_value(msgBuf, scan_data);
                        scan_data.msgOffset = scan_data.msgOffset + 2;
                        scan_data.size = 2;
                        scan_point_data.flags = (int16_t)read_value(msgBuf, scan_data);
                        scan_data.msgOffset = scan_data.msgOffset + 4;
                        scan_point_data.radial_distance = (uint16_t)read_value(msgBuf, scan_data);
                        scan_data.msgOffset = scan_data.msgOffset + 6;
                        scan_point_data.echo_pulse_width = (uint16_t)read_value(msgBuf, scan_data);

                        lux_scan_msg.scan_points.push_back(scan_point_data);
                    }
                    lux_scan_msg.header.stamp = now;
                    lux_scan_msg.header.frame_id = frame_id;
                    scan_data_pub.publish(lux_scan_msg);
                    lux_scan_msg.scan_points.clear();
                }
                // ibeo LUX object data
                if (data_type == 0x2221)
                {
                    ROS_INFO("reading object data");
                    TCPMsg   object_data;
                    object_data.size = 8;
                    object_data.msgOffset = start_byte;
                    object_data.factor = 1;
                    object_data.valueOffset = 0;
                    lux_object_msg.scan_start_time = (uint64_t)read_value(msgBuf, object_data);
                    object_data.msgOffset = start_byte + 8;
                    object_data.size = 2;
                    lux_object_msg.num_of_objects = (uint16_t)read_value(msgBuf, object_data);

                    ros_ibeo_lux::object_list scan_object;
                    ros_ibeo_lux::point2D     object_point;
                    object_data.msgOffset = start_byte + 10;

                    for(int k = 0; k < lux_object_msg.num_of_objects; k++)
                    {

                        scan_object.ID = (uint16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 2;
                        scan_object.age = (uint16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 4;
                        scan_object.prediction_age = (uint16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 6;
                        scan_object.relative_timestamp = (uint16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 8;
                        scan_object.reference_point.x = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 10;
                        scan_object.reference_point.y = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 12;
                        scan_object.reference_point_sigma.x = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 14;
                        scan_object.reference_point_sigma.y = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 16;
                        scan_object.closest_point.x = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 18;
                        scan_object.closest_point.y = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 20;
                        scan_object.bounding_box_center.x = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 22;
                        scan_object.bounding_box_center.y = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 24;
                        scan_object.bounding_box_width = (uint16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 26;
                        scan_object.bounding_box_length = (uint16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 28;
                        scan_object.object_box_center.x = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 30;
                        scan_object.object_box_center.y = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 32;
                        scan_object.object_box_size.x = (uint16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 34;
                        scan_object.object_box_size.y = (uint16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 36;
                        scan_object.object_box_orientation = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 38;
                        scan_object.absolute_velocity.x = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 40;
                        scan_object.absolute_velocity.y = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 42;
                        scan_object.absolute_velocity_sigma.x = (uint16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 44;
                        scan_object.absolute_velocity_sigma.y = (uint16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 46;
                        scan_object.relative_velocity.x = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 48;
                        scan_object.relative_velocity.y = (int16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 50;
                        scan_object.classification = (uint16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 52;
                        scan_object.classification_age = (uint16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 54;
                        scan_object.classification_certaiinty = (uint16_t)read_value(msgBuf, object_data);
                        object_data.msgOffset = object_data.msgOffset + 56;
                        scan_object.number_of_contour_points = (uint16_t)read_value(msgBuf, object_data);

                        for(int j =0; j< scan_object.number_of_contour_points; j++)
                        {
                            object_data.msgOffset = object_data.msgOffset + 2*j;
                            object_point.x = (int16_t)read_value(msgBuf, object_data);
                            object_data.msgOffset = object_data.msgOffset + 2*(j + 1);
                            object_point.y = (int16_t)read_value(msgBuf, object_data);
                            scan_object.list_of_contour_points.push_back(object_point);
                        }

                        lux_object_msg.object.push_back(scan_object);
                        scan_object.list_of_contour_points.clear();
                    }
                    lux_object_msg.header.stamp = now;
                    lux_object_msg.header.frame_id = frame_id;
                    object_data_pub.publish(lux_object_msg);
                    lux_object_msg.object.clear();
                }


                if (data_type == 0x2805)
                {
                    ROS_INFO("reading vehicle state data");
                    TCPMsg   vehicle_state_data;
                    vehicle_state_data.size = 8;
                    vehicle_state_data.msgOffset = start_byte;
                    vehicle_state_data.factor = 1;
                    vehicle_state_data.valueOffset = 0;
                    lux_vehicle_state_msg.timestamp = (uint64_t)read_value(msgBuf, vehicle_state_data);
                    vehicle_state_data.size = 2;
                    vehicle_state_data.msgOffset = start_byte + 8;
                    lux_vehicle_state_msg.scan_number = (uint16_t)read_value(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 10;
                    lux_vehicle_state_msg.error_flags = (uint16_t)read_value(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 12;
                    lux_vehicle_state_msg.longitudinal_velocity = (int16_t)read_value(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 14;
                    lux_vehicle_state_msg.steering_wheel_angle = (int16_t)read_value(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 16;
                    lux_vehicle_state_msg.front_wheel_angle = (int16_t)read_value(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 20;
                    vehicle_state_data.size = 4;
                    lux_vehicle_state_msg.vehicle_x = (int32_t)read_value(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 24;
                    lux_vehicle_state_msg.vehicle_y = (int32_t)read_value(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 28;
                    vehicle_state_data.size = 2;
                    lux_vehicle_state_msg.course_angle = (int16_t)read_value(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 30;
                    lux_vehicle_state_msg.time_difference = (uint16_t)read_value(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 32;
                    lux_vehicle_state_msg.diff_in_x = (int16_t)read_value(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 34;
                    lux_vehicle_state_msg.diff_in_y = (int16_t)read_value(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 36;
                    lux_vehicle_state_msg.diff_in_heading = (int16_t)read_value(msgBuf, vehicle_state_data);
                    vehicle_state_data.msgOffset = start_byte + 40;
                    lux_vehicle_state_msg.current_yaw_rate = (int16_t)read_value(msgBuf, vehicle_state_data);

                    lux_vehicle_state_msg.header.stamp = now;
                    lux_vehicle_state_msg.header.frame_id = frame_id;
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

