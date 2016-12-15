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
#include <as_ibeo_lux.hpp>
#include <TCPMsg.hpp>

//Ros
#include <ros/ros.h>

//Tx
#include <ros_ibeo_lux/lux_scan_data.h>
#include <ros_ibeo_lux/scan_point.h>


using namespace std;
using namespace boost;
using boost::asio::ip::tcp;
namespace po = boost::program_options;


const size_t ERROR_UNHANDLED_EXCEPTION = 2;
//const unsigned int ESR_XCP_PAYLOAD_SIZE =8568;
const uint8_t  LUX_MESSAGE_DATA_OFFSET = 24;
const   unsigned int LUX_PAYLOAD_SIZE = 100000;
size_t magicWord = 0xAFFEC0C2;



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
    ros::Publisher objectsPub = n.advertise<ros_ibeo_lux::lux_scan_data>("lux_scan_data", 1);
    
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
                first_four_bytes = (uint32_t)read_value(msgBuf, header_msg);
                if (first_four_bytes == magicWord)
                {
                    ROS_INFO("Found the header msg");
                    header_msg.msgOffset = 8;
                    data_size = (uint32_t)read_value(msgBuf, header_msg);
                    header_msg.msgOffset = 14;
                    header_msg.size = 2;
                    data_type = (uint16_t)read_value(msgBuf, header_msg);
                    start_byte = i;
                    package_rcvd = true;
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
                    objectsPub.publish(lux_scan_msg);
                    lux_scan_msg.scan_points.clear();
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

