/*
 *   ros_ibeo_lux.cpp - ROS implementation of the Ibeo LUX driver.
 *   Copyright (C) 2017 AutonomouStuff, Co.
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 *   USA
 */

// Sys
#include <csignal>

#include <ibeo_core.h>
#include <network_interface/network_interface.h>

//Ros
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>

//Tx
#include <network_interface/TCPFrame.h>

#include <ibeo_msgs/IbeoDataHeader.h>
#include <ibeo_msgs/CameraImage.h>
#include <ibeo_msgs/ErrorWarning.h>
#include <ibeo_msgs/ScanData2202.h>
#include <ibeo_msgs/ScanPoint2204.h>
#include <ibeo_msgs/ScannerInfo2204.h>
#include <ibeo_msgs/ScanData2204.h>
#include <ibeo_msgs/ScanPoint2205.h>
#include <ibeo_msgs/ScannerInfo2205.h>
#include <ibeo_msgs/ScanData2205.h>
#include <ibeo_msgs/Object2221.h>
#include <ibeo_msgs/ObjectData2221.h>
#include <ibeo_msgs/Object2225.h>
#include <ibeo_msgs/ObjectData2225.h>
#include <ibeo_msgs/Object2280.h>
#include <ibeo_msgs/ObjectData2280.h>
#include <ibeo_msgs/HostVehicleState2805.h>
#include <ibeo_msgs/HostVehicleState2806.h>
#include <ibeo_msgs/HostVehicleState2807.h>

// supplemental objects/messages

#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>

using std::string;
using std::vector;
using namespace AS::Network;
using namespace AS::Drivers::Ibeo;

TCPInterface tcp_interface;

ros::Time ntp_to_ros_time(NTPTime time)
{
  return ros::Time(((time & 0xFFFF0000) >> 32), (time & 0x0000FFFF));
}

visualization_msgs::Marker createWireframeMarker(float center_x, float center_y, float size_x, float size_y, float size_z)
{
  visualization_msgs::Marker box;
  box.type = visualization_msgs::Marker::LINE_LIST;
  box.action = visualization_msgs::Marker::ADD;
  box.pose.position.x = center_x;
  box.pose.position.y = center_y;
  box.scale.x = 0.05;
  geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;

  size_y = (size_y <= 0.1f)? 0.1f : size_y;
  size_x = (size_x <= 0.1f)? 0.1f : size_x;

  float half_x = (0.5) * size_x;
  float half_y = (0.5) * size_y;

  p1.x = half_x;
  p1.y = half_y;
  p1.z = size_z;
  p2.x = half_x;
  p2.y = -half_y;
  p2.z = size_z;
  p3.x = -half_x;
  p3.y = -half_y;
  p3.z = size_z;
  p4.x = -half_x;
  p4.y = half_y;
  p4.z = size_z;
  p5 = p1;
  p5.z = -size_z;
  p6 = p2;
  p6.z = -size_z;
  p7 = p3;
  p7.z = -size_z;
  p8 = p4;
  p8.z = -size_z;

  box.points.reserve(24);
  
  box.points.push_back(p1);
  box.points.push_back(p2);

  box.points.push_back(p2);
  box.points.push_back(p3);

  box.points.push_back(p3);
  box.points.push_back(p4);

  box.points.push_back(p4);
  box.points.push_back(p1);

  box.points.push_back(p1);
  box.points.push_back(p5);

  box.points.push_back(p2);
  box.points.push_back(p6);

  box.points.push_back(p3);
  box.points.push_back(p7);

  box.points.push_back(p4);
  box.points.push_back(p8);

  box.points.push_back(p5);
  box.points.push_back(p6);

  box.points.push_back(p6);
  box.points.push_back(p7);

  box.points.push_back(p7);
  box.points.push_back(p8);

  box.points.push_back(p8);
  box.points.push_back(p5);

  return box;
}

// Main routine
int main(int argc, char **argv)
{
  //int c;
  string ip_address = "192.168.0.1";
  int port = 12002;
  string frame_id = "ibeo_lux";
  bool is_fusion = false;

  // ROS initialization
  ros::init(argc, argv, "ibeo_lux");
  ros::NodeHandle n;
  ros::NodeHandle priv("~");
  bool exit = false;

  if (priv.getParam("ip_address", ip_address))
  {
    ROS_INFO("Ibeo LUX - Got ip_address: %s", ip_address.c_str());

    if (ip_address == "" )
    {
      ROS_ERROR("Ibeo LUX - IP Address Invalid");
      exit = true;
    }
  }

  if (priv.getParam("port", port))
  {
    ROS_INFO("Ibeo LUX - Got port: %d", port);

    if (port < 0)
    {
      ROS_ERROR("Ibeo LUX - Port Invalid");
      exit = true;
    }
  }

  if (priv.getParam("is_fusion", is_fusion))
  {
    ROS_INFO("Ibeo LUX - Is Fusion ECU: %s", (is_fusion)? "true" : "false");
  }

  if (priv.getParam("sensor_frame_id", frame_id))
  {
    ROS_INFO("Ibeo LUX - Got sensor frame ID: %s", frame_id.c_str());
  }

  if(exit)
    return 0;

  // Advertise messages to send
  ros::Publisher raw_tcp_pub = n.advertise<network_interface::TCPFrame>("tcp_tx", 1);
  ros::Publisher pointcloud_pub = n.advertise<pcl::PointCloud <pcl::PointXYZ> >("as_tx/point_cloud", 1);
  ros::Publisher object_markers_pub = n.advertise<visualization_msgs::MarkerArray>("as_tx/objects", 1);
  ros::Publisher object_contour_points_pub = n.advertise<visualization_msgs::Marker>("as_tx/object_contour_points", 1);

  ros::Publisher scan_data_pub, object_data_pub, vehicle_state_pub, error_warn_pub;
  ros::Publisher fusion_scan_2204_pub, fusion_scan_2205_pub, fusion_object_2225_pub, fusion_object_2280_pub, fusion_img_2403_pub, fusion_vehicle_2806_pub, fusion_vehicle_2807_pub;

  //LUX Sensor Only
  if(!is_fusion)
  {
    scan_data_pub = n.advertise<ibeo_msgs::ScanData2202>("parsed_tx/lux_scan_data", 1);
    object_data_pub = n.advertise<ibeo_msgs::ObjectData2221>("parsed_tx/lux_object_data", 1);
    vehicle_state_pub = n.advertise<ibeo_msgs::HostVehicleState2805>("parsed_tx/lux_vehicle_state", 1);
    error_warn_pub = n.advertise<ibeo_msgs::ErrorWarning>("parsed_tx/lux_error_warning", 1);
  }
  else //Fusion ECU Only
  {
    fusion_scan_2204_pub = n.advertise<ibeo_msgs::ScanData2204>("parsed_tx/fusion_scan_data_2204", 1);
    fusion_scan_2205_pub = n.advertise<ibeo_msgs::ScanData2205>("parsed_tx/fusion_scan_data_2205", 1);
    fusion_object_2225_pub = n.advertise<ibeo_msgs::ObjectData2225>("parsed_tx/fusion_object_data_2225", 1);
    fusion_object_2280_pub = n.advertise<ibeo_msgs::ObjectData2280>("parsed_tx/fusion_object_data_2280", 1);
    fusion_img_2403_pub = n.advertise<ibeo_msgs::CameraImage>("parsed_tx/fusion_image_2403", 1);
    fusion_vehicle_2806_pub = n.advertise<ibeo_msgs::HostVehicleState2806>("parsed_tx/lux_fusion_vehicle_state_2806", 1);
    fusion_vehicle_2807_pub = n.advertise<ibeo_msgs::HostVehicleState2807>("parsed_tx/lux_fusion_vehicle_state_2807", 1);
  }

  // Wait for time to be valid
  while (ros::Time::now().nsec == 0);

  ibeo_msgs::IbeoDataHeader lux_header_msg;
  ibeo_msgs::ScanData2202 lux_scan_msg;
  ibeo_msgs::ObjectData2221 lux_object_msg;
  ibeo_msgs::HostVehicleState2805 lux_vehicle_state_msg;
  ibeo_msgs::ErrorWarning lux_error_warning_msg;
  ibeo_msgs::ScanData2204 lux_fusion_scan_2204;
  ibeo_msgs::ScanData2205 lux_fusion_scan_2205;
  ibeo_msgs::ObjectData2225 lux_fusion_object_2225;
  ibeo_msgs::ObjectData2280 lux_fusion_object_2280;
  ibeo_msgs::CameraImage lux_fusion_image_msg;
  ibeo_msgs::HostVehicleState2806 lux_vehicle_state_2806_msg;
  ibeo_msgs::HostVehicleState2807 lux_vehicle_state_2807_msg;
  
  network_interface::TCPFrame tcp_raw_msg;

  IbeoDataHeader header_tx;
  ScanData2202 lux_scan_data_tx;
  ObjectData2221 lux_object_data_tx;
  HostVehicleState2805 lux_vehicle_state_tx;
  ErrorWarning lux_error_warning_tx;
  ScanData2204 fusion_scan_data_2204_tx;
  ScanData2205 fusion_scan_data_2205_tx;
  ObjectData2225 fusion_object_data_2225_tx;
  ObjectData2280 fusion_object_data_2280_tx;
  CameraImage fusion_image_tx;
  HostVehicleState2806 fusion_vehicle_state_2806_tx;
  HostVehicleState2807 fusion_vehicle_state_2807_tx;

  return_statuses status;

  ros::Rate loop_rate = (is_fusion)? ros::Rate(1100) : ros::Rate(40);
  // Loop as long as module should run
  
  unsigned char head_msg[IBEO_HEADER_SIZE]; 
  bool fusion_filter_sent = false;

  while (ros::ok())
  {
    if (!tcp_interface.is_open())
    {
      if (is_fusion)
        fusion_filter_sent = false;

      status = tcp_interface.open(ip_address.c_str(), port);

      if (status != OK)
        ROS_WARN("Ibeo LUX - Unable to connect to sensor at %s: %d - %s", ip_address.c_str(), status, return_status_desc(status).c_str());

      ros::Duration(1.0).sleep();
    }
    else
    {
      if(is_fusion && !fusion_filter_sent)
      {
        unsigned char set_filter_cmd[32] = {0xaf, 0xfe, 0xc0, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x02, 0x00, 0x00, 0xff,0xff};

        ROS_INFO_THROTTLE(3, "Ibeo LUX - Sending Fusion filter command to begin transmission.");

        if ((status = tcp_interface.write(set_filter_cmd, sizeof(set_filter_cmd))) != OK)
        {
          ROS_ERROR_THROTTLE(3, "Ibeo LUX - Failed to send Fusion filter command.");
        }
        else
        {
          fusion_filter_sent = true;
        }
      }

      //TODO: Change to same parsing method as ScaLa
      status = tcp_interface.read_exactly(head_msg, sizeof(head_msg), IBEO_HEADER_SIZE);
      header_tx.parse(head_msg);

      /*
      if(offset > 0)
      {
        ROS_WARN("LUX/LUX Fusion TCP Data out of sync. Offset by %d bytes", offset);

        unsigned char temparray[IBEO_HEADER_SIZE];
        memcpy(temparray, head_msg + offset, (IBEO_HEADER_SIZE - offset) * sizeof(char));
        status = tcp_interface.read_exactly(head_msg, sizeof(head_msg), offset);
        memcpy(temparray + IBEO_HEADER_SIZE - offset, head_msg, offset * sizeof(char));
        memcpy(head_msg, temparray, IBEO_HEADER_SIZE * sizeof(char));
      }
      */
      
      unsigned char data_msg[header_tx.message_size];

      status = tcp_interface.read_exactly(data_msg, sizeof(data_msg), header_tx.message_size);
      tcp_raw_msg.address = ip_address;
      tcp_raw_msg.port = port;
      tcp_raw_msg.data.clear();

      for(unsigned int i = 0; i < IBEO_HEADER_SIZE; i++)
      {
        tcp_raw_msg.data.push_back(head_msg[i]);
      }

      for(unsigned int i = 0; i < header_tx.message_size; i++)
      {
        tcp_raw_msg.data.push_back(data_msg[i]);
      }

      tcp_raw_msg.size = tcp_raw_msg.data.size();
      tcp_raw_msg.header.stamp = ros::Time::now();
      raw_tcp_pub.publish(tcp_raw_msg);

      lux_header_msg.message_size = header_tx.message_size;
      lux_header_msg.device_id = header_tx.device_id;
      lux_header_msg.data_type_id = header_tx.data_type_id;

      if(!is_fusion && lux_header_msg.data_type_id == ScanData2202::DATA_TYPE)
      {
        ROS_DEBUG("Ibeo LUX - Reading scan data 0x2202");

        pcl::PointCloud <pcl::PointXYZL> pcl_cloud_2202;
        pcl::PointXYZL cloud_point_2202;
        
        lux_scan_msg.ibeo_header = lux_header_msg;

        // header needs to be set before parse since the offset that the
        // tx message uses is contained in the header.
        // shouldnt be required when a sync process is added if the MAGIC WORD is offset
        //lux_scan_data_tx.header_ = header_tx;
        lux_scan_data_tx.parse(data_msg);

        lux_scan_msg.scan_number = lux_scan_data_tx.scan_number;
        lux_scan_msg.scanner_status = lux_scan_data_tx.scanner_status;
        lux_scan_msg.sync_phase_offset = lux_scan_data_tx.sync_phase_offset;
        lux_scan_msg.scan_start_time = ntp_to_ros_time(lux_scan_data_tx.scan_start_time);
        lux_scan_msg.scan_end_time = ntp_to_ros_time(lux_scan_data_tx.scan_end_time);
        lux_scan_msg.angle_ticks_per_rotation = lux_scan_data_tx.angle_ticks_per_rotation;
        lux_scan_msg.start_angle_ticks = lux_scan_data_tx.start_angle_ticks;
        lux_scan_msg.end_angle_ticks = lux_scan_data_tx.end_angle_ticks;
        lux_scan_msg.scan_points_count = lux_scan_data_tx.scan_points_count;
        lux_scan_msg.mounting_yaw_angle_ticks = lux_scan_data_tx.mounting_yaw_angle_ticks;
        lux_scan_msg.mounting_pitch_angle_ticks = lux_scan_data_tx.mounting_pitch_angle_ticks;
        lux_scan_msg.mounting_roll_angle_ticks = lux_scan_data_tx.mounting_roll_angle_ticks;
        lux_scan_msg.mounting_position_x = lux_scan_data_tx.mounting_position_x;
        lux_scan_msg.mounting_position_y = lux_scan_data_tx.mounting_position_y;
        lux_scan_msg.mounting_position_z = lux_scan_data_tx.mounting_position_z;
        lux_scan_msg.ground_labeled = lux_scan_data_tx.ground_labeled;
        lux_scan_msg.dirt_labeled = lux_scan_data_tx.dirt_labeled;
        lux_scan_msg.rain_labeled = lux_scan_data_tx.rain_labeled;
        lux_scan_msg.mirror_side = static_cast<uint8_t>(lux_scan_data_tx.mirror_side);

        pcl_cloud_2202.reserve(lux_scan_msg.scan_points_count);
        lux_scan_msg.scan_point_list.clear();

        ibeo_msgs::ScanPoint2202 msg_point;
        ScanPoint2202 tx_point;

        for(unsigned int k = 0; k < lux_scan_msg.scan_points_count; k++)
        {
          tx_point = lux_scan_data_tx.scan_point_list[k];
          msg_point.layer = tx_point.layer;
          msg_point.echo = tx_point.echo;
          msg_point.transparent_point = tx_point.transparent_point;
          msg_point.clutter_atmospheric = tx_point.clutter_atmospheric;
          msg_point.ground = tx_point.ground;
          msg_point.dirt = tx_point.dirt;
          msg_point.horizontal_angle = tx_point.horizontal_angle;
          msg_point.radial_distance = tx_point.radial_distance;
          msg_point.echo_pulse_width = tx_point.echo_pulse_width;
          
          lux_scan_msg.scan_point_list.push_back(msg_point);
          
          // filter out flagged points
          if(!msg_point.transparent_point &&
             !msg_point.clutter_atmospheric &&
             !msg_point.ground &&
             !msg_point.dirt)
          {
            double phi;
            switch (msg_point.layer)
            {
              case 0: phi = -1.6 * M_PI / 180.0; break;
              case 1: phi = -0.8 * M_PI / 180.0; break;
              case 2: phi =  0.8 * M_PI / 180.0; break;
              case 3: phi =  1.6 * M_PI / 180.0; break;
              default: phi = 0.0; break;
            }

            cloud_point_2202.x = (double)msg_point.radial_distance / 100 * cos(ticks_to_angle(msg_point.horizontal_angle, lux_scan_msg.angle_ticks_per_rotation)) * cos(phi);
            cloud_point_2202.y = (double)msg_point.radial_distance / 100 * sin(ticks_to_angle(msg_point.horizontal_angle, lux_scan_msg.angle_ticks_per_rotation)) * cos(phi);
            cloud_point_2202.z = (double)msg_point.radial_distance / 100 * sin(phi);

            pcl_cloud_2202.points.push_back(cloud_point_2202);
          }
        }

        lux_scan_msg.header.frame_id = frame_id;
        lux_scan_msg.header.stamp = ros::Time::now();
        scan_data_pub.publish(lux_scan_msg);

        pcl_cloud_2202.header.frame_id = frame_id;
        pcl_conversions::toPCL(ros::Time::now(), pcl_cloud_2202.header.stamp);
        pointcloud_pub.publish(pcl_cloud_2202);
                  
      }
      else if (!is_fusion && lux_header_msg.data_type_id == ObjectData2221::DATA_TYPE)
      {
        //ROS_DEBUG("Ibeo LUX - Reading object data 0x2221");
        visualization_msgs::MarkerArray object_markers;

        lux_object_msg.ibeo_header = lux_header_msg;

        //lux_object_data_tx.header_ = header_tx;
        lux_object_data_tx.parse(data_msg);
        
        lux_object_msg.scan_start_timestamp = ntp_to_ros_time(lux_object_data_tx.scan_start_timestamp);
        lux_object_msg.number_of_objects = lux_object_data_tx.number_of_objects;

        ibeo_msgs::Object2221 scan_object;
        Object2221 object_tx;
        ibeo_msgs::Point2Di object_point;
        Point2Di point_tx;
        lux_object_msg.object_list.clear();

        for(int k = 0; k < lux_object_msg.number_of_objects; k++)
        {
          object_tx = lux_object_data_tx.object_list[k];
          scan_object.id = object_tx.id;
          scan_object.age = object_tx.age;
          scan_object.prediction_age = object_tx.prediction_age;
          scan_object.relative_timestamp = object_tx.relative_timestamp;
          scan_object.reference_point.x = object_tx.reference_point.x;
          scan_object.reference_point.y = object_tx.reference_point.y;
          scan_object.reference_point_sigma.x = object_tx.reference_point_sigma.x;
          scan_object.reference_point_sigma.y = object_tx.reference_point_sigma.y;
          scan_object.closest_point.x = object_tx.closest_point.x;
          scan_object.closest_point.y = object_tx.closest_point.y;
          scan_object.bounding_box_center.x = 0.01 * object_tx.bounding_box_center.x;
          scan_object.bounding_box_center.y = 0.01 * object_tx.bounding_box_center.y;
          scan_object.bounding_box_width = 0.01 * object_tx.bounding_box_width;
          scan_object.bounding_box_length = 0.01 * object_tx.bounding_box_length;
          scan_object.object_box_center.x = 0.01 * object_tx.object_box_center.x;
          scan_object.object_box_center.y = 0.01 * object_tx.object_box_center.y;
          scan_object.object_box_size.size_x = 0.01 * object_tx.object_box_size.size_x;
          scan_object.object_box_size.size_y = 0.01 * object_tx.object_box_size.size_y;
          scan_object.object_box_orientation = (float)object_tx.object_box_orientation;
          scan_object.absolute_velocity.x = object_tx.absolute_velocity.x;
          scan_object.absolute_velocity.y = object_tx.absolute_velocity.y;
          scan_object.absolute_velocity_sigma.size_x = object_tx.absolute_velocity_sigma.size_x;
          scan_object.absolute_velocity_sigma.size_y = object_tx.absolute_velocity_sigma.size_y;
          scan_object.relative_velocity.x = object_tx.relative_velocity.x;
          scan_object.relative_velocity.y = object_tx.relative_velocity.y;
          scan_object.classification = static_cast<uint8_t>(object_tx.classification);
          scan_object.classification_age = object_tx.classification_age;
          scan_object.classification_certainty = object_tx.classification_certainty;
          scan_object.number_of_contour_points = object_tx.number_of_contour_points;

          lux_object_msg.object_list.push_back(scan_object);
          
          tf::Quaternion quaternion = tf::createQuaternionFromYaw(scan_object.object_box_orientation * 100/180 * M_PI);

          visualization_msgs::Marker object_marker = createWireframeMarker(scan_object.object_box_center.x,
                                                                           scan_object.object_box_center.y,
                                                                           scan_object.object_box_size.size_x,
                                                                           scan_object.object_box_size.size_y,
                                                                           0.75);
          object_marker.header.frame_id = frame_id;
          object_marker.id  = scan_object.id;
          object_marker.pose.orientation.x = quaternion.x();
          object_marker.pose.orientation.y = quaternion.y();
          object_marker.pose.orientation.z = quaternion.z();
          object_marker.pose.orientation.w = quaternion.w();
          object_marker.lifetime = ros::Duration(0.2);
          object_marker.color.a = 0.5;
          object_marker.color.r = object_marker.color.g = object_marker.color.b = 1.0;
          object_marker.frame_locked = false;

          std::string label;

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
              // Truck - dark green
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

          object_marker.ns = label;
          
          visualization_msgs::Marker object_label;
          object_label.header.frame_id = frame_id;
          object_label.id = scan_object.id + 1000;
          object_label.ns = label;
          object_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          object_label.action = visualization_msgs::Marker::ADD;
          object_label.pose.position.x = scan_object.object_box_center.x;
          object_label.pose.position.y = scan_object.object_box_center.y;
          object_label.pose.position.z = 0.5;
          object_label.text = label;
          object_label.scale.z = 0.5;
          object_label.lifetime = object_marker.lifetime;
          object_label.color.r = object_label.color.g = object_label.color.b = 1;
          object_label.color.a = 0.5;
          
          visualization_msgs::Marker object_point_marker;
          object_point_marker.header.frame_id = frame_id;
          object_point_marker.type = visualization_msgs::Marker::POINTS;
          object_point_marker.action = visualization_msgs::Marker::ADD;
          object_point_marker.ns = label;
          object_point_marker.lifetime = object_marker.lifetime;
          object_point_marker.scale.x = 0.05;
          object_point_marker.scale.y = 0.05;
          object_point_marker.color.r = 0;
          object_point_marker.color.g = 1;
          object_point_marker.color.b = 0;
          object_point_marker.color.a = 0.7;

          scan_object.contour_point_list.clear();
          geometry_msgs::Point dis_object_point;

          for(int j = 0; j < scan_object.number_of_contour_points; j++)
          {
            point_tx.x = object_tx.contour_point_list[k].x;
            point_tx.y = object_tx.contour_point_list[k].y;
            object_point.x = 0.01 * point_tx.x;
            object_point.y = 0.01 * point_tx.y;

            scan_object.contour_point_list.push_back(object_point);
            
            dis_object_point.x = object_point.x;
            dis_object_point.y = object_point.y;

            object_point_marker.points.push_back(dis_object_point);
          }
          
          object_point_marker.header.stamp = ros::Time::now();
          object_contour_points_pub.publish(object_point_marker);

          object_label.header.stamp = ros::Time::now();
          object_markers.markers.push_back(object_label);

          object_marker.header.stamp = ros::Time::now();
          object_markers.markers.push_back(object_marker);
        }

        lux_object_msg.header.frame_id = frame_id;
        lux_object_msg.header.stamp = ros::Time::now();
        object_data_pub.publish(lux_object_msg);
        object_markers_pub.publish(object_markers);
      }
      else if(!is_fusion && lux_header_msg.data_type_id == HostVehicleState2805::DATA_TYPE)
      {
        ROS_DEBUG("Ibeo LUX - Reading vehicle state data 0x2805");
        lux_vehicle_state_msg.ibeo_header  = lux_header_msg;

        //lux_vehicle_state_tx.header_ = header_tx;
        lux_vehicle_state_tx.parse(data_msg);

        lux_vehicle_state_msg.timestamp = ntp_to_ros_time(lux_vehicle_state_tx.timestamp);
        lux_vehicle_state_msg.scan_number = lux_vehicle_state_tx.scan_number;
        lux_vehicle_state_msg.error_flags = lux_vehicle_state_tx.error_flags;
        lux_vehicle_state_msg.longitudinal_velocity = lux_vehicle_state_tx.longitudinal_velocity;
        lux_vehicle_state_msg.steering_wheel_angle = lux_vehicle_state_tx.steering_wheel_angle;
        lux_vehicle_state_msg.front_wheel_angle = lux_vehicle_state_tx.front_wheel_angle;
        lux_vehicle_state_msg.x_position = lux_vehicle_state_tx.x_position;
        lux_vehicle_state_msg.y_position = lux_vehicle_state_tx.y_position;
        lux_vehicle_state_msg.course_angle = lux_vehicle_state_tx.course_angle;
        lux_vehicle_state_msg.time_difference = lux_vehicle_state_tx.time_difference;
        lux_vehicle_state_msg.x_difference = lux_vehicle_state_tx.x_difference;
        lux_vehicle_state_msg.y_difference = lux_vehicle_state_tx.y_difference;
        lux_vehicle_state_msg.heading_difference = lux_vehicle_state_tx.heading_difference;
        lux_vehicle_state_msg.current_yaw_rate = lux_vehicle_state_tx.current_yaw_rate;

        lux_vehicle_state_msg.header.frame_id = frame_id;
        lux_vehicle_state_msg.header.stamp = ros::Time::now();
        vehicle_state_pub.publish(lux_vehicle_state_msg);
      }
      else if(!is_fusion && lux_header_msg.data_type_id == ErrorWarning::DATA_TYPE)
      {
        ROS_DEBUG("Ibeo LUX - Reading lux errors and warnings data 0x2030");
        lux_error_warning_msg.ibeo_header  = lux_header_msg;
       
        lux_error_warning_tx.parse(data_msg);
       
				lux_error_warning_msg.err_internal_error = lux_error_warning_msg.err_internal_error;
				lux_error_warning_msg.err_motor_1_fault = lux_error_warning_msg.err_motor_1_fault;
				lux_error_warning_msg.err_buffer_error_xmt_incomplete = lux_error_warning_msg.err_buffer_error_xmt_incomplete;
				lux_error_warning_msg.err_buffer_error_overflow = lux_error_warning_msg.err_buffer_error_overflow;
				lux_error_warning_msg.err_apd_over_temperature = lux_error_warning_msg.err_apd_over_temperature;
				lux_error_warning_msg.err_apd_under_temperature = lux_error_warning_msg.err_apd_under_temperature;
				lux_error_warning_msg.err_apd_temperature_sensor_defect = lux_error_warning_msg.err_apd_temperature_sensor_defect;
				lux_error_warning_msg.err_motor_2_fault = lux_error_warning_msg.err_motor_2_fault;
				lux_error_warning_msg.err_motor_3_fault = lux_error_warning_msg.err_motor_3_fault;
				lux_error_warning_msg.err_motor_4_fault = lux_error_warning_msg.err_motor_4_fault;
				lux_error_warning_msg.err_motor_5_fault = lux_error_warning_msg.err_motor_5_fault;
				lux_error_warning_msg.err_int_no_scan_data = lux_error_warning_msg.err_int_no_scan_data;
				lux_error_warning_msg.err_int_communication_error = lux_error_warning_msg.err_int_communication_error;
				lux_error_warning_msg.err_int_incorrect_scan_data = lux_error_warning_msg.err_int_incorrect_scan_data;
				lux_error_warning_msg.err_config_fpga_not_configurable = lux_error_warning_msg.err_config_fpga_not_configurable;
				lux_error_warning_msg.err_config_incorrect_config_data = lux_error_warning_msg.err_config_incorrect_config_data;
				lux_error_warning_msg.err_config_contains_incorrect_params = lux_error_warning_msg.err_config_contains_incorrect_params;
				lux_error_warning_msg.err_timeout_data_processing = lux_error_warning_msg.err_timeout_data_processing;
				lux_error_warning_msg.err_timeout_env_model_computation_reset = lux_error_warning_msg.err_timeout_env_model_computation_reset;
				lux_error_warning_msg.wrn_int_communication_error = lux_error_warning_msg.wrn_int_communication_error;
				lux_error_warning_msg.wrn_low_temperature = lux_error_warning_msg.wrn_low_temperature;
				lux_error_warning_msg.wrn_high_temperature = lux_error_warning_msg.wrn_high_temperature;
				lux_error_warning_msg.wrn_int_motor_1 = lux_error_warning_msg.wrn_int_motor_1;
				lux_error_warning_msg.wrn_sync_error = lux_error_warning_msg.wrn_sync_error;
				lux_error_warning_msg.wrn_laser_1_start_pulse_missing = lux_error_warning_msg.wrn_laser_1_start_pulse_missing;
				lux_error_warning_msg.wrn_laser_2_start_pulse_missing = lux_error_warning_msg.wrn_laser_2_start_pulse_missing;
				lux_error_warning_msg.wrn_can_interface_blocked = lux_error_warning_msg.wrn_can_interface_blocked;
				lux_error_warning_msg.wrn_eth_interface_blocked = lux_error_warning_msg.wrn_eth_interface_blocked;
				lux_error_warning_msg.wrn_incorrect_can_data_rcvd = lux_error_warning_msg.wrn_incorrect_can_data_rcvd;
				lux_error_warning_msg.wrn_int_incorrect_scan_data = lux_error_warning_msg.wrn_int_incorrect_scan_data;
				lux_error_warning_msg.wrn_eth_unkwn_incomplete_data = lux_error_warning_msg.wrn_eth_unkwn_incomplete_data;
				lux_error_warning_msg.wrn_incorrect_or_forbidden_cmd_rcvd = lux_error_warning_msg.wrn_incorrect_or_forbidden_cmd_rcvd;
				lux_error_warning_msg.wrn_memory_access_failure = lux_error_warning_msg.wrn_memory_access_failure;
				lux_error_warning_msg.wrn_int_overflow = lux_error_warning_msg.wrn_int_overflow;
				lux_error_warning_msg.wrn_ego_motion_data_missing = lux_error_warning_msg.wrn_ego_motion_data_missing;
				lux_error_warning_msg.wrn_incorrect_mounting_params = lux_error_warning_msg.wrn_incorrect_mounting_params;
				lux_error_warning_msg.wrn_no_obj_comp_due_to_scan_freq = lux_error_warning_msg.wrn_no_obj_comp_due_to_scan_freq;

        lux_error_warning_msg.header.frame_id = frame_id;
        lux_error_warning_msg.header.stamp = ros::Time::now();
        error_warn_pub.publish(lux_error_warning_msg);
      }
      else if(is_fusion && lux_header_msg.data_type_id == ScanData2204::DATA_TYPE)
      {       
        ROS_DEBUG("Ibeo LUX - Reading FUSION SYSTEM/ECU scan data 0x2204");

        //publish the point cloud
        pcl::PointCloud <pcl::PointXYZL> pcl_cloud_2204;
        pcl::PointXYZL cloud_point_2204;

        lux_fusion_scan_2204.ibeo_header = lux_header_msg;

        fusion_scan_data_2204_tx.parse(data_msg);

        lux_fusion_scan_2204.scan_start_time = fusion_scan_data_2204_tx.scan_start_time; 
        lux_fusion_scan_2204.scan_end_time_offset = fusion_scan_data_2204_tx.scan_end_time_offset;
        lux_fusion_scan_2204.ground_labeled = fusion_scan_data_2204_tx.ground_labeled;
        lux_fusion_scan_2204.dirt_labeled = fusion_scan_data_2204_tx.dirt_labeled;
        lux_fusion_scan_2204.rain_labeled = fusion_scan_data_2204_tx.rain_labeled;
        lux_fusion_scan_2204.mirror_side = static_cast<uint8_t>(fusion_scan_data_2204_tx.mirror_side);
        lux_fusion_scan_2204.coordinate_system = static_cast<uint8_t>(fusion_scan_data_2204_tx.coordinate_system);
        lux_fusion_scan_2204.scan_number = fusion_scan_data_2204_tx.scan_number;
        lux_fusion_scan_2204.scan_points = fusion_scan_data_2204_tx.scan_points;
        lux_fusion_scan_2204.number_of_scanner_infos = fusion_scan_data_2204_tx.number_of_scanner_infos;

        ibeo_msgs::ScannerInfo2204 scan_info_2204;
        ScannerInfo2204 info_tx;
        lux_fusion_scan_2204.scanner_info_list.clear();

        for(int k = 0; k < lux_fusion_scan_2204.number_of_scanner_infos; k++)
        {
          info_tx = fusion_scan_data_2204_tx.scanner_info_list[k];
          scan_info_2204.device_id = info_tx.device_id;
          scan_info_2204.scanner_type = info_tx.scanner_type;
          scan_info_2204.scan_number = info_tx.scan_number;
          scan_info_2204.start_angle = info_tx.start_angle;
          scan_info_2204.end_angle = info_tx.end_angle;
          scan_info_2204.yaw_angle = info_tx.yaw_angle;
          scan_info_2204.pitch_angle = info_tx.pitch_angle;
          scan_info_2204.roll_angle = info_tx.roll_angle;
          scan_info_2204.offset_x = info_tx.offset_x;
          scan_info_2204.offset_y = info_tx.offset_y;
          scan_info_2204.offset_z = info_tx.offset_z;
        
          lux_fusion_scan_2204.scanner_info_list.push_back(scan_info_2204);
        }

        pcl_cloud_2204.reserve(lux_fusion_scan_2204.scan_points);

        ibeo_msgs::ScanPoint2204 scan_point_2204;
        ScanPoint2204 point_tx;
        lux_fusion_scan_2204.scan_point_list.clear();

        for(int k = 0; k < lux_fusion_scan_2204.scan_points; k++)
        {
          point_tx = fusion_scan_data_2204_tx.scan_point_list[k];
          scan_point_2204.x_position = point_tx.x_position;
          scan_point_2204.y_position = point_tx.y_position;
          scan_point_2204.z_position = point_tx.z_position;
          scan_point_2204.echo_width = point_tx.echo_width;
          scan_point_2204.device_id = point_tx.device_id;
          scan_point_2204.layer = point_tx.layer;
          scan_point_2204.echo = point_tx.echo;
          scan_point_2204.time_offset = point_tx.time_offset;
          scan_point_2204.ground = point_tx.ground;
          scan_point_2204.dirt = point_tx.dirt;
          scan_point_2204.precipitation = point_tx.precipitation;

          //cloud_point_2204.label = scan_point_2204.flags;
          cloud_point_2204.x = scan_point_2204.x_position;
          cloud_point_2204.y = scan_point_2204.y_position;
          cloud_point_2204.z = scan_point_2204.z_position;
          pcl_cloud_2204.points.push_back(cloud_point_2204);

          lux_fusion_scan_2204.scan_point_list.push_back(scan_point_2204);
        }

        lux_fusion_scan_2204.header.frame_id = frame_id;
        lux_fusion_scan_2204.header.stamp = ros::Time::now();
        fusion_scan_2204_pub.publish(lux_fusion_scan_2204);

        pcl_cloud_2204.header.frame_id = frame_id;
        pcl_conversions::toPCL(ros::Time::now(), pcl_cloud_2204.header.stamp);
        pointcloud_pub.publish(pcl_cloud_2204);
      }
      else if(is_fusion && lux_header_msg.data_type_id == ScanData2205::DATA_TYPE)
      {
        ROS_DEBUG("Ibeo LUX - Reading FUSION SYSTEM/ECU scan data 0x2205");

        pcl::PointCloud <pcl::PointXYZL> pcl_cloud_2205;
        pcl::PointXYZL cloud_point_2205;

        lux_fusion_scan_2205.ibeo_header = lux_header_msg;
        
        fusion_scan_data_2205_tx.parse(data_msg);

        lux_fusion_scan_2205.scan_start_time = ntp_to_ros_time(fusion_scan_data_2205_tx.scan_start_time);
        lux_fusion_scan_2205.scan_end_time_offset = fusion_scan_data_2205_tx.scan_end_time_offset;
        lux_fusion_scan_2205.fused_scan = fusion_scan_data_2205_tx.fused_scan;
        lux_fusion_scan_2205.mirror_side = static_cast<uint8_t>(fusion_scan_data_2205_tx.mirror_side);
        lux_fusion_scan_2205.coordinate_system = static_cast<uint8_t>(fusion_scan_data_2205_tx.coordinate_system);
        lux_fusion_scan_2205.scan_number = fusion_scan_data_2205_tx.scan_number;
        lux_fusion_scan_2205.scan_points = fusion_scan_data_2205_tx.scan_points;
        lux_fusion_scan_2205.number_of_scanner_infos = fusion_scan_data_2205_tx.number_of_scanner_infos;

        ibeo_msgs::ScannerInfo2205 scan_info_2205;
        ScannerInfo2205 info_tx;
        lux_fusion_scan_2205.scanner_info_list.clear();

        for(int k = 0; k < lux_fusion_scan_2205.number_of_scanner_infos; k++)
        {
          info_tx = fusion_scan_data_2205_tx.scanner_info_list[k];
          scan_info_2205.device_id = info_tx.device_id;
          scan_info_2205.scanner_type = info_tx.scanner_type;
          scan_info_2205.scan_number = info_tx.scan_number;
          scan_info_2205.start_angle = info_tx.start_angle;
          scan_info_2205.end_angle = info_tx.end_angle;
          scan_info_2205.scan_start_time = ntp_to_ros_time(info_tx.scan_start_time);
          scan_info_2205.scan_end_time = ntp_to_ros_time(info_tx.scan_end_time);
          scan_info_2205.scan_start_time_from_device = ntp_to_ros_time(info_tx.scan_start_time_from_device);
          scan_info_2205.scan_end_time_from_device = ntp_to_ros_time(info_tx.scan_end_time_from_device);
          scan_info_2205.scan_frequency = info_tx.scan_frequency;
          scan_info_2205.beam_tilt = info_tx.beam_tilt;
          scan_info_2205.scan_flags = info_tx.scan_flags;
          scan_info_2205.mounting_position.yaw_angle = info_tx.mounting_position.yaw_angle;
          scan_info_2205.mounting_position.pitch_angle = info_tx.mounting_position.pitch_angle;
          scan_info_2205.mounting_position.roll_angle = info_tx.mounting_position.roll_angle;
          scan_info_2205.mounting_position.x_position = info_tx.mounting_position.x_position;
          scan_info_2205.mounting_position.y_position = info_tx.mounting_position.y_position;
          scan_info_2205.mounting_position.z_position = info_tx.mounting_position.z_position;

          scan_info_2205.resolutions[0].resolution_start_angle = info_tx.resolutions[0].resolution_start_angle;
          scan_info_2205.resolutions[0].resolution = info_tx.resolutions[0].resolution;

          scan_info_2205.resolutions[1].resolution_start_angle = info_tx.resolutions[1].resolution_start_angle;
          scan_info_2205.resolutions[1].resolution = info_tx.resolutions[1].resolution;

          scan_info_2205.resolutions[2].resolution_start_angle = info_tx.resolutions[2].resolution_start_angle;
          scan_info_2205.resolutions[2].resolution = info_tx.resolutions[2].resolution;

          scan_info_2205.resolutions[3].resolution_start_angle = info_tx.resolutions[3].resolution_start_angle;
          scan_info_2205.resolutions[3].resolution = info_tx.resolutions[3].resolution;

          scan_info_2205.resolutions[4].resolution_start_angle = info_tx.resolutions[4].resolution_start_angle;
          scan_info_2205.resolutions[4].resolution = info_tx.resolutions[4].resolution;

          scan_info_2205.resolutions[5].resolution_start_angle = info_tx.resolutions[5].resolution_start_angle;
          scan_info_2205.resolutions[5].resolution = info_tx.resolutions[5].resolution;

          scan_info_2205.resolutions[6].resolution_start_angle = info_tx.resolutions[6].resolution_start_angle;
          scan_info_2205.resolutions[6].resolution = info_tx.resolutions[6].resolution;

          scan_info_2205.resolutions[7].resolution_start_angle = info_tx.resolutions[7].resolution_start_angle;
          scan_info_2205.resolutions[7].resolution = info_tx.resolutions[7].resolution;

          lux_fusion_scan_2205.scanner_info_list.push_back(scan_info_2205);
        }

        pcl_cloud_2205.reserve(lux_fusion_scan_2205.scan_points);

        ibeo_msgs::ScanPoint2205  scan_point_2205;
        ScanPoint2205  point_tx;
        lux_fusion_scan_2205.scan_point_list.clear();
        
        for(int k = 0; k < lux_fusion_scan_2205.scan_points; k++)
        {
          point_tx = fusion_scan_data_2205_tx.scan_point_list[k];
          scan_point_2205.x_position = (float) point_tx.x_position;
          scan_point_2205.y_position = (float) point_tx.y_position;
          scan_point_2205.z_position = (float) point_tx.z_position;
          scan_point_2205.echo_width = (float) point_tx.echo_width;
          scan_point_2205.device_id = point_tx.device_id;
          scan_point_2205.layer = point_tx.layer;
          scan_point_2205.echo = point_tx.echo;
          scan_point_2205.time_offset = point_tx.time_offset;
          scan_point_2205.ground = point_tx.ground;
          scan_point_2205.dirt = point_tx.dirt;
          scan_point_2205.precipitation = point_tx.precipitation;
          scan_point_2205.transparent = point_tx.transparent;

          lux_fusion_scan_2205.scan_point_list.push_back(scan_point_2205);
          
          //cloud_point_2205.label = scan_point_2205.flags;
          cloud_point_2205.x = scan_point_2205.x_position;
          cloud_point_2205.y = scan_point_2205.y_position;
          cloud_point_2205.z = scan_point_2205.z_position;
          pcl_cloud_2205.points.push_back(cloud_point_2205);
        }

        lux_fusion_scan_2205.header.frame_id = frame_id;
        lux_fusion_scan_2205.header.stamp = ros::Time::now();
        fusion_scan_2205_pub.publish(lux_fusion_scan_2205);

        pcl_cloud_2205.header.frame_id = frame_id;
        pcl_conversions::toPCL(ros::Time::now(), pcl_cloud_2205.header.stamp);
        pointcloud_pub.publish(pcl_cloud_2205);
      }
      else if(is_fusion && lux_header_msg.data_type_id == ObjectData2225::DATA_TYPE)
      {
        ROS_DEBUG("Ibeo LUX - Reading Fusion object data 0x2225");
        visualization_msgs::MarkerArray object_markers_2225;

        lux_fusion_object_2225.ibeo_header = lux_header_msg;

        fusion_object_data_2225_tx.parse(data_msg);

        lux_fusion_object_2225.mid_scan_timestamp = ntp_to_ros_time(fusion_object_data_2225_tx.mid_scan_timestamp);
        lux_fusion_object_2225.number_of_objects = fusion_object_data_2225_tx.number_of_objects;

        ibeo_msgs::Object2225 scan_object;
        Object2225 object_tx;
        lux_fusion_object_2225.object_list.clear();

        for(int k = 0; k < lux_fusion_object_2225.number_of_objects; k++)
        {
          object_tx = fusion_object_data_2225_tx.object_list[k];
          scan_object.id = object_tx.id;
          scan_object.age = object_tx.age;
          scan_object.timestamp = ntp_to_ros_time(object_tx.timestamp);
          scan_object.hidden_status_age = object_tx.hidden_status_age;
          scan_object.classification = static_cast<uint8_t>(object_tx.classification);
          scan_object.classification_certainty = object_tx.classification_certainty;
          scan_object.classification_age = object_tx.classification_age;
          scan_object.bounding_box_center.x = object_tx.bounding_box_center.x;
          scan_object.bounding_box_center.y = object_tx.bounding_box_center.y;
          scan_object.bounding_box_size.x = object_tx.bounding_box_size.x;
          scan_object.bounding_box_size.y = object_tx.bounding_box_size.y;
          scan_object.object_box_center.x = object_tx.object_box_center.x;
          scan_object.object_box_center.y = object_tx.object_box_center.y;
          scan_object.object_box_center_sigma.x = object_tx.object_box_center_sigma.x;
          scan_object.object_box_center_sigma.y = object_tx.object_box_center_sigma.y;
          scan_object.object_box_size.x = object_tx.object_box_size.x;
          scan_object.object_box_size.y = object_tx.object_box_size.y;
          scan_object.yaw_angle = object_tx.yaw_angle;
          scan_object.relative_velocity.x = object_tx.relative_velocity.x;
          scan_object.relative_velocity.y = object_tx.relative_velocity.y;
          scan_object.relative_velocity_sigma.x = object_tx.relative_velocity_sigma.x;
          scan_object.relative_velocity_sigma.y = object_tx.relative_velocity_sigma.y;
          scan_object.absolute_velocity.x = object_tx.absolute_velocity.x;
          scan_object.absolute_velocity.y = object_tx.absolute_velocity.y;
          scan_object.absolute_velocity_sigma.x = object_tx.absolute_velocity_sigma.x;
          scan_object.absolute_velocity_sigma.y = object_tx.absolute_velocity_sigma.y;
          scan_object.number_of_contour_points = object_tx.number_of_contour_points;
          scan_object.closest_point_index = object_tx.closest_point_index;

          lux_fusion_object_2225.object_list.push_back(scan_object);
          tf::Quaternion quaternion = tf::createQuaternionFromYaw(scan_object.yaw_angle); 
          visualization_msgs::Marker object_marker_2225 = createWireframeMarker(scan_object.object_box_center.x,
                                                                                scan_object.object_box_center.y,
                                                                                scan_object.object_box_size.x,
                                                                                scan_object.object_box_size.y,
                                                                                0.75);
          object_marker_2225.header.frame_id = frame_id;
          object_marker_2225.id  = scan_object.id;
          object_marker_2225.lifetime = ros::Duration(0.2);
          object_marker_2225.pose.orientation.x = quaternion.x();
          object_marker_2225.pose.orientation.y = quaternion.y();
          object_marker_2225.pose.orientation.z = quaternion.z();
          object_marker_2225.pose.orientation.w = quaternion.w();
          object_marker_2225.color.a = 0.5;
          object_marker_2225.color.r = object_marker_2225.color.g = object_marker_2225.color.b = 1.0;
          object_marker_2225.frame_locked = false;

          std::string label;

          switch (scan_object.classification)
          {
            case 0:
              label = "Unclassified";
              // Unclassified - white
              break;
            case 1:
              label = "Unknown Small";
              // Unknown small - blue
              object_marker_2225.color.r = object_marker_2225.color.g = 0;
              break;
            case 2:
              label = "Unknown Big";
              // Unknown big - dark blue
              object_marker_2225.color.r = object_marker_2225.color.g = 0;
              object_marker_2225.color.b = 0.5;
              break;
            case 3:
              label = "Pedestrian";
              // Pedestrian - red
              object_marker_2225.color.g = object_marker_2225.color.b = 0;
              break;
            case 4:
              label = "Bike";
              // Bike - dark red
              object_marker_2225.color.g = object_marker_2225.color.b = 0;
              object_marker_2225.color.r = 0.5;
              break;
            case 5:
              label = "Car";
              // Car - green
              object_marker_2225.color.b = object_marker_2225.color.r = 0;
              break;
            case 6:
              label = "Truck";
              // Truck - dark gree
              object_marker_2225.color.b = object_marker_2225.color.r = 0;
              object_marker_2225.color.g = 0.5;
              break;
            case 12:
              label = "Under drivable";
              // Under drivable - grey
              object_marker_2225.color.r = object_marker_2225.color.b = object_marker_2225.color.g = 0.7;
              break;
            case 13:
              label = "Over drivable";
              // Over drivable - dark grey
              object_marker_2225.color.r = object_marker_2225.color.b = object_marker_2225.color.g = 0.4;
              break;
            default:
              label = "Unknown";
              object_marker_2225.color.r = object_marker_2225.color.b = object_marker_2225.color.g = 0.0;
              break;
          }
          object_marker_2225.ns = label;

          visualization_msgs::Marker   object_label;
          object_label.header.frame_id = frame_id;
          object_label.ns = label;
          object_label.id = scan_object.id + 1000;
          object_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          object_label.action = visualization_msgs::Marker::ADD;
          object_label.pose.position.x = scan_object.object_box_center.x;
          object_label.pose.position.y = scan_object.object_box_center.y;
          object_label.pose.position.z = 0.5;
          object_label.text = label;
          object_label.scale.z = 0.5;
          object_label.lifetime = object_marker_2225.lifetime;
          object_label.color.r = object_label.color.g = object_label.color.b = 1;
          object_label.color.a = 0.5;
          
          ibeo_msgs::Point2Df object_point;
          Point2Df point_tx;
          scan_object.contour_point_list.clear();
          
          visualization_msgs::Marker object_point_marker;
          object_point_marker.header.frame_id = frame_id;
          object_point_marker.type = visualization_msgs::Marker::POINTS;
          object_point_marker.action = visualization_msgs::Marker::ADD;
          object_point_marker.ns = label;
          object_point_marker.points.reserve(scan_object.number_of_contour_points);
          object_point_marker.lifetime = object_marker_2225.lifetime;
          object_point_marker.scale.x = 0.1;
          object_point_marker.scale.y = 0.1;
          object_point_marker.color.r = 1;
          object_point_marker.color.g = 1;
          object_point_marker.color.b = 1;
          object_point_marker.color.a = 0.7;
          
          for(int j =0; j< scan_object.number_of_contour_points; j++)
          {
            point_tx = object_tx.contour_point_list[j];
            object_point.x = point_tx.x;
            object_point.y = point_tx.y;

            scan_object.contour_point_list.push_back(object_point);
            geometry_msgs::Point dis_object_point;
            dis_object_point.x = object_point.x;
            dis_object_point.y = object_point.y;
            object_point_marker.points.push_back(dis_object_point);
          }

          object_marker_2225.header.stamp = ros::Time::now();
          object_markers_2225.markers.push_back(object_marker_2225);

          object_label.header.stamp = ros::Time::now();
          object_markers_2225.markers.push_back(object_label);

          object_point_marker.header.stamp = ros::Time::now();
          object_contour_points_pub.publish(object_point_marker);
        }

        lux_fusion_object_2225.header.frame_id = frame_id;
        lux_fusion_object_2225.header.stamp = ros::Time::now();
        fusion_object_2225_pub.publish(lux_fusion_object_2225);

        object_markers_pub.publish(object_markers_2225);
      }
      else if(is_fusion && lux_header_msg.data_type_id == ObjectData2280::DATA_TYPE)
      {
        ROS_DEBUG("Ibeo LUX - Reading Fusion object data 0x2280");
        visualization_msgs::MarkerArray object_markers_2280;

        lux_fusion_object_2280.ibeo_header = lux_header_msg;

        fusion_object_data_2280_tx.parse(data_msg);

        lux_fusion_object_2280.mid_scan_timestamp = ntp_to_ros_time(fusion_object_data_2280_tx.mid_scan_timestamp);
        lux_fusion_object_2280.number_of_objects = fusion_object_data_2280_tx.number_of_objects;

        ibeo_msgs::Object2280 scan_object;
        Object2280 object_tx;
        lux_fusion_object_2280.objects.clear();

        for(int k = 0; k < lux_fusion_object_2280.number_of_objects; k++)
        {
          object_tx = fusion_object_data_2280_tx.object_list[k];
          scan_object.id = object_tx.id;
          scan_object.tracking_model = static_cast<uint8_t>(object_tx.tracking_model);
          scan_object.mobility_of_dyn_object_detected = object_tx.mobility_of_dyn_object_detected;
          scan_object.motion_model_validated = object_tx.motion_model_validated;
          scan_object.object_age = object_tx.object_age;
          scan_object.timestamp = ntp_to_ros_time(object_tx.timestamp);
          scan_object.object_prediction_age = object_tx.object_prediction_age;
          scan_object.classification = static_cast<uint8_t>(object_tx.classification);
          scan_object.classification_certainty = object_tx.classification_certainty;
          scan_object.classification_age = object_tx.classification_age;
          scan_object.object_box_center.x = object_tx.object_box_center.x;
          scan_object.object_box_center.y = object_tx.object_box_center.y;
          scan_object.object_box_size.x = object_tx.object_box_size.x;
          scan_object.object_box_size.y = object_tx.object_box_size.y;
          scan_object.object_box_orientation_angle = object_tx.object_box_orientation_angle;
          scan_object.object_box_orientation_angle_sigma = object_tx.object_box_orientation_angle_sigma;
          scan_object.relative_velocity.x = object_tx.relative_velocity.x;
          scan_object.relative_velocity.y = object_tx.relative_velocity.y;
          scan_object.relative_velocity_sigma.x = object_tx.relative_velocity_sigma.x;
          scan_object.relative_velocity_sigma.y = object_tx.relative_velocity_sigma.y;
          scan_object.absolute_velocity.x = object_tx.absolute_velocity.x;
          scan_object.absolute_velocity.y = object_tx.absolute_velocity.y;
          scan_object.absolute_velocity_sigma.x = object_tx.absolute_velocity_sigma.x;
          scan_object.absolute_velocity_sigma.y = object_tx.absolute_velocity_sigma.y;
          scan_object.number_of_contour_points = object_tx.number_of_contour_points;
          scan_object.closest_point_index = object_tx.closest_point_index;
          scan_object.reference_point_location = static_cast<uint8_t>(object_tx.reference_point_location);
          scan_object.reference_point_coordinate.x = object_tx.reference_point_coordinate.x;
          scan_object.reference_point_coordinate.y = object_tx.reference_point_coordinate.y;
          scan_object.reference_point_coordinate_sigma.x = object_tx.reference_point_coordinate_sigma.x;
          scan_object.reference_point_coordinate_sigma.y = object_tx.reference_point_coordinate_sigma.y;
          scan_object.object_priority = object_tx.object_priority;
          scan_object.object_existence_measurement = object_tx.object_existence_measurement;

          lux_fusion_object_2280.objects.push_back(scan_object);
          tf::Quaternion quaternion = tf::createQuaternionFromYaw(scan_object.object_box_orientation_angle); 
          visualization_msgs::Marker   object_marker_2280 = createWireframeMarker(scan_object.object_box_center.x,
                                                                                  scan_object.object_box_center.y,
                                                                                  scan_object.object_box_size.x,
                                                                                  scan_object.object_box_size.y,
                                                                                  0.75);;
          object_marker_2280.header.frame_id = frame_id;
          object_marker_2280.id = scan_object.id;
          object_marker_2280.lifetime = ros::Duration(0.2);
          object_marker_2280.pose.orientation.x = quaternion.x();
          object_marker_2280.pose.orientation.y = quaternion.y();
          object_marker_2280.pose.orientation.z = quaternion.z();
          object_marker_2280.pose.orientation.w = quaternion.w();
          object_marker_2280.color.a = 0.5;
          object_marker_2280.color.r = object_marker_2280.color.g = object_marker_2280.color.b = 1.0;
          object_marker_2280.frame_locked = false;

          std::string label;

          switch (scan_object.classification)
          {
            case 0:
              label = "Unclassified";
              // Unclassified - white
              break;
            case 1:
              label = "Unknown Small";
              // Unknown small - blue
              object_marker_2280.color.r = object_marker_2280.color.g = 0;
              break;
            case 2:
              label = "Unknown Big";
              // Unknown big - dark blue
              object_marker_2280.color.r = object_marker_2280.color.g = 0;
              object_marker_2280.color.b = 0.5;
              break;
            case 3:
              label = "Pedestrian";
              // Pedestrian - red
              object_marker_2280.color.g = object_marker_2280.color.b = 0;
              break;
            case 4:
              label = "Bike";
              // Bike - dark red
              object_marker_2280.color.g = object_marker_2280.color.b = 0;
              object_marker_2280.color.r = 0.5;
              break;
            case 5:
              label = "Car";
              // Car - green
              object_marker_2280.color.b = object_marker_2280.color.r = 0;
              break;
            case 6:
              label = "Truck";
              // Truck - dark gree
              object_marker_2280.color.b = object_marker_2280.color.r = 0;
              object_marker_2280.color.g = 0.5;
              break;
            case 12:
              label = "Under drivable";
              // Under drivable - grey
              object_marker_2280.color.r = object_marker_2280.color.b = object_marker_2280.color.g = 0.7;
              break;
            case 13:
              label = "Over drivable";
              // Over drivable - dark grey
              object_marker_2280.color.r = object_marker_2280.color.b = object_marker_2280.color.g = 0.4;
              break;
            default:
              label = "Unknown";
              object_marker_2280.color.r = object_marker_2280.color.b = object_marker_2280.color.g = 0.0;
              break;
          }
          object_marker_2280.ns = label;

          visualization_msgs::Marker object_label;
          object_label.header.frame_id = frame_id;
          object_label.ns = label;
          object_label.id = scan_object.id + 1000;
          object_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          object_label.action = visualization_msgs::Marker::ADD;
          object_label.pose.position.x = scan_object.object_box_center.x;
          object_label.pose.position.y = scan_object.object_box_center.y;
          object_label.pose.position.z = 0.5;
          object_label.text = label;
          object_label.scale.z = 0.5;
          object_label.lifetime = object_marker_2280.lifetime;
          object_label.color.r = object_label.color.g = object_label.color.b = 1;
          object_label.color.a = 0.5;
          
          ibeo_msgs::Point2Df object_point;
          Point2Df point_tx;
          scan_object.contour_point_list.clear();
          visualization_msgs::Marker object_point_marker;
          object_point_marker.header.frame_id = frame_id;
          object_point_marker.type = visualization_msgs::Marker::POINTS;
          object_point_marker.action = visualization_msgs::Marker::ADD;
          object_point_marker.lifetime = object_marker_2280.lifetime;
          object_point_marker.ns = label;
          object_point_marker.points.reserve(scan_object.number_of_contour_points);
          object_point_marker.scale.x = 0.1;
          object_point_marker.scale.y = 0.1;
          object_point_marker.color.r = 0;
          object_point_marker.color.g = 1;
          object_point_marker.color.b = 0;
          object_point_marker.color.a = 0.7;
          scan_object.contour_point_list.clear();
          geometry_msgs::Point dis_object_point;

          for(unsigned int j = 0; j < scan_object.number_of_contour_points; j++)
          {
            point_tx = object_tx.contour_point_list[j];
            object_point.x = point_tx.x;
            object_point.y = point_tx.y;

            scan_object.contour_point_list.push_back(object_point);
            
            geometry_msgs::Point dis_object_point;
            dis_object_point.x = object_point.x;
            dis_object_point.y = object_point.y;

            object_point_marker.points.push_back(dis_object_point);
          }
          object_marker_2280.header.stamp = ros::Time::now();
          object_markers_2280.markers.push_back(object_marker_2280);

          object_label.header.stamp = ros::Time::now();
          object_markers_2280.markers.push_back(object_label);

          object_point_marker.header.stamp = ros::Time::now();
          object_contour_points_pub.publish(object_point_marker);
        }

        fusion_object_2280_pub.publish(lux_fusion_object_2280);

        object_markers_pub.publish(object_markers_2280);
      }
      else if(is_fusion && lux_header_msg.data_type_id == CameraImage::DATA_TYPE)
      {
        ROS_DEBUG("Ibeo LUX - Reading FUSION SYSTEM/ECU image 2403");
        lux_fusion_image_msg.ibeo_header = lux_header_msg;

        //fusion_image_tx.header_ = header_tx;
        fusion_image_tx.parse(data_msg);

        lux_fusion_image_msg.image_format = fusion_image_tx.image_format;
        lux_fusion_image_msg.us_since_power_on = fusion_image_tx.us_since_power_on;
        lux_fusion_image_msg.timestamp = ntp_to_ros_time(fusion_image_tx.timestamp);
        lux_fusion_image_msg.device_id = fusion_image_tx.device_id;
        lux_fusion_image_msg.mounting_position.yaw_angle = fusion_image_tx.mounting_position.yaw_angle;
        lux_fusion_image_msg.mounting_position.pitch_angle = fusion_image_tx.mounting_position.pitch_angle;
        lux_fusion_image_msg.mounting_position.roll_angle = fusion_image_tx.mounting_position.roll_angle;
        lux_fusion_image_msg.mounting_position.x_position = fusion_image_tx.mounting_position.x_position;
        lux_fusion_image_msg.mounting_position.y_position = fusion_image_tx.mounting_position.y_position;
        lux_fusion_image_msg.mounting_position.z_position = fusion_image_tx.mounting_position.z_position;
        lux_fusion_image_msg.horizontal_opening_angle = fusion_image_tx.horizontal_opening_angle;
        lux_fusion_image_msg.vertical_opening_angle = fusion_image_tx.vertical_opening_angle;
        lux_fusion_image_msg.image_width = fusion_image_tx.image_width;
        lux_fusion_image_msg.image_height = fusion_image_tx.image_height;
        lux_fusion_image_msg.compressed_size = fusion_image_tx.compressed_size;
        lux_fusion_image_msg.image_buffer = fusion_image_tx.image_buffer; 
        
        lux_fusion_image_msg.header.frame_id = frame_id;
        lux_fusion_image_msg.header.stamp = ros::Time::now();
        fusion_img_2403_pub.publish(lux_fusion_image_msg);
      }
      else if(is_fusion && lux_header_msg.data_type_id == HostVehicleState2806::DATA_TYPE)
      {
        ROS_DEBUG("Ibeo LUX - Reading Fusion vehicle state data 0x2806");
        lux_vehicle_state_2806_msg.ibeo_header = lux_header_msg;

        fusion_vehicle_state_2806_tx.parse(data_msg);

        lux_vehicle_state_2806_msg.timestamp = ntp_to_ros_time(fusion_vehicle_state_2806_tx.timestamp);
        lux_vehicle_state_2806_msg.distance_x = fusion_vehicle_state_2806_tx.distance_x;
        lux_vehicle_state_2806_msg.distance_y = fusion_vehicle_state_2806_tx.distance_y;
        lux_vehicle_state_2806_msg.course_angle = fusion_vehicle_state_2806_tx.course_angle;
        lux_vehicle_state_2806_msg.longitudinal_velocity = fusion_vehicle_state_2806_tx.longitudinal_velocity;
        lux_vehicle_state_2806_msg.yaw_rate = fusion_vehicle_state_2806_tx.yaw_rate;
        lux_vehicle_state_2806_msg.steering_wheel_angle = fusion_vehicle_state_2806_tx.steering_wheel_angle;
        lux_vehicle_state_2806_msg.front_wheel_angle = fusion_vehicle_state_2806_tx.front_wheel_angle;
        lux_vehicle_state_2806_msg.vehicle_width = fusion_vehicle_state_2806_tx.vehicle_width;
        lux_vehicle_state_2806_msg.vehicle_front_to_front_axle = fusion_vehicle_state_2806_tx.vehicle_front_to_front_axle;
        lux_vehicle_state_2806_msg.rear_axle_to_front_axle = fusion_vehicle_state_2806_tx.rear_axle_to_front_axle;
        lux_vehicle_state_2806_msg.rear_axle_to_vehicle_rear = fusion_vehicle_state_2806_tx.rear_axle_to_vehicle_rear;
        lux_vehicle_state_2806_msg.steer_ratio_poly_0 = fusion_vehicle_state_2806_tx.steer_ratio_poly_0;
        lux_vehicle_state_2806_msg.steer_ratio_poly_1 = fusion_vehicle_state_2806_tx.steer_ratio_poly_1;
        lux_vehicle_state_2806_msg.steer_ratio_poly_2 = fusion_vehicle_state_2806_tx.steer_ratio_poly_2;
        lux_vehicle_state_2806_msg.steer_ratio_poly_3 = fusion_vehicle_state_2806_tx.steer_ratio_poly_3;

        lux_vehicle_state_msg.header.frame_id = frame_id;
        lux_vehicle_state_msg.header.stamp = ros::Time::now();
        fusion_vehicle_2806_pub.publish(lux_vehicle_state_msg);
      }
      else if(is_fusion && lux_header_msg.data_type_id == HostVehicleState2807::DATA_TYPE)
      {
        ROS_DEBUG("Ibeo LUX - Reading Fusion vehicle state data 0x2807");
        lux_vehicle_state_2807_msg.ibeo_header  = lux_header_msg;

        fusion_vehicle_state_2807_tx.parse(data_msg);

        lux_vehicle_state_2807_msg.timestamp = ntp_to_ros_time(fusion_vehicle_state_2807_tx.timestamp);
        lux_vehicle_state_2807_msg.distance_x = fusion_vehicle_state_2807_tx.distance_x;
        lux_vehicle_state_2807_msg.distance_y = fusion_vehicle_state_2807_tx.distance_y;
        lux_vehicle_state_2807_msg.course_angle = fusion_vehicle_state_2807_tx.course_angle;
        lux_vehicle_state_2807_msg.longitudinal_velocity = fusion_vehicle_state_2807_tx.longitudinal_velocity;
        lux_vehicle_state_2807_msg.yaw_rate = fusion_vehicle_state_2807_tx.yaw_rate;
        lux_vehicle_state_2807_msg.steering_wheel_angle = fusion_vehicle_state_2807_tx.steering_wheel_angle;
        lux_vehicle_state_2807_msg.cross_acceleration = fusion_vehicle_state_2807_tx.cross_acceleration;
        lux_vehicle_state_2807_msg.front_wheel_angle = fusion_vehicle_state_2807_tx.front_wheel_angle;
        lux_vehicle_state_2807_msg.vehicle_width = fusion_vehicle_state_2807_tx.vehicle_width;
        lux_vehicle_state_2807_msg.vehicle_front_to_front_axle = fusion_vehicle_state_2807_tx.vehicle_front_to_front_axle;
        lux_vehicle_state_2807_msg.rear_axle_to_front_axle = fusion_vehicle_state_2807_tx.rear_axle_to_front_axle;
        lux_vehicle_state_2807_msg.rear_axle_to_vehicle_rear = fusion_vehicle_state_2807_tx.rear_axle_to_vehicle_rear;
        lux_vehicle_state_2807_msg.steer_ratio_poly_0 = fusion_vehicle_state_2807_tx.steer_ratio_poly_0;
        lux_vehicle_state_2807_msg.steer_ratio_poly_1 = fusion_vehicle_state_2807_tx.steer_ratio_poly_1;
        lux_vehicle_state_2807_msg.steer_ratio_poly_2 = fusion_vehicle_state_2807_tx.steer_ratio_poly_2;
        lux_vehicle_state_2807_msg.steer_ratio_poly_3 = fusion_vehicle_state_2807_tx.steer_ratio_poly_3;
        lux_vehicle_state_2807_msg.longitudinal_acceleration = fusion_vehicle_state_2807_tx.longitudinal_acceleration;

        lux_vehicle_state_2807_msg.header.frame_id = frame_id;
        lux_vehicle_state_2807_msg.header.stamp = ros::Time::now();
        fusion_vehicle_2807_pub.publish(lux_vehicle_state_2807_msg);
      }
    } //If sensor is connected

    ros::spinOnce();
    loop_rate.sleep();
  }

  status = tcp_interface.close();

  if (status != OK)
    ROS_ERROR("Ibeo LUX - Closing the connection to the LUX failed: %i - %s", status, return_status_desc(status).c_str());

  return 0;
}

