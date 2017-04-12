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
#include <csignal>

#include <core_ibeo_lux.h>
#include <network_interface/network_interface.h>

//Ros
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>

//Tx
#include <network_interface/TCPFrame.h>

#include <ibeo_lux_msgs/LuxHeader.h>
// single lux
#include <ibeo_lux_msgs/LuxErrorWarning.h>
#include <ibeo_lux_msgs/LuxObject.h>
#include <ibeo_lux_msgs/LuxObjectData.h>
#include <ibeo_lux_msgs/LuxVehicleState.h>
#include <ibeo_lux_msgs/LuxScanData.h>

#include <ibeo_lux_msgs/FusionScanData2204.h>
#include <ibeo_lux_msgs/FusionScanInfo2204.h>
#include <ibeo_lux_msgs/FusionScanData2205.h>
#include <ibeo_lux_msgs/FusionScanInfo2205.h>
#include <ibeo_lux_msgs/FusionScanPoint.h>
#include <ibeo_lux_msgs/FusionObjectData2225.h>
#include <ibeo_lux_msgs/FusionObject2225.h>
#include <ibeo_lux_msgs/FusionObjectData2280.h>
#include <ibeo_lux_msgs/FusionObject2280.h>
#include <ibeo_lux_msgs/FusionImage.h>
#include <ibeo_lux_msgs/FusionVehicleState2806.h>
#include <ibeo_lux_msgs/FusionVehicleState2807.h>
// supplemental objects/messages

#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>


using namespace std;
using namespace AS::Network;

TCPInterface tcp_interface;

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
  ros::Rate loop_rate(1.0/0.01);
  bool exit = false;
  if (priv.getParam("ip_address", ip_address))
  {
    ROS_INFO("Got ip_address: %s", ip_address.c_str());
    if (ip_address == "" )
    {
     ROS_ERROR("IP Address Invalid");
     exit = true;
    }
  }
  if (priv.getParam("port", port))
  {
    ROS_INFO("Got port: %d", port);
    if (port < 0)
    {
     ROS_ERROR("Port Invalid");
     exit = true;
    }
  }
  if (priv.getParam("is_fusion", is_fusion))
  {
    ROS_INFO("is Fusion ECU: %s", (is_fusion)? "true" : "false");
  }
  if (priv.getParam("sensor_frame_id", frame_id))
  {
    ROS_INFO("Got sensor frame ID: %s", frame_id.c_str());
  }
  if(exit)
    return 0;

  // Advertise messages to send
  ros::Publisher raw_tcp_pub = n.advertise<network_interface::TCPFrame>("tcp_tx", 1);
  ros::Publisher scan_data_pub, object_data_pub, vehicle_state_pub, error_warn_pub, scan_pointcloud_2202_pub, object_markers_2221_pub, object_marker_2221_pub;
  ros::Publisher fusion_scan_2204_pub, fusion_scan_2205_pub, fusion_object_2225_pub, fusion_object_2280_pub, fusion_img_2403_pub, fusion_vehicle_2806_pub, fusion_vehicle_2807_pub;
  ros::Publisher scan_pointcloud_2204_pub, scan_pointcloud_2205_pub, object_markers_2225_pub, object_markers_2280_pub;

  //LUX Sensor Only
  if(!is_fusion)
  {
  scan_data_pub = n.advertise<ibeo_lux_msgs::LuxScanData>("parsed_tx/lux_scan_data", 1);
  object_data_pub = n.advertise<ibeo_lux_msgs::LuxObjectData>("parsed_tx/lux_object_data", 1);
  vehicle_state_pub = n.advertise<ibeo_lux_msgs::LuxVehicleState>("parsed_tx/lux_vehicle_state", 1);
  error_warn_pub = n.advertise<ibeo_lux_msgs::LuxErrorWarning>("parsed_tx/lux_error_warning", 1);

  scan_pointcloud_2202_pub = n.advertise<pcl::PointCloud <pcl::PointXYZ> >("as_tx/lux_point_cloud_2202", 1);
  object_markers_2221_pub = n.advertise<visualization_msgs::MarkerArray>("as_tx/object_markers_array_2221", 1);
  object_marker_2221_pub = n.advertise<visualization_msgs::Marker>("as_tx/object_contour_points_2221", 1);
  }//Fusion ECU Only
  else
  {
  fusion_scan_2204_pub = n.advertise<ibeo_lux_msgs::FusionScanData2204>("parsed_tx/fusion_scan_data_2204", 1);
  fusion_scan_2205_pub = n.advertise<ibeo_lux_msgs::FusionScanData2205>("parsed_tx/fusion_scan_data_2205", 1);
  fusion_object_2225_pub = n.advertise<ibeo_lux_msgs::FusionObjectData2225>("parsed_tx/fusion_object_data_2225", 1);
  fusion_object_2280_pub = n.advertise<ibeo_lux_msgs::FusionObjectData2280>("parsed_tx/fusion_object_data_2280", 1);
  fusion_img_2403_pub = n.advertise<ibeo_lux_msgs::FusionImage>("parsed_tx/fusion_image_2403", 1);
  fusion_vehicle_2806_pub = n.advertise<ibeo_lux_msgs::FusionVehicleState2806>("parsed_tx/lux_fusion_vehicle_state_2806", 1);
  fusion_vehicle_2807_pub = n.advertise<ibeo_lux_msgs::FusionVehicleState2807>("parsed_tx/lux_fusion_vehicle_state_2807", 1);

  scan_pointcloud_2204_pub = n.advertise<pcl::PointCloud <pcl::PointXYZ> >("as_tx/lux_fusion_point_cloud_2204", 1);
  scan_pointcloud_2205_pub = n.advertise<pcl::PointCloud <pcl::PointXYZ> >("as_tx/lux_fusion_point_cloud_2205", 1);
  object_markers_2225_pub = n.advertise<visualization_msgs::MarkerArray>("as_tx/object_markers_array_2225", 1);
  object_markers_2280_pub = n.advertise<visualization_msgs::MarkerArray>("as_tx/object_markers_array_2280", 1);
  }

  // Wait for time to be valid
  while (ros::Time::now().nsec == 0);

  ibeo_lux_msgs::LuxHeader lux_header_msg;
  ibeo_lux_msgs::LuxScanData lux_scan_msg;
  ibeo_lux_msgs::LuxObjectData lux_object_msg;
  ibeo_lux_msgs::LuxVehicleState lux_vehicle_state_msg;
  ibeo_lux_msgs::LuxErrorWarning lux_error_warning_msg;
  ibeo_lux_msgs::FusionScanData2204 lux_fusion_scan_2204;
  ibeo_lux_msgs::FusionScanData2205 lux_fusion_scan_2205;
  ibeo_lux_msgs::FusionObjectData2225 lux_fusion_object_2225;
  ibeo_lux_msgs::FusionObjectData2280 lux_fusion_object_2280;
  ibeo_lux_msgs::FusionImage lux_fusion_image_msg;
  ibeo_lux_msgs::FusionVehicleState2806 lux_vehicle_state_2806_msg;
  ibeo_lux_msgs::FusionVehicleState2807 lux_vehicle_state_2807_msg;
  
  network_interface::TCPFrame tcp_raw_msg;

  LuxHeader_TX_Message header_tx;
  LuxScanData_TX_Message lux_scan_data_tx;
  LuxObjectData_TX_Message lux_object_data_tx;
  LuxVehicleState_TX_Message lux_vehicle_state_tx;
  LuxErrorWarning_TX_Message lux_error_warning_tx;
  FusionScanData2204_TX_Message fusion_scan_data_2204_tx;
  FusionScanData2205_TX_Message fusion_scan_data_2205_tx;
  FusionObjectData2225_TX_Message fusion_object_data_2225_tx;
  FusionObjectData2280_TX_Message fusion_object_data_2280_tx;
  FusionImage_TX_Message fusion_image_tx;
  FusionVehicleState2806_TX_Message fusion_vehicle_state_2806_tx;
  FusionVehicleState2807_TX_Message fusion_vehicle_state_2807_tx;
  return_statuses status = tcp_interface.open(ip_address.c_str(), port);
  if(status == ok)
  {
    ROS_INFO("LUX connected");
    // Loop as long as module should run
    
    unsigned char head_msg[LUX_HEADER_SIZE]; 
    if(is_fusion)
    {
      unsigned char set_filter_cmd[32] = {0xaf, 0xfe, 0xc0, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x02, 0x00, 0x00, 0xff,0xff};
      status = tcp_interface.send(set_filter_cmd, sizeof(set_filter_cmd));
    }
    while (ros::ok())
    {
      status = tcp_interface.read_exactly(head_msg, sizeof(head_msg), LUX_HEADER_SIZE);
      int offset = header_tx.parse(head_msg);
      if(offset > 0)
      {
        ROS_WARN("LUX/LUX Fusion TCP Data out of sync. Offset by %d bytes", offset);
        unsigned char temparray[LUX_HEADER_SIZE];
        memcpy(temparray, head_msg + offset, (LUX_HEADER_SIZE - offset) * sizeof(char));
        status = tcp_interface.read_exactly(head_msg,sizeof(head_msg), offset);
        memcpy(temparray + LUX_HEADER_SIZE - offset, head_msg, offset * sizeof(char));
        memcpy(head_msg, temparray, LUX_HEADER_SIZE * sizeof(char));
      }
      
      unsigned char data_msg[header_tx.message_size_];

      status = tcp_interface.read_exactly(data_msg, sizeof(data_msg), header_tx.message_size_);
      tcp_raw_msg.address = ip_address;
      tcp_raw_msg.port = port;
      tcp_raw_msg.data.clear();
      for(int i = 0; i < LUX_HEADER_SIZE; i++)
      {
        tcp_raw_msg.data.push_back(head_msg[i]);
      }
      for(int i = 0; i < header_tx.message_size_; i++)
      {
        tcp_raw_msg.data.push_back(data_msg[i]);
      }
      tcp_raw_msg.size = tcp_raw_msg.data.size();
      
      tcp_raw_msg.header.stamp = ros::Time::now();
      raw_tcp_pub.publish(tcp_raw_msg);

      lux_header_msg.message_size = header_tx.message_size_;
      lux_header_msg.device_id = header_tx.device_id_;
      lux_header_msg.data_type = header_tx.data_type_;

      if(!is_fusion && lux_header_msg.data_type == 0x2202)
      {
        ROS_INFO("reading scan data 0x2202");

        pcl::PointCloud <pcl::PointXYZL> pcl_cloud_2202;
        pcl::PointXYZL cloud_point_2202;
        
        lux_scan_msg.lux_header = lux_header_msg;

        // header needs to be set before parse since the offset that the
        // tx message uses is contained in the header.
        // shouldnt be required when a sync process is added if the MAGIC WORD is offset
        lux_scan_data_tx.header_ = header_tx;
        lux_scan_data_tx.parse(data_msg);

        lux_scan_msg.scan_number = lux_scan_data_tx.scan_number_;
        lux_scan_msg.scan_status = lux_scan_data_tx.scan_status_;
        lux_scan_msg.sync_phase_offset = lux_scan_data_tx.sync_phase_offset_;
        lux_scan_msg.scan_start_time = lux_scan_data_tx.scan_start_time_;
        lux_scan_msg.scan_end_time = lux_scan_data_tx.scan_end_time_;
        lux_scan_msg.angle_ticks = lux_scan_data_tx.angle_ticks_;
        lux_scan_msg.start_angle = lux_scan_data_tx.start_angle_;
        lux_scan_msg.end_angle = lux_scan_data_tx.end_angle_;
        lux_scan_msg.num_scan_pts = lux_scan_data_tx.num_scan_pts_;
        lux_scan_msg.mounting_yaw_angle = lux_scan_data_tx.mounting_yaw_angle_;
        lux_scan_msg.mounting_pitch_angle = lux_scan_data_tx.mounting_pitch_angle_;
        lux_scan_msg.mounting_roll_angle = lux_scan_data_tx.mounting_roll_angle_;
        lux_scan_msg.mounting_position_x = lux_scan_data_tx.mounting_position_x_;
        lux_scan_msg.mounting_position_y = lux_scan_data_tx.mounting_position_y_;
        lux_scan_msg.mounting_position_z = lux_scan_data_tx.mounting_position_z_;
        lux_scan_msg.scan_flags = lux_scan_data_tx.scan_flags_;

        pcl_cloud_2202.reserve(lux_scan_msg.num_scan_pts);
        lux_scan_msg.scan_points.clear();

        ibeo_lux_msgs::ScanPoint msg_point;
        ScanPoint tx_point;
        for(int k = 0; k < lux_scan_msg.num_scan_pts; k++)
        {
          tx_point = lux_scan_data_tx.scan_points_[k];
          msg_point.layer = tx_point.layer_;
          msg_point.echo = tx_point.echo_;
          msg_point.flags = tx_point.flags_;
          msg_point.horizontal_angle = tx_point.horizontal_angle_;
          msg_point.radial_distance = tx_point.radial_distance_;
          msg_point.echo_pulse_width = tx_point.echo_pulse_width_;
          
          lux_scan_msg.scan_points.push_back(msg_point);

          
          cloud_point_2202.label = msg_point.flags;
          // filter out flagged points
          if(!(msg_point.flags & 0x0F))
          {
            double phi;
            switch (msg_point.layer)
            {
              case 0: phi = -1.6 * PI / 180.0; break;
              case 1: phi = -0.8 * PI / 180.0; break;
              case 2: phi =  0.8 * PI / 180.0; break;
              case 3: phi =  1.6 * PI / 180.0; break;
              default: phi = 0.0; break;
            }

            cloud_point_2202.x = (double)msg_point.radial_distance/100*cos(convertAngle(msg_point.horizontal_angle,lux_scan_msg.angle_ticks))*cos(phi);
            cloud_point_2202.y = (double)msg_point.radial_distance/100*sin(convertAngle(msg_point.horizontal_angle,lux_scan_msg.angle_ticks))*cos(phi);
            cloud_point_2202.z = (double)msg_point.radial_distance/100*sin(phi);

            pcl_cloud_2202.points.push_back(cloud_point_2202);
          }
        }

        lux_scan_msg.header.frame_id = frame_id;
        lux_scan_msg.header.stamp = ros::Time::now();
        scan_data_pub.publish(lux_scan_msg);

        pcl_cloud_2202.header.frame_id = frame_id;
        pcl_conversions::toPCL(ros::Time::now(), pcl_cloud_2202.header.stamp);
        scan_pointcloud_2202_pub.publish(pcl_cloud_2202);
                  
      }
      // ibeo LUX object data
      else if (!is_fusion && lux_header_msg.data_type == 0x2221)
      {
        ROS_INFO("reading object data 0x2221");
        visualization_msgs::MarkerArray   object_markers;

        lux_object_msg.lux_header = lux_header_msg;

        lux_object_data_tx.header_ = header_tx;
        lux_object_data_tx.parse(data_msg);
        
        lux_object_msg.scan_start_time = lux_object_data_tx.scan_start_time_;
        lux_object_msg.num_of_objects = lux_object_data_tx.num_of_objects_;

        ibeo_lux_msgs::LuxObject scan_object;
        LuxObject object_tx;
        ibeo_lux_msgs::Point2D     object_point;
        Point2D point_tx;
        lux_object_msg.objects.clear();
        for(int k = 0; k < lux_object_msg.num_of_objects; k++)
        {
          object_tx = lux_object_data_tx.objects_[k];
          scan_object.ID = object_tx.ID_;
          scan_object.age = object_tx.age_;
          scan_object.prediction_age = object_tx.prediction_age_;
          scan_object.relative_timestamp = object_tx.relative_timestamp_;
          scan_object.reference_point.x = object_tx.reference_point_.x_;
          scan_object.reference_point.y = object_tx.reference_point_.y_;
          scan_object.reference_point_sigma.x = object_tx.reference_point_sigma_.x_;
          scan_object.reference_point_sigma.y = object_tx.reference_point_sigma_.y_;
          scan_object.closest_point.x = object_tx.closest_point_.x_;
          scan_object.closest_point.y = object_tx.closest_point_.y_;
          scan_object.bounding_box_center.x = 0.01 * object_tx.bounding_box_center_.x_;
          scan_object.bounding_box_center.y = 0.01 * object_tx.bounding_box_center_.y_;
          scan_object.bounding_box_width = 0.01 * object_tx.bounding_box_width_;
          scan_object.bounding_box_length = 0.01 * object_tx.bounding_box_length_;
          scan_object.object_box_center.x = 0.01 * object_tx.object_box_center_.x_;
          scan_object.object_box_center.y = 0.01 * object_tx.object_box_center_.y_;
          scan_object.object_box_size.x = 0.01 * object_tx.object_box_size_.x_;
          scan_object.object_box_size.y = 0.01 * object_tx.object_box_size_.y_;
          scan_object.object_box_orientation = (float)object_tx.object_box_orientation_;
          scan_object.absolute_velocity.x = object_tx.absolute_velocity_.x_;
          scan_object.absolute_velocity.y = object_tx.absolute_velocity_.y_;
          scan_object.absolute_velocity_sigma.x = object_tx.absolute_velocity_sigma_.x_;
          scan_object.absolute_velocity_sigma.y = object_tx.absolute_velocity_sigma_.y_;
          scan_object.relative_velocity.x = object_tx.relative_velocity_.x_;
          scan_object.relative_velocity.y = object_tx.relative_velocity_.y_;
          scan_object.classification = object_tx.classification_;
          scan_object.classification_age = object_tx.classification_age_;
          scan_object.classification_certainty = object_tx.classification_certainty_;
          scan_object.number_of_contour_points = object_tx.number_of_contour_points_;

          lux_object_msg.objects.push_back(scan_object);
          
          tf::Quaternion quaternion = tf::createQuaternionFromYaw(scan_object.object_box_orientation * 100/180 * M_PI);

          visualization_msgs::Marker   object_marker = createWireframeMarker(scan_object.object_box_center.x, scan_object.object_box_center.y,
              scan_object.object_box_size.x, scan_object.object_box_size.y, 0.75);
          object_marker.header.frame_id = frame_id;
          object_marker.id  = scan_object.ID;
          object_marker.pose.orientation.x = quaternion.x();
          object_marker.pose.orientation.y = quaternion.y();
          object_marker.pose.orientation.z = quaternion.z();
          object_marker.pose.orientation.w = quaternion.w();
          object_marker.lifetime = ros::Duration(0.2);
          object_marker.color.a = 0.5;
          object_marker.color.r = object_marker.color.g = object_marker.color.b = 1.0;
          object_marker.frame_locked = false;

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
          
          visualization_msgs::Marker   object_label;
          object_label.header.frame_id = frame_id;
          object_label.id  = scan_object.ID + 1000;
          object_label.ns = label;
          object_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          object_label.action = visualization_msgs::Marker::ADD;
          object_label.pose.position.x = scan_object.object_box_center.x;
          object_label.pose.position.y = scan_object.object_box_center.y;
          object_label.pose.position.z = 0.5;
          object_label.text = label;
          object_label.scale.z = 0.2;
          object_label.lifetime = object_marker.lifetime;
          object_label.color.r = object_label.color.g = object_label.color.b = 1;
          object_label.color.a = 0.5;
          
          visualization_msgs::Marker      object_point_marker;
          object_point_marker.header.frame_id = frame_id;
          object_point_marker.type = visualization_msgs::Marker::POINTS;
          object_point_marker.action = visualization_msgs::Marker::ADD;
          object_point_marker.ns = label;
          object_point_marker.scale.x = 0.05;
          object_point_marker.scale.y = 0.05;
          object_point_marker.color.r = 0;
          object_point_marker.color.g = 1;
          object_point_marker.color.b = 0;
          object_point_marker.color.a = 0.7;

          scan_object.list_of_contour_points.clear();
          geometry_msgs::Point    dis_object_point;
          for(int j =0; j< scan_object.number_of_contour_points; j++)
          {
            point_tx = object_tx.list_of_contour_points_[k];
            object_point.x = 0.01 * point_tx.x_;
            object_point.y = 0.01 * point_tx.y_;

            scan_object.list_of_contour_points.push_back(object_point);
            
            dis_object_point.x = object_point.x;
            dis_object_point.y = object_point.y;

            object_point_marker.points.push_back(dis_object_point);
          }
          
          object_point_marker.header.stamp = ros::Time::now();
          object_marker_2221_pub.publish(object_point_marker);

          object_label.header.stamp = ros::Time::now();
          object_markers.markers.push_back(object_label);

          object_marker.header.stamp = ros::Time::now();
          object_markers.markers.push_back(object_marker);
        }

        lux_object_msg.header.frame_id = frame_id;
        lux_object_msg.header.stamp = ros::Time::now();
        object_data_pub.publish(lux_object_msg);
        object_markers_2221_pub.publish(object_markers);
      }
      //vehicle state 2805
      else if(!is_fusion && lux_header_msg.data_type == 0x2805)
      {
        ROS_INFO("reading vehicle state data 0x2805");
        lux_vehicle_state_msg.lux_header  = lux_header_msg;

        lux_vehicle_state_tx.header_ = header_tx;
        lux_vehicle_state_tx.parse(data_msg);

        lux_vehicle_state_msg.timestamp = lux_vehicle_state_tx.timestamp_;
        lux_vehicle_state_msg.scan_number = lux_vehicle_state_tx.scan_number_;
        lux_vehicle_state_msg.error_flags = lux_vehicle_state_tx.error_flags_;
        lux_vehicle_state_msg.longitudinal_velocity = lux_vehicle_state_tx.longitudinal_velocity_;
        lux_vehicle_state_msg.steering_wheel_angle = lux_vehicle_state_tx.steering_wheel_angle_;
        lux_vehicle_state_msg.front_wheel_angle = lux_vehicle_state_tx.front_wheel_angle_;
        lux_vehicle_state_msg.vehicle_x = lux_vehicle_state_tx.vehicle_x_;
        lux_vehicle_state_msg.vehicle_y = lux_vehicle_state_tx.vehicle_y_;
        lux_vehicle_state_msg.course_angle = lux_vehicle_state_tx.course_angle_;
        lux_vehicle_state_msg.time_difference = lux_vehicle_state_tx.time_difference_;
        lux_vehicle_state_msg.diff_in_x = lux_vehicle_state_tx.diff_in_x_;
        lux_vehicle_state_msg.diff_in_y = lux_vehicle_state_tx.diff_in_y_;
        lux_vehicle_state_msg.diff_in_heading = lux_vehicle_state_tx.diff_in_heading_;
        lux_vehicle_state_msg.current_yaw_rate = lux_vehicle_state_tx.current_yaw_rate_;

        lux_vehicle_state_msg.header.frame_id = frame_id;
        lux_vehicle_state_msg.header.stamp = ros::Time::now();
        vehicle_state_pub.publish(lux_vehicle_state_msg);
      }
      // error and warning
      else if(!is_fusion && lux_header_msg.data_type == 0x2030)
      {
        ROS_INFO("reading lux errors and warnings data 0x2030");
        lux_error_warning_msg.lux_header  = lux_header_msg;
       
        lux_error_warning_tx.header_ = header_tx;
        lux_error_warning_tx.parse(data_msg);
       
        lux_error_warning_msg.error_register1 = lux_error_warning_tx.error_register1_;
        lux_error_warning_msg.error_register2 = lux_error_warning_tx.error_register2_;
        lux_error_warning_msg.warning_register1 = lux_error_warning_tx.warning_register1_;
        lux_error_warning_msg.warning_register2 = lux_error_warning_tx.warning_register2_;

        lux_error_warning_msg.header.frame_id = frame_id;
        lux_error_warning_msg.header.stamp = ros::Time::now();
        error_warn_pub.publish(lux_error_warning_msg);
      }
      // Fusion scan data 2204
      else if(is_fusion && lux_header_msg.data_type == 0x2204)
      {       
        ROS_INFO("reading FUSION SYSTEM/ECU scan data 0x2204");

        //publish the point cloud
        pcl::PointCloud <pcl::PointXYZL> pcl_cloud_2204;
        pcl::PointXYZL cloud_point_2204;

        lux_fusion_scan_2204.lux_header = lux_header_msg;

        fusion_scan_data_2204_tx.header_ = header_tx;
        fusion_scan_data_2204_tx.parse(data_msg);

        lux_fusion_scan_2204.scan_start_time = fusion_scan_data_2204_tx.scan_start_time_; 
        lux_fusion_scan_2204.scan_end_time_offset = fusion_scan_data_2204_tx.scan_end_time_offset_;
        lux_fusion_scan_2204.flags = fusion_scan_data_2204_tx.flags_;
        lux_fusion_scan_2204.scan_number = fusion_scan_data_2204_tx.scan_number_;
        lux_fusion_scan_2204.num_scan_pts = fusion_scan_data_2204_tx.num_scan_pts_;
        lux_fusion_scan_2204.num_scan_info = fusion_scan_data_2204_tx.num_scan_info_;

        ibeo_lux_msgs::FusionScanInfo2204   scan_info_2204;
        FusionScanInfo2204 info_tx;
        lux_fusion_scan_2204.scan_info_list.clear();
        for(int k = 0; k < lux_fusion_scan_2204.num_scan_info; k++)
        {
          info_tx = fusion_scan_data_2204_tx.scan_info_list_[k];
          scan_info_2204.device_id = info_tx.device_id_;
          scan_info_2204.type = info_tx.type_;
          scan_info_2204.number = info_tx.number_;
          scan_info_2204.start_angle = info_tx.start_angle_;
          scan_info_2204.end_angle = info_tx.end_angle_;
          scan_info_2204.yaw_angle = info_tx.yaw_angle_;
          scan_info_2204.pitch_angle = info_tx.pitch_angle_;
          scan_info_2204.roll_angle = info_tx.roll_angle_;
          scan_info_2204.mount_offset_x = info_tx.mount_offset_x_;
          scan_info_2204.mount_offset_y = info_tx.mount_offset_y_;
          scan_info_2204.mount_offset_z = info_tx.mount_offset_z_;
        
          lux_fusion_scan_2204.scan_info_list.push_back(scan_info_2204);
        }


        pcl_cloud_2204.reserve(lux_fusion_scan_2204.num_scan_pts);

        ibeo_lux_msgs::FusionScanPoint  scan_point_2204;
        FusionScanPoint point_tx;
        lux_fusion_scan_2204.scan_point_list.clear();
        for(int k = 0; k < lux_fusion_scan_2204.num_scan_pts; k++)
        {
          point_tx = fusion_scan_data_2204_tx.scan_point_list_[k];
          scan_point_2204.x_position = point_tx.x_position_;
          scan_point_2204.y_position = point_tx.y_position_;
          scan_point_2204.z_position = point_tx.z_position_;
          scan_point_2204.echo_width = point_tx.echo_width_;
          scan_point_2204.device_id = point_tx.device_id_;
          scan_point_2204.layer = point_tx.layer_;
          scan_point_2204.echo = point_tx.echo_;
          scan_point_2204.time_stamp = point_tx.time_stamp_;
          scan_point_2204.flags = point_tx.flags_;

          cloud_point_2204.label = scan_point_2204.flags;
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
        scan_pointcloud_2204_pub.publish(pcl_cloud_2204);
      }
      // Fusion scan data 2205
      else if(is_fusion && lux_header_msg.data_type == 0x2205)
      {
        ROS_INFO("reading FUSION SYSTEM/ECU scan data 0x2205");

        pcl::PointCloud <pcl::PointXYZL> pcl_cloud_2205;
        pcl::PointXYZL cloud_point_2205;

        lux_fusion_scan_2205.lux_header = lux_header_msg;
        
        fusion_scan_data_2205_tx.header_ = header_tx;
        fusion_scan_data_2205_tx.parse(data_msg);

        lux_fusion_scan_2205.scan_start_time = fusion_scan_data_2205_tx.scan_start_time_;
        lux_fusion_scan_2205.scan_end_time_offset = fusion_scan_data_2205_tx.scan_end_time_offset_;
        lux_fusion_scan_2205.flags = fusion_scan_data_2205_tx.flags_;
        lux_fusion_scan_2205.scan_number = fusion_scan_data_2205_tx.scan_number_;
        lux_fusion_scan_2205.num_scan_pts = fusion_scan_data_2205_tx.num_scan_pts_;
        lux_fusion_scan_2205.num_scan_info = fusion_scan_data_2205_tx.num_scan_info_;

        ibeo_lux_msgs::FusionScanInfo2205   scan_info_2205;
        FusionScanInfo2205   info_tx;
        lux_fusion_scan_2205.scan_info_list.clear();
        for(int k = 0; k < lux_fusion_scan_2205.num_scan_info; k++)
        {
          info_tx = fusion_scan_data_2205_tx.scan_info_list_[k];
          scan_info_2205.device_id = info_tx.device_id_;
          scan_info_2205.type = info_tx.type_;
          scan_info_2205.number = info_tx.number_;
          scan_info_2205.start_angle = info_tx.start_angle_;
          scan_info_2205.end_angle = info_tx.end_angle_;
          scan_info_2205.scan_start_time = info_tx.scan_start_time_;
          scan_info_2205.scan_end_time = info_tx.scan_end_time_;
          scan_info_2205.device_start_time = info_tx.device_start_time_;
          scan_info_2205.device_end_time = info_tx.device_end_time_;
          scan_info_2205.scan_frequency = info_tx.scan_frequency_;
          scan_info_2205.beam_tilt = info_tx.beam_tilt_;
          scan_info_2205.scan_flags = info_tx.scan_flags_;
          scan_info_2205.mount_position.yaw_angle = info_tx.mount_position_.yaw_angle_;
          scan_info_2205.mount_position.pitch_angle = info_tx.mount_position_.pitch_angle_;
          scan_info_2205.mount_position.roll_angle = info_tx.mount_position_.roll_angle_;
          scan_info_2205.mount_position.offset_x = info_tx.mount_position_.offset_x_;
          scan_info_2205.mount_position.offset_y = info_tx.mount_position_.offset_y_;
          scan_info_2205.mount_position.offset_z = info_tx.mount_position_.offset_z_;

          scan_info_2205.resolution1.resolution_start_angle = info_tx.resolution1_.resolution_start_angle_;
          scan_info_2205.resolution1.resolution = info_tx.resolution1_.resolution_;

          scan_info_2205.resolution2.resolution_start_angle = info_tx.resolution1_.resolution_start_angle_;
          scan_info_2205.resolution2.resolution = info_tx.resolution1_.resolution_;

          scan_info_2205.resolution3.resolution_start_angle = info_tx.resolution1_.resolution_start_angle_;
          scan_info_2205.resolution3.resolution = info_tx.resolution1_.resolution_;

          scan_info_2205.resolution4.resolution_start_angle = info_tx.resolution1_.resolution_start_angle_;
          scan_info_2205.resolution4.resolution = info_tx.resolution1_.resolution_;

          scan_info_2205.resolution5.resolution_start_angle = info_tx.resolution1_.resolution_start_angle_;
          scan_info_2205.resolution5.resolution = info_tx.resolution1_.resolution_;

          scan_info_2205.resolution6.resolution_start_angle = info_tx.resolution1_.resolution_start_angle_;
          scan_info_2205.resolution6.resolution = info_tx.resolution1_.resolution_;

          scan_info_2205.resolution7.resolution_start_angle = info_tx.resolution1_.resolution_start_angle_;
          scan_info_2205.resolution7.resolution = info_tx.resolution1_.resolution_;

          scan_info_2205.resolution8.resolution_start_angle = info_tx.resolution1_.resolution_start_angle_;
          scan_info_2205.resolution8.resolution = info_tx.resolution1_.resolution_;

          lux_fusion_scan_2205.scan_info_list.push_back(scan_info_2205);
        }

        pcl_cloud_2205.reserve(lux_fusion_scan_2205.num_scan_pts);

        ibeo_lux_msgs::FusionScanPoint  scan_point_2205;
        FusionScanPoint  point_tx;
        lux_fusion_scan_2205.scan_point_list.clear();
        for(int k = 0; k < lux_fusion_scan_2205.num_scan_pts; k++)
        {
          point_tx = fusion_scan_data_2205_tx.scan_point_list_[k];
          scan_point_2205.x_position = (float) point_tx.x_position_;
          scan_point_2205.y_position = (float) point_tx.y_position_;
          scan_point_2205.z_position = (float) point_tx.z_position_;
          scan_point_2205.echo_width = (float) point_tx.echo_width_;
          scan_point_2205.device_id = point_tx.device_id_;
          scan_point_2205.layer = point_tx.layer_;
          scan_point_2205.echo = point_tx.echo_;
          scan_point_2205.time_stamp = point_tx.time_stamp_;
          scan_point_2205.flags = point_tx.flags_;

          lux_fusion_scan_2205.scan_point_list.push_back(scan_point_2205);
          
          cloud_point_2205.label = scan_point_2205.flags;
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
        scan_pointcloud_2205_pub.publish(pcl_cloud_2205);
      }
      // Fusion object data 2225
      else if(is_fusion && lux_header_msg.data_type == 0x2225)
      {
        ROS_INFO("reading Fusion object data 0x2225");
        visualization_msgs::MarkerArray   object_markers_2225;

        lux_fusion_object_2225.lux_header = lux_header_msg;

        fusion_object_data_2225_tx.header_ = header_tx;
        fusion_object_data_2225_tx.parse(data_msg);

        lux_fusion_object_2225.mid_scan_timestamp = fusion_object_data_2225_tx.mid_scan_timestamp_;
        lux_fusion_object_2225.num_of_objects = fusion_object_data_2225_tx.num_of_objects_;

        ibeo_lux_msgs::FusionObject2225 scan_object;
        FusionObject2225 object_tx;
        lux_fusion_object_2225.objects.clear();
        for(int k = 0; k < lux_fusion_object_2225.num_of_objects; k++)
        {
          object_tx = fusion_object_data_2225_tx.objects_[k];
          scan_object.ID = object_tx.ID_;
          scan_object.object_age = object_tx.object_age_;
          scan_object.time_stamp = object_tx.time_stamp_;
          scan_object.object_hidden_age = object_tx.object_hidden_age_;
          scan_object.classification = object_tx.classification_;
          scan_object.classification_certainty = object_tx.classification_certainty_;
          scan_object.classification_age = object_tx.classification_age_;
          scan_object.bounding_box_center.x = object_tx.bounding_box_center_.x_;
          scan_object.bounding_box_center.y = object_tx.bounding_box_center_.y_;
          scan_object.bounding_box_size.x = object_tx.bounding_box_size_.x_;
          scan_object.bounding_box_size.y = object_tx.bounding_box_size_.y_;
          scan_object.object_box_center.x = object_tx.object_box_center_.x_;
          scan_object.object_box_center.y = object_tx.object_box_center_.y_;
          scan_object.object_box_center_sigma.x = object_tx.object_box_center_sigma_.x_;
          scan_object.object_box_center_sigma.y = object_tx.object_box_center_sigma_.y_;
          scan_object.object_box_size.x = object_tx.object_box_size_.x_;
          scan_object.object_box_size.y = object_tx.object_box_size_.y_;
          scan_object.yaw_angle = object_tx.yaw_angle_;
          scan_object.relative_velocity.x = object_tx.relative_velocity_.x_;
          scan_object.relative_velocity.y = object_tx.relative_velocity_.y_;
          scan_object.relative_velocity_sigma.x = object_tx.relative_velocity_sigma_.x_;
          scan_object.relative_velocity_sigma.y = object_tx.relative_velocity_sigma_.y_;
          scan_object.absolute_velocity.x = object_tx.absolute_velocity_.x_;
          scan_object.absolute_velocity.y = object_tx.absolute_velocity_.y_;
          scan_object.absolute_velocity_sigma.x = object_tx.absolute_velocity_sigma_.x_;
          scan_object.absolute_velocity_sigma.y = object_tx.absolute_velocity_sigma_.y_;
          scan_object.number_of_contour_points = object_tx.number_of_contour_points_;
          scan_object.closest_point_index = object_tx.closest_point_index_;

          lux_fusion_object_2225.objects.push_back(scan_object);
          tf::Quaternion quaternion = tf::createQuaternionFromYaw(scan_object.yaw_angle); 
          visualization_msgs::Marker   object_marker_2225 = createWireframeMarker(scan_object.object_box_center.x, scan_object.object_box_center.y,
              scan_object.object_box_size.x, scan_object.object_box_size.y, 0.75);
          object_marker_2225.header.frame_id = frame_id;
          object_marker_2225.id  = scan_object.ID;
          object_marker_2225.lifetime = ros::Duration(0.2);
          object_marker_2225.pose.orientation.x = quaternion.x();
          object_marker_2225.pose.orientation.y = quaternion.y();
          object_marker_2225.pose.orientation.z = quaternion.z();
          object_marker_2225.pose.orientation.w = quaternion.w();
          object_marker_2225.color.a = 0.5;
          object_marker_2225.color.r = object_marker_2225.color.g = object_marker_2225.color.b = 1.0;
          object_marker_2225.frame_locked = false;

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
          object_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          object_label.action = visualization_msgs::Marker::ADD;
          object_label.pose.position.x = scan_object.object_box_center.x;
          object_label.pose.position.y = scan_object.object_box_center.y;
          object_label.pose.position.z = 0.5;
          object_label.text = label;
          object_label.scale.z = 0.2;
          object_label.lifetime = object_marker_2225.lifetime;
          object_label.color.r = object_label.color.g = object_label.color.b = 1;
          object_label.color.a = 0.5;
          
          ibeo_lux_msgs::Float2D object_point;
          Float2D point_tx;
          scan_object.list_of_contour_points.clear();
          
          visualization_msgs::Marker      object_point_marker;
          object_point_marker.header.frame_id = frame_id;
          object_point_marker.type = visualization_msgs::Marker::POINTS;
          object_point_marker.action = visualization_msgs::Marker::ADD;
          object_point_marker.ns = label;
          object_point_marker.points.reserve(scan_object.number_of_contour_points);
          object_point_marker.scale.x = 0.1;
          object_point_marker.scale.y = 0.1;
          object_point_marker.color.r = 1;
          object_point_marker.color.g = 1;
          object_point_marker.color.b = 1;
          object_point_marker.color.a = 0.7;
          
          for(int j =0; j< scan_object.number_of_contour_points; j++)
          {
            point_tx = object_tx.list_of_contour_points_[j];
            object_point.x = point_tx.x_;
            object_point.y = point_tx.y_;

            scan_object.list_of_contour_points.push_back(object_point);
            geometry_msgs::Point    dis_object_point;
            dis_object_point.x = object_point.x;
            dis_object_point.y = object_point.y;
            object_point_marker.points.push_back(dis_object_point);
          }

          object_marker_2225.header.stamp = ros::Time::now();
          object_markers_2225.markers.push_back(object_marker_2225);

          object_point_marker.header.stamp = ros::Time::now();
          object_markers_2225.markers.push_back(object_point_marker);

          object_label.header.stamp = ros::Time::now();
          object_markers_2225.markers.push_back(object_label);
        }

        lux_fusion_object_2225.header.frame_id = frame_id;
        lux_fusion_object_2225.header.stamp = ros::Time::now();
        fusion_object_2225_pub.publish(lux_fusion_object_2225);

        object_markers_2225_pub.publish(object_markers_2225);
      }
      // Fusion object data 2280
      else if(is_fusion && lux_header_msg.data_type == 0x2280)
      {
      
        ROS_INFO("reading Fusion object data 0x2280");
        visualization_msgs::MarkerArray   object_markers_2280;

        lux_fusion_object_2280.lux_header = lux_header_msg;

        fusion_object_data_2280_tx.header_ = header_tx;
        fusion_object_data_2280_tx.parse(data_msg);

        lux_fusion_object_2280.mid_scan_timestamp = fusion_object_data_2280_tx.mid_scan_timestamp_;
        lux_fusion_object_2280.num_of_objects = fusion_object_data_2280_tx.num_of_objects_;

        ibeo_lux_msgs::FusionObject2280 scan_object;
        FusionObject2280 object_tx;
        lux_fusion_object_2280.objects.clear();
        for(int k = 0; k < lux_fusion_object_2280.num_of_objects; k++)
        {
          object_tx = fusion_object_data_2280_tx.objects_[k];
          scan_object.ID = object_tx.ID_;
          scan_object.flags = object_tx.flags_;
          scan_object.object_age = object_tx.object_age_;
          scan_object.time_stamp = object_tx.time_stamp_;
          scan_object.object_prediction_age = object_tx.object_prediction_age_;
          scan_object.classification = object_tx.classification_;
          scan_object.classification_quality = object_tx.classification_quality_;
          scan_object.classification_age = object_tx.classification_age_;
          scan_object.object_box_center.x = object_tx.object_box_center_.x_;
          scan_object.object_box_center.y = object_tx.object_box_center_.y_;
          scan_object.object_box_size.x = object_tx.object_box_size_.x_;
          scan_object.object_box_size.y = object_tx.object_box_size_.y_;
          scan_object.object_course_angle = object_tx.object_course_angle_;
          scan_object.object_course_angle_sigma = object_tx.object_course_angle_sigma_;
          scan_object.relative_velocity.x = object_tx.relative_velocity_.x_;
          scan_object.relative_velocity.y = object_tx.relative_velocity_.y_;
          scan_object.relative_velocity_sigma.x = object_tx.relative_velocity_sigma_.x_;
          scan_object.relative_velocity_sigma.y = object_tx.relative_velocity_sigma_.y_;
          scan_object.absolute_velocity.x = object_tx.absolute_velocity_.x_;
          scan_object.absolute_velocity.y = object_tx.absolute_velocity_.y_;
          scan_object.absolute_velocity_sigma.x = object_tx.absolute_velocity_sigma_.x_;
          scan_object.absolute_velocity_sigma.y = object_tx.absolute_velocity_sigma_.y_;
          scan_object.number_of_contour_points = object_tx.number_of_contour_points_;
          scan_object.closest_point_index = object_tx.closest_point_index_;
          scan_object.reference_point_location = object_tx.reference_point_location_;
          scan_object.reference_point_coordinate.x = object_tx.reference_point_coordinate_.x_;
          scan_object.reference_point_coordinate.y = object_tx.reference_point_coordinate_.y_;
          scan_object.reference_point_coordinate_sigma.x = object_tx.reference_point_coordinate_sigma_.x_;
          scan_object.reference_point_coordinate_sigma.y = object_tx.reference_point_coordinate_sigma_.y_;
          scan_object.object_priority = object_tx.object_priority_;

          lux_fusion_object_2280.objects.push_back(scan_object);
          tf::Quaternion quaternion = tf::createQuaternionFromYaw(scan_object.object_course_angle); 
          visualization_msgs::Marker   object_marker_2280 = createWireframeMarker(scan_object.object_box_center.x, scan_object.object_box_center.y,
              scan_object.object_box_size.x, scan_object.object_box_size.y, 0.75);;
          object_marker_2280.header.frame_id = frame_id;
          object_marker_2280.id  = scan_object.ID;
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

          visualization_msgs::Marker   object_label;
          object_label.header.frame_id = frame_id;
          object_label.ns = label;
          object_label.id = scan_object.ID + 1000;
          object_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          object_label.action = visualization_msgs::Marker::ADD;
          object_label.pose.position.x = scan_object.object_box_center.x;
          object_label.pose.position.y = scan_object.object_box_center.y;
          object_label.pose.position.z = 0.5;
          object_label.text = label;
          object_label.scale.z = 0.2;
          object_label.lifetime = object_marker_2280.lifetime;
          object_label.color.r = object_label.color.g = object_label.color.b = 1;
          object_label.color.a = 0.5;
          
          ibeo_lux_msgs::Float2D     object_point;
          Float2D point_tx;
          scan_object.list_of_contour_points.clear();
          visualization_msgs::Marker      object_point_marker;
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
          scan_object.list_of_contour_points.clear();
          geometry_msgs::Point    dis_object_point;
          for(int j =0; j< scan_object.number_of_contour_points; j++)
          {
            point_tx = object_tx.list_of_contour_points_[j];
            object_point.x = point_tx.x_;
            object_point.y = point_tx.y_;

            scan_object.list_of_contour_points.push_back(object_point);
            
            geometry_msgs::Point    dis_object_point;
            dis_object_point.x = object_point.x;
            dis_object_point.y = object_point.y;

            object_point_marker.points.push_back(dis_object_point);
          }
          object_marker_2280.header.stamp = ros::Time::now();
          object_markers_2280.markers.push_back(object_marker_2280);

          object_point_marker.header.stamp = ros::Time::now();
          object_markers_2280.markers.push_back(object_point_marker);

          object_label.header.stamp = ros::Time::now();
          object_markers_2280.markers.push_back(object_label);
        }

        fusion_object_2280_pub.publish(lux_fusion_object_2280);

        object_markers_2280_pub.publish(object_markers_2280);
      }
      // Fusion image data 2403
      else if(is_fusion && lux_header_msg.data_type == 0x2403)
      {
      
        ROS_INFO("reading FUSION SYSTEM/ECU image 2403");
        lux_fusion_image_msg.lux_header = lux_header_msg;

        fusion_image_tx.header_ = header_tx;
        fusion_image_tx.parse(data_msg);

        lux_fusion_image_msg.image_format = fusion_image_tx.image_format_;
        lux_fusion_image_msg.time_stamp = fusion_image_tx.time_stamp_;
        lux_fusion_image_msg.time_stamp_ntp = fusion_image_tx.time_stamp_ntp_;
        lux_fusion_image_msg.ID = fusion_image_tx.ID_;
        lux_fusion_image_msg.mounting_position.yaw_angle = fusion_image_tx.mounting_position_.yaw_angle_;
        lux_fusion_image_msg.mounting_position.pitch_angle = fusion_image_tx.mounting_position_.pitch_angle_;
        lux_fusion_image_msg.mounting_position.roll_angle = fusion_image_tx.mounting_position_.roll_angle_;
        lux_fusion_image_msg.mounting_position.offset_x = fusion_image_tx.mounting_position_.offset_x_;
        lux_fusion_image_msg.mounting_position.offset_y = fusion_image_tx.mounting_position_.offset_y_;
        lux_fusion_image_msg.mounting_position.offset_z = fusion_image_tx.mounting_position_.offset_z_;
        lux_fusion_image_msg.horizontal_opening_angle = fusion_image_tx.horizontal_opening_angle_;
        lux_fusion_image_msg.vertical_opening_angle = fusion_image_tx.vertical_opening_angle_;
        lux_fusion_image_msg.image_width = fusion_image_tx.image_width_;
        lux_fusion_image_msg.image_height = fusion_image_tx.image_height_;
        lux_fusion_image_msg.compress_size = fusion_image_tx.compress_size_;
        lux_fusion_image_msg.image_bytes = fusion_image_tx.image_bytes_; 
        
        fusion_img_2403_pub.publish(lux_fusion_image_msg);
      }
      //Fusion vehicle state 2806
      else if(is_fusion && lux_header_msg.data_type == 0x2806)
      {
        ROS_INFO("reading Fusion vehicle state data 0x2806");
        lux_vehicle_state_2806_msg.lux_header  = lux_header_msg;

        fusion_vehicle_state_2806_tx.header_ = header_tx;
        fusion_vehicle_state_2806_tx.parse(data_msg);

        lux_vehicle_state_2806_msg.time_stamp = fusion_vehicle_state_2806_tx.time_stamp_;
        lux_vehicle_state_2806_msg.distance_x = fusion_vehicle_state_2806_tx.distance_x_;
        lux_vehicle_state_2806_msg.distance_y = fusion_vehicle_state_2806_tx.distance_y_;
        lux_vehicle_state_2806_msg.course_angle = fusion_vehicle_state_2806_tx.course_angle_;
        lux_vehicle_state_2806_msg.longitudinal_velocity = fusion_vehicle_state_2806_tx.longitudinal_velocity_;
        lux_vehicle_state_2806_msg.yaw_rate = fusion_vehicle_state_2806_tx.yaw_rate_;
        lux_vehicle_state_2806_msg.steering_wheel_angle = fusion_vehicle_state_2806_tx.steering_wheel_angle_;
        lux_vehicle_state_2806_msg.front_wheel_angle = fusion_vehicle_state_2806_tx.front_wheel_angle_;
        lux_vehicle_state_2806_msg.vehicle_width = fusion_vehicle_state_2806_tx.vehicle_width_;
        lux_vehicle_state_2806_msg.vehicle_front_to_front_axle = fusion_vehicle_state_2806_tx.vehicle_front_to_front_axle_;
        lux_vehicle_state_2806_msg.rear_axle_to_front_axle = fusion_vehicle_state_2806_tx.rear_axle_to_front_axle_;
        lux_vehicle_state_2806_msg.rear_axle_to_vehicle_rear = fusion_vehicle_state_2806_tx.rear_axle_to_vehicle_rear_;
        lux_vehicle_state_2806_msg.steer_ratio_poly0 = fusion_vehicle_state_2806_tx.steer_ratio_poly0_;
        lux_vehicle_state_2806_msg.steer_ratio_poly1 = fusion_vehicle_state_2806_tx.steer_ratio_poly1_;
        lux_vehicle_state_2806_msg.steer_ratio_poly2 = fusion_vehicle_state_2806_tx.steer_ratio_poly2_;
        lux_vehicle_state_2806_msg.steer_ratio_poly3 = fusion_vehicle_state_2806_tx.steer_ratio_poly3_;

        lux_vehicle_state_msg.header.frame_id = frame_id;
        lux_vehicle_state_msg.header.stamp = ros::Time::now();
        fusion_vehicle_2806_pub.publish(lux_vehicle_state_msg);
      }
      //Fusion vehicle state 2807
      else if(is_fusion && lux_header_msg.data_type == 0x2807)
      {
        ROS_INFO("reading Fusion vehicle state data 0x2807");
        lux_vehicle_state_2807_msg.lux_header  = lux_header_msg;

        fusion_vehicle_state_2807_tx.header_ = header_tx;
        fusion_vehicle_state_2807_tx.parse(data_msg);

        lux_vehicle_state_2807_msg.time_stamp = fusion_vehicle_state_2807_tx.time_stamp_;
        lux_vehicle_state_2807_msg.distance_x = fusion_vehicle_state_2807_tx.distance_x_;
        lux_vehicle_state_2807_msg.distance_y = fusion_vehicle_state_2807_tx.distance_y_;
        lux_vehicle_state_2807_msg.course_angle = fusion_vehicle_state_2807_tx.course_angle_;
        lux_vehicle_state_2807_msg.longitudinal_velocity = fusion_vehicle_state_2807_tx.longitudinal_velocity_;
        lux_vehicle_state_2807_msg.yaw_rate = fusion_vehicle_state_2807_tx.yaw_rate_;
        lux_vehicle_state_2807_msg.steering_wheel_angle = fusion_vehicle_state_2807_tx.steering_wheel_angle_;
        lux_vehicle_state_2807_msg.front_wheel_angle = fusion_vehicle_state_2807_tx.front_wheel_angle_;
        lux_vehicle_state_2807_msg.vehicle_width = fusion_vehicle_state_2807_tx.vehicle_width_;
        lux_vehicle_state_2807_msg.vehicle_front_to_front_axle = fusion_vehicle_state_2807_tx.vehicle_front_to_front_axle_;
        lux_vehicle_state_2807_msg.rear_axle_to_front_axle = fusion_vehicle_state_2807_tx.rear_axle_to_front_axle_;
        lux_vehicle_state_2807_msg.rear_axle_to_vehicle_rear = fusion_vehicle_state_2807_tx.rear_axle_to_vehicle_rear_;
        lux_vehicle_state_2807_msg.steer_ratio_poly0 = fusion_vehicle_state_2807_tx.steer_ratio_poly0_;
        lux_vehicle_state_2807_msg.steer_ratio_poly1 = fusion_vehicle_state_2807_tx.steer_ratio_poly1_;
        lux_vehicle_state_2807_msg.steer_ratio_poly2 = fusion_vehicle_state_2807_tx.steer_ratio_poly2_;
        lux_vehicle_state_2807_msg.steer_ratio_poly3 = fusion_vehicle_state_2807_tx.steer_ratio_poly3_;
        lux_vehicle_state_2807_msg.longitudinal_acceleration = fusion_vehicle_state_2807_tx.longitudinal_acceleration_;

        lux_vehicle_state_2807_msg.header.frame_id = frame_id;
        lux_vehicle_state_2807_msg.header.stamp = ros::Time::now();
        fusion_vehicle_2807_pub.publish(lux_vehicle_state_2807_msg);
      }
    }
  }
  else
  {
    ROS_ERROR("Connection to LUX could not be opened");
  }
  tcp_interface.close();
  return 0;
}

