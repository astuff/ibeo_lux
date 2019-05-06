/*
 * Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
 *
 * This file is part of the ibeo_lux ROS 1.0 driver which is released under the MIT license.
 * See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
 */

#include <ibeo_lux/ibeo_lux_ros_msg_handler.h>
#include <network_interface/network_interface.h>
#include <network_interface/network_utils.h>

// Tx
#include <network_interface/TCPFrame.h>

#include <unordered_map>
#include <vector>
#include <queue>
#include <deque>
#include <string>

using std::string;
using std::vector;
using namespace AS::Network;
using namespace AS::Drivers::IbeoLux;

TCPInterface tcp_interface;
IbeoLuxRosMsgHandler handler;
std::unordered_map<int64_t, ros::Publisher> pub_list;

// Main routine
int main(int argc, char **argv)
{
  // int c;
  string ip_address = "192.168.0.1";
  int port = 12002;
  string frame_id = "ibeo_lux";
  bool is_fusion = false;
  int buf_size = AS::Drivers::Ibeo::IBEO_PAYLOAD_SIZE;
  std::deque<uint8_t> grand_buffer;
  std::queue<std::vector<uint8_t>> messages;

  // ROS initialization
  ros::init(argc, argv, "ibeo_lux");
  ros::NodeHandle n;
  ros::NodeHandle priv("~");
  bool exit = false;

  if (priv.getParam("ip_address", ip_address))
  {
    ROS_INFO("Ibeo LUX - Got ip_address: %s", ip_address.c_str());

    if (ip_address == "")
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
    ROS_INFO("Ibeo LUX - Is Fusion ECU: %s", (is_fusion) ? "true" : "false");
  }

  if (priv.getParam("sensor_frame_id", frame_id))
  {
    ROS_INFO("Ibeo LUX - Got sensor frame ID: %s", frame_id.c_str());
  }

  if (exit)
    return 0;

  // Advertise messages to send
  ros::Publisher raw_tcp_pub = n.advertise<network_interface::TCPFrame>("tcp_tx", 1);
  ros::Publisher pointcloud_pub = n.advertise<pcl::PointCloud <pcl::PointXYZ> >("as_tx/point_cloud", 1);
  ros::Publisher object_markers_pub = n.advertise<visualization_msgs::MarkerArray>("as_tx/objects", 1);
  ros::Publisher object_contour_points_pub = n.advertise<visualization_msgs::Marker>("as_tx/object_contour_points", 1);

  ros::Publisher scan_data_pub, object_data_pub, vehicle_state_pub, error_warn_pub;
  ros::Publisher fusion_scan_2204_pub,
    fusion_scan_2205_pub,
    fusion_object_2225_pub,
    fusion_object_2280_pub,
    fusion_img_2403_pub,
    fusion_vehicle_2806_pub,
    fusion_vehicle_2807_pub;

  // LUX Sensor Only
  if (!is_fusion)
  {
    scan_data_pub = n.advertise<ibeo_msgs::ScanData2202>("parsed_tx/scan_data_2202", 1);
    object_data_pub = n.advertise<ibeo_msgs::ObjectData2221>("parsed_tx/object_data_2221", 1);
    vehicle_state_pub = n.advertise<ibeo_msgs::HostVehicleState2805>("parsed_tx/host_vehicle_state_2805", 1);
    error_warn_pub = n.advertise<ibeo_msgs::ErrorWarning>("parsed_tx/error_warning", 1);

    pub_list.insert(std::make_pair(ErrorWarning::DATA_TYPE, error_warn_pub));
    pub_list.insert(std::make_pair(ScanData2202::DATA_TYPE, scan_data_pub));
    pub_list.insert(std::make_pair(ObjectData2221::DATA_TYPE, object_data_pub));
    pub_list.insert(std::make_pair(HostVehicleState2805::DATA_TYPE, vehicle_state_pub));
  }
  else  // Fusion ECU Only
  {
    fusion_scan_2204_pub = n.advertise<ibeo_msgs::ScanData2204>("parsed_tx/scan_data_2204", 1);
    fusion_scan_2205_pub = n.advertise<ibeo_msgs::ScanData2205>("parsed_tx/scan_data_2205", 1);
    fusion_object_2225_pub = n.advertise<ibeo_msgs::ObjectData2225>("parsed_tx/object_data_2225", 1);
    fusion_object_2280_pub = n.advertise<ibeo_msgs::ObjectData2280>("parsed_tx/object_data_2280", 1);
    fusion_img_2403_pub = n.advertise<ibeo_msgs::CameraImage>("parsed_tx/camera_image", 1);
    fusion_vehicle_2806_pub = n.advertise<ibeo_msgs::HostVehicleState2806>("parsed_tx/host_vehicle_state_2806", 1);
    fusion_vehicle_2807_pub = n.advertise<ibeo_msgs::HostVehicleState2807>("parsed_tx/host_vehicle_state_2807", 1);

    pub_list.insert(std::make_pair(ScanData2204::DATA_TYPE, fusion_scan_2204_pub));
    pub_list.insert(std::make_pair(ScanData2205::DATA_TYPE, fusion_scan_2205_pub));
    pub_list.insert(std::make_pair(ObjectData2225::DATA_TYPE, fusion_object_2225_pub));
    pub_list.insert(std::make_pair(ObjectData2280::DATA_TYPE, fusion_object_2280_pub));
    pub_list.insert(std::make_pair(CameraImage::DATA_TYPE, fusion_img_2403_pub));
    pub_list.insert(std::make_pair(HostVehicleState2806::DATA_TYPE, fusion_vehicle_2806_pub));
    pub_list.insert(std::make_pair(HostVehicleState2807::DATA_TYPE, fusion_vehicle_2807_pub));
  }

  // Wait for time to be valid
  ros::Time::waitForValid();

  network_interface::TCPFrame tcp_raw_msg;

  ReturnStatuses status;

  ros::Rate loop_rate = (is_fusion) ? ros::Rate(1100) : ros::Rate(40);
  // Loop as long as module should run

  bool fusion_filter_sent = false;

  while (ros::ok())
  {
    if (!tcp_interface.is_open())
    {
      if (is_fusion)
        fusion_filter_sent = false;

      status = tcp_interface.open(ip_address.c_str(), port);

      if (status != ReturnStatuses::OK)
        ROS_WARN("Ibeo LUX - Unable to connect to sensor at %s: %d - %s",
            ip_address.c_str(),
            static_cast<int>(status), return_status_desc(status).c_str());

      ros::Duration(1.0).sleep();
    }
    else
    {
      if (is_fusion && !fusion_filter_sent)
      {
        std::vector<uint8_t> set_filter_cmd =
          {0xaf, 0xfe, 0xc0, 0xc2,
           0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x08,
           0x00, 0x00, 0x20, 0x10,
           0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x00,
           0x00, 0x05, 0x00, 0x02,
           0x00, 0x00, 0xff, 0xff};

        ROS_INFO_THROTTLE(3, "Ibeo LUX - Sending Fusion filter command to begin transmission.");

        status = tcp_interface.write(set_filter_cmd);

        if (status != ReturnStatuses::OK)
          ROS_ERROR_THROTTLE(3, "Ibeo LUX - Failed to send Fusion filter command.");
        else
          fusion_filter_sent = true;

        ros::Duration(0.5).sleep();
      }
      else
      {
        buf_size = IBEO_PAYLOAD_SIZE;
        std::vector<uint8_t> msg_buf;
        msg_buf.reserve(buf_size);

        status = tcp_interface.read(&msg_buf);  // Read a (big) chunk.

        if (status != ReturnStatuses::OK && status != ReturnStatuses::NO_MESSAGES_RECEIVED)
        {
          ROS_WARN("Ibeo ScaLa - Failed to read from socket: %d - %s",
              static_cast<int>(status),
              return_status_desc(status).c_str());
        }
        else if (status == ReturnStatuses::OK)
        {
          buf_size = msg_buf.size();
          grand_buffer.insert(grand_buffer.end(), msg_buf.begin(), msg_buf.end());

          int first_mw = 0;

          while (true)
          {
            first_mw = find_magic_word(grand_buffer, MAGIC_WORD);

            if (first_mw == -1)  // no magic word found. move along.
            {
              break;
            }
            else  // magic word found. pull out message from grand buffer and add it to the message list.
            {
              if (first_mw > 0)
                grand_buffer.erase(
                    grand_buffer.begin(),
                    grand_buffer.begin() + first_mw);  // Unusable data in beginning of buffer.

              // From here on, the detected Magic Word should be at the beginning of the grand_buffer.

              IbeoDataHeader header;
              header.parse(std::vector<uint8_t>(grand_buffer.begin(), grand_buffer.begin() + IBEO_HEADER_SIZE));

              auto total_msg_size = IBEO_HEADER_SIZE + header.message_size;

              if (grand_buffer.size() < total_msg_size)
                break;  // Incomplete message left in grand buffer. Wait for next read.

              std::vector<uint8_t> msg(
                  grand_buffer.begin(),
                  grand_buffer.begin() + total_msg_size);
              messages.push(msg);
              grand_buffer.erase(
                  grand_buffer.begin(),
                  grand_buffer.begin() + total_msg_size);
            }

            if (!ros::ok())
              break;
          }
        }

        while (!messages.empty())
        {
          auto message = messages.front();

          network_interface::TCPFrame raw_frame;
          raw_frame.address = ip_address;
          raw_frame.port = port;
          raw_frame.size = message.size();
          raw_frame.data.insert(raw_frame.data.end(), message.begin(), message.end());
          raw_frame.header.frame_id = frame_id;
          raw_frame.header.stamp = ros::Time::now();

          raw_tcp_pub.publish(raw_frame);

          IbeoDataHeader ibeo_header;
          ibeo_header.parse(message);

          // Instantiate a parser class of the correct type.
          auto class_parser = IbeoTxMessage::make_message(ibeo_header.data_type_id);
          // Look up the message publisher for this type.
          auto pub = pub_list.find(ibeo_header.data_type_id);

          // Only parse message types we know how to handle.
          if (class_parser != NULL && pub != pub_list.end())
          {
            // Parse the raw data into the class.
            class_parser->parse(message);
            // Create a new message of the correct type and publish it.
            handler.fillAndPublish(ibeo_header.data_type_id, frame_id, pub->second, class_parser.get());

            if (class_parser->has_scan_points)
            {
              pcl::PointCloud<pcl::PointXYZL> pcl_cloud;
              pcl_cloud.header.frame_id = frame_id;
              // pcl_cloud.header.stamp = ibeo_header.time;
              pcl_conversions::toPCL(ros::Time::now(), pcl_cloud.header.stamp);
              std::vector<Point3DL> scan_points = class_parser->get_scan_points();
              handler.fillPointcloud(scan_points, &pcl_cloud);
              pointcloud_pub.publish(pcl_cloud);
            }

            if (class_parser->has_contour_points)
            {
              visualization_msgs::Marker marker;
              marker.header.frame_id = frame_id;
              marker.header.stamp = ros::Time::now();
              std::vector<Point3D> contour_points = class_parser->get_contour_points();

              if (contour_points.size() > 0)
              {
                handler.fillContourPoints(contour_points, &marker, frame_id);
                object_contour_points_pub.publish(marker);
              }
            }

            if (class_parser->has_objects)
            {
              std::vector<IbeoObject> objects = class_parser->get_objects();
              visualization_msgs::MarkerArray marker_array;
              handler.fillMarkerArray(objects, &marker_array, frame_id);

              for (visualization_msgs::Marker m : marker_array.markers)
              {
                m.header.frame_id = frame_id;
              }

              object_markers_pub.publish(marker_array);
            }
          }

          messages.pop();
        }  // Message parse loop
      }    // If fusion filter sent or != fusion
    }      // If sensor is connected

    loop_rate.sleep();
    // ros::spinOnce();  // No callbacks yet
  }

  status = tcp_interface.close();

  if (status != ReturnStatuses::OK)
    ROS_ERROR(
        "Ibeo LUX - Closing the connection to the LUX failed: %i - %s",
        static_cast<int>(status),
        return_status_desc(status).c_str());

  return 0;
}
