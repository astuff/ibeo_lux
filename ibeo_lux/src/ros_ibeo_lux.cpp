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

#include <unordered_map>

#include <ibeo_lux_ros_msg_handler.h>
#include <network_interface/network_interface.h>

//Tx
#include <network_interface/TCPFrame.h>

using std::string;
using std::vector;
using namespace AS::Network;
using namespace AS::Drivers::Ibeo;
using namespace AS::Drivers::IbeoLux;

TCPInterface tcp_interface;

// Main routine
int main(int argc, char **argv)
{
  //int c;
  string ip_address = "192.168.0.1";
  int port = 12002;
  string frame_id = "ibeo_lux";
  bool is_fusion = false;
  size_t bytes_read;
  int buf_size = IBEO_PAYLOAD_SIZE;
  std::vector<unsigned char> grand_buffer;
  std::vector<std::vector<unsigned char>> messages;

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

  std::unordered_map<unsigned short, IbeoLuxRosMsgHandler> handler_list;

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
    scan_data_pub = n.advertise<ibeo_msgs::ScanData2202>("parsed_tx/scan_data_2202", 1);
    object_data_pub = n.advertise<ibeo_msgs::ObjectData2221>("parsed_tx/object_data_2221", 1);
    vehicle_state_pub = n.advertise<ibeo_msgs::HostVehicleState2805>("parsed_tx/host_vehicle_state_2805", 1);
    error_warn_pub = n.advertise<ibeo_msgs::ErrorWarning>("parsed_tx/error_warning", 1);

    IbeoLuxRosMsgHandler handler_2030(error_warn_pub, frame_id);
    IbeoLuxRosMsgHandler handler_2202(scan_data_pub, frame_id);
    IbeoLuxRosMsgHandler handler_2221(object_data_pub, frame_id);
    IbeoLuxRosMsgHandler handler_2805(vehicle_state_pub, frame_id);

    handler_list.insert(std::make_pair(ErrorWarning::DATA_TYPE, handler_2030));
    handler_list.insert(std::make_pair(ScanData2202::DATA_TYPE, handler_2202));
    handler_list.insert(std::make_pair(ObjectData2221::DATA_TYPE, handler_2221));
    handler_list.insert(std::make_pair(HostVehicleState2805::DATA_TYPE, handler_2805));
  }
  else //Fusion ECU Only
  {
    fusion_scan_2204_pub = n.advertise<ibeo_msgs::ScanData2204>("parsed_tx/scan_data_2204", 1);
    fusion_scan_2205_pub = n.advertise<ibeo_msgs::ScanData2205>("parsed_tx/scan_data_2205", 1);
    fusion_object_2225_pub = n.advertise<ibeo_msgs::ObjectData2225>("parsed_tx/object_data_2225", 1);
    fusion_object_2280_pub = n.advertise<ibeo_msgs::ObjectData2280>("parsed_tx/object_data_2280", 1);
    fusion_img_2403_pub = n.advertise<ibeo_msgs::CameraImage>("parsed_tx/camera_image", 1);
    fusion_vehicle_2806_pub = n.advertise<ibeo_msgs::HostVehicleState2806>("parsed_tx/host_vehicle_state_2806", 1);
    fusion_vehicle_2807_pub = n.advertise<ibeo_msgs::HostVehicleState2807>("parsed_tx/host_vehicle_state_2807", 1);

    IbeoLuxRosMsgHandler handler_2204(fusion_scan_2204_pub, frame_id);
    IbeoLuxRosMsgHandler handler_2205(fusion_scan_2205_pub, frame_id);
    IbeoLuxRosMsgHandler handler_2225(fusion_object_2225_pub, frame_id);
    IbeoLuxRosMsgHandler handler_2280(fusion_object_2280_pub, frame_id);
    IbeoLuxRosMsgHandler handler_2403(fusion_img_2403_pub, frame_id);
    IbeoLuxRosMsgHandler handler_2806(fusion_vehicle_2806_pub, frame_id);
    IbeoLuxRosMsgHandler handler_2807(fusion_vehicle_2807_pub, frame_id);

    handler_list.insert(std::make_pair(ScanData2204::DATA_TYPE, handler_2204));
    handler_list.insert(std::make_pair(ScanData2205::DATA_TYPE, handler_2205));
    handler_list.insert(std::make_pair(ObjectData2225::DATA_TYPE, handler_2225));
    handler_list.insert(std::make_pair(ObjectData2280::DATA_TYPE, handler_2280));
    handler_list.insert(std::make_pair(CameraImage::DATA_TYPE, handler_2403));
    handler_list.insert(std::make_pair(HostVehicleState2806::DATA_TYPE, handler_2806));
    handler_list.insert(std::make_pair(HostVehicleState2807::DATA_TYPE, handler_2807));
  }

  // Wait for time to be valid
  while (ros::Time::now().nsec == 0);
  
  network_interface::TCPFrame tcp_raw_msg;

  return_statuses status;

  ros::Rate loop_rate = (is_fusion)? ros::Rate(1100) : ros::Rate(40);
  // Loop as long as module should run
  
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

        status = tcp_interface.write(set_filter_cmd, sizeof(set_filter_cmd));

        if (status != OK) 
          ROS_ERROR_THROTTLE(3, "Ibeo LUX - Failed to send Fusion filter command.");
        else
          fusion_filter_sent = true;

        ros::Duration(0.5).sleep();
      }
      else
      {
        buf_size = IBEO_PAYLOAD_SIZE;
        std::unique_ptr<unsigned char[]> msg_buf(new unsigned char[buf_size + 1]);

        status = tcp_interface.read(msg_buf.get(), buf_size, bytes_read); //Read a (big) chunk.

        if (status != OK && status != NO_MESSAGES_RECEIVED)
        {
          ROS_WARN("Ibeo ScaLa - Failed to read from socket: %d - %s", status, return_status_desc(status).c_str());
        }
        else if (status == OK)
        {
          buf_size = bytes_read;
          grand_buffer.insert(grand_buffer.end(), msg_buf.get(), msg_buf.get() + bytes_read);

          int first_mw = 0;
          //ROS_INFO("Finished reading %d bytes of data. Total buffer size is %d.",bytes_read, grand_buffer.size());

          //TODO: FMEA on following loop.
          while (true)
          {
            first_mw = find_magic_word((uint8_t*) grand_buffer.data() + 1, grand_buffer.size(), MAGIC_WORD);

            if(first_mw == -1) // no magic word found. move along.
            {
               break;
            }
            else  // magic word found. pull out message from grand buffer and add it to the message list.
            {
              std::vector<unsigned char> msg;
              msg.insert(msg.end(),grand_buffer.begin(), grand_buffer.begin() + first_mw + 1);
              messages.push_back(msg);
              grand_buffer.erase(grand_buffer.begin(), grand_buffer.begin() + first_mw + 1);
            }
          }
        }

        if (!messages.empty())
        {
          for (unsigned int i = 0; i < messages.size(); i++)
          {
						network_interface::TCPFrame raw_frame;
						raw_frame.address = ip_address;
						raw_frame.port = port;
						raw_frame.size = messages[i].size();
						raw_frame.data.insert(raw_frame.data.begin(), messages[i].begin(), messages[i].end());
						raw_frame.header.frame_id = frame_id;
						raw_frame.header.stamp = ros::Time::now();

						raw_tcp_pub.publish(raw_frame);

            IbeoDataHeader ibeo_header;
            ibeo_header.parse(messages[i].data());

            auto class_parser = IbeoTxMessage::make_message(ibeo_header.data_type_id); //Instantiate a parser class of the correct type.
            auto msg_handler = handler_list.find(ibeo_header.data_type_id); //Look up the message handler for this type.

            //Only parse message types we know how to handle.
            if (class_parser != NULL && msg_handler != handler_list.end())
            {
              class_parser->parse(messages[i].data()); //Parse the raw data into the class.
              msg_handler->second.fillAndPublish(ibeo_header.data_type_id, class_parser); //Create a new message of the correct type and publish it.

              if (class_parser->has_scan_points)
              {
                pcl::PointCloud<pcl::PointXYZL> pcl_cloud;
                pcl_cloud.header.frame_id = frame_id;
                //pcl_cloud.header.stamp = ibeo_header.time;
                pcl_conversions::toPCL(ros::Time::now(), pcl_cloud.header.stamp);
                std::vector<Point3DL> scan_points = class_parser->get_scan_points();
                msg_handler->second.fillPointcloud(scan_points, pcl_cloud);
                pointcloud_pub.publish(pcl_cloud);
              }

              if (class_parser->has_contour_points)
              {
                visualization_msgs::Marker marker;
                marker.header.frame_id = frame_id;
                marker.header.stamp = ros::Time::now();
                std::vector<Point3D> contour_points = class_parser->get_contour_points();

                if( contour_points.size() > 0 )
                {
                  msg_handler->second.fillContourPoints(contour_points, marker);
                  object_contour_points_pub.publish(marker);
                }
              }

              if (class_parser->has_objects)
              {
                std::vector<IbeoObject> objects = class_parser->get_objects();
                visualization_msgs::MarkerArray marker_array;
                msg_handler->second.fillMarkerArray(objects, marker_array);

                for( visualization_msgs::Marker m : marker_array.markers )
                {
                  m.header.frame_id = frame_id;
                }

                object_markers_pub.publish(marker_array);
              }
            }
					} //Message parse loop

          messages.clear();
        } //Messages were found
      } //If fusion filter sent or != fusion
    } //If sensor is connected

    loop_rate.sleep();
    //ros::spinOnce(); // No callbacks yet
  }

  status = tcp_interface.close();

  if (status != OK)
    ROS_ERROR("Ibeo LUX - Closing the connection to the LUX failed: %i - %s", status, return_status_desc(status).c_str());

  return 0;
}

