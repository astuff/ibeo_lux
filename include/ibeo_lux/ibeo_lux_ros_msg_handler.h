/*
 * Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
 *
 * This file is part of the ibeo_lux ROS 1.0 driver which is released under the MIT license.
 * See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
 */

#ifndef IBEO_LUX_IBEO_LUX_ROS_MSG_HANDLER_H
#define IBEO_LUX_IBEO_LUX_ROS_MSG_HANDLER_H

#include <ibeo_lux/ibeo_lux_common.h>

#include <string>
#include <vector>

using namespace AS::Drivers::Ibeo;

namespace AS
{
namespace Drivers
{
namespace IbeoLux
{
class IbeoLuxRosMsgHandler
{
public:
  void fillAndPublish(const uint16_t& type_id,
                      const std::string& frame_id,
                      const ros::Publisher& pub,
                      IbeoTxMessage * parser_class);
  void fillPointcloud(
      const std::vector<Point3DL>& points,
      pcl::PointCloud<pcl::PointXYZL> * new_msg);
  void fillContourPoints(
      const std::vector<Point3D>& points,
      visualization_msgs::Marker * new_msg,
      const std::string& frame_id);
  void fillMarkerArray(
    const std::vector<IbeoObject>& objects,
    visualization_msgs::MarkerArray * new_msg,
    const std::string& frame_id);

private:
  ros::Time ntp_to_ros_time(const NTPTime& time);

  void fillIbeoHeader(
      const IbeoDataHeader& class_header,
      ibeo_msgs::IbeoDataHeader * msg_header);
  void fill2030(
    IbeoTxMessage * parser_class,
    ibeo_msgs::ErrorWarning * new_msg,
    const std::string& frame_id);
  void fill2202(
    IbeoTxMessage * parser_class,
    ibeo_msgs::ScanData2202 * new_msg,
    const std::string& frame_id);
  void fill2204(
    IbeoTxMessage * parser_class,
    ibeo_msgs::ScanData2204 * new_msg,
    const std::string& frame_id);
  void fill2205(
    IbeoTxMessage * parser_class,
    ibeo_msgs::ScanData2205 * new_msg,
    const std::string& frame_id);
  void fill2209(
    IbeoTxMessage * parser_class,
    ibeo_msgs::ScanData2209 * new_msg,
    const std::string& frame_id);
  void fill2221(
    IbeoTxMessage * parser_class,
    ibeo_msgs::ObjectData2221 * new_msg,
    const std::string& frame_id);
  void fill2225(
    IbeoTxMessage * parser_class,
    ibeo_msgs::ObjectData2225 * new_msg,
    const std::string& frame_id);
  void fill2280(
    IbeoTxMessage * parser_class,
    ibeo_msgs::ObjectData2280 * new_msg,
    const std::string& frame_id);
  void fill2403(
    IbeoTxMessage * parser_class,
    ibeo_msgs::CameraImage * new_msg,
    const std::string& frame_id);
  void fill2805(
    IbeoTxMessage * parser_class,
    ibeo_msgs::HostVehicleState2805 * new_msg,
    const std::string& frame_id);
  void fill2806(
    IbeoTxMessage * parser_class,
    ibeo_msgs::HostVehicleState2806 * new_msg,
    const std::string& frame_id);
  void fill2807(
    IbeoTxMessage * parser_class,
    ibeo_msgs::HostVehicleState2807 * new_msg,
    const std::string& frame_id);
  visualization_msgs::Marker createWireframeMarker(
      const float& center_x,
      const float& center_y,
      float size_x,
      float size_y,
      const float& size_z);
};
}  // namespace IbeoLux
}  // namespace Drivers
}  // namespace AS

#endif  // IBEO_LUX_IBEO_LUX_ROS_MSG_HANDLER_H
