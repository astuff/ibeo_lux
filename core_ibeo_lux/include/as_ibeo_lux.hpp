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

#ifndef AS_IBEO_LUX_HPP
#define AS_IBEO_LUX_HPP

//C++ Includes
#include <iostream>

//OS Includes
#include <unistd.h>

#include <TCPMsg.hpp>

class lux_scan_data_2202 {
public:
    uint16_t   scan_number;
    uint16_t   scan_status;
    uint16_t   sync_phase_offset;
    uint64_t   scan_start_time;
    uint64_t   scan_end_time;
    uint16_t   angle_ticks;
    int16_t    start_angle;
    int16_t    end_angle;
    uint16_t   num_scan_pts;
    int16_t    mounting_yaw_angle;
    int16_t    mounting_pitch_angle;
    int16_t    mounting_roll_angle;
    int16_t    mounting_position_x;
    int16_t    mounting_position_y;
    int16_t    mounting_position_z;
    uint16_t   scan_flags;
    //void parse();
};


class Lux_Object_Data {
public:
    uint64_t       scan_start_time;
    uint16_t       num_of_objects;
    //void parse();
};



class scan_point
{
public:
    uint8_t  layer;
    uint8_t  echo;
    uint8_t  flags;
    int16_t  horizontal_angle;
    uint16_t radial_distance;
    uint16_t echo_pulse_width;

    //void get_point(TCPMsg msg);
};

class object_list
{
public:
    uint16_t    ID;
    uint16_t    age;
    uint16_t    prediction_age;
    uint16_t    relative_timestamp;
    point2D     reference_point;
    point2D     reference_point_sigma;
    point2D     closest_point;
    point2D     bounding_box_center;
    uint16_t    bounding_box_width;
    uint16_t    bounding_box_length;
    point2D     object_box_center;
    size2D      object_box_size;
    int16_t     object_box_orientation;
    point2D     absolute_velocity;
    size2D      absolute_velocity_sigma;
    point2D     relative_velocity;
    uint16_t    classification;
    uint16_t    classification_age;
    uint16_t    classification_certaiinty;
    uint16_t    number_of_contour_points;
};

class  point2D
{
    int16_t     x;
    int16_t     y;
};

class  size2D
{
    uint16_t    x;
    uint16_t    y;
};

/*class ESR_Status1_Message {
public:
    uint16_t timeStamp;
    uint8_t rollingCount;
    bool commError;
    int16_t curvature;
    uint16_t scanId;
    float yawRate;
    float vehicleSpeedCalc;

    void parse(unsigned char *in);
};*/

#endif
