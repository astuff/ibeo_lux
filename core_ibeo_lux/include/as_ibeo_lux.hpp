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

class Lux_Scan_Data {
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
    //scan_point scan_points[256];
    //TCPMsg     lux_scan_msg();
    //void parse(TCPMsg msg);
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
