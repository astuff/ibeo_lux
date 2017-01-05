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

// little endian
double read_little_endian(std::array<unsigned char, LUX_PAYLOAD_SIZE> &bufArray, TCPMsg msg) {
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
double read_big_endian(std::array<unsigned char, LUX_PAYLOAD_SIZE> &bufArray, TCPMsg msg) {
    unsigned long rcvData = 0;

    for (unsigned int i = 0; i <  msg.size; i++) {
        rcvData <<= 8;

        rcvData |= bufArray[(msg.msgOffset) + i];
    }

    double retVal = ((double)rcvData * msg.factor) - msg.valueOffset;

    return retVal;
}

double convertAngle(int angle, int angle_tick_per_rotation)
{
    return 2.0 * PI * static_cast<double>(angle / angle_tick_per_rotation);
}

unsigned char   read_one_byte(std::array<unsigned char, LUX_PAYLOAD_SIZE> &bufArray, uint16_t index){

    return (bufArray[index]);
}