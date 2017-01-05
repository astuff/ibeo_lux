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

const size_t ERROR_UNHANDLED_EXCEPTION = 2;
//const unsigned int ESR_XCP_PAYLOAD_SIZE =8568;
const uint8_t  LUX_MESSAGE_DATA_OFFSET = 24;
const   unsigned int LUX_PAYLOAD_SIZE = 100000;
const size_t magicWord = 0xAFFEC0C2;
const double PI = 3.1415926535897;


// little endian
double read_little_endian(std::array<unsigned char, LUX_PAYLOAD_SIZE> &bufArray, TCPMsg msg);

// big endian
double read_big_endian(std::array<unsigned char, LUX_PAYLOAD_SIZE> &bufArray, TCPMsg msg);

double convertAngle(int angle, int angle_tick_per_rotation);

unsigned char   read_one_byte(std::array<unsigned char, LUX_PAYLOAD_SIZE> &bufArray, uint16_t index);


#endif
