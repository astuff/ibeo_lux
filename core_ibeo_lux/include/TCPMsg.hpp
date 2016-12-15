#ifndef TCP_MSG_HPP
#define TCP_MSG_HPP

#include <cinttypes>

class TCPMsg
{
public:
    TCPMsg();
    ~TCPMsg();
    uint16_t msgOffset;
    uint16_t size;
    double factor;
    double valueOffset;

    //double read_value(std::array<unsigned char, LUX_PAYLOAD_SIZE> &bufArray, TCPMsg msg);
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
#endif
