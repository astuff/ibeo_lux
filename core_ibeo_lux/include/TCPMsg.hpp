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

#endif
