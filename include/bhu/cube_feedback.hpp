#pragma once
#include <mavlink/ardupilotmega/mavlink.h>
#include <array>
#include <cstring>
#include <stdexcept>
#include <string>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace bhu::cube {
inline const char* tune(const std::string& event) {
    if(event=="ready")return "MFT180L16O5CC";
    if(event=="start")return "MFT120L2O5C";
    if(event=="stop")return "MFT180L16O5CCC";
    if(event=="saved")return "MFT120L4O5CG";
    if(event=="valid")return "MFT180L8O5CEG";
    if(event=="lost")return "MFT180L8O5GEC";
    if(event=="error")return "MFT180L16O4CCCCCC";
    if(event=="move")return "MFT120L8O5CEG";
    if(event=="settle")return "MFT120L2O4G";
    if(event=="record_start" || event=="record_stop")return "MFT150L8O5CP8CP8C";
    return nullptr;
}
inline mavlink_message_t play(const std::string& event) {
    auto value=tune(event);if(!value)throw std::invalid_argument("Unknown Cube cue");
    std::array<char,30> first{};std::array<char,200> second{};
    std::strncpy(first.data(),value,first.size()-1);
    mavlink_message_t message{};
    mavlink_msg_play_tune_pack(1,197,&message,1,1,first.data(),second.data());
    return message;
}
// Feedback is optional, local and nonblocking. It never opens the serial port.
class Feedback {
    int fd_=-1; sockaddr_in address_{};
public:
    explicit Feedback(unsigned port=0) {
        if(!port)return;
        if(port>65535)throw std::invalid_argument("Invalid Cube feedback port");
        fd_=::socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
        if(fd_<0)throw std::runtime_error("Cannot create Cube feedback socket");
        address_.sin_family=AF_INET;address_.sin_addr.s_addr=htonl(INADDR_LOOPBACK);address_.sin_port=htons(port);
    }
    ~Feedback(){if(fd_>=0)::close(fd_);}
    void send(const std::string& event)const {
        if(fd_<0)return;
        if(!tune(event))throw std::invalid_argument("Unknown Cube cue");
        if(::sendto(fd_,event.data(),event.size(),MSG_DONTWAIT,reinterpret_cast<const sockaddr*>(&address_),sizeof(address_))!=ssize_t(event.size()))
            throw std::runtime_error("Cannot send Cube feedback event");
    }
};
}
