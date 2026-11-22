#pragma once
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <stdexcept>
#include <string>

namespace bhu {
// Fixed recording-start event only. The existing Cube owner selects the tune.
class RecordingCue {
    int fd_=-1;
    sockaddr_in destination_{};
    std::string token_;
    uint64_t started_=0,lastSend_=0;
    bool acknowledged_=false;
public:
    RecordingCue(unsigned port,uint64_t token):token_(std::to_string(token)){
        if(!port)return;
        if(port>65535)throw std::invalid_argument("Invalid recording cue port");
        fd_=::socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
        if(fd_<0)throw std::runtime_error("Cannot open recording cue socket");
        destination_.sin_family=AF_INET;destination_.sin_addr.s_addr=htonl(INADDR_LOOPBACK);destination_.sin_port=htons(port);
    }
    ~RecordingCue(){if(fd_>=0)::close(fd_);}
    void poll(bool recordingReady,uint64_t now){
        if(fd_<0||acknowledged_)return;
        std::array<char,128> bytes{};sockaddr_in from{};socklen_t size=sizeof(from);
        auto n=::recvfrom(fd_,bytes.data(),bytes.size(),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&from),&size);
        if(n>0&&from.sin_addr.s_addr==destination_.sin_addr.s_addr&&from.sin_port==destination_.sin_port&&std::string(bytes.data(),n)=="VIOACK1 "+token_){acknowledged_=true;return;}
        if(!recordingReady)return;
        if(!started_)started_=now;
        if(now-started_>30000000||now-lastSend_<1000000)return;
        auto event="VIOREC1 "+token_+" "+std::to_string(now);
        ::sendto(fd_,event.data(),event.size(),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&destination_),sizeof(destination_));lastSend_=now;
    }
    bool acknowledged()const{return acknowledged_;}
    bool requested()const{return lastSend_!=0;}
};
}
