#pragma once
#include "native.hpp"
#include <netinet/in.h>
#include <sys/socket.h>

namespace tof {
// Local handoff to the existing Cube USB writer; this class never opens serial.
class CubeUdp {
    Fd tx_,rx_;
    sockaddr_in destination_{};
public:
    CubeUdp(int target_port,int feedback_port):
        tx_(::socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0)),
        rx_(::socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0)) {
        check(tx_.get()>=0 && rx_.get()>=0,os_error("Create Cube handoff sockets"));
        destination_.sin_family=AF_INET;destination_.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        destination_.sin_port=htons(uint16_t(target_port));
        sockaddr_in local=destination_;local.sin_port=htons(uint16_t(feedback_port));
        check(::bind(rx_.get(),reinterpret_cast<sockaddr*>(&local),sizeof(local))==0,
            os_error("Bind Cube feedback port"));
    }
    void write(const std::vector<uint8_t>& bytes) {
        check(::sendto(tx_.get(),bytes.data(),bytes.size(),0,
            reinterpret_cast<const sockaddr*>(&destination_),sizeof(destination_))==ssize_t(bytes.size()),
            os_error("Send ToF handoff"));
    }
    std::vector<uint8_t> read() {
        std::array<uint8_t,MAVLINK_MAX_PACKET_LEN> bytes{};
        sockaddr_in sender{};socklen_t size=sizeof(sender);
        auto n=::recvfrom(rx_.get(),bytes.data(),bytes.size(),0,reinterpret_cast<sockaddr*>(&sender),&size);
        if (n<0 && errno!=EAGAIN && errno!=EINTR) fail(os_error("Read Cube feedback"));
        if (n<=0 || sender.sin_addr.s_addr!=htonl(INADDR_LOOPBACK)) return {};
        return {bytes.begin(),bytes.begin()+n};
    }
};
}
