#pragma once
#include <mavlink/ardupilotmega/mavlink.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <cerrno>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace bhu::extnav {

// Narrow input interface: one complete unsigned MAVLink2 ODOMETRY datagram.
// Producer time_usec must use this host's CLOCK_MONOTONIC capture-time domain.
// Only the newest valid pose is retained, avoiding a serial backlog.
class RelayFilter {
    std::optional<mavlink_message_t> pending_;
    uint64_t last_capture_=0,last_sent_=0,max_age_us_,min_interval_us_;
    uint8_t system_,component_;
    int quality_min_;
public:
    uint64_t received=0,rejected=0,forwarded=0,superseded=0;
    explicit RelayFilter(uint8_t system=1,uint8_t component=197,int quality_min=60,uint64_t max_age_us=150000,uint64_t min_interval_us=100000)
        :max_age_us_(max_age_us),min_interval_us_(min_interval_us),system_(system),component_(component),quality_min_(quality_min) {
        if(!system || !component || quality_min<1 || quality_min>100)
            throw std::invalid_argument("Invalid external-nav relay identity/quality");
    }
    bool ingest(const uint8_t* bytes,size_t size,uint64_t now_us) {
        ++received;
        auto reject=[&] {++rejected;return false;};
        if(size<12 || size>MAVLINK_MAX_PACKET_LEN || bytes[0]!=MAVLINK_STX || bytes[2]!=0)return reject();
        mavlink_message_t buffer{},message{};mavlink_status_t state{},status{};
        unsigned complete=0;
        for(size_t i=0;i<size;++i) {
            auto framing=mavlink_frame_char_buffer(&buffer,&state,bytes[i],&message,&status);
            if(framing==MAVLINK_FRAMING_OK) {++complete;if(i+1!=size)return reject();}
            else if(framing==MAVLINK_FRAMING_BAD_CRC || framing==MAVLINK_FRAMING_BAD_SIGNATURE)return reject();
        }
        if(complete!=1 || mavlink_msg_get_send_buffer_length(&message)!=size
           || message.msgid!=MAVLINK_MSG_ID_ODOMETRY || message.sysid!=system_ || message.compid!=component_)
            return reject();
        mavlink_odometry_t o{};mavlink_msg_odometry_decode(&message,&o);
        if(o.frame_id!=MAV_FRAME_LOCAL_FRD || o.child_frame_id!=MAV_FRAME_BODY_FRD
           || o.estimator_type!=MAV_ESTIMATOR_TYPE_VIO || o.quality<quality_min_ || o.quality>100
           || !o.time_usec || o.time_usec<=last_capture_ || o.time_usec>now_us || (max_age_us_ && now_us-o.time_usec>max_age_us_))
            return reject();
        for(float v:{o.x,o.y,o.z,o.vx,o.vy,o.vz,o.rollspeed,o.pitchspeed,o.yawspeed})
            if(!std::isfinite(v))return reject();
        double norm=0;
        for(float q:o.q) {if(!std::isfinite(q))return reject();norm+=double(q)*q;}
        if(std::abs(norm-1)>1e-3)return reject();
        if(!std::isnan(o.pose_covariance[0])) {
            for(float value:o.pose_covariance)if(!std::isfinite(value))return reject();
            for(int i:{0,6,11,15,18,20})if(o.pose_covariance[i]<0)return reject();
        }
        if(pending_)++superseded;
        pending_=message;last_capture_=o.time_usec;return true;
    }
    std::optional<mavlink_message_t> take(uint64_t now_us) {
        if(!pending_)return std::nullopt;
        auto stamp=mavlink_msg_odometry_get_time_usec(&*pending_);
        if(stamp>now_us || (max_age_us_ && now_us-stamp>max_age_us_)) {pending_.reset();++rejected;return std::nullopt;}
        // Legacy defaults cap at10 Hz; the ArduCopter timing policy adds no delay.
        if(last_sent_ && now_us-last_sent_<min_interval_us_)return std::nullopt;
        auto result=pending_;pending_.reset();last_sent_=now_us;++forwarded;return result;
    }
};

class RelaySocket {
    int fd_=-1;
public:
    RelayFilter filter;
    explicit RelaySocket(unsigned port=0,uint8_t system=1,int quality_min=60,uint64_t max_age_us=150000,uint64_t min_interval_us=100000):filter(system,197,quality_min,max_age_us,min_interval_us) {
        if(!port)return;
        if(port>65535)throw std::invalid_argument("Invalid external-nav UDP port");
        fd_=::socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
        if(fd_<0)throw std::runtime_error("Create external-nav UDP socket: "+std::string(std::strerror(errno)));
        sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(uint16_t(port));
        address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        if(::bind(fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0) {
            auto error=std::string(std::strerror(errno));::close(fd_);fd_=-1;
            throw std::runtime_error("Bind external-nav UDP socket: "+error);
        }
    }
    ~RelaySocket(){if(fd_>=0)::close(fd_);}
    RelaySocket(const RelaySocket&)=delete;
    RelaySocket& operator=(const RelaySocket&)=delete;
    template<class Clock> std::optional<mavlink_message_t> poll(Clock now_us) {
        if(fd_<0)return std::nullopt;
        std::array<uint8_t,1024> bytes{};
        for(unsigned i=0;i<64;++i) {
            auto n=::recv(fd_,bytes.data(),bytes.size(),MSG_DONTWAIT);
            if(n<0) {
                if(errno==EAGAIN || errno==EWOULDBLOCK)break;
                if(errno==EINTR)continue;
                throw std::runtime_error("Read external-nav UDP socket");
            }
            filter.ingest(bytes.data(),size_t(n),now_us());
        }
        return filter.take(now_us());
    }
};
} // namespace bhu::extnav
