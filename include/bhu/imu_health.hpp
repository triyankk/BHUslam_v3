#pragma once
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
namespace bhu {
// Gemini's enumerated SDK ranges: +/-4g (SDK uses 9.8 m/s^2 per g), +/-1000 deg/s.
// A sample within 1% of a rail is suspect; this cannot restore clipped motion.
inline bool imu_near_limit(bool acceleration,const std::array<double,3>& v) {
    const double limit=.99*(acceleration?39.2:17.453292519943295);
    for(double x:v)if(!std::isfinite(x)||std::abs(x)>=limit)return true;
    return false;
}
class ImuHealth {
    std::atomic<uint64_t> last_clip_{0},acc_clips_{0},gyro_clips_{0};
public:
    void observe(bool acceleration,const std::array<double,3>& v,uint64_t host_us) {
        if(!imu_near_limit(acceleration,v))return;
        (acceleration?acc_clips_:gyro_clips_).fetch_add(1);
        auto old=last_clip_.load();while(old<host_us&&!last_clip_.compare_exchange_weak(old,host_us)){}
    }
    bool healthy(uint64_t now)const{auto t=last_clip_.load();return !t||(now>=t&&now-t>200000);}
    uint64_t acceleration_clips()const{return acc_clips_.load();}
    uint64_t gyro_clips()const{return gyro_clips_.load();}
    uint64_t last_clip_host_us()const{return last_clip_.load();}
};
}
