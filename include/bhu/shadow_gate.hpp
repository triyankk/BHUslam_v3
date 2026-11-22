#pragma once
#include "bhu/external_nav.hpp"

namespace bhu::extnav {
// Diagnostic only: preserve the existing hard guards, replace only recovery
// frame count with elapsed capture time. Never used for actual publication.
class TimedShadowGate {
    DurationGate gate_;
public:
    explicit TimedShadowGate(uint64_t duration_us):gate_(duration_us) {}
    bool accept(const Sample& sample,uint64_t now) {
        return bool(gate_.accept(sample,now));
    }
};

// Read-only diagnostics. Support conditions are added by the estimator adapter.
inline unsigned hard_rejection_mask(const Sample& s,uint64_t now,uint64_t max_age_us=150000) {
    unsigned mask=0;
    if(!s.imu_initialized)mask|=1;
    if(!s.frame_alignment_valid)mask|=2;
    if(!s.T_W_C.matrix().allFinite() || !rotation_valid(s.T_W_C.rotation()))mask|=4;
    if(!s.velocity_valid || !s.camera_velocity_W.allFinite() || !s.angular_velocity_C.allFinite())mask|=8;
    if(!s.capture_host_us || (max_age_us && s.capture_host_us<=now && now-s.capture_host_us>max_age_us))mask|=16;
    if(s.capture_host_us>now)mask|=32;
    if(!s.imu_healthy)mask|=64;
    return mask;
}
}
