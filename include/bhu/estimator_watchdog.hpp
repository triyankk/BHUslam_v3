#pragma once
#include <cstdint>
namespace bhu {
// Reinitialization is permitted only after continuously observed fresh disarm,
// inactive recording, adequate image support and persistent invalid motion.
class EstimatorWatchdog {
    uint64_t bad_since_=0,last_reset_=0;
public:
    bool poll(uint64_t now,bool initialized,bool motion_valid,bool supported,
              bool fresh_disarmed,bool recording_active) {
        if(!initialized||motion_valid||!supported||!fresh_disarmed||recording_active){bad_since_=0;return false;}
        if(!bad_since_||now<bad_since_)bad_since_=now;
        if(now-bad_since_<2000000 || (last_reset_&&now-last_reset_<10000000))return false;
        bad_since_=0;last_reset_=now;return true;
    }
};
}
