#include "bhu/estimator_watchdog.hpp"
#include "bhu/imu_health.hpp"
#include <iostream>
#include <stdexcept>
static void need(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main()try{
    bhu::ImuHealth health;
    health.observe(true,{0,0,9.8},1000000);need(health.healthy(1000000),"Stationary gravity is not saturation");
    health.observe(true,{0,39.2,0},1100000);
    need(!health.healthy(1100000)&&!health.healthy(1300000),"Saturation closes health for 200 ms");
    need(health.healthy(1300001)&&health.acceleration_clips()==1,"Clipping holdoff expires without inventing samples");
    health.observe(false,{17.45,0,0},1400000);need(!health.healthy(1400000)&&health.gyro_clips()==1,"Gyro rail detected");
    health.observe(true,{39.2,0,0},1350000);need(health.last_clip_host_us()==1400000,"Callbacks cannot move latest clip time backwards");
    bhu::EstimatorWatchdog watchdog;
    for(uint64_t t=1000000;t<=5000000;t+=100000)need(!watchdog.poll(t,true,false,true,false,false),"Never reset armed or unconfirmed vehicle");
    for(uint64_t t=5100000;t<=8000000;t+=100000)need(!watchdog.poll(t,true,false,true,true,true),"Never reset an active recording or disarm tail");
    need(!watchdog.poll(8100000,true,false,true,true,false),"Begin disarmed invalid-motion observation");
    need(!watchdog.poll(10099999,true,false,true,true,false),"Require full two seconds");
    need(watchdog.poll(10100000,true,false,true,true,false),"Recover persistently invalid disarmed estimator");
    for(uint64_t t=10200000;t<20100000;t+=100000)need(!watchdog.poll(t,true,false,true,true,false),"Ten-second reset cooldown");
    need(watchdog.poll(20100000,true,false,true,true,false),"Cooldown permits another recovery if still invalid");
    need(!watchdog.poll(20200000,true,true,true,true,false),"Valid motion clears failure history");
    need(!watchdog.poll(40000000,true,false,false,true,false),"Do not initialize blindly without support");
    need(!watchdog.poll(43000000,false,false,true,true,false),"Do not repeatedly reset while initializing");
    std::cout<<"PASS: IMU rails/holdoff and disarmed-only estimator recovery, recording protection and cooldown\n";
    return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
