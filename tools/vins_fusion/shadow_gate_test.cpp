#include "bhu/shadow_gate.hpp"
#include <iostream>
using namespace bhu::extnav;
static void need(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
static Sample sample(uint64_t t){Sample s;s.capture_host_us=t;s.quality=80;s.tracking_ok=s.imu_initialized=s.frame_alignment_valid=s.velocity_valid=true;return s;}
int main()try{
    DurationGate publication(500000);
    TimedShadowGate reference(500000);
    uint64_t recovery=1000000;
    auto feed=[&](Sample s,uint64_t now){
        auto result=publication.accept(s,now);
        need(bool(result)==reference.accept(s,now),"Publication timing matches tested shadow");
        return result;
    };
    for(unsigned i=0;i<=5;++i){auto s=sample(recovery+i*100000);s.quality=40;
        auto result=feed(s,s.capture_host_us);
        need(bool(result)==(i==5),"Publication requires continuous 500 ms");
        if(result)need(*result==0,"First output has reset counter zero");
    }
    auto failure=sample(1600000);failure.tracking_ok=false;
    need(!feed(failure,failure.capture_host_us),"Publication stops on failed support");
    for(unsigned i=0;i<=5;++i){auto s=sample(1700000+i*100000);auto result=feed(s,s.capture_host_us);
        need(bool(result)==(i==5),"Full duration after failure");
        if(result)need(*result==0,"Continuous estimator recovery preserves reset counter");
    }
    auto continuous=sample(2300000);auto result=feed(continuous,2300000);
    need(result&&*result==0,"Continuous output preserves reset counter");
    for(unsigned i=0;i<=5;++i){auto s=sample(2500000+i*100000);s.reset_epoch=1;auto r=feed(s,s.capture_host_us);
        need(bool(r)==(i==5),"Gap and epoch restart duration");
        if(r)need(*r==1,"Actual epoch change marks discontinuity");
    }
    // Weak stereo may last seconds without losing the estimator coordinate frame.
    DurationGate continuity(500000);
    for(uint64_t t=1000000;t<=1600000;t+=100000)continuity.accept(sample(t),t);
    for(uint64_t t=1700000;t<=4000000;t+=100000){auto s=sample(t);s.tracking_ok=false;need(!continuity.accept(s,t),"Withhold all weak-support poses");}
    for(uint64_t t=4100000;t<=4600000;t+=100000){auto r=continuity.accept(sample(t),t);if(t==4600000)need(r&&*r==0,"Long observed support gap is not a map reset");else need(!r,"Full support recovery remains required");}
    need(continuity.recoveries()==1,"Count recovery separately from reset");
    auto jump=sample(4700000);jump.T_W_C.translation().x()=1;
    need(!continuity.accept(jump,4700000),"Pose jump closes publication immediately");
    for(uint64_t t=4800000;t<=5200000;t+=100000){jump.capture_host_us=t;auto r=continuity.accept(jump,t);if(t==5200000)need(r&&*r==1,"Pose jump announced once after recovery");else need(!r,"Jump requires recovery duration");}
    need(continuity.last_reset_causes()&2,"Position jump diagnosis");
    jump.capture_host_us=5300000;jump.T_W_C.linear()=Eigen::AngleAxisd(.6,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    need(!continuity.accept(jump,5300000),"Attitude jump closes publication");
    for(uint64_t t=5400000;t<=5800000;t+=100000){jump.capture_host_us=t;auto r=continuity.accept(jump,t);if(t==5800000)need(r&&*r==2,"Attitude jump reset once");}
    need(continuity.last_reset_causes()&4,"Attitude jump diagnosis");
    jump.capture_host_us=7200000;need(!continuity.accept(jump,7200000),"Unobserved interval requires recovery");
    for(uint64_t t=7300000;t<=7700000;t+=100000){jump.capture_host_us=t;auto r=continuity.accept(jump,t);if(t==7700000)need(r&&*r==3,"Unknown continuity remains explicit reset");}
    need(continuity.last_reset_causes()&8,"Unobserved interval diagnosis");
    DurationGate stalled(500000);
    for(uint64_t t=1000000;t<=1600000;t+=100000)stalled.accept(sample(t),t);
    for(uint64_t t:{1800000u,2000000u}){auto s=sample(t);s.velocity_valid=false;need(!stalled.accept(s,t),"Skipped-frame derivative remains invalid");}
    for(uint64_t t=2100000;t<=2600000;t+=100000){auto r=stalled.accept(sample(t),t);need(bool(r)==(t==2600000),"Processing stall still requires full recovery");if(r)need(*r==0,"Coherent stationary pose after a subsecond stall is not a frame reset");}
    DurationGate moving(500000);
    const Eigen::Vector3d speed(1.2,-.4,.1);
    for(uint64_t t=1000000;t<=5000000;t+=100000){
        auto s=sample(t);double elapsed=(t-1000000)*1e-6;
        s.T_W_C.translation()=speed*elapsed;s.camera_velocity_W=speed;
        s.T_W_C.linear()=Eigen::AngleAxisd(.8*elapsed,Eigen::Vector3d::UnitZ()).toRotationMatrix();
        s.angular_velocity_C=Eigen::Vector3d(0,0,.8);s.tracking_ok=t<2000000||t>=3000000;
        auto r=moving.accept(s,t);if(r)need(*r==0,"Smooth translation and rotation must not trigger coordinate reset");
        if(t==5000000)need(bool(r),"Moving estimator resumes publication");
    }
    DurationGate startup(500000);
    for(uint64_t t=1000000;t<=1400000;t+=100000)need(!startup.accept(sample(t),t),"Initial qualification");
    auto startupJump=sample(1500000);startupJump.T_W_C.translation().x()=1;
    need(!startup.accept(startupJump,1500000),"Pre-publication jump restarts qualification");
    for(uint64_t t=1600000;t<=2000000;t+=100000){startupJump.capture_host_us=t;auto r=startup.accept(startupJump,t);need(bool(r)==(t==2000000),"Full qualification after initial jump");if(r)need(*r==0,"First coordinate frame has no previous published reset");}
    Gate frame15(15);
    uint64_t stamp=1000000;
    for(unsigned i=1;i<=15;++i,stamp+=50000)
        need(bool(frame15.accept(sample(stamp),stamp))==(i==15),"15-frame gate opens exactly on sample 15");
    auto low=sample(stamp);low.tracking_ok=false;
    need(!frame15.accept(low,stamp),"Feature-support failure closes 15-frame gate immediately");
    for(unsigned i=1;i<=15;++i){stamp+=50000;need(bool(frame15.accept(sample(stamp),stamp))==(i==15),"15 fresh samples required after failure");}
    stamp+=50000;need(!frame15.accept(sample(stamp),stamp+150001),"15-frame gate preserves 150 ms age limit");
    stamp+=200000;auto reset=sample(stamp);reset.reset_epoch=1;
    need(!frame15.accept(reset,stamp),"15-frame gate resets after gap and epoch change");
    for(uint64_t interval:{33333u,66667u,100000u}){
        TimedShadowGate timed(500000);uint64_t first=0;
        for(uint64_t t=1000000;t<=2000000;t+=interval){bool pass=timed.accept(sample(t),t);if(pass&&!first)first=t;need(!pass||t>=1500000,"No early timed recovery");}
        need(first>=1500000&&first<1500000+interval,"Recovery duration independent of frame rate");
    }
    TimedShadowGate timed(500000);
    for(uint64_t t=1000000;t<=1500000;t+=100000)timed.accept(sample(t),t);
    auto bad=sample(1600000);bad.tracking_ok=false;need(!timed.accept(bad,1600000),"Tracking failure closes immediately");
    for(uint64_t t=1700000;t<2200000;t+=100000)need(!timed.accept(sample(t),t),"Full recovery after failure");
    need(timed.accept(sample(2200000),2200000),"Recovers after duration");
    auto epoch=sample(2300000);epoch.reset_epoch=1;need(!timed.accept(epoch,2300000),"Epoch resets warmup");
    auto gap=sample(2500000);gap.reset_epoch=1;need(!timed.accept(gap,2500000),"Capture gap resets warmup without epoch change");
    need(!timed.accept(sample(2600000),2800000),"Stale pose rejected");
    need(!timed.accept(sample(2900000),2800000),"Future pose rejected");
    need(!timed.accept(sample(2900000),2900000),"Duplicate rejected");
    bad=sample(3000000);bad.T_W_C.translation().x()=std::numeric_limits<double>::quiet_NaN();need(!timed.accept(bad,3000000),"Nonfinite pose rejected");need(hard_rejection_mask(bad,3000000)&4,"Nonfinite reason recorded");
    bad=sample(3100000);bad.velocity_valid=false;need(!timed.accept(bad,3100000),"Invalid velocity rejected");
    bad=sample(3200000);bad.T_W_C.linear()(0,0)=2;need(!timed.accept(bad,3200000),"Nonrigid pose rejected");
    bad=sample(3300000);bad.imu_healthy=false;need(!timed.accept(bad,3300000),"Clipped IMU closes publication");need(hard_rejection_mask(bad,3300000)&64,"Clipped IMU reason recorded");
    Gate slow(30),fast(30);uint64_t slowFirst=0,fastFirst=0;
    for(unsigned i=0;i<35;i++){auto a=sample(1000000+i*100000);if(slow.accept(a,a.capture_host_us)&&!slowFirst)slowFirst=a.capture_host_us;auto b=sample(1000000+i*33333);if(fast.accept(b,b.capture_host_us)&&!fastFirst)fastFirst=b.capture_host_us;}
    need(slowFirst-fastFirst>1900000,"Frame-count recovery depends on rate");
    // Operator-selected policy: no Jetson pose-age, gap or recovery wait.
    DurationGate direct(0,0,0,0);
    auto delayed=sample(1000000);
    need(bool(direct.accept(delayed,1400000)),"Direct policy accepts delayed first valid pose immediately");
    need(!direct.accept(delayed,1400000),"Direct policy rejects duplicate capture");
    delayed.capture_host_us=1100000;delayed.tracking_ok=false;
    need(!direct.accept(delayed,1400000),"Direct policy keeps visual-support guard");
    delayed.capture_host_us=1200000;delayed.tracking_ok=true;
    need(bool(direct.accept(delayed,1500000)),"Direct policy recovers without waiting");
    need(direct.reset_counter()==0,"Recovery does not invent an origin reset");
    delayed.capture_host_us=3000000;
    need(bool(direct.accept(delayed,3300000))&&direct.reset_counter()==0,"Long observation gap alone is not an origin reset");
    delayed.capture_host_us=3100000;delayed.reset_epoch=1;
    need(bool(direct.accept(delayed,3300000))&&direct.reset_counter()==1,"Actual estimator reset propagated immediately");
    delayed.capture_host_us=3400000;
    need(!direct.accept(delayed,3300000),"Direct policy rejects future timestamps");
    delayed.capture_host_us=3500000;delayed.imu_healthy=false;
    need(!direct.accept(delayed,3600000),"Direct policy keeps IMU health guard");
    need(!(hard_rejection_mask(sample(1000000),1400000,0)&16),"Direct diagnostic mask has no age rejection");
    // Candidate: 500 ms startup/fault qualification, 100 ms only after a <=200 ms interruption.
    DurationGate brief(500000,100000,200000);
    for(uint64_t t=1000000;t<=1500000;t+=50000)need(bool(brief.accept(sample(t),t))==(t==1500000),"Candidate retains 500 ms startup");
    auto weak=sample(1550000);weak.tracking_ok=false;need(!brief.accept(weak,1550000),"Candidate closes immediately");
    for(uint64_t t=1600000;t<=1700000;t+=50000)need(bool(brief.accept(sample(t),t))==(t==1700000),"Brief continuous recovery takes 100 ms");
    need(brief.reset_counter()==0&&brief.qualification_duration_us()==100000,"Brief recovery keeps frame reset counter");
    auto clipped=sample(1750000);clipped.imu_healthy=false;need(!brief.accept(clipped,1750000),"Candidate rejects IMU fault");
    for(uint64_t t=1800000;t<=2300000;t+=50000)need(bool(brief.accept(sample(t),t))==(t==2300000),"IMU fault requires full 500 ms");
    auto shift=sample(2350000);shift.reset_epoch=1;need(!brief.accept(shift,2350000),"Epoch change closes candidate");
    for(uint64_t t=2400000;t<=2850000;t+=50000){shift.capture_host_us=t;need(bool(brief.accept(shift,t))==(t==2850000),"Epoch recovery requires 500 ms");}
    need(brief.reset_counter()==1,"Candidate announces real reset");
    shift.capture_host_us=2900000;shift.tracking_ok=false;brief.accept(shift,2900000);
    for(uint64_t t=3200000;t<=3700000;t+=50000){shift.capture_host_us=t;shift.tracking_ok=true;need(bool(brief.accept(shift,t))==(t==3700000),"Long interruption retains 500 ms");}
    auto stale=shift;stale.capture_host_us=3750000;need(!brief.accept(stale,3900001),"Candidate keeps 150 ms freshness");
    DurationGate jumping(500000,100000,200000);
    for(uint64_t t=1000000;t<=1500000;t+=50000)jumping.accept(sample(t),t);
    auto changed=sample(1550000);changed.T_W_C.translation().x()=1;
    need(!jumping.accept(changed,1550000),"Position jump closes brief candidate");
    for(uint64_t t=1600000;t<=2050000;t+=50000){changed.capture_host_us=t;need(bool(jumping.accept(changed,t))==(t==2050000),"Pose jump still requires full recovery");}
    std::cout<<"PASS: timed shadow recovery across rates, immediate failure closure, reset/gap/stale/future/duplicate/finite/velocity guards\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
