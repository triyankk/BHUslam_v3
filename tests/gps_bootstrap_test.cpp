#include "native.hpp"
#include "bhu/gps_bootstrap.hpp"
#include "bhu/observation_guard.hpp"
#include <iostream>
using namespace tof;
using bhu::extnav::GpsBootstrap;
static constexpr int32_t lat=250000000,lon=830000000,alt=80000;
static void clock(GpsBootstrap& b,uint64_t now,uint32_t boot=0,int options=8){mavlink_message_t m{};mavlink_system_time_t t{};t.time_boot_ms=boot?boot:uint32_t(now/1000);mavlink_msg_system_time_encode(1,1,&m,&t);b.observe(m,now);mavlink_param_value_t p{};std::memcpy(p.param_id,"AHRS_OPTIONS",12);p.param_value=options;mavlink_msg_param_value_encode(1,1,&m,&p);b.observe(m,now);}
static void gps(GpsBootstrap&b,uint64_t now,uint64_t stamp=0,int satellites=12){mavlink_message_t m{};mavlink_gps_raw_int_t g{};g.time_usec=stamp?stamp:now;g.lat=lat;g.lon=lon;g.alt=alt;g.fix_type=3;g.satellites_visible=satellites;g.h_acc=800;g.v_acc=1500;g.vel=0;mavlink_msg_gps_raw_int_encode(1,1,&m,&g);b.observe(m,now);}
static void origin(GpsBootstrap&b,uint64_t now){mavlink_message_t m{};mavlink_gps_global_origin_t p{};p.latitude=lat;p.longitude=lon;p.altitude=alt;mavlink_msg_gps_global_origin_encode(1,1,&m,&p);b.observe(m,now);}
static void home(GpsBootstrap&b,uint64_t now,int offset=0){mavlink_message_t m{};mavlink_home_position_t p{};p.latitude=lat+offset;p.longitude=lon;p.altitude=alt;mavlink_msg_home_position_encode(1,1,&m,&p);b.observe(m,now);}
static GpsBootstrap stable(){GpsBootstrap b;for(uint64_t n=1000000;n<=6000000;n+=1000000){clock(b,n);gps(b,n);}origin(b,6000000);return b;}
int main()try{
    auto b=stable();auto m=b.poll(6000000,true,false);if(!m)std::cerr<<b.snapshot(6000000).dump()<<"\n";check(m&&m->msgid==MAVLINK_MSG_ID_COMMAND_INT,"Fresh disarmed fix requests Home");mavlink_command_int_t c{};mavlink_msg_command_int_decode(&*m,&c);check(c.command==179&&c.frame==0&&c.param1==0&&c.x==lat&&c.y==lon&&std::abs(c.z-80)<.001,"Home packet units and command");check(!b.ready,"Sending is not confirmation");home(b,6000100);m=b.poll(6000100,true,false);check(m&&m->msgid==MAVLINK_MSG_ID_NAMED_VALUE_INT&&b.ready,"Actual Cube Home confirms reference");
    clock(b,12000000);check(!b.poll(12000000,true,true)&&b.ready&&b.state=="reference_verified","GPS disconnect does not clear same-boot reference");clock(b,13000000,100);check(!b.poll(13000000,true,false)&&!b.ready&&b.boot_generation==1,"Reboot discards old reference");
    b=stable();home(b,6000000,10000);check(!b.poll(6000000,true,false)&&b.state=="existing_home_conflict","Never overwrite conflicting Home");
    b=stable();check(!b.poll(6000000,true,true)&&b.state=="armed_without_verified_reference","Never initialize armed");
    b=stable();check(!b.poll(6000000,false,false)&&b.state=="waiting_profile","Only explicit bootstrap profile");clock(b,6000000,0,24);check(!b.poll(6000000,true,false)&&b.state=="waiting_no_origin_reuse","No old-origin reuse");
    b=stable();clock(b,9000000);check(!b.poll(9000000,true,false)&&b.state=="waiting_stable_gps","Stale GPS rejected");
    b=stable();gps(b,6100000,0,4);for(uint64_t n=6200000;n<=11200000;n+=1000000){clock(b,n);gps(b,n);}origin(b,11200000);check(bool(b.poll(11200000,true,false)),"Quality recovery starts a new stable window");
    GpsBootstrap repeated;for(uint64_t n=1000000;n<=6000000;n+=1000000){clock(repeated,n);gps(repeated,n,1000000);}origin(repeated,6000000);check(!repeated.poll(6000000,true,false),"Repeated cached GPS is not fresh");
    b=stable();gps(b,6000100,0,4);check(!b.poll(6000100,true,false),"Poor GPS rejected");
    b=stable();for(uint64_t n=6000000;n<=12000000;n+=2000000){clock(b,n);gps(b,n);origin(b,n);auto request=b.poll(n,true,false);check(bool(request)==(n<12000000),"At most three Home attempts");}check(b.state=="home_request_limit","Bounded requests");
    auto profiles=bhu::extnav::ObservationGuard::profiles();auto hybrid=profiles.at("vio_optflow_compass");hybrid["GPS1_TYPE"]=9;check(hybrid==profiles.at("vio_optflow_compass_gps_bootstrap"),"Only driver differs from selected hybrid");
    std::cout<<"PASS GPS bootstrap: packet, readback, stale/poor/repeated GPS, conflicts, armed, reboot, disconnect, bounded retries\n";
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
