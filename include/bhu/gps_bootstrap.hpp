#pragma once
#include <mavlink/ardupilotmega/mavlink.h>
#include <json.hpp>
#include <cmath>
#include <optional>
#include <string>
#include <cstdint>
#include <cstring>
namespace bhu::extnav {
// Navigation-reference initialization only. Never selects sources, arms, or moves.
class GpsBootstrap {
public:
    struct Point {int32_t lat=0,lon=0,alt=0;}; // e7 degrees and AMSL millimetres
private:
    Point gps_,origin_,home_,requested_;
    bool has_origin_=false,has_home_=false,requested_home_=false,notified_=false,boot_known_=false;
    uint64_t gps_stamp_=0,gps_host_=0,good_since_=0,options_host_=0,boot_host_=0,origin_host_=0,home_host_=0,last_request_=0,last_notice_=0;
    uint32_t boot_ms_=0,options_=16;unsigned gps_samples_=0,attempts_=0;
    bool quality_=false;
    void reset(){gps_stamp_=gps_host_=good_since_=options_host_=origin_host_=home_host_=last_request_=last_notice_=0;gps_samples_=attempts_=0;has_origin_=has_home_=requested_home_=notified_=ready=false;quality_=false;}
    static bool fresh(uint64_t t,uint64_t now,uint64_t age){return t&&now>=t&&now-t<=age;}
public:
    bool ready=false;unsigned boot_generation=0;std::string state="waiting_profile";
    static bool valid(Point p){return p.lat>=-900000000&&p.lat<=900000000&&p.lon>=-1800000000&&p.lon<=1800000000&&!(p.lat==0&&p.lon==0&&p.alt==0);}
    static double distance(Point a,Point b){const double lat=(double(a.lat)+b.lat)*.5e-7*.017453292519943295;return std::hypot((double(a.lat)-b.lat)*.01113194907932736,(double(a.lon)-b.lon)*.01113194907932736*std::cos(lat));}
    static bool near(Point a,Point b){return distance(a,b)<=5&&std::abs(double(a.alt)-b.alt)<=5000;}
    bool gps_fresh(uint64_t now)const{return quality_&&fresh(gps_host_,now,2500000);}
    // True only for an observed Cube reboot, not a GPS disconnect or map reset.
    bool observe(const mavlink_message_t&m,uint64_t now){
        if(m.sysid!=1||m.compid!=1)return false;
        if(m.msgid==MAVLINK_MSG_ID_SYSTEM_TIME){mavlink_system_time_t clock{};mavlink_msg_system_time_decode(&m,&clock);auto t=clock.time_boot_ms;bool reboot=boot_known_&&uint64_t(t)+1000<boot_ms_;if(reboot){reset();++boot_generation;}if(!boot_known_||reboot||t>=boot_ms_){boot_known_=true;boot_ms_=t;boot_host_=now;}return reboot;}
        if(m.msgid==MAVLINK_MSG_ID_PARAM_VALUE){mavlink_param_value_t p{};mavlink_msg_param_value_decode(&m,&p);if(std::string(p.param_id,strnlen(p.param_id,16))=="AHRS_OPTIONS"&&std::isfinite(p.param_value)&&p.param_value>=0&&p.param_value<=65535&&std::floor(p.param_value)==p.param_value){options_=uint32_t(p.param_value);options_host_=now;}}
        if(m.msgid==MAVLINK_MSG_ID_GPS_RAW_INT){mavlink_gps_raw_int_t g{};mavlink_msg_gps_raw_int_decode(&m,&g);gps_={g.lat,g.lon,g.alt};quality_=g.fix_type>=3&&g.satellites_visible>=8&&g.satellites_visible!=255&&g.h_acc>0&&g.h_acc<=2000&&g.v_acc>0&&g.v_acc<=5000&&g.vel<=50&&valid(gps_);
            if(!quality_){good_since_=0;gps_samples_=0;}else if(g.time_usec&&g.time_usec!=gps_stamp_){if(!good_since_||!fresh(gps_host_,now,2500000)||g.time_usec<gps_stamp_){good_since_=now;gps_samples_=0;}gps_host_=now;++gps_samples_;}gps_stamp_=g.time_usec;
        }
        if(m.msgid==MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN){mavlink_gps_global_origin_t p{};mavlink_msg_gps_global_origin_decode(&m,&p);Point n{p.latitude,p.longitude,p.altitude};if(ready&&has_origin_&&(n.lat!=origin_.lat||n.lon!=origin_.lon||n.alt!=origin_.alt)){ready=false;state="origin_changed";}origin_=n;has_origin_=valid(n);origin_host_=now;}
        if(m.msgid==MAVLINK_MSG_ID_HOME_POSITION){mavlink_home_position_t p{};mavlink_msg_home_position_decode(&m,&p);Point n{p.latitude,p.longitude,p.altitude};if(ready&&has_home_&&!near(n,home_)){ready=false;state="home_changed";}home_=n;has_home_=valid(n);home_host_=now;}
        return false;
    }
    std::optional<mavlink_message_t> poll(uint64_t now,bool qualified_profile,bool armed){
        if(!qualified_profile){state="waiting_profile";return std::nullopt;}
        if(!boot_known_||!fresh(boot_host_,now,3000000)){state="waiting_cube_clock";return std::nullopt;}
        if(!fresh(options_host_,now,3000000)||(options_&16)){state="waiting_no_origin_reuse";return std::nullopt;}
        if(ready){state="reference_verified";if(!armed&&gps_fresh(now)&&(!last_notice_||now-last_notice_>=2000000)){last_notice_=now;mavlink_message_t m{};mavlink_msg_named_value_int_pack(1,197,&m,boot_ms_,"NAVREF_OK",1);return m;}return std::nullopt;}
        if(armed){state="armed_without_verified_reference";return std::nullopt;}
        if(!gps_fresh(now)||!good_since_||now-good_since_<5000000||gps_samples_<3){state="waiting_stable_gps";return std::nullopt;}
        if(!has_origin_||!fresh(origin_host_,now,5000000)){state="waiting_cube_origin";return std::nullopt;}
        if(distance(origin_,gps_)>100){state="origin_far_from_launch";return std::nullopt;}
        if(has_home_&&fresh(home_host_,now,5000000)){
            if(!near(home_,requested_home_?requested_:gps_)){state="existing_home_conflict";return std::nullopt;}
            ready=true;state="reference_verified";
            if(!notified_){notified_=true;last_notice_=now;mavlink_message_t m{};mavlink_msg_named_value_int_pack(1,197,&m,boot_ms_,"NAVREF_OK",1);return m;}return std::nullopt;
        }
        if(attempts_>=3){state="home_request_limit";return std::nullopt;}
        state="waiting_home_readback";if(last_request_&&now-last_request_<2000000)return std::nullopt;
        if(!requested_home_){requested_=gps_;requested_home_=true;}if(!near(requested_,gps_)){state="launch_moved";return std::nullopt;}
        mavlink_message_t m{};mavlink_msg_command_int_pack(1,197,&m,1,1,MAV_FRAME_GLOBAL,MAV_CMD_DO_SET_HOME,0,0,0,0,0,0,requested_.lat,requested_.lon,requested_.alt*.001f);last_request_=now;++attempts_;return m;
    }
    nlohmann::json snapshot(uint64_t now)const{return {{"state",state},{"reference_verified",ready},{"gps_fresh",gps_fresh(now)},{"stable_gps_samples",gps_samples_},{"cube_boot_ms",boot_known_?nlohmann::json(boot_ms_):nlohmann::json(nullptr)},{"boot_generation",boot_generation},{"home_requests",attempts_},{"origin",has_origin_?nlohmann::json{{"lat_e7",origin_.lat},{"lon_e7",origin_.lon},{"alt_msl_mm",origin_.alt}}:nlohmann::json(nullptr)},{"home",has_home_?nlohmann::json{{"lat_e7",home_.lat},{"lon_e7",home_.lon},{"alt_msl_mm",home_.alt}}:nlohmann::json(nullptr)},{"fix_policy","5s advancing GPS,3D+,8 satellites,hacc<=2m,vacc<=5m,speed<=0.5m/s"},{"flight_ready",false}};}
};
}
