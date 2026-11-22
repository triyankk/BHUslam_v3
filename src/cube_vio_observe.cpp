#include "native.hpp"
#include "bhu/extnav_relay.hpp"
#include "bhu/observation_guard.hpp"
#include "bhu/gps_bootstrap.hpp"
#include <csignal>
#include <iostream>
static volatile std::sig_atomic_t running=1;
static void stop(int){running=0;}
int main(int argc,char**argv)try{
    using namespace tof;
    if(argc==2 && std::string(argv[1])=="--print-profiles"){Json profiles=Json::object();for(const auto& [name,parameters]:bhu::extnav::ObservationGuard::profiles())profiles[name]=parameters;std::cout<<profiles.dump(2)<<'\n';return 0;}
    check(argc==2,"cube_vio_observe NEW_OUTPUT_DIRECTORY | --print-profiles");fs::path out=argv[1];
    check(!fs::exists(out),"Output already exists");fs::create_directories(out);
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
    bhu::extnav::RelaySocket relay(14600,1,40,0,0);bhu::extnav::ObservationGuard guard;bhu::extnav::GpsBootstrap bootstrap;std::string priorBootstrap;
    Fd telemetry(::socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0));
    sockaddr_in here{};here.sin_family=AF_INET;here.sin_addr.s_addr=htonl(INADDR_LOOPBACK);here.sin_port=htons(14604);
    check(telemetry.get()>=0&&::bind(telemetry.get(),reinterpret_cast<sockaddr*>(&here),sizeof(here))==0,"Bind telemetry tap");
    auto destination=here;destination.sin_port=htons(14603);
    std::ofstream events(out/"events.jsonl");Parser parser;uint64_t handed=0,blocked=0,lastPose=0,lastStatus=0;std::string prior,priorProfile;
    auto log=[&](Json row){row["host_monotonic_us"]=uint64_t(now()*1e6);events<<row.dump()<<'\n';events.flush();check(bool(events),"Forwarding log failed");};
    while(running){uint64_t current=uint64_t(now()*1e6);std::array<uint8_t,1024> bytes{};
        for(int i=0;i<128;++i){auto n=::recv(telemetry.get(),bytes.data(),bytes.size(),MSG_DONTWAIT);if(n<0)break;
            parser.feed(bytes.data(),size_t(n),[&](const mavlink_message_t&m){if(m.sysid!=1||m.compid!=1)return;
                if(bootstrap.observe(m,current)){guard.started_disarmed=false;guard.pinned_profile.clear();log({{"event","cube_reboot"}});}
                if(m.msgid==MAVLINK_MSG_ID_HEARTBEAT){mavlink_heartbeat_t h{};mavlink_msg_heartbeat_decode(&m,&h);guard.observe_heartbeat(bool(h.base_mode&MAV_MODE_FLAG_SAFETY_ARMED),current);}
                if(m.msgid==MAVLINK_MSG_ID_PARAM_VALUE){mavlink_param_value_t p{};mavlink_msg_param_value_decode(&m,&p);guard.parameter(std::string(p.param_id,strnlen(p.param_id,16)),p.param_value,current);}
                if(m.msgid==MAVLINK_MSG_ID_STATUSTEXT){mavlink_statustext_t s{};mavlink_msg_statustext_decode(&m,&s);std::string text(s.text,strnlen(s.text,50));if(text.find("Using EKF Source Set ")!=std::string::npos&&text.find("Using EKF Source Set 1")==std::string::npos)guard.nonprimary_observed=true;log({{"event","cube_statustext"},{"text",text}});}
                if(m.msgid==MAVLINK_MSG_ID_SYS_STATUS){auto enabled=mavlink_msg_sys_status_get_onboard_control_sensors_enabled(&m),health=mavlink_msg_sys_status_get_onboard_control_sensors_health(&m);log({{"event","cube_vision_health"},{"vision_enabled",bool(enabled&128)},{"vision_healthy",bool(health&128)},{"reception_proven",false}});}
                if(m.msgid==MAVLINK_MSG_ID_EKF_STATUS_REPORT)log({{"event","cube_ekf_status"},{"flags",unsigned(mavlink_msg_ekf_status_report_get_flags(&m))}});
            });
        }
        auto reason=guard.reason(current);auto profile=guard.profile(current);if(profile!=priorProfile){log({{"event","navigation_profile"},{"profile",profile},{"pinned_profile",guard.pinned_profile},{"armed",guard.armed}});priorProfile=profile;}if(reason!=prior){log({{"event","forwarding_gate"},{"reason",reason.empty()?"ready":reason}});prior=reason;}
        if(auto setup=bootstrap.poll(current,reason.empty()&&profile=="vio_optflow_compass_gps_bootstrap",guard.armed)){auto wire=packet(*setup);check(::sendto(telemetry.get(),wire.data(),wire.size(),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&destination),sizeof(destination))==ssize_t(wire.size()),"Bootstrap handoff failed");log({{"event","bootstrap_handoff"},{"message_id",unsigned(setup->msgid)}});}
        if(bootstrap.state!=priorBootstrap){log({{"event","navigation_bootstrap"},{"details",bootstrap.snapshot(current)}});priorBootstrap=bootstrap.state;}
        if(auto message=relay.poll([]{return uint64_t(now()*1e6);})){if(reason.empty()){
            auto wire=packet(*message);check(::sendto(telemetry.get(),wire.data(),wire.size(),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&destination),sizeof(destination))==ssize_t(wire.size()),"Owner handoff failed");
            ++handed;lastPose=mavlink_msg_odometry_get_time_usec(&*message);log({{"event","owner_handoff"},{"profile",profile},{"capture_host_us",lastPose},{"age_ms",(current-lastPose)*.001},{"quality",int(mavlink_msg_odometry_get_quality(&*message))},{"reset_counter",unsigned(mavlink_msg_odometry_get_reset_counter(&*message))},{"cube_reception_proven",false}});
        }else ++blocked;}
        if(current-lastStatus>=1000000){Json params=Json::object();for(auto&[name,value]:guard.values)params[name]=value.first;
            auto tmp=out/"status.json.tmp";save_json(tmp,{{"running",true},{"host_monotonic_us",current},{"mode","profile-guarded VINS pose forwarding"},{"profile",profile},{"pinned_profile",guard.pinned_profile},{"armed",guard.armed},{"source_switch_commands",0},{"parameter_writes",0},{"fusion_validated",false},{"source_configuration_ok",reason.empty()},{"block_reason",reason},{"parameters",params},{"navigation_bootstrap",bootstrap.snapshot(current)},{"received",relay.filter.received},{"rejected",relay.filter.rejected},{"superseded",relay.filter.superseded},{"guard_blocked",blocked},{"owner_handoffs",handed},{"last_capture_host_us",lastPose},{"pose_freshness_policy","arducopter"},{"max_pose_age_ms",nullptr},{"max_forward_hz",nullptr},{"cube_reception_proven",false},{"reception_evidence","Cube BIN VISP/VISV; active source evidence XKFS.SS=0"}});fs::rename(tmp,out/"status.json");lastStatus=current;}
        pause_ms(5);
    }
    log({{"event","stopped"},{"owner_handoffs",handed}});return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
