#include "native.hpp"
#include "bhu/extnav_relay.hpp"
#include "bhu/cube_feedback.hpp"
#include <csignal>
#include <iostream>

static volatile std::sig_atomic_t running=1;
static void stop(int){running=0;}
int main(int argc,char** argv) try {
    using namespace tof;
    std::string uart="/dev/serial/by-id/usb-CubePilot_CubeOrange+_36003F000B51333338373339-if00",cue;
    int baud=115200,nav_port=14600,cue_port=14601,quality_min=60;double duration=1200,startup_seconds=8;
    fs::path output;bool feedbackOnly=false;
    for(int i=1;i<argc;++i) {
        std::string key=argv[i];
        if(key=="--feedback-only"){feedbackOnly=true;continue;}
        if(key=="--help"){std::cout<<"cube_vio_bridge --uart DEVICE --baud 115200 --seconds 1200 --output NEW_DIRECTORY [--cue ready] [--nav-port 14600 --cue-port 14601] [--quality-min 60]\n";return 0;}
        check(i+1<argc,"Missing option value");std::string value=argv[++i];
        if(key=="--uart")uart=value;else if(key=="--baud")baud=std::stoi(value);
        else if(key=="--seconds")duration=std::stod(value);else if(key=="--output")output=value;
        else if(key=="--startup-seconds")startup_seconds=std::stod(value);
        else if(key=="--nav-port")nav_port=std::stoi(value);else if(key=="--cue-port")cue_port=std::stoi(value);
        else if(key=="--quality-min")quality_min=std::stoi(value);
        else if(key=="--cue")cue=value;else fail("Unknown option "+key);
    }
    check(!output.empty() && !fs::exists(output),"Supply a new bridge output directory");
    check(quality_min>=1 && quality_min<=100,"Invalid relay quality minimum");
    check(std::isfinite(duration) && duration>0 && duration<=7200 && nav_port>0 && nav_port<=65535 && cue_port>0 && cue_port<=65535 && nav_port!=cue_port,"Invalid bridge limits");
    check(cue.empty() || bhu::cube::tune(cue),"Unknown cue");
    check(std::isfinite(startup_seconds) && startup_seconds>=1 && startup_seconds<=120,"Invalid startup timeout");
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
    Serial serial(uart,baud);Parser parser;
    bhu::extnav::RelaySocket relay(feedbackOnly?0:nav_port,1,quality_min);
    Fd feedback(::socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0));
    sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);address.sin_port=htons(cue_port);
    check(feedback.get()>=0 && ::bind(feedback.get(),reinterpret_cast<sockaddr*>(&address),sizeof(address))==0,"Cannot bind Cube cue socket");
    fs::create_directories(output);std::ofstream events(output/"events.jsonl"),wire(output/"cube.tlog",std::ios::binary);
    auto log=[&](const Json& row){events<<row.dump()<<'\n';events.flush();};
    double start=now(),last_heartbeat=0,last_cue=0,next_request=0;bool ready=false,armed=false;std::string last_cue_event;
    uint64_t heartbeats=0,cues=0,link_drops=0,forwarded=0;bool ever_link=false,link_good=false;
    auto sound=[&](const std::string& event){serial.write(packet(bhu::cube::play(event)));++cues;last_cue=now();last_cue_event=event;log({{"event","cue_sent"},{"cue",event},{"host_s",now()},{"audible_confirmation",false}});};
    while(running && now()-start<duration) {
        double current=now();
        if(current>=next_request) {
            // A parameter response is also evidence that this serial link is bidirectional.
            serial.write(packet(parameter_request(1,"MAV_SYSID",1,197)));
            next_request=current+2;
        }
        auto bytes=serial.read();
        parser.feed(bytes.data(),bytes.size(),[&](const mavlink_message_t& message){
            auto raw=packet(message);uint64_t stamp=uint64_t(now()*1e6);std::array<uint8_t,8> prefix{};
            for(int i=0;i<8;++i)prefix[7-i]=uint8_t(stamp>>(i*8));wire.write(reinterpret_cast<char*>(prefix.data()),8);wire.write(reinterpret_cast<char*>(raw.data()),raw.size());
            if(message.sysid!=1 || message.compid!=1)return;
            if(message.msgid==MAVLINK_MSG_ID_HEARTBEAT){mavlink_heartbeat_t h{};mavlink_msg_heartbeat_decode(&message,&h);last_heartbeat=now();armed=h.base_mode&MAV_MODE_FLAG_SAFETY_ARMED;++heartbeats;}
            if(message.msgid==MAVLINK_MSG_ID_PARAM_VALUE){mavlink_param_value_t p{};mavlink_msg_param_value_decode(&message,&p);if(std::string(p.param_id,strnlen(p.param_id,16))=="MAV_SYSID" && p.param_value==1)ever_link=true;}
            if(message.msgid==MAVLINK_MSG_ID_STATUSTEXT){mavlink_statustext_t s{};mavlink_msg_statustext_decode(&message,&s);log({{"event","statustext"},{"text",std::string(s.text,strnlen(s.text,50))},{"severity",s.severity}});}
            if(message.msgid==MAVLINK_MSG_ID_EKF_STATUS_REPORT){mavlink_ekf_status_report_t e{};mavlink_msg_ekf_status_report_decode(&message,&e);log({{"event","ekf_status"},{"host_s",now()},{"flags",unsigned(e.flags)},{"pos_horiz_variance",float(e.pos_horiz_variance)},{"velocity_variance",float(e.velocity_variance)}});}
            if(message.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED){mavlink_local_position_ned_t p{};mavlink_msg_local_position_ned_decode(&message,&p);log({{"event","local_position_ned"},{"host_s",now()},{"x",float(p.x)},{"y",float(p.y)},{"z",float(p.z)}});}
        });
        bool healthy=ever_link && last_heartbeat && current-last_heartbeat<3 && !armed;
        if(healthy && !ready){
            ready=true;
            for(auto request:{std::pair{MAVLINK_MSG_ID_EKF_STATUS_REPORT,1000000},std::pair{MAVLINK_MSG_ID_LOCAL_POSITION_NED,200000}}) {
                mavlink_message_t message{};mavlink_msg_command_long_pack(1,197,&message,1,1,MAV_CMD_SET_MESSAGE_INTERVAL,0,request.first,request.second,0,0,0,0,0);serial.write(packet(message));
            }
            save_json(output/"ready.json",{{"uart",uart},{"disarmed",true},{"bidirectional_link",true}});std::cout<<"CUBE_LINK_READY"<<std::endl;if(!cue.empty())sound(cue);
        }
        if(link_good && !healthy){++link_drops;log({{"event","link_unhealthy_or_armed"}});}
        link_good=healthy;
        check(!armed,"Cube became armed: stopping the bench bridge");
        check(ready || current-start<startup_seconds,"Timed out waiting for a verified bidirectional Cube link");
        check(!ready || (last_heartbeat && current-last_heartbeat<5),"Cube heartbeat lost");
        std::array<char,64> data{};
        for(unsigned i=0;i<16;++i){auto n=::recv(feedback.get(),data.data(),data.size(),MSG_DONTWAIT);if(n<0)break;std::string event(data.data(),n);if(healthy && bhu::cube::tune(event) && (event!=last_cue_event || current-last_cue>.25))sound(event);}
        if(auto message=relay.poll([]{return uint64_t(now()*1e6);});message && healthy){
            const double writeStart=now();serial.write(packet(*message));const double writeEnd=now();++forwarded;
            const auto captureUs=mavlink_msg_odometry_get_time_usec(&*message);
            log({{"event","odometry_forwarded"},{"capture_us",captureUs},{"host_s",writeEnd},
                 {"age_at_serial_write_ms",writeEnd*1000-double(captureUs)*.001},
                 {"serial_write_ms",(writeEnd-writeStart)*1000}});
        }
        pause_ms(5);
    }
    save_json(output/"summary.json",{{"uart",uart},{"navigation_enabled",!feedbackOnly},{"quality_min",quality_min},{"heartbeats",heartbeats},{"cues_sent",cues},{"link_drops",link_drops},{"odometry_forwarded",forwarded},{"odometry_rejected",relay.filter.rejected},{"uart_bad_packets",parser.bad},{"parameter_writes",0},{"fusion_validated",false}});
    return ready?0:1;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
