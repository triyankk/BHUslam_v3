#include "native.hpp"
#include "bhu/extnav_relay.hpp"
#include "bhu/observation_guard.hpp"
#include <pty.h>
#include <iostream>

int main() try {
    using namespace tof;
    using Guard=bhu::extnav::ObservationGuard;
    uint64_t tick=10000000;
    auto load=[&](Guard& guard,const std::string& profile,uint64_t stamp){
        for(auto [name,value]:Guard::profiles().at(profile))guard.parameter(name,value,stamp);
    };
    for(const auto& [profile,parameters]:Guard::profiles()) {
        Guard guard;
        check(!guard.reason(tick).empty(),"No telemetry blocks forwarding");
        guard.observe_heartbeat(false,tick);load(guard,profile,tick);
        check(guard.profile(tick)==profile,"Exact navigation profile identified");
        check(guard.reason(tick).empty()==(profile!="gps_recording"),"Only enabled profiles permit forwarding");
        guard.observe_heartbeat(true,tick+1);
        check(guard.reason(tick+1).empty()==(profile!="gps_recording"),"Qualified profile persists while armed");
        guard.parameter("VISO_POS_X",.15f,tick);
        check(guard.reason(tick+1)=="Unsupported Cube navigation profile","Wrong mount blocks output");load(guard,profile,tick);
        guard.parameter("EK3_SRC_OPTIONS",1,tick);
        check(guard.reason(tick+1)=="Unsupported Cube navigation profile","Fuse-all velocities blocks output");load(guard,profile,tick);
        guard.parameter("VISO_POS_X",std::numeric_limits<float>::quiet_NaN(),tick);
        check(guard.reason(tick+1)=="Nonfinite VISO_POS_X","NaN parameter blocks output");load(guard,profile,tick);
        guard.heartbeat=tick+5000001;
        check(!guard.reason(tick+5000001).empty(),"Stale audit blocks with live heartbeat");
        guard.heartbeat=tick;guard.nonprimary_observed=true;
        check(!guard.reason(tick).empty(),"Observed source switch latches off");
    }
    Guard guard;load(guard,"gps_observation",tick);guard.observe_heartbeat(false,tick);
    check(guard.reason(tick).empty(),"GPS observation qualifies disarmed");
    guard.observe_heartbeat(true,tick+1);load(guard,"gps_free_vio",tick+1);
    check(guard.profile(tick+1)=="gps_free_vio"&&!guard.reason(tick+1).empty(),"In-flight profile changes blocked");
    guard.observe_heartbeat(false,tick+2);check(guard.reason(tick+2).empty(),"VIO profile may qualify after disarm");
    guard.observe_heartbeat(true,tick+3);check(guard.reason(tick+3).empty(),"VIO forwarding works armed after qualification");
    guard.observe_heartbeat(true,tick+2000000);load(guard,"gps_free_vio",tick+2000000);
    check(!guard.reason(tick+2000000).empty(),"Armed heartbeat gap requires new disarmed startup");
    Guard startup;load(startup,"gps_free_vio",tick);startup.observe_heartbeat(true,tick);
    check(!startup.reason(tick).empty(),"Armed startup cannot qualify VIO");
    Guard mixed;load(mixed,"gps_observation",tick);mixed.observe_heartbeat(false,tick);
    mixed.parameter("EK3_SRC1_VELXY",5,tick);mixed.parameter("EK3_SRC1_VELZ",0,tick);
    check(!mixed.reason(tick).empty(),"Old GPS/flow hybrid is outside the approved profiles");
    // Reserve an available local UDP port, then use the production relay.
    Fd sender(::socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0));
    check(sender.get()>=0,"Create test UDP sender");
    sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    {
        Fd probe(::socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0));
        check(::bind(probe.get(),reinterpret_cast<sockaddr*>(&address),sizeof(address))==0,"Find UDP port");
        socklen_t length=sizeof(address);
        check(::getsockname(probe.get(),reinterpret_cast<sockaddr*>(&address),&length)==0,"Read UDP port");
    }
    bhu::extnav::RelaySocket relay(ntohs(address.sin_port),1,40,0,0);
    uint64_t stamp=uint64_t(now()*1e6)-400000;
    mavlink_message_t message{};float q[4]={1,0,0,0},unknown[21]{};
    unknown[0]=std::numeric_limits<float>::quiet_NaN();
    mavlink_msg_odometry_pack(1,197,&message,stamp,MAV_FRAME_LOCAL_FRD,MAV_FRAME_BODY_FRD,
        1,2,3,q,.1,.2,.3,0,0,0,unknown,unknown,7,MAV_ESTIMATOR_TYPE_VIO,40);
    auto bytes=packet(message);
    check(::sendto(sender.get(),bytes.data(),bytes.size(),0,reinterpret_cast<sockaddr*>(&address),sizeof(address))==ssize_t(bytes.size()),"Send local odometry");
    std::optional<mavlink_message_t> received;
    for(int i=0;i<20 && !received;++i) {
        received=relay.poll([] {return uint64_t(now()*1e6);});
        if(!received)pause_ms(1);
    }
    check(received.has_value(),"Receive odometry through production UDP relay");
    int master_fd=-1,slave_fd=-1;char path[128]{};
    check(::openpty(&master_fd,&slave_fd,path,nullptr,nullptr)==0,"Create simulated UART");
    Fd master(master_fd);::close(slave_fd);
    Serial serial(path,57600);
    serial.write(packet(*received));
    Parser decoder;unsigned accepted=0;
    double deadline=now()+1;
    while(!accepted && now()<deadline) {
        pollfd waiting{master.get(),POLLIN,0};if(::poll(&waiting,1,20)<=0)continue;
        std::array<uint8_t,512> data{};auto n=::read(master.get(),data.data(),data.size());
        if(n<=0)continue;
        decoder.feed(data.data(),size_t(n),[&](const mavlink_message_t& wire) {
            check(wire.msgid==MAVLINK_MSG_ID_ODOMETRY && wire.sysid==1 && wire.compid==197,"UART message identity");
            mavlink_odometry_t o{};mavlink_msg_odometry_decode(&wire,&o);
            check(o.time_usec==stamp && o.reset_counter==7 && o.x==1 && o.y==2 && o.z==3 && o.quality==40,"UART fields/capture timestamp/quality 40 preserved");
            ++accepted;
        });
    }
    check(accepted==1 && decoder.bad==0,"Valid UART frame/CRC");
    mavlink_msg_command_long_pack(1,197,&message,1,1,MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0);
    bytes=packet(message);
    check(::sendto(sender.get(),bytes.data(),bytes.size(),0,reinterpret_cast<sockaddr*>(&address),sizeof(address))==ssize_t(bytes.size()),"Send rejected fixture");
    for(int i=0;i<20;++i) {
        check(!relay.poll([] {return uint64_t(now()*1e6);}),"Relay rejects non-odometry command");
        if(relay.filter.rejected)break;
        pause_ms(1);
    }
    check(relay.filter.forwarded==1 && relay.filter.rejected==1,"Transport counters");
    std::cout<<"PASS: loopback UDP -> odometry-only relay -> native UART -> pseudo-terminal decoding; arm command rejected\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
