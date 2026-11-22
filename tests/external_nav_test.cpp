#include "bhu/external_nav.hpp"
#include "bhu/extnav_relay.hpp"
#include <iostream>

using namespace bhu::extnav;
static void require(bool ok,const char* message) {if(!ok)throw std::runtime_error(message);}
static Sample sample(uint64_t t) {
    Sample s;s.capture_host_us=t;s.quality=60;
    s.tracking_ok=s.imu_initialized=s.frame_alignment_valid=s.velocity_valid=true;
    return s;
}
int main() try {
    // Forward-facing upright camera: optical x right, y down, z forward.
    Alignment a;a.R_B_C<<0,0,1, 1,0,0, 0,1,0;
    a.T_L_W.linear()=a.R_B_C;
    auto s=sample(1000000);s.T_W_C.translation()<<1,2,3;
    s.camera_velocity_W<<1,2,3;s.angular_velocity_C<<.1,.2,.3;
    auto p=convert(s,a);
    require((p.camera_position_L-Eigen::Vector3d(3,1,2)).norm()<1e-9,"Optical/FRD position axes");
    require(p.q_L_B.angularDistance(Eigen::Quaterniond::Identity())<1e-9,"Vehicle quaternion mounting correction");
    require((p.camera_velocity_B-Eigen::Vector3d(3,1,2)).norm()<1e-9,"Velocity body axes");
    require((p.angular_velocity_B-Eigen::Vector3d(.3,.1,.2)).norm()<1e-9,"Angular rate body axes");
    // Measured downward mount: optical forward has positive body down.
    Alignment pitched;
    const double pitch=22.8*3.14159265358979323846/180;
    pitched.R_B_C=Eigen::AngleAxisd(-pitch,Eigen::Vector3d::UnitY()).toRotationMatrix()*a.R_B_C;
    require((pitched.R_B_C*Eigen::Vector3d::UnitZ()-Eigen::Vector3d(std::cos(pitch),0,std::sin(pitch))).norm()<1e-9,"Downward camera optical axis/sign");
    auto mounted=sample(1000000);
    // A level vehicle sees this camera attitude in local FRD coordinates.
    mounted.T_W_C.linear()=pitched.R_B_C;
    mounted.T_W_C.translation()<<.13,0,.05;
    mounted.camera_velocity_W<<.2,.3,.4;
    mounted.angular_velocity_C=pitched.R_B_C.transpose()*Eigen::Vector3d(.1,.2,.3);
    auto corrected=convert(mounted,pitched);
    require(corrected.q_L_B.angularDistance(Eigen::Quaterniond::Identity())<1e-9,"22.8 degree mount removed from body attitude");
    require((corrected.camera_position_L-Eigen::Vector3d(.13,0,.05)).norm()<1e-9,"Keep camera origin; Cube corrects lever arm once");
    require((corrected.angular_velocity_B-Eigen::Vector3d(.1,.2,.3)).norm()<1e-9,"Measured mount rotates angular rates consistently");
    mounted.T_W_C.linear()=Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitY()).toRotationMatrix()*pitched.R_B_C;
    corrected=convert(mounted,pitched);
    require(corrected.q_L_B.angularDistance(Eigen::Quaterniond(Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitY())))<1e-9,"Preserve actual vehicle pitch after mount correction");
    // A world-frame yaw changes position but not the body-frame velocity.
    a.T_L_W.linear()=Eigen::AngleAxisd(1.5707963267948966,Eigen::Vector3d::UnitZ()).toRotationMatrix()*a.R_B_C;
    p=convert(s,a);
    require((p.camera_position_L-Eigen::Vector3d(-1,3,2)).norm()<1e-8,"Local alignment rotation");
    require((p.camera_velocity_B-Eigen::Vector3d(3,1,2)).norm()<1e-8,"Velocity is body, not world");
    auto message=encode(s,p,255);
    std::array<uint8_t,MAVLINK_MAX_PACKET_LEN> bytes{};auto n=mavlink_msg_to_send_buffer(bytes.data(),&message);
    mavlink_message_t buffer{},decoded{};mavlink_status_t state{},status{};unsigned accepted=0;
    for(unsigned i=0;i<n;++i)if(mavlink_frame_char_buffer(&buffer,&state,bytes[i],&decoded,&status)==MAVLINK_FRAMING_OK)++accepted;
    require(accepted==1 && decoded.msgid==MAVLINK_MSG_ID_ODOMETRY,"MAVLink wire/CRC roundtrip");
    mavlink_odometry_t odom{};mavlink_msg_odometry_decode(&decoded,&odom);
    require(odom.frame_id==MAV_FRAME_LOCAL_FRD && odom.child_frame_id==MAV_FRAME_BODY_FRD,"MAVLink frames");
    require(odom.reset_counter==255 && odom.quality==60 && odom.time_usec==1000000,"Reset/quality/capture time");
    require(std::abs(odom.q[0]-std::sqrt(.5))<1e-6 && std::abs(odom.q[3]-std::sqrt(.5))<1e-6,"MAVLink quaternion wxyz");
    require(std::isnan(odom.pose_covariance[0]) && std::isnan(odom.velocity_covariance[0]),"Unknown covariance");
    RelayFilter relay;
    require(relay.ingest(bytes.data(),n,1001000),"Relay accepts current odometry");
    auto forwarded=relay.take(1001000);
    require(forwarded && forwarded->msgid==MAVLINK_MSG_ID_ODOMETRY,"Relay forward");
    require(!relay.take(1200000),"Relay never replays old sample");
    require(!relay.ingest(bytes.data(),n,1001000),"Relay rejects duplicates");
    bytes[12]^=1;require(!relay.ingest(bytes.data(),n,1001000),"Relay rejects corrupt CRC");bytes[12]^=1;
    RelayFilter stale;require(!stale.ingest(bytes.data(),n,1200000),"Relay rejects stale acquisition time");
    RelayFilter future;require(!future.ingest(bytes.data(),n,999999),"Relay rejects future acquisition time");
    auto pack=[&](uint64_t stamp,int quality=60,uint8_t component=197) {
        auto input=sample(stamp);input.quality=quality;
        auto wire=encode(input,p,0,1,component);
        return mavlink_msg_to_send_buffer(bytes.data(),&wire);
    };
    RelayFilter fusionRelay(1,197,40);
    n=pack(1000000,39);require(!fusionRelay.ingest(bytes.data(),n,1001000),"Fusion rejects quality 39");
    n=pack(1000000,40);require(fusionRelay.ingest(bytes.data(),n,1001000),"Fusion accepts quality 40");
    auto fusionForwarded=fusionRelay.take(1001000);
    require(fusionForwarded&&mavlink_msg_odometry_get_quality(&*fusionForwarded)==40,"Quality 40 forwarded unchanged");
    n=pack(1200000,59);require(fusionRelay.ingest(bytes.data(),n,1201000),"Fusion accepts quality 59");
    require(fusionRelay.take(1201000).has_value(),"Previously blocked quality forwarded");
    n=pack(1400000,40);require(!fusionRelay.ingest(bytes.data(),n,1550001),"Fusion retains stale rejection");
    require(!fusionRelay.ingest(bytes.data(),n,1399999),"Fusion retains future rejection");
    n=pack(1033333);require(relay.ingest(bytes.data(),n,1034333),"Relay ingests latest");
    require(!relay.take(1034333),"Relay rate cap");
    n=pack(1066666);require(relay.ingest(bytes.data(),n,1067666),"Relay replaces pending sample");
    forwarded=relay.take(1101000);
    require(forwarded && mavlink_msg_odometry_get_time_usec(&*forwarded)==1066666,"Relay chooses newest only");
    n=pack(1133333,59);require(!relay.ingest(bytes.data(),n,1134333),"Relay enforces quality minimum");
    n=pack(1133333,60,191);require(!relay.ingest(bytes.data(),n,1134333),"Relay rejects wrong component");
    n=pack(1133333);require(relay.ingest(bytes.data(),n,1134333),"Relay pending sample");
    require(!relay.take(1300000),"Relay rechecks freshness before UART");
    RelayFilter cubeTiming(1,197,40,0,0);
    n=pack(1000000,40);require(cubeTiming.ingest(bytes.data(),n,1400000),"Cube timing accepts old capture without rewriting timestamp");
    auto delayedWire=cubeTiming.take(1600000);
    require(delayedWire && mavlink_msg_odometry_get_time_usec(&*delayedWire)==1000000,"No final relay age cutoff");
    require(!cubeTiming.take(1800000),"No repeated old pose to hide Cube timeout");
    require(!cubeTiming.ingest(bytes.data(),n,1800000),"Cube timing retains duplicate rejection");
    n=pack(1000001,40);require(cubeTiming.ingest(bytes.data(),n,1600001)&&cubeTiming.take(1600001),"Direct relay does not throttle a new sample");
    n=pack(2000000,40);require(!cubeTiming.ingest(bytes.data(),n,1999999),"Cube timing retains future rejection");
    Gate gate(2);
    require(!gate.accept(s,1001000),"Warmup sample must not publish");
    s.capture_host_us+=33333;auto reset=gate.accept(s,s.capture_host_us+1000);
    require(reset && *reset==0,"Continuous good samples publish");
    require(!gate.accept(s,s.capture_host_us+1000),"Duplicate must not publish");
    s.capture_host_us+=33333;require(!gate.accept(s,s.capture_host_us),"Recovery warmup");
    s.capture_host_us+=33333;reset=gate.accept(s,s.capture_host_us);
    require(reset && *reset==1,"Recovery increments reset counter");
    s.capture_host_us+=33333;s.imu_initialized=false;
    require(!gate.accept(s,s.capture_host_us),"Reject uninitialized VIO");
    s.imu_initialized=true;s.capture_host_us+=33333;s.reset_epoch=1;
    require(!gate.accept(s,s.capture_host_us),"Map switch warmup");
    s.capture_host_us+=33333;reset=gate.accept(s,s.capture_host_us);
    require(reset && *reset==2,"Map reset increment");
    s.capture_host_us+=33333;require(!gate.accept(s,s.capture_host_us+150001),"Stale capture rejected");
    s.capture_host_us+=33333;require(!gate.accept(s,s.capture_host_us-1),"Future capture rejected");
    s.capture_host_us+=33333;s.velocity_valid=false;require(!gate.accept(s,s.capture_host_us),"Unknown velocity rejected");
    s.velocity_valid=true;s.capture_host_us+=33333;s.T_W_C.translation().x()=std::numeric_limits<double>::quiet_NaN();
    require(!gate.accept(s,s.capture_host_us),"Nonfinite pose rejected");
    a.R_B_C=-Eigen::Matrix3d::Identity();bool rejected=false;
    try {convert(sample(1000000),a);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"Reflected camera mount rejected");
    std::cout<<"PASS: frame conversion, quaternion order, MAVLink CRC, covariance, freshness, warmup and resets\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
