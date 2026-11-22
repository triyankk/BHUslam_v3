#include "bhu/flight_recording.hpp"
#include "bhu/recording_cue.hpp"
#include <iostream>
static void check(bool v){if(!v)throw std::runtime_error("Flight/cue test failed");}
static std::string state(int tick,bool armed,bool connected=true){return nlohmann::json{{"running",true},{"tick",tick},{"cube",{{"connected",connected},{"armed",armed},{"heartbeat_age_ms",10}}}}.dump();}
int main(){
 using F=bhu::FlightRecording;F f("");
 check(f.observe(state(0,true),1000000)==F::None); // Ignore initial cached snapshot.
 check(f.observe(state(1,false),1500000)==F::None);
 check(f.fresh_disarmed(1500000)&&!f.fresh_disarmed(3000001));
 check(f.observe(state(2,true),2000000)==F::Start&&f.active());
 check(!f.fresh_disarmed(2000000));
 check(f.observe("corrupt",2500000)==F::None&&f.active());
 check(f.observe(state(3,false,false),3000000)==F::None&&f.active());
 check(!f.fresh_disarmed(3000000));
 check(f.observe(state(4,false),3500000)==F::None);
 check(f.observe(state(5,true),4000000)==F::None); // Short disarm/rearm remains one recording.
 for(int i=0;i<6;i++)check(f.observe(state(6+i,false),4500000+i*500000)==F::None);
 check(f.observe(state(12,false),7500000)==F::Stop&&!f.active());
 check(f.observe(state(13,true),8000000)==F::Start);
 // Cue is silent until confirmed disk recording, ACK stops retries.
 int server=::socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0);check(server>=0);
 sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);check(::bind(server,reinterpret_cast<sockaddr*>(&a),sizeof(a))==0);socklen_t n=sizeof(a);::getsockname(server,reinterpret_cast<sockaddr*>(&a),&n);
 bhu::RecordingCue cue(ntohs(a.sin_port),123);char bytes[128];sockaddr_in peer{};n=sizeof(peer);
 cue.poll(false,10000000);check(::recvfrom(server,bytes,sizeof(bytes),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&peer),&n)<0);
 cue.poll(true,11000000);auto count=::recvfrom(server,bytes,sizeof(bytes),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&peer),&n);check(count>0&&std::string(bytes,count)=="VIOREC1 123 11000000");
 std::string ack="VIOACK1 123";::sendto(server,ack.data(),ack.size(),0,reinterpret_cast<sockaddr*>(&peer),n);cue.poll(true,12000000);check(cue.acknowledged());
 cue.poll(true,20000000);check(::recv(server,bytes,sizeof(bytes),MSG_DONTWAIT)<0);::close(server);
 std::cout<<"PASS: multi-flight arm/disarm tail, no false disarm on link loss, and recording-only ACKed cue\n";
}
