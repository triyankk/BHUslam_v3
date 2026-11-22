#include "native.hpp"
#include <pty.h>
#include <sys/wait.h>
#include <iostream>
#include <map>
#include <set>
using namespace tof;
static void fixture(const char*tool,bool armed,bool occupied){
 int master,slave;char path[128]{};check(openpty(&master,&slave,path,nullptr,nullptr)==0,"PTY");Fd port(master);fcntl(master,F_SETFL,O_NONBLOCK);
 auto out=fs::temp_directory_path()/("bhu-responsive-"+std::to_string(getpid())+std::to_string(armed)+std::to_string(occupied));fs::remove_all(out);
 auto child=fork();check(child>=0,"fork");if(!child){close(master);close(slave);execl(tool,tool,"--uart",path,"--output",out.c_str(),"--responsive-avoidance-only","--apply",nullptr);_exit(127);}
 std::map<std::string,float> p{{"RNGFND1_TYPE",27},{"RNGFND1_ORIENT",25},{"RNGFND1_MIN",.1f},{"RNGFND1_MAX",180},{"RNGFND1_POS_Z",.1f},{"RNGFND2_TYPE",occupied?10.f:0.f},{"RNGFND2_ORIENT",25},{"RNGFND2_MIN",.01f},{"RNGFND2_MAX",5},{"PRX1_TYPE",2},{"PRX2_TYPE",occupied?0.f:4.f},{"PRX_FILT",.25f},{"AVOID_ACCEL_MAX",.5f},{"AVOID_BEHAVE",1},{"AVOID_MARGIN",2},{"FLOW_TYPE",6},{"FLOW_POS_Z",.1f},{"CAN_P2_DRIVER",1},{"CAN_D1_PROTOCOL",1},{"EK3_SRC1_POSXY",6},{"EK3_SRC1_VELXY",5},{"GPS1_TYPE",9},{"ARMING_SKIPCHK",304374}};
 std::set<std::string> allowed{"PRX_FILT","AVOID_ACCEL_MAX"};unsigned writes=0,reboots=0;Parser parser;int status=0;bool done=false;auto start=now();double next=0;
 auto send=[&](mavlink_message_t&m){auto bytes=packet(m);check(write(master,bytes.data(),bytes.size())==ssize_t(bytes.size()),"write fixture");};
 while(now()-start<15){if(now()>=next){mavlink_message_t m{};mavlink_msg_heartbeat_pack(1,1,&m,2,3,armed?128:0,0,3);send(m);next=now()+.1;}
  std::array<uint8_t,4096> bytes{};auto n=read(master,bytes.data(),bytes.size());if(n>0)parser.feed(bytes.data(),n,[&](const mavlink_message_t&m){
   if(m.msgid==MAVLINK_MSG_ID_PARAM_REQUEST_READ){mavlink_param_request_read_t q{};mavlink_msg_param_request_read_decode(&m,&q);std::string name(q.param_id,strnlen(q.param_id,16));mavlink_message_t r{};char field[16]{};memcpy(field,name.data(),name.size());mavlink_msg_param_value_pack(1,1,&r,field,p[name],9,100,0);send(r);}
   else if(m.msgid==MAVLINK_MSG_ID_PARAM_SET){check(!armed&&!occupied,"Unsafe fixture must have zero writes");mavlink_param_set_t q{};mavlink_msg_param_set_decode(&m,&q);std::string name(q.param_id,strnlen(q.param_id,16));check(allowed.count(name),"No unrelated writes");p[name]=q.param_value;++writes;}
   else if(m.msgid==MAVLINK_MSG_ID_COMMAND_LONG){mavlink_command_long_t q{};mavlink_msg_command_long_decode(&m,&q);check(!armed&&!occupied&&q.command==MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN&&q.param1==1,"Only authorized reboot");++reboots;}
   else fail("Unexpected Cube command");
  });
  if(waitpid(child,&status,WNOHANG)==child){done=true;break;}pause_ms(2);
 }
 if(!done){kill(child,SIGKILL);waitpid(child,&status,0);}close(slave);check(done&&WIFEXITED(status),"Fixture completed");
 if(armed||occupied)check(WEXITSTATUS(status)==1&&writes==0&&reboots==0,"Fail closed without writes/reboot");
 else {check(WEXITSTATUS(status)==0&&writes==2&&reboots==0,"Two scoped changes without reboot");auto before=Json::parse(read_text(out/"before.json")),after=Json::parse(read_text(out/"after.json"));for(auto it=before.begin();it!=before.end();++it)if(!allowed.count(it.key()))check(after[it.key()]==it.value(),"All unrelated settings preserved");check(after["PRX_FILT"]==2&&after["AVOID_ACCEL_MAX"]==1,"Responsive tuning readback");check(fs::file_size(out/"restore.param")>0,"Durable rollback");}
 fs::remove_all(out);
}
int main(int argc,char**argv)try{check(argc==2,"Config tool required");fixture(argv[1],false,false);fixture(argv[1],true,false);fixture(argv[1],false,true);std::cout<<"PASS scoped avoidance tuning, backup/readback, source preservation, armed/occupied rejection\n";}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
