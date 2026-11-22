// Offline, receive-only audit of the project's timestamp-prefixed MAVLink logs.
#include <mavlink/ardupilotmega/mavlink.h>
#include <json.hpp>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
using J=nlohmann::json;namespace fs=std::filesystem;
int main(int argc,char**argv)try{
 if(argc<3)throw std::runtime_error("Usage: cube_tlog_audit OUTPUT.json INPUT.tlog [...]");
 J files=J::array();uint64_t totalRecords=0,totalEkf=0,totalLocal=0,totalHealthy=0,totalOrigin=0;
 for(int arg=2;arg<argc;arg++){
  fs::path file=argv[arg];std::ifstream in(file,std::ios::binary);if(!in)throw std::runtime_error("Cannot open "+file.string());
  uint64_t records=0,bad=0,truncated=0,ekf=0,healthy=0,local=0,origin=0,heartbeats=0,armed=0;std::map<unsigned,uint64_t> flags,ids;std::set<std::string> messages;uint64_t first=0,last=0;
  while(true){std::array<uint8_t,8> prefix{};in.read(reinterpret_cast<char*>(prefix.data()),8);if(!in.gcount())break;if(in.gcount()!=8){++truncated;break;}
   uint64_t stamp=0;for(auto b:prefix)stamp=(stamp<<8)|b;if(!records)first=stamp;last=stamp;
   std::array<uint8_t,MAVLINK_MAX_PACKET_LEN> bytes{};in.read(reinterpret_cast<char*>(bytes.data()),2);if(in.gcount()!=2){++truncated;break;}
   if(bytes[0]!=MAVLINK_STX&&bytes[0]!=MAVLINK_STX_MAVLINK1)throw std::runtime_error("Unknown framing in "+file.string());
   const bool v2=bytes[0]==MAVLINK_STX;const unsigned header=v2?10:6;
   in.read(reinterpret_cast<char*>(bytes.data()+2),header-2);if(in.gcount()!=header-2){++truncated;break;}
   const unsigned tail=bytes[1]+2+(v2&&(bytes[2]&MAVLINK_IFLAG_SIGNED)?13:0);
   in.read(reinterpret_cast<char*>(bytes.data()+header),tail);if(in.gcount()!=tail){++truncated;break;}
   mavlink_message_t buffer{},message{};mavlink_status_t state{},status{};unsigned complete=0;
   for(unsigned i=0;i<header+tail;i++)if(mavlink_frame_char_buffer(&buffer,&state,bytes[i],&message,&status)==MAVLINK_FRAMING_OK)++complete;
   ++records;if(complete!=1){++bad;continue;}if(message.sysid!=1||message.compid!=1)continue;++ids[message.msgid];
   if(message.msgid==MAVLINK_MSG_ID_EKF_STATUS_REPORT){mavlink_ekf_status_report_t r{};mavlink_msg_ekf_status_report_decode(&message,&r);++ekf;++flags[r.flags];if(!(r.flags&EKF_UNINITIALIZED)&&(r.flags&EKF_ATTITUDE)&&(r.flags&(EKF_POS_HORIZ_REL|EKF_POS_HORIZ_ABS)))++healthy;}
   else if(message.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED)++local;
   else if(message.msgid==MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN)++origin;
   else if(message.msgid==MAVLINK_MSG_ID_HEARTBEAT){mavlink_heartbeat_t r{};mavlink_msg_heartbeat_decode(&message,&r);++heartbeats;if(r.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)++armed;}
   else if(message.msgid==MAVLINK_MSG_ID_STATUSTEXT){mavlink_statustext_t r{};mavlink_msg_statustext_decode(&message,&r);std::string s(r.text,strnlen(r.text,50));if(s.find("EKF")!=s.npos||s.find("VisOdom")!=s.npos||s.find("origin")!=s.npos||s.find("Origin")!=s.npos||s.find("external")!=s.npos)messages.insert(s);}
  }
  J flagJson=J::object(),idJson=J::object();for(auto[k,v]:flags)flagJson[std::to_string(k)]=v;for(auto[k,v]:ids)idJson[std::to_string(k)]=v;
  files.push_back({{"file",file.string()},{"records",records},{"bad_crc_records",bad},{"truncated_records",truncated},{"span_s",last>=first?(last-first)*1e-6:0},{"ekf_reports",ekf},{"ekf_flags",flagJson},{"healthy_horizontal_status_reports",healthy},{"local_position_samples",local},{"origin_messages",origin},{"heartbeats",heartbeats},{"armed_heartbeats",armed},{"autopilot_message_ids",idJson},{"relevant_statustext",messages}});
  totalRecords+=records;totalEkf+=ekf;totalLocal+=local;totalHealthy+=healthy;totalOrigin+=origin;
 }
 J result={{"files",files},{"totals",{{"files",files.size()},{"records",totalRecords},{"ekf_reports",totalEkf},{"healthy_horizontal_status_reports",totalHealthy},{"local_position_samples",totalLocal},{"origin_messages",totalOrigin}}},{"fusion_proven",false},{"scope","BHUslamV3 saved Cube receive telemetry only; not an estimator replay or a new live test"},{"note","Healthy telemetry alone would not establish which measurement source was fused. Origin absence from these logs does not establish that no origin existed unless explicitly queried."}};
 std::ofstream out(argv[1]);if(!out)throw std::runtime_error("Cannot create audit output");out<<result.dump(2)<<'\n';std::cout<<result["totals"].dump(2)<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
