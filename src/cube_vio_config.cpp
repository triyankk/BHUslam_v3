#include "native.hpp"
#include "bhu/observation_guard.hpp"
#include <iostream>
#include <map>

// Explicit bench configuration, with a durable before snapshot and requested
// readback after each write. Never selects an EKF source or changes arming.
int main(int argc,char** argv) try {
    using namespace tof;
    std::string uart="/dev/serial/by-id/usb-CubePilot_CubeOrange+_36003F000B51333338373339-if00";
    fs::path output;bool responsiveAvoidance=false,apply=false,reboot=false,qualityOnly=false,gpsShadowOnly=false,gpsRecording=false,gpsFree=false,clearUnused=false,hybrid=false,hybridPrimary=false,disableOriginReuse=false,gpsBootstrap=false,hflowRear=false,disableProximity=false,enableProximity=false,observationOnly=false,mountOnly=false;int baud=115200,disarmedLog=-1;
    for(int i=1;i<argc;++i){std::string key=argv[i];if(key=="--apply"){apply=true;continue;}if(key=="--reboot"){reboot=true;continue;}if(key=="--quality-only"){qualityOnly=true;continue;}
        if(key=="--gps-shadow-only"){gpsShadowOnly=true;continue;}
        if(key=="--gps-recording-only"){gpsRecording=true;continue;}
        if(key=="--gps-free-only"){gpsFree=true;continue;}
        if(key=="--vio-optflow-compass-only"){hybrid=true;continue;}
        if(key=="--vio-optflow-compass-primary-only"){hybrid=true;hybridPrimary=true;continue;}
        if(key=="--responsive-avoidance-only"){responsiveAvoidance=true;continue;}
        if(key=="--hflow-down-rear-only"){hflowRear=true;continue;}
        if(key=="--gps-bootstrap-only"){gpsBootstrap=true;continue;}
        if(key=="--disable-origin-reuse-only"){disableOriginReuse=true;continue;}
        if(key=="--clear-unused-sources-only"){clearUnused=true;continue;}
        if(key=="--mount-only"){mountOnly=true;continue;}
        if(key=="--observation-only"){observationOnly=true;continue;}
        if(key=="--enable-proximity-only"){enableProximity=true;continue;}
        if(key=="--disable-proximity-only"){disableProximity=true;continue;}
        if(key=="--help"){std::cout<<"cube_vio_config --output NEW_DIRECTORY [--uart DEVICE --baud 115200] [--apply] [--reboot] [--responsive-avoidance-only | --quality-only | --gps-shadow-only | --gps-recording-only | --gps-free-only | --vio-optflow-compass-only | --vio-optflow-compass-primary-only | --hflow-down-rear-only | --gps-bootstrap-only | --disable-origin-reuse-only | --clear-unused-sources-only | --disable-proximity-only | --enable-proximity-only | --observation-only | --mount-only | --disarmed-log 0/1]\n";return 0;}
        check(i+1<argc,"Missing option value");std::string value=argv[++i];if(key=="--uart")uart=value;else if(key=="--baud")baud=std::stoi(value);else if(key=="--output")output=value;else if(key=="--disarmed-log"){disarmedLog=std::stoi(value);check(disarmedLog==0||disarmedLog==1,"Disarmed log must be 0 or 1");}else fail("Unknown option "+key);}
    check(!output.empty() && !fs::exists(output),"Supply a new configuration output directory");
    check(!reboot || apply,"Reboot requires --apply");
    check(int(responsiveAvoidance)+int(qualityOnly)+int(gpsShadowOnly)+int(gpsRecording)+int(gpsFree)+int(clearUnused)+int(hybrid)+int(disableOriginReuse)+int(gpsBootstrap)+int(hflowRear)+int(disableProximity)+int(enableProximity)+int(observationOnly)+int(mountOnly)+int(disarmedLog>=0)<=1,"Select only one configuration mode");
    const std::map<std::string,float> preset={{"VISO_TYPE",3},{"VISO_ORIENT",0},{"VISO_POS_X",.13f},{"VISO_POS_Y",0},{"VISO_POS_Z",.05f},{"VISO_QUAL_MIN",40},
        {"EK3_SRC2_POSXY",6},{"EK3_SRC2_VELXY",0},{"EK3_SRC2_POSZ",1},{"EK3_SRC2_VELZ",0},{"EK3_SRC2_YAW",1}};
    std::map<std::string,float> unused;for(auto set:{2,3})for(auto axis:{"POSXY","VELXY","POSZ","VELZ","YAW"})unused["EK3_SRC"+std::to_string(set)+"_"+axis]=std::string(axis)=="POSZ"?1.f:0.f;
    auto wanted=hybrid?bhu::extnav::ObservationGuard::profiles().at("vio_optflow_compass"):clearUnused?unused:gpsFree?bhu::extnav::ObservationGuard::profiles().at("gps_free_vio"):disarmedLog>=0?std::map<std::string,float>{{"LOG_DISARMED",float(disarmedLog)}}:gpsRecording?std::map<std::string,float>{{"EK3_SRC1_VELXY",3},{"EK3_SRC1_VELZ",3},{"VISO_TYPE",0}}:mountOnly?std::map<std::string,float>{{"VISO_POS_X",.13f},{"VISO_POS_Y",0},{"VISO_POS_Z",.05f}}:observationOnly?std::map<std::string,float>{{"VISO_TYPE",3}}:enableProximity?std::map<std::string,float>{{"PRX1_TYPE",2}}:disableProximity?std::map<std::string,float>{{"PRX1_TYPE",0}}:(gpsShadowOnly?std::map<std::string,float>{{"VISO_TYPE",0}}:(qualityOnly?std::map<std::string,float>{{"VISO_QUAL_MIN",40}}:preset));
    if(responsiveAvoidance)wanted={{"PRX_FILT",2},{"AVOID_ACCEL_MAX",1}};
    if(hflowRear)wanted={{"RNGFND1_ORIENT",4},{"RNGFND2_TYPE",24},{"RNGFND2_ORIENT",25},{"RNGFND2_MIN",.08f},{"RNGFND2_MAX",30},{"RNGFND2_ADDR",0},{"PRX2_TYPE",4}};
    if(gpsBootstrap)wanted={{"GPS1_TYPE",9},{"AHRS_OPTIONS",0}};
    if(disableOriginReuse)wanted={{"AHRS_OPTIONS",0}};
    if(hybridPrimary)wanted={{"EK3_SRC1_POSXY",6},{"EK3_SRC1_VELXY",5}};
    else if(gpsFree||hybrid)wanted.insert(unused.begin(),unused.end());
    std::vector<std::string> names;for(const auto& pair:preset)names.push_back(pair.first);
    for(auto name:{"EK3_SRC1_POSXY","EK3_SRC1_VELXY","EK3_SRC1_POSZ","EK3_SRC1_VELZ","EK3_SRC1_YAW","RNGFND1_TYPE","RNGFND1_ORIENT","RNGFND2_TYPE","RNGFND2_ORIENT","PRX1_TYPE","SERIAL1_BAUD","SERIAL1_PROTOCOL","NTF_BUZZ_ENABLE","NTF_BUZZ_VOLUME","NTF_BUZZ_TYPES"})names.emplace_back(name);
    if(responsiveAvoidance||enableProximity||gpsFree||clearUnused||hybrid)for(auto name:{"OA_TYPE","OA_BR_TYPE","OA_BR_LOOKAHEAD","OA_MARGIN_MAX","OA_DB_SIZE","OA_DB_ALT_MIN","OA_DB_EXPIRE","AVOID_ENABLE","AVOID_MARGIN","AVOID_ALT_MIN","PRX_IGN_GND","PRX_FILT","RC7_OPTION"})names.emplace_back(name);
    if(gpsShadowOnly||gpsRecording||gpsFree||clearUnused||hybrid||gpsBootstrap||hflowRear||observationOnly||mountOnly||enableProximity)for(auto name:{"EK3_SRC3_POSXY","EK3_SRC3_VELXY","EK3_SRC3_POSZ","EK3_SRC3_VELZ","EK3_SRC3_YAW","EK3_SRC_OPTIONS","GPS1_TYPE","GPS2_TYPE","SERIAL2_BAUD","SERIAL2_PROTOCOL","SERIAL2_OPTIONS","BRD_SER2_RTSCTS","ARMING_SKIPCHK"})names.emplace_back(name);
    if(gpsRecording||gpsFree||clearUnused||hybrid||gpsBootstrap||hflowRear||disableOriginReuse)for(auto name:{"AHRS_EKF_TYPE","EK3_ENABLE","COMPASS_ENABLE","COMPASS_USE","COMPASS_USE2","COMPASS_USE3","AHRS_OPTIONS","AHRS_ORIGIN_LAT","AHRS_ORIGIN_LON","AHRS_ORIGIN_ALT","LOG_BITMASK","LOG_BACKEND_TYPE","LOG_DISARMED","RC7_OPTION","OA_TYPE","OA_BR_TYPE","OA_BR_LOOKAHEAD","OA_MARGIN_MAX","AVOID_ENABLE"})names.emplace_back(name);
    if(gpsBootstrap)for(auto name:{"CAN_P1_DRIVER","CAN_P2_DRIVER","CAN_P1_BITRATE","CAN_P2_BITRATE","CAN_D1_PROTOCOL","CAN_D2_PROTOCOL","COMPASS_DEV_ID","COMPASS_PRIO1_ID","COMPASS_EXTERNAL"})names.emplace_back(name);
    if(hflowRear){for(auto slot:{1,2})for(auto field:{"TYPE","ORIENT","MIN","MAX","ADDR","POS_X","POS_Y","POS_Z","GNDCLR"})names.push_back("RNGFND"+std::to_string(slot)+"_"+field);for(auto name:{"PRX2_TYPE","PRX2_ORIENT","PRX2_YAW_CORR","FLOW_TYPE","FLOW_POS_X","FLOW_POS_Y","FLOW_POS_Z","CAN_P2_DRIVER","CAN_D1_PROTOCOL"})names.emplace_back(name);}
    if(responsiveAvoidance)for(auto name:{"PRX2_TYPE","AVOID_ACCEL_MAX","AVOID_BEHAVE","AVOID_BACKUP_SPD","RNGFND_FILT","EK3_SRC_OPTIONS","GPS1_TYPE","GPS2_TYPE"})names.emplace_back(name);
    if(disarmedLog>=0)names.emplace_back("LOG_DISARMED");
    if(observationOnly||gpsFree||clearUnused||hybrid||gpsBootstrap||hflowRear||disableOriginReuse)for(const auto& [name,value]:bhu::extnav::ObservationGuard::profiles().at("gps_recording"))if(std::find(names.begin(),names.end(),name)==names.end())names.push_back(name);
    Serial serial(uart,baud);Parser parser;std::map<std::string,float> values;double heartbeat=0;bool armed=true;
    auto receive=[&]{auto bytes=serial.read();parser.feed(bytes.data(),bytes.size(),[&](const mavlink_message_t& message){if(message.sysid!=1 || message.compid!=1)return;
        if(message.msgid==MAVLINK_MSG_ID_HEARTBEAT){mavlink_heartbeat_t h{};mavlink_msg_heartbeat_decode(&message,&h);heartbeat=now();armed=h.base_mode&MAV_MODE_FLAG_SAFETY_ARMED;}
        if(message.msgid==MAVLINK_MSG_ID_PARAM_VALUE){mavlink_param_value_t p{};mavlink_msg_param_value_decode(&message,&p);values[std::string(p.param_id,strnlen(p.param_id,16))]=p.param_value;}});};
    auto get=[&](const std::string& name){
        // Drain stale replies before issuing a new named request.
        for(int i=0;i<4;++i){receive();pause_ms(2);}values.erase(name);double begin=now(),next=0;
        while(now()-begin<2){if(now()>=next){serial.write(packet(parameter_request(1,name,1,197)));next=now()+.4;}receive();if(values.count(name))return values.at(name);pause_ms(5);}
        fail("No parameter readback: "+name);return 0.f;};
    Json before=Json::object();
    for(const auto& name:names){try{before[name]=get(name);}catch(...){if(name.rfind("NTF_",0)!=0)throw;before[name]=nullptr;}}
    check(heartbeat && now()-heartbeat<3 && !armed,"Cube must be disarmed with a recent heartbeat");
    if(disableOriginReuse||gpsBootstrap){const auto options=before.at("AHRS_OPTIONS").get<float>();check(std::isfinite(options)&&options>=0&&options<=65535&&std::floor(options)==options,"Invalid AHRS_OPTIONS bitmask");wanted["AHRS_OPTIONS"]=float(uint32_t(options)&~16u);}
    if(hflowRear){
        check(before["RNGFND1_TYPE"]==27,"Expected existing TF03 serial rangefinder in slot1");
        check(before["RNGFND1_ORIENT"]==25||before["RNGFND1_ORIENT"]==4,"Unexpected relocated rangefinder orientation");
        check(before["RNGFND2_TYPE"]==0||before["RNGFND2_TYPE"]==24,"Slot2 is occupied by another rangefinder");
        check(before["PRX1_TYPE"]==2,"Preserve the selected front MAVLink proximity backend");
        check(before["PRX2_TYPE"]==0||before["PRX2_TYPE"]==4,"Proximity slot2 is occupied");
        check(before["PRX2_ORIENT"]==0&&before["PRX2_YAW_CORR"]==0,"Unexpected rear proximity transform");
        check(before["FLOW_TYPE"]==6&&before["CAN_P2_DRIVER"]==1&&before["CAN_D1_PROTOCOL"]==1,"Expected working H-Flow DroneCAN setup");
        for(auto axis:{"X","Y","Z"}){auto value=before.at("FLOW_POS_"+std::string(axis)).get<float>();check(std::isfinite(value)&&std::abs(value)<=5,"Invalid existing flow position offset");wanted["RNGFND2_POS_"+std::string(axis)]=value;}
    }
    if(gpsBootstrap){
        for(auto [name,value]:bhu::extnav::ObservationGuard::profiles().at("vio_optflow_compass"))if(name!="GPS1_TYPE")check(std::abs(before.at(name).get<float>()-value)<1e-5,"Unexpected bootstrap configuration: "+name);
        check(before["GPS1_TYPE"]==0||before["GPS1_TYPE"]==9,"Unexpected GPS1 driver");
        check(before["CAN_P1_DRIVER"]==1&&before["CAN_D1_PROTOCOL"]==1&&before["CAN_P1_BITRATE"]==1000000,"Expected existing DroneCAN bus1 configuration");
        check(before["COMPASS_DEV_ID"]==1313809&&before["COMPASS_PRIO1_ID"]==1313809&&before["COMPASS_EXTERNAL"]==0&&before["COMPASS_USE"]==1&&before["COMPASS_USE2"]==0&&before["COMPASS_USE3"]==0,"Expected selected internal compass");
    }
    if(responsiveAvoidance){
        check(!reboot,"Responsive avoidance tuning requires no reboot");
        check(before["PRX1_TYPE"]==2&&before["PRX2_TYPE"]==4,"Expected existing front MAVLink and rear RangeFinder proximity");
        check(before["AVOID_BEHAVE"]==1&&before["AVOID_MARGIN"]==2,"Expected selected Stop behaviour and 2m margin");
    }
    if(gpsRecording){
        for(auto [name,value]:std::map<std::string,float>{{"AHRS_EKF_TYPE",3},{"EK3_ENABLE",1},{"EK3_SRC1_POSXY",3},{"EK3_SRC1_POSZ",1},{"EK3_SRC1_YAW",1},{"EK3_SRC_OPTIONS",0}})check(before.at(name)==value,"Unexpected GPS recording configuration: "+name);
        check(before.at("GPS1_TYPE").get<float>()!=0,"GPS recording requires an enabled GPS");
    }
    if(clearUnused)for(auto [name,value]:bhu::extnav::ObservationGuard::profiles().at("gps_free_vio"))check(std::abs(before.at(name).get<float>()-value)<1e-5,"Unexpected GPS-free configuration: "+name);
    if(enableProximity)check(before.at("OA_TYPE")==1,"BendyRuler is not selected; review configuration before enabling proximity");
    if(hybridPrimary)for(auto [name,value]:bhu::extnav::ObservationGuard::profiles().at("vio_optflow_compass"))if(!wanted.count(name))check(std::abs(before.at(name).get<float>()-value)<1e-5,"Unexpected hybrid configuration: "+name);
    if(observationOnly)for(auto [name,value]:bhu::extnav::ObservationGuard::profiles().at("gps_observation"))if(name!="VISO_TYPE")check(std::abs(before.at(name).get<float>()-value)<1e-5,"Unexpected GPS observation configuration: "+name);
    fs::create_directories(output);save_json(output/"before.json",before);Json requested=Json::object();for(auto [name,value]:wanted)requested[name]=value;save_json(output/"requested.json",requested);
    {std::ofstream restore(output/"restore.param");restore<<std::setprecision(9);for(auto [name,value]:wanted)restore<<name<<' '<<before.at(name).get<float>()<<'\n';check(bool(restore),"Cannot save restore parameters");}
    Json changes=Json::array();save_json(output/"changes.json",changes);
    if(apply)for(auto [name,value]:wanted){if(std::abs(before.at(name).get<float>()-value)<1e-5)continue;
        receive();check(heartbeat && now()-heartbeat<3 && !armed,"Cube must remain disarmed");char field[16]{};std::memcpy(field,name.data(),name.size());mavlink_message_t message{};
        mavlink_msg_param_set_pack(1,197,&message,1,1,field,value,MAV_PARAM_TYPE_REAL32);serial.write(packet(message));pause_ms(100);
        float observed=get(name);changes.push_back({{"name",name},{"requested",value},{"readback",observed}});save_json(output/"changes.json",changes);
        check(std::abs(observed-value)<1e-5,"Parameter write/readback mismatch: "+name);
    }
    Json after=Json::object();for(const auto& name:names){if(before[name].is_null())continue;after[name]=get(name);}
    save_json(output/"after.json",after);
    for(const auto& name:names)if(!wanted.count(name) && !before[name].is_null())check(after[name]==before[name],"Unrelated parameter changed: "+name);
    save_json(output/"summary.json",{{"responsive_avoidance_only",responsiveAvoidance},{"applied",apply},{"quality_only",qualityOnly},{"gps_shadow_only",gpsShadowOnly},{"gps_recording_only",gpsRecording},{"gps_free_only",gpsFree},{"vio_optflow_compass_only",hybrid},{"primary_sources_only",hybridPrimary},{"disable_origin_reuse_only",disableOriginReuse},{"gps_bootstrap_only",gpsBootstrap},{"hflow_down_rear_only",hflowRear},{"clear_unused_sources_only",clearUnused},{"disable_proximity_only",disableProximity},{"enable_proximity_only",enableProximity},{"observation_only",observationOnly},{"disarmed_log",disarmedLog},{"mount_only",mountOnly},{"reboot_requested",reboot},{"writes",changes.size()},{"source_selected",false},{"fusion_validated",false},{"uart_bad_packets",parser.bad}});
    if(reboot){receive();check(heartbeat && now()-heartbeat<3 && !armed,"Cube must remain disarmed before reboot");mavlink_message_t message{};mavlink_msg_command_long_pack(1,197,&message,1,1,MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,0,1,0,0,0,0,0,0);serial.write(packet(message));pause_ms(500);}
    std::cout<<"Configuration snapshot: "<<output<<"; applied="<<apply<<"; writes="<<changes.size()<<"; source selection unchanged\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
