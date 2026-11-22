// Read-only MAVLink inspection. Does not send PARAM_SET, mode or arm commands.
#include "native.hpp"
#include <iostream>
#include <set>

int main(int argc,char** argv) try {
    using namespace tof;
    std::string uart="/dev/ttyTHS1";int baud=57600,system=1;double duration=12;
    fs::path output;bool proximityAudit=false;
    for(int i=1;i<argc;++i) {
        std::string key=argv[i];
        if(key=="--proximity-audit"){proximityAudit=true;continue;}
        if(key=="--help") {
            std::cout<<"cube_extnav_inspect --uart /dev/ttyTHS1 --baud 57600 --system-id 1 --duration 12 --output NEW.json [--proximity-audit]\n"
                     <<"Requires exclusive UART access; requests firmware and parameters only.\n";return 0;
        }
        check(i+1<argc,"Missing value for "+key);std::string value=argv[++i];
        if(key=="--uart")uart=value;
        else if(key=="--baud")baud=std::stoi(value);
        else if(key=="--system-id")system=std::stoi(value);
        else if(key=="--duration")duration=std::stod(value);
        else if(key=="--output")output=value;
        else fail("Unknown option "+key);
    }
    check(system>=1 && system<=255 && std::isfinite(duration) && duration>=2 && duration<=120,"Invalid inspector limits");
    check(output.empty() || !fs::exists(output),"Output already exists");
    std::vector<std::string> names={"MAV_SYSID","SYSID_THISMAV","SERIAL1_PROTOCOL","SERIAL1_BAUD",
        "SERIAL2_PROTOCOL","SERIAL2_BAUD","SERIAL2_OPTIONS","BRD_SER2_RTSCTS",
        "BRD_SER1_RTSCTS","AHRS_OPTIONS","AHRS_ORIGIN_LAT","AHRS_ORIGIN_LON","AHRS_ORIGIN_ALT","COMPASS_ENABLE","COMPASS_USE","COMPASS_USE2","COMPASS_USE3","COMPASS_DEV_ID","COMPASS_DEV_ID2","COMPASS_DEV_ID3","COMPASS_PRIO1_ID","COMPASS_PRIO2_ID","COMPASS_PRIO3_ID","COMPASS_EXTERNAL","COMPASS_EXTERN2","COMPASS_EXTERN3","COMPASS_OFS_X","COMPASS_OFS_Y","COMPASS_OFS_Z","COMPASS_OFS2_X","COMPASS_OFS2_Y","COMPASS_OFS2_Z","COMPASS_OFS3_X","COMPASS_OFS3_Y","COMPASS_OFS3_Z","AHRS_EKF_TYPE","EK3_ENABLE","EK3_SRC1_POSXY","EK3_SRC1_VELXY",
        "EK3_SRC1_POSZ","EK3_SRC1_VELZ","EK3_SRC1_YAW","EK3_SRC2_POSXY","EK3_SRC2_VELXY",
        "EK3_SRC2_POSZ","EK3_SRC2_VELZ","EK3_SRC2_YAW","EK3_SRC3_POSXY","EK3_SRC3_VELXY",
        "EK3_SRC3_POSZ","EK3_SRC3_VELZ","EK3_SRC3_YAW","EK3_SRC_OPTIONS","VISO_TYPE","VISO_POS_X",
        "VISO_POS_Y","VISO_POS_Z","VISO_ORIENT","VISO_DELAY_MS","VISO_QUAL_MIN","VISO_POS_M_NSE",
        "VISO_YAW_M_NSE","VISO_VEL_M_NSE","GPS1_TYPE","RNGFND1_TYPE","RNGFND1_ORIENT",
        "RNGFND2_TYPE","RNGFND2_ORIENT","PRX1_TYPE","AVOID_ENABLE","AVOID_MARGIN","AVOID_ALT_MIN",
        "OA_BR_TYPE","OA_DB_ALT_MIN","OA_DB_EXPIRE","PRX_IGN_GND","AVOID_BEHAVE","OA_TYPE","OA_BR_LOOKAHEAD","OA_MARGIN_MAX","OA_DB_SIZE","RC7_OPTION","RC8_OPTION","RC9_OPTION",
        "NTF_BUZZ_ENABLE","NTF_BUZZ_TYPES","NTF_BUZZ_VOLUME","ARMING_CHECK","ARMING_SKIPCHK","LOG_BACKEND_TYPE",
        "LOG_DISARMED","LOG_BITMASK","FLOW_TYPE","GPS2_TYPE", "SCR_ENABLE",
        "RC1_OPTION","RC2_OPTION","RC3_OPTION","RC4_OPTION","RC5_OPTION","RC6_OPTION",
        "RC10_OPTION","RC11_OPTION","RC12_OPTION","RC13_OPTION","RC14_OPTION","RC15_OPTION","RC16_OPTION"};
    for(auto slot:{1,2,3})for(auto field:{"TYPE","ORIENT","MIN","MAX","ADDR","POS_X","POS_Y","POS_Z","GNDCLR"}){auto name="RNGFND"+std::to_string(slot)+"_"+field;if(std::find(names.begin(),names.end(),name)==names.end())names.push_back(name);}
    for(auto name:{"PRX2_TYPE","PRX3_TYPE","CAN_P1_DRIVER","CAN_P2_DRIVER","CAN_D1_PROTOCOL","CAN_D2_PROTOCOL","CAN_D1_UC_NODE","CAN_D2_UC_NODE","FLOW_POS_X","FLOW_POS_Y","FLOW_POS_Z","FLOW_ORIENT_YAW","SERIAL3_PROTOCOL","SERIAL3_BAUD","SERIAL4_PROTOCOL","SERIAL4_BAUD"})names.emplace_back(name);
    Serial port(uart,baud);Parser parser;
    Json report={{"uart",uart},{"baud",baud},{"system_id",system},{"parameter_writes",0},
                 {"parameters",Json::object()},{"heartbeat_count",0}};
    double start=now(),next=start,nextProximity=start;size_t request=0;
    while(now()-start<duration) {
        if(proximityAudit&&now()>=nextProximity){mavlink_message_t m{};mavlink_msg_command_long_pack(254,190,&m,uint8_t(system),1,MAV_CMD_REQUEST_MESSAGE,0,MAVLINK_MSG_ID_DISTANCE_SENSOR,0,0,0,0,0,0);port.write(packet(m));mavlink_msg_command_long_pack(254,190,&m,uint8_t(system),1,MAV_CMD_REQUEST_MESSAGE,0,MAVLINK_MSG_ID_OPTICAL_FLOW,0,0,0,0,0,0);port.write(packet(m));nextProximity=now()+.2;}

        if(now()>=next) {
            auto name=names[request++%names.size()];
            port.write(packet(parameter_request(uint8_t(system),name,254,190)));
            if(request%names.size()==1) {
                mavlink_message_t message{};
                mavlink_msg_command_long_pack(254,190,&message,uint8_t(system),1,MAV_CMD_REQUEST_MESSAGE,0,
                                              MAVLINK_MSG_ID_AUTOPILOT_VERSION,0,0,0,0,0,0);
                port.write(packet(message));
                for(auto id:{MAVLINK_MSG_ID_SYS_STATUS,MAVLINK_MSG_ID_EKF_STATUS_REPORT,MAVLINK_MSG_ID_GPS_RAW_INT,MAVLINK_MSG_ID_HOME_POSITION,MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN}){
                    mavlink_msg_command_long_pack(254,190,&message,uint8_t(system),1,MAV_CMD_REQUEST_MESSAGE,0,id,0,0,0,0,0,0);port.write(packet(message));
                }
            }
            next=now()+.08;
        }
        auto bytes=port.read();
        parser.feed(bytes.data(),bytes.size(),[&](const mavlink_message_t& message) {
            if(proximityAudit&&message.sysid==system&&message.msgid==MAVLINK_MSG_ID_DISTANCE_SENSOR){mavlink_distance_sensor_t d{};mavlink_msg_distance_sensor_decode(&message,&d);report["range_packets"].push_back({{"host_monotonic_s",now()},{"component",message.compid},{"id",d.id},{"orientation",d.orientation},{"distance_cm",unsigned(d.current_distance)},{"min_cm",unsigned(d.min_distance)},{"max_cm",unsigned(d.max_distance)},{"signal_quality",unsigned(d.signal_quality)}});if(d.orientation==0)report["front_packets"].push_back({{"host_monotonic_s",now()},{"component",message.compid},{"id",d.id},{"distance_cm",unsigned(d.current_distance)},{"boot_ms",uint32_t(d.time_boot_ms)}});}
            if(message.sysid!=system || message.compid!=1)return;
            if(message.msgid==MAVLINK_MSG_ID_PARAM_VALUE) {
                mavlink_param_value_t p{};mavlink_msg_param_value_decode(&message,&p);
                std::string name(p.param_id,strnlen(p.param_id,16));
                if(std::find(names.begin(),names.end(),name)!=names.end() && std::isfinite(p.param_value))
                    report["parameters"][name]=float(p.param_value);
            } else if(message.msgid==MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t h{};mavlink_msg_heartbeat_decode(&message,&h);
                report["heartbeat_count"]=report["heartbeat_count"].get<unsigned>()+1;
                report["armed"]=bool(h.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
                report["autopilot"]=unsigned(h.autopilot);
            } else if(message.msgid==MAVLINK_MSG_ID_EKF_STATUS_REPORT){mavlink_ekf_status_report_t s{};mavlink_msg_ekf_status_report_decode(&message,&s);report["ekf_flags"]=unsigned(s.flags);
            } else if(message.msgid==MAVLINK_MSG_ID_OPTICAL_FLOW){mavlink_optical_flow_t f{};mavlink_msg_optical_flow_decode(&message,&f);report["flow_packets"].push_back({{"host_monotonic_s",now()},{"quality",unsigned(f.quality)},{"ground_distance_m",float(f.ground_distance)},{"velocity_x_mps",float(f.flow_comp_m_x)},{"velocity_y_mps",float(f.flow_comp_m_y)}});
            } else if(message.msgid==MAVLINK_MSG_ID_GPS_RAW_INT){mavlink_gps_raw_int_t g{};mavlink_msg_gps_raw_int_decode(&message,&g);report["gps"]={{"fix_type",unsigned(g.fix_type)},{"satellites",unsigned(g.satellites_visible)},{"eph_cm",unsigned(g.eph)}};
            } else if(message.msgid==MAVLINK_MSG_ID_HOME_POSITION){mavlink_home_position_t h{};mavlink_msg_home_position_decode(&message,&h);report["home"]={{"lat_e7",int(h.latitude)},{"lon_e7",int(h.longitude)},{"alt_msl_mm",int(h.altitude)}};
            } else if(message.msgid==MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN){mavlink_gps_global_origin_t h{};mavlink_msg_gps_global_origin_decode(&message,&h);report["origin"]={{"lat_e7",int(h.latitude)},{"lon_e7",int(h.longitude)},{"alt_msl_mm",int(h.altitude)}};
            } else if(message.msgid==MAVLINK_MSG_ID_SYS_STATUS){mavlink_sys_status_t s{};mavlink_msg_sys_status_decode(&message,&s);report["sensors"]={{"present",unsigned(s.onboard_control_sensors_present)},{"enabled",unsigned(s.onboard_control_sensors_enabled)},{"health",unsigned(s.onboard_control_sensors_health)}};
            } else if(message.msgid==MAVLINK_MSG_ID_STATUSTEXT){mavlink_statustext_t s{};mavlink_msg_statustext_decode(&message,&s);report["status_texts"].push_back(std::string(s.text,strnlen(s.text,50)));
            } else if(message.msgid==MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
                mavlink_autopilot_version_t v{};mavlink_msg_autopilot_version_decode(&message,&v);
                uint32_t version=v.flight_sw_version;
                report["flight_sw_version"]=version;
                report["firmware"]=std::to_string(version>>24)+"."+std::to_string((version>>16)&255)+"."+std::to_string((version>>8)&255);
                report["firmware_hash"]=std::string(reinterpret_cast<const char*>(v.flight_custom_version),8);
            }
        });
        pause_ms(5);
    }
    report["uart_bad_packets"]=parser.bad;
    report["missing_parameters"]=Json::array();
    for(const auto& name:names)if(!report["parameters"].contains(name))report["missing_parameters"].push_back(name);
    report["link_verified"]=report["heartbeat_count"].get<unsigned>()>0 && !report["parameters"].empty();
    if(!output.empty()) {
        if(!output.parent_path().empty())fs::create_directories(output.parent_path());
        save_json(output,report);
    }
    std::cout<<report.dump(2)<<'\n';
    return report["link_verified"].get<bool>()?0:1;
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
