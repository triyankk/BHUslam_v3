#pragma once
#include <map>
#include <string>
#include <cstdint>
#include <cmath>
namespace bhu::extnav {
// Configuration is not proof of active EKF selection or successful fusion.
// Exact profiles share one packet path; this guard never changes Cube settings.
struct ObservationGuard {
    using Parameters=std::map<std::string,float>;
    static const std::map<std::string,Parameters>& profiles() {
        static const auto result=[] {
            Parameters gps{{"AHRS_EKF_TYPE",3},{"EK3_ENABLE",1},
                {"EK3_SRC1_POSXY",3},{"EK3_SRC1_VELXY",3},{"EK3_SRC1_POSZ",1},
                {"EK3_SRC1_VELZ",3},{"EK3_SRC1_YAW",1},{"EK3_SRC_OPTIONS",0},
                {"VISO_TYPE",3},{"VISO_QUAL_MIN",40},{"VISO_ORIENT",0},
                {"VISO_POS_X",.13f},{"VISO_POS_Y",0},{"VISO_POS_Z",.05f},
                {"GPS1_TYPE",9},{"GPS2_TYPE",0},{"COMPASS_ENABLE",1}};
            auto recording=gps;recording["VISO_TYPE"]=0;
            auto vio=gps;
            for(auto name:{"EK3_SRC1_POSXY","EK3_SRC1_VELXY","EK3_SRC1_VELZ","EK3_SRC1_YAW"})vio[name]=6;
            vio["GPS1_TYPE"]=0;vio["COMPASS_ENABLE"]=0;
            auto hybrid=vio;hybrid["EK3_SRC1_VELXY"]=5;hybrid["EK3_SRC1_YAW"]=1;hybrid["COMPASS_ENABLE"]=1;
            auto bootstrap=hybrid;bootstrap["GPS1_TYPE"]=9;
            return std::map<std::string,Parameters>{{"vio_optflow_compass_gps_bootstrap",bootstrap},{"vio_optflow_compass",hybrid},{"gps_observation",gps},
                {"gps_recording",recording},{"gps_free_vio",vio}};
        }();
        return result;
    }
    // All profiles have the same audited keys; values here are the GPS example.
    const Parameters expected=profiles().at("gps_observation");
    std::map<std::string,std::pair<float,uint64_t>> values;
    uint64_t heartbeat=0;
    bool armed=true,started_disarmed=false,nonprimary_observed=false;
    std::string pinned_profile;
    void observe_heartbeat(bool is_armed,uint64_t now) {
        if(heartbeat && (now<heartbeat || now-heartbeat>1500000)) {
            started_disarmed=false;pinned_profile.clear();
        }
        heartbeat=now;armed=is_armed;if(!armed)started_disarmed=true;
    }
    void parameter(const std::string& name,float value,uint64_t now) {
        if(expected.count(name))values[name]={value,now};
    }
    std::string profile(uint64_t now) const {
        for(const auto& [name,unused]:expected) {
            auto it=values.find(name);
            if(it==values.end()||!it->second.second||now<it->second.second
                ||now-it->second.second>5000000||!std::isfinite(it->second.first))return "";
        }
        for(const auto& [profile,parameters]:profiles()) {
            bool match=true;
            for(const auto& [name,wanted]:parameters)
                if(std::abs(values.at(name).first-wanted)>1e-5f){match=false;break;}
            if(match)return profile;
        }
        return "";
    }
    std::string reason(uint64_t now) {
        if(!heartbeat||now<heartbeat||now-heartbeat>1500000)return "Cube heartbeat stale";
        if(!started_disarmed)return "No disarmed startup heartbeat";
        if(nonprimary_observed)return "Non-primary source selection observed";
        for(const auto& [name,unused]:expected) {
            auto it=values.find(name);
            if(it==values.end()||!it->second.second||now<it->second.second||now-it->second.second>5000000)return "Missing/stale "+name;
            if(!std::isfinite(it->second.first))return "Nonfinite "+name;
        }
        auto selected=profile(now);
        if(selected.empty())return "Unsupported Cube navigation profile";
        if(!armed)pinned_profile=selected;
        else if(pinned_profile!=selected)return "Navigation profile not qualified disarmed";
        if(selected=="gps_recording")return "VIO backend disabled (GPS recording)";
        return "";
    }
};
}
