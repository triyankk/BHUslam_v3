#pragma once
#include <json.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdint>

namespace bhu {
// Flight control stays independent. This only gates observation files.
class FlightRecording {
    std::string path_,last_;
    uint64_t changed_=0,stopAt_=0;
    bool observed_=false,active_=false,initial_=false,disarmed_=false;
public:
    enum Event {None,Start,Stop};
    explicit FlightRecording(std::string path):path_(std::move(path)){}
    Event observe(const std::string& text,uint64_t now){
        if(!initial_){initial_=true;last_=text;return None;}
        if(text!=last_){last_=text;changed_=now;observed_=true;}
        bool valid=false,armed=false;
        try{
            auto s=nlohmann::json::parse(text);const auto& c=s.at("cube");double age=c.at("heartbeat_age_ms").get<double>();
            valid=observed_&&now-changed_<=1500000&&s.at("running").get<bool>()&&c.at("connected").get<bool>()&&age>=0&&age<=1500;
            armed=c.at("armed").get<bool>();
        }catch(...){}
        // Link loss is not disarm. Keep the current recording through outages.
        disarmed_=valid&&!armed;
        if(!valid){stopAt_=0;return None;}
        if(armed){stopAt_=0;if(!active_){active_=true;return Start;}}
        else if(active_){if(!stopAt_)stopAt_=now+3000000;if(now>=stopAt_){active_=false;stopAt_=0;return Stop;}}
        return None;
    }
    Event poll(uint64_t now){std::ifstream f(path_);std::ostringstream s;s<<f.rdbuf();return observe(s.str(),now);}
    bool active()const{return active_;}
    bool fresh_disarmed(uint64_t now)const{return disarmed_&&now>=changed_&&now-changed_<=1500000;}
};
}
