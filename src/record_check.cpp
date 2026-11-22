// Offline dataset integrity/timing check. This never runs SLAM or opens hardware.
#include <json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

namespace fs=std::filesystem;using Json=nlohmann::json;
static std::vector<std::string> cells(const std::string& line){std::stringstream s(line);std::string v;std::vector<std::string> out;while(std::getline(s,v,','))out.push_back(v);return out;}
static void need(bool yes,const std::string& reason){if(!yes)throw std::runtime_error(reason);}
struct Timing{size_t n=0;double first=0,last=0,gap=0;void add(double t){need(std::isfinite(t) && t>0 && (!n || t>last),"Nonmonotonic/nonfinite timestamp");if(!n)first=t;else gap=std::max(gap,t-last);last=t;++n;}Json json()const{return {{"samples",n},{"first_s",first},{"last_s",last},{"rate_hz",n>1?(n-1)/(last-first):0},{"max_gap_ms",gap*1000}};}};
static Timing imu(const fs::path& file,double start,Json& stats,bool accel){
    std::ifstream f(file);need(bool(f),"Missing "+file.string());std::string line;std::getline(f,line);Timing timing;double sum=0,square=0;size_t count=0;
    while(std::getline(f,line)){auto c=cells(line);need(c.size()==4,"Malformed IMU CSV");double t=std::stod(c[0]);timing.add(t);double norm2=0;for(int i=1;i<4;++i){double v=std::stod(c[i]);need(std::isfinite(v),"Nonfinite IMU value");norm2+=v*v;}
        if(t>=start && t<start+10){double norm=std::sqrt(norm2);sum+=norm;square+=norm2;++count;}}
    need(timing.n>=2,"Insufficient IMU samples");std::string key=accel?"first_10s_acceleration":"first_10s_angular_rate";
    stats[key]={{"samples",count},{"norm_mean",count?sum/count:0},{"norm_rms",count?std::sqrt(square/count):0},{"units",accel?"m/s^2":"rad/s"},{"stationarity_assumed_not_verified",true}};return timing;
}
static Timing native(const fs::path& file){
    std::ifstream f(file);need(bool(f),"Missing native IMU CSV");std::string line;std::getline(f,line);Timing timing;
    while(std::getline(f,line)){auto c=cells(line);need(c.size()==7,"Malformed native IMU CSV");timing.add(std::stod(c[0])*1e-6);for(int i=1;i<7;++i)need(std::isfinite(std::stod(c[i])),"Nonfinite native IMU value");}return timing;
}
int main(int argc,char** argv)try{
    need(argc==2,"Usage: gemini_record_check RECORDING_DIRECTORY");fs::path root=argv[1];std::error_code error;fs::remove(root/"recording-check.json",error);need(!error,"Cannot replace prior recording check");std::ifstream metaFile(root/"recording.json");Json meta=Json::object();if(metaFile)metaFile>>meta;
    std::ifstream f(root/"frames.csv");need(bool(f),"Missing frames.csv");std::string line;std::getline(f,line);Timing frames;double skew=0;uint64_t leftPrevious=0,rightPrevious=0,gaps=0;std::string firstLeft,lastLeft;bool diagnostic=meta.value("diagnostic",false);
    while(std::getline(f,line)){auto c=cells(line);need(c.size()==7,"Malformed frame CSV");frames.add(std::stod(c[0]));
        double delta=std::abs(std::stod(c[3])-std::stod(c[4]));skew=std::max(skew,delta);need(delta<=500,"Stereo skew exceeds 500 us");
        auto left=std::stoull(c[5]),right=std::stoull(c[6]);if(frames.n>1){need(left>leftPrevious && right>rightPrevious,"Nonmonotonic frame indices");gaps+=left-leftPrevious-1+right-rightPrevious-1;}leftPrevious=left;rightPrevious=right;
        for(int side=1;side<=2;++side){fs::path relative=c[side];need(!relative.is_absolute() && relative.string().find("..") == std::string::npos,"Invalid image path");need(fs::exists(root/relative) && fs::file_size(root/relative)>0,"Missing stereo image");
            if(diagnostic){auto raw=root/(side==1?"native_left":"native_right")/relative.filename();need(fs::exists(raw) && fs::file_size(raw)>0,"Missing native image");}}
        if(firstLeft.empty())firstLeft=c[1];lastLeft=c[1];
    }
    need(frames.n>=2,"Insufficient recorded stereo pairs");
    for(const auto& name:{firstLeft,lastLeft}){auto image=cv::imread((root/name).string(),cv::IMREAD_GRAYSCALE);need(!image.empty(),"Cannot decode recorded image");if(meta.contains("width"))need(image.cols==meta["width"].get<int>() && image.rows==meta["height"].get<int>(),"Image dimensions differ from metadata");}
    Json result={{"schema",1},{"stereo",frames.json()},{"stereo_max_skew_us",skew},{"combined_left_right_index_gaps",gaps},{"diagnostic",diagnostic},{"vio_validated",false}};
    double start=meta.value("start_device_s",frames.first);auto acc=imu(root/"accel.csv",start,result,true),gyro=imu(root/"gyro.csv",start,result,false);
    result["accel"]=acc.json();result["gyro"]=gyro.json();result["imu_brackets_all_images"]=acc.first<frames.first && gyro.first<frames.first && acc.last>frames.last && gyro.last>frames.last;
    if(diagnostic){auto a=native(root/"accel_native.csv"),g=native(root/"gyro_native.csv");need(a.n==acc.n && g.n==gyro.n,"Native/replay IMU sample counts differ");need(std::abs(a.first-acc.first)<1e-8 && std::abs(a.last-acc.last)<1e-8 && std::abs(g.first-gyro.first)<1e-8 && std::abs(g.last-gyro.last)<1e-8,"Native/replay IMU timestamps differ");result["accel_native"]=a.json();result["gyro_native"]=g.json();}
    result["warnings"]=Json::array();
    if(acc.gap>.02 || gyro.gap>.02)result["warnings"].push_back("IMU gap exceeds replay synchronizer's 20 ms limit");
    if(gaps)result["warnings"].push_back("Frame-index gaps present; inspect capture timing");
    if(!result["imu_brackets_all_images"].get<bool>())result["warnings"].push_back("Replay may trim images without bracketing IMU");
    if(meta.contains("complete") && !meta["complete"].get<bool>())result["warnings"].push_back("Capture marked incomplete");
    if(diagnostic && meta.value("mode",std::string{})!="boot_timed" && !meta.value("protocol_duration_complete",false))result["warnings"].push_back("Full 10/60/10-second protocol was not completed");
    if(meta.value("mode",std::string{})=="boot_timed" && !meta.value("timed_duration_complete",false))result["warnings"].push_back("Timed recording was interrupted before its requested duration");
    result["integrity_pass"]=true;
    std::ofstream output(root/"recording-check.json");output<<result.dump(2)<<'\n';need(bool(output),"Cannot write check report");std::cout<<result.dump(2)<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
