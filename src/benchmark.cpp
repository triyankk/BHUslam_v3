#include "bhu/benchmark.hpp"
#include <json.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <netinet/in.h>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <sys/resource.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifndef BHU_TEST_PAGE
#define BHU_TEST_PAGE "web/vio-test.html"
#endif
namespace bhu::bench {
using Json=nlohmann::json;namespace fs=std::filesystem;
static uint64_t us(){return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static std::string read(const fs::path& path){std::ifstream f(path);return {std::istreambuf_iterator<char>(f),{}};}
static double number(const fs::path& path){std::ifstream f(path);double v;return f>>v?v:std::numeric_limits<double>::quiet_NaN();}
static void save(const fs::path& path,const Json& j){std::ofstream f(path);f<<j.dump(2)<<'\n';if(!f)throw std::runtime_error("Cannot write "+path.string());}
static std::vector<std::string> split(const std::string& line){std::stringstream s(line);std::string cell;std::vector<std::string> v;while(std::getline(s,cell,','))v.push_back(cell);return v;}
static Json distribution(std::vector<double> v){
    v.erase(std::remove_if(v.begin(),v.end(),[](double n){return !std::isfinite(n);}),v.end());std::sort(v.begin(),v.end());
    auto q=[&](double p){if(v.empty())return std::numeric_limits<double>::quiet_NaN();double n=(v.size()-1)*p;size_t i=size_t(n);return v[i]+(n-i)*(v[std::min(i+1,v.size()-1)]-v[i]);};
    return {{"p50",q(.5)},{"p95",q(.95)},{"p99",q(.99)}};
}
static void plot_png(const fs::path& directory,const Json& points,const Json& metrics,const std::string& profile){
    cv::Mat image(900,1400,CV_8UC3,cv::Scalar(31,22,16));
    auto text=[&](const std::string& value,int x,int y,double scale=.7,cv::Scalar color=cv::Scalar(225,220,210)){
        cv::putText(image,value,{x,y},cv::FONT_HERSHEY_SIMPLEX,scale,color,1,cv::LINE_AA);
    };
    text("Gemini 336L | VIO test | "+profile,35,45,1.0);
    text(std::to_string(metrics["tracked"].get<size_t>())+" / "+std::to_string(metrics["frames"].get<size_t>())+" frames tracked; initialized frames "+std::to_string(metrics["initialized_frames"].get<size_t>()),35,80);
    text("Green: initialized VIO. Amber: visual tracking only. Gaps separate segments.",35,115);
    if(points.empty())text("No tracked poses in this test.",100,350,1.0);
    else {
        std::array<double,3> minimum{1e100,1e100,1e100},maximum{-1e100,-1e100,-1e100},center{};
        for(const auto& p:points)for(int i=0;i<3;++i){double v=p[i+1];minimum[i]=std::min(minimum[i],v);maximum[i]=std::max(maximum[i],v);}
        double range=.15;for(int i=0;i<3;++i){center[i]=(minimum[i]+maximum[i])*.5;range=std::max(range,maximum[i]-minimum[i]);}
        const double yaw=-.6,pitch=.65,scale=420/range;
        auto project=[&](double x,double y,double z){x-=center[0];y-=center[1];z-=center[2];double a=std::cos(yaw)*x-std::sin(yaw)*y,b=std::sin(yaw)*x+std::cos(yaw)*y;return cv::Point(int(700+a*scale),int(465-(std::cos(pitch)*z-std::sin(pitch)*b)*scale));};
        for(int axis=0;axis<3;++axis){auto end=center;end[axis]+=range*.3;auto p=project(end[0],end[1],end[2]);cv::line(image,{700,465},p,cv::Scalar(130,120,110),1,cv::LINE_AA);text(std::string(1,"XYZ"[axis])+" (m)",p.x+5,p.y);}
        for(size_t i=1;i<points.size();++i){const auto& a=points[i-1];const auto& b=points[i];if(a[5]!=b[5])continue;cv::line(image,project(a[1],a[2],a[3]),project(b[1],b[2],b[3]),a[4].get<bool>()&&b[4].get<bool>()?cv::Scalar(182,223,66):cv::Scalar(97,183,239),2,cv::LINE_AA);}
        auto a=points.front(),b=points.back();cv::circle(image,project(a[1],a[2],a[3]),6,cv::Scalar(255,255,255),2,cv::LINE_AA);auto end=project(b[1],b[2],b[3]);cv::rectangle(image,end-cv::Point(5,5),end+cv::Point(5,5),cv::Scalar(255,255,255),cv::FILLED);
        std::ostringstream extent;extent<<std::fixed<<std::setprecision(2)<<"Extent (m): X "<<maximum[0]-minimum[0]<<"  Y "<<maximum[1]-minimum[1]<<"  Z "<<maximum[2]-minimum[2];text(extent.str(),35,785);
    }
    text("Online estimates before final loop optimization. Coordinates can change at resets.",35,830,.6);
    text("No ground truth: this is a trajectory estimate, not a measured accuracy result.",35,865,.6);
    if(!cv::imwrite((directory/"trajectory.png").string(),image))throw std::runtime_error("Cannot write trajectory PNG");
}
static std::string html(const Json& data){
    std::string page=read(BHU_TEST_PAGE),encoded=data.dump();
    for(size_t pos=0;(pos=encoded.find('<',pos))!=std::string::npos;pos+=6)encoded.replace(pos,1,"\\u003c");
    auto marker=page.find("/*VIO_REPORT*/null");
    if(marker==std::string::npos)throw std::runtime_error("Missing trajectory report template");
    page.replace(marker,std::string("/*VIO_REPORT*/null").size(),encoded);return page;
}
std::string power_profile(){
    FILE* stream=::popen("/usr/sbin/nvpmodel -q 2>&1","r");
    if(!stream)return "UNKNOWN";
    std::string text;char buffer[256];while(std::fgets(buffer,sizeof(buffer),stream))text+=buffer;
    ::pclose(stream);auto pos=text.find("NV Power Mode:");if(pos==std::string::npos)return "UNKNOWN";
    auto value=text.substr(pos+14);value=value.substr(0,value.find('\n'));
    auto first=value.find_first_not_of(" \t\r");if(first==std::string::npos)return "UNKNOWN";
    return value.substr(first,value.find_last_not_of(" \t\r")-first+1);
}

struct Benchmark::Impl {
    fs::path output,power_directory;
    mutable std::mutex mutex;Json meta,points=Json::array();
    std::string phase="STARTING";
    uint64_t began=0,ended=0,processed=0,tracked=0,inertial=0;
    std::vector<double> latencies;
    double cpu_previous=0,time_previous=0;
    std::ofstream frames,events,resources;
    std::atomic<bool> finish{false};std::thread worker;int socket=-1;
    Impl(const fs::path& directory,int port,const std::string& required):output(directory) {
        auto profile=power_profile();
        if(profile!=required)throw std::runtime_error("Power profile is "+profile+"; expected "+required+". Select 15W explicitly, or request a named comparison profile.");
        fs::create_directories(output);
        meta={{"schema",1},{"power_profile_start",profile},{"required_power_profile",required},{"phase",phase},
              {"trigger","test_vio"},{"qr_rate_hz",2},{"navigation_transmission",false},{"buzzer_enabled",false},
              {"loop_closure_enabled",true},{"loop_event_instrumentation",true},{"power_domain","VDD_IN module input; not total battery consumption"},
              {"cpu_governor",read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")},
              {"cpu_max_khz",number("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq")},
              {"cpu_min_khz",number("/sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq")}};
        frames.open(output/"frames.csv");events.open(output/"events.jsonl");resources.open(output/"resources.csv");
        if(!frames || !events || !resources)throw std::runtime_error("Cannot open benchmark logs");
        frames<<"device_s,elapsed_s,state,imu_initialized,quality,tracking_ms,age_ms,qr_ms,wall_ms,segment,frame_index,x,y,z,qw,qx,qy,qz\n"<<std::setprecision(15);
        resources<<"elapsed_s,process_cpu_pct,rss_mb,peak_rss_mb,input_w,cpu_khz,temperature_max_c\n";
        std::error_code error;
        for(const auto& directory:fs::directory_iterator("/sys/class/hwmon",error))
            if(read(directory.path()/"in1_label").find("VDD_IN")!=std::string::npos){power_directory=directory.path();break;}
        if(port) {
            if(port<1 || port>65535)throw std::runtime_error("Invalid dashboard port");
            socket=::socket(AF_INET,SOCK_STREAM|SOCK_CLOEXEC,0);
            if(socket<0)throw std::runtime_error("Cannot create dashboard socket");
            int one=1;::setsockopt(socket,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
            sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);address.sin_port=htons(uint16_t(port));
            if(::bind(socket,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0 || ::listen(socket,4)<0){::close(socket);socket=-1;throw std::runtime_error("Cannot bind localhost dashboard port");}
        }
        save(output/"run.json",meta);
        try {worker=std::thread([this]{run();});}catch(...){if(socket>=0)::close(socket);throw;}
    }
    ~Impl(){finish=true;if(worker.joinable())worker.join();if(socket>=0)::close(socket);}
    Json snapshot(){
        std::unique_lock<std::mutex> lock(mutex);Json selected=Json::array();
        size_t stride=std::max<size_t>(1,(points.size()+2499)/2500);
        for(size_t i=0;i<points.size();i+=stride)selected.push_back(points[i]);
        if(!points.empty() && (points.size()-1)%stride)selected.push_back(points.back());
        double duration=began?((ended?ended:us())-began)*1e-6:0;
        auto times=latencies;
        Json result={{"phase",phase},{"duration_s",duration},{"processed",processed},{"tracked",tracked},{"initialized_frames",inertial},
                {"fps",duration>0?processed/duration:0},
                {"power_profile",meta["power_profile_start"]},{"points",selected},{"live",true}};
        lock.unlock();result["tracking_ms"]=distribution(std::move(times));return result;
    }
    void sample(){
        uint64_t begin=0;{std::lock_guard<std::mutex> lock(mutex);if(!began || ended)return;begin=began;}
        double t=(us()-begin)*1e-6;rusage usage{};::getrusage(RUSAGE_SELF,&usage);
        double cpu=usage.ru_utime.tv_sec+usage.ru_utime.tv_usec*1e-6+usage.ru_stime.tv_sec+usage.ru_stime.tv_usec*1e-6;
        double cpu_pct=time_previous>0?100*(cpu-cpu_previous)/(t-time_previous):std::numeric_limits<double>::quiet_NaN();
        cpu_previous=cpu;time_previous=t;
        std::ifstream statm("/proc/self/statm");long total=0,resident=0;statm>>total>>resident;
        double watts=std::numeric_limits<double>::quiet_NaN();
        if(!power_directory.empty())watts=number(power_directory/"in1_input")*number(power_directory/"curr1_input")/1e6;
        double temp=std::numeric_limits<double>::quiet_NaN();std::error_code error;
        for(const auto& directory:fs::directory_iterator("/sys/class/thermal",error)) {
            if(directory.path().filename().string().find("thermal_zone")!=0)continue;
            double value=number(directory.path()/"temp")/1000;
            if(std::isfinite(value) && (!std::isfinite(temp) || value>temp))temp=value;
        }
        std::lock_guard<std::mutex> lock(mutex);if(ended)return;
        resources<<std::setprecision(10)<<t<<','<<cpu_pct<<','<<resident*::sysconf(_SC_PAGESIZE)/1048576.0<<','
                 <<usage.ru_maxrss/1024.0<<','<<watts<<','<<number("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")<<','<<temp<<'\n';resources.flush();
    }
    void serve(){
        if(socket<0){std::this_thread::sleep_for(std::chrono::milliseconds(100));return;}
        pollfd waiting{socket,POLLIN,0};if(::poll(&waiting,1,100)<=0)return;
        int client=::accept4(socket,nullptr,nullptr,SOCK_CLOEXEC);if(client<0)return;
        timeval timeout{0,200000};::setsockopt(client,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));::setsockopt(client,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
        char buffer[2048];auto count=::recv(client,buffer,sizeof(buffer),0);
        std::string request=count>0?std::string(buffer,size_t(count)):"";
        std::string body,type="text/plain",status="404 Not Found";
        if(request.rfind("GET /api/state ",0)==0){body=snapshot().dump();type="application/json";status="200 OK";}
        else if(request.rfind("GET / ",0)==0){body=html(nullptr);type="text/html; charset=utf-8";status="200 OK";}
        std::string response="HTTP/1.1 "+status+"\r\nContent-Type: "+type+"\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
        size_t offset=0;while(offset<response.size()){auto n=::send(client,response.data()+offset,response.size()-offset,MSG_NOSIGNAL);if(n<=0)break;offset+=size_t(n);}::close(client);
    }
    void run(){
        auto next=us();
        try {while(!finish){serve();if(us()>=next){sample();next=us()+1000000;}}}
        catch(const std::exception& e){std::lock_guard<std::mutex> lock(mutex);meta["dashboard_error"]=e.what();}
    }
};
Benchmark::Benchmark(const fs::path& output,int port,const std::string& profile):impl_(new Impl(output,port,profile)){}
Benchmark::~Benchmark(){
    if(!impl_)return;
    try {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if(!impl_->ended){impl_->ended=us();impl_->meta.update({{"phase","ERROR"},{"stop_reason","exception_or_incomplete"},
            {"duration_s",impl_->began?(impl_->ended-impl_->began)*1e-6:0}});save(impl_->output/"run.json",impl_->meta);}
    }catch(...){} // destructor cleanup must not mask the original camera error
}
void Benchmark::status(const std::string& phase){std::lock_guard<std::mutex> lock(impl_->mutex);impl_->phase=phase;}
void Benchmark::integration(bool navigation,bool buzzer){std::lock_guard<std::mutex> lock(impl_->mutex);impl_->meta["navigation_transmission"]=navigation;impl_->meta["buzzer_enabled"]=buzzer;impl_->meta["buzzer_transport"]=buzzer?"Cube MAVLink PLAY_TUNE":"disabled";save(impl_->output/"run.json",impl_->meta);}
void Benchmark::start(double device_s,uint64_t host_us){
    std::lock_guard<std::mutex> lock(impl_->mutex);impl_->began=host_us;impl_->phase="RUNNING";
    impl_->meta.update({{"start_device_s",device_s},{"start_host_us",host_us},{"phase","RUNNING"}});
    impl_->events<<Json({{"event","TEST_START"},{"device_s",device_s},{"host_us",host_us}}).dump()<<'\n';impl_->events.flush();
    save(impl_->output/"run.json",impl_->meta);
}
bool Benchmark::started()const{std::lock_guard<std::mutex> lock(impl_->mutex);return impl_->began!=0;}
double Benchmark::elapsed(uint64_t host_us)const{std::lock_guard<std::mutex> lock(impl_->mutex);return impl_->began?(host_us-impl_->began)*1e-6:0;}
void Benchmark::frame(const Frame& f){
    std::lock_guard<std::mutex> lock(impl_->mutex);++impl_->processed;if(f.state==2)++impl_->tracked;if(f.initialized)++impl_->inertial;
    impl_->latencies.push_back(f.tracking_ms);
    impl_->frames<<f.device_s<<','<<f.elapsed_s<<','<<f.state<<','<<f.initialized<<','<<f.quality<<','<<f.tracking_ms<<','<<f.age_ms<<','<<f.qr_ms<<','<<f.wall_ms<<','<<f.segment<<','<<f.index<<','<<f.x<<','<<f.y<<','<<f.z<<','<<f.qw<<','<<f.qx<<','<<f.qy<<','<<f.qz<<'\n';
    if(f.state==2 && std::isfinite(f.x) && std::isfinite(f.y) && std::isfinite(f.z))impl_->points.push_back({f.elapsed_s,f.x,f.y,f.z,f.initialized,f.segment});
}
void Benchmark::stop(const std::string& reason,uint64_t host_us,uint64_t overwritten){
    auto profile=power_profile();std::lock_guard<std::mutex> lock(impl_->mutex);impl_->ended=host_us;impl_->phase="FINALIZING";
    impl_->meta.update({{"stop_reason",reason},{"stop_host_us",host_us},{"duration_s",impl_->began?(host_us-impl_->began)*1e-6:0},
                        {"video_overwrites",overwritten},{"power_profile_end",profile},{"phase","FINALIZING"},
                        {"power_profile_changed",profile!=impl_->meta["power_profile_start"].get<std::string>()}});
    impl_->events<<Json({{"event","TEST_STOP"},{"reason",reason},{"host_us",host_us}}).dump()<<'\n';impl_->events.flush();impl_->frames.flush();impl_->resources.flush();
    save(impl_->output/"run.json",impl_->meta);
}

void create_report(const fs::path& directory){
    Json meta=Json::parse(read(directory/"run.json"));
    std::ifstream file(directory/"frames.csv");if(!file)throw std::runtime_error("Missing frames.csv");
    std::string line;std::getline(file,line);
    Json points=Json::array(),loops=Json::array();
    std::vector<double> tracking,ages,qr,wall;size_t n=0,tracked=0,initialized=0,initialized_tracked=0;
    std::unordered_map<int64_t,std::pair<uint64_t,bool>> tracked_samples;
    bool end_initialized=false;double first_init=-1,elapsed_last=0;
    while(std::getline(file,line)) {
        auto c=split(line);if(c.size()!=18)throw std::runtime_error("Malformed benchmark frame row");
        ++n;int state=std::stoi(c[2]);bool init=std::stoi(c[3]);double elapsed=std::stod(c[1]);elapsed_last=elapsed;
        tracked+=state==2;initialized+=init;initialized_tracked+=init && state==2;end_initialized=init;
        if(init && first_init<0)first_init=elapsed;
        tracking.push_back(std::stod(c[5]));ages.push_back(std::stod(c[6]));qr.push_back(std::stod(c[7]));wall.push_back(std::stod(c[8]));
        double x=std::stod(c[11]),y=std::stod(c[12]),z=std::stod(c[13]);
        if(state==2 && std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            auto segment=std::stoull(c[9]);points.push_back({elapsed,x,y,z,init,segment});
            tracked_samples[std::llround(std::stod(c[0])*1e6)]={segment,init};
        }
    }
    std::ifstream events(directory/"loop-events.csv");
    while(std::getline(events,line)){auto c=split(line);if(c.size()==3)loops.push_back({{"device_s",std::stod(c[0])},{"keyframe",std::stoull(c[1])},{"matched_keyframe",std::stoull(c[2])}});}
    std::string console=read(directory/"console.log");size_t resets=0,pos=0;
    while((pos=console.find("LM: Active map reset, Done!!!",pos))!=std::string::npos){++resets;++pos;}
    std::ifstream resources(directory/"resources.csv");std::getline(resources,line);
    std::vector<double> watts,cpu,rss,temperature;double energy=0,last_t=-1,last_w=0,first_t=-1;
    while(std::getline(resources,line)) {
        auto c=split(line);if(c.size()!=7)continue;double t=std::stod(c[0]),w=std::stod(c[4]);
        for(auto item:{std::make_pair(1,&cpu),{2,&rss},{6,&temperature}}){double v=std::stod(c[item.first]);if(std::isfinite(v))item.second->push_back(v);}
        if(std::isfinite(w)){watts.push_back(w);if(last_t>=0)energy+=(t-last_t)*(w+last_w)*.5;else first_t=t;last_t=t;last_w=w;}
    }
    double duration=meta.value("duration_s",elapsed_last);
    Json metrics={{"frames",n},{"tracked",tracked},{"initialized_frames",initialized},{"initialized_tracked_frames",initialized_tracked},
                  {"imu_initialized_at_end",end_initialized},{"active_map_resets",fs::exists(directory/"console.log")?Json(resets):Json(nullptr)},
                  {"accepted_loop_closures",meta.value("loop_event_instrumentation",true)?Json(loops.size()):Json(nullptr)},
                  {"initialization_time_s",first_init>=0?Json(first_init):Json(nullptr)},
                  {"duration_s",duration},{"processing_fps",duration>0?n/duration:0},
                  {"tracking_ms",distribution(tracking)},{"capture_age_ms",distribution(ages)},{"qr_ms_per_frame",distribution(qr)},
                  {"frame_wall_ms",distribution(wall)},{"input_power_w",distribution(watts)},{"process_cpu_pct",distribution(cpu)},
                  {"rss_mb",distribution(rss)},{"temperature_c",distribution(temperature)},
                  {"sampled_input_energy_j",watts.size()>1?Json(energy):Json(nullptr)},
                  {"energy_sample_coverage_s",last_t>first_t?last_t-first_t:0},
                  {"ground_truth_available",false},{"sustained_vio_validated",false}};
    save(directory/"metrics.json",metrics);
    plot_png(directory,points,metrics,meta["power_profile_start"]);
    Json optimized=Json::array();std::ifstream tum(directory/"trajectory_optimized.txt");
    while(std::getline(tum,line)) {
        std::stringstream row(line);double t,x,y,z;
        if(row>>t>>x>>y>>z && std::isfinite(t) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            auto match=tracked_samples.find(std::llround(t*1e6));
            if(match!=tracked_samples.end())optimized.push_back({t-meta.value("start_device_s",0.0),x,y,z,match->second.first,match->second.second});
        }
    }
    auto previous_phase=meta.value("phase",std::string());
    std::string phase=(previous_phase=="ERROR" || previous_phase=="INCOMPLETE")?"INCOMPLETE":"COMPLETE";
    Json report={{"phase",phase},{"power_profile",meta["power_profile_start"]},{"duration_s",duration},
                 {"processed",n},{"tracked",tracked},{"initialized_frames",initialized},{"fps",metrics["processing_fps"]},
                 {"tracking_ms",metrics["tracking_ms"]},{"metrics",metrics},{"metadata",meta},{"points",points},
                 {"loop_events",loops},{"optimized_points",optimized},{"live",false}};
    std::ofstream page(directory/"report.html");page<<html(report);if(!page)throw std::runtime_error("Cannot write HTML report");
    save(directory/"plot_data.json",report);
    meta["phase"]=phase;save(directory/"run.json",meta);
}
} // namespace bhu::bench
