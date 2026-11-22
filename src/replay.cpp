#include "System.h"
#include "bhu/imu.hpp"
#include "bhu/sparse_map.hpp"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <sys/resource.h>

namespace fs=std::filesystem;
static std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> v;std::stringstream s(line);std::string cell;
    while(std::getline(s,cell,','))v.push_back(cell);
    return v;
}
static std::vector<bhu::Sample> samples(const fs::path& p) {
    std::ifstream f(p);if(!f)throw std::runtime_error("Cannot read "+p.string());
    std::string line;std::getline(f,line);std::vector<bhu::Sample> out;
    while(std::getline(f,line)) {
        auto v=split(line);if(v.size()!=4)throw std::runtime_error("Malformed IMU CSV");
        out.push_back({std::stod(v[0]),{std::stod(v[1]),std::stod(v[2]),std::stod(v[3])}});
    }
    return out;
}

int main(int argc,char**argv) try {
    if(argc<3) {
        std::cerr<<"Usage: gemini_replay RECORDING VOCABULARY [--inertial] [--monocular] [--allow-provisional] [--viewer] [--realtime] [--sparse-map] [--max-frames N] [--settings FILE] [--output DIRECTORY]\n";
        return 2;
    }
    fs::path root=argv[1],vocabulary=argv[2],settings=root/"orb.yaml",output;
    bool inertial=false,provisional=false,viewer=false,realtime=false,sparseMap=false,monocular=false;
    size_t frameLimit=0;
    int cvThreads=0;
    for(int i=3;i<argc;++i) {
        std::string flag=argv[i];
        if(flag=="--inertial")inertial=true;
        else if(flag=="--sparse-map")sparseMap=true;
        else if(flag=="--monocular")monocular=true;
        else if(flag=="--max-frames" && i+1<argc)frameLimit=std::stoul(argv[++i]);
        else if(flag=="--opencv-threads" && i+1<argc)cvThreads=std::stoi(argv[++i]);
        else if(flag=="--allow-provisional")provisional=true;
        else if(flag=="--viewer")viewer=true;
        else if(flag=="--realtime")realtime=true;
        else if(flag=="--settings" && i+1<argc)settings=argv[++i];
        else if(flag=="--output" && i+1<argc)output=argv[++i];
        else throw std::runtime_error("Unknown or incomplete option: "+flag);
    }
    if(!fs::exists(vocabulary))throw std::runtime_error("Missing vocabulary");
    if(cvThreads>0)cv::setNumThreads(cvThreads);
    cv::FileStorage config(settings.string(),cv::FileStorage::READ);
    if(!config.isOpened())throw std::runtime_error("Cannot open settings");
    if(inertial && int(config["Calibration.imuValidated"])!=1 && !provisional)
        throw std::runtime_error("IMU noise/extrinsics are provisional. Calibrate first, or use --allow-provisional for an experimental run.");
    double imuOffset=0;
    if(!config["IMU.TimeOffsetSeconds"].empty())imuOffset=double(config["IMU.TimeOffsetSeconds"]);
    if(!std::isfinite(imuOffset))throw std::runtime_error("Nonfinite IMU time offset");
    config.release();
    std::vector<bhu::Imu> imu;
    if(inertial) {
        if(fs::exists(root/"INVALID_UNITS.md"))
            throw std::runtime_error("Recording is marked as having invalid IMU units");
        auto acc=samples(root/"accel.csv"),gyro=samples(root/"gyro.csv");
        for(auto& s:acc)s.t+=imuOffset;
        for(auto& s:gyro)s.t+=imuOffset;
        imu=bhu::synchronize(acc,gyro);
        if(imu.size()<2)throw std::runtime_error("No overlapping accelerometer and gyroscope coverage");
    }
    struct Frame{double t;std::string left,right;};std::vector<Frame> frames;
    std::ifstream input(root/"frames.csv");std::string line;std::getline(input,line);
    while(std::getline(input,line)) {
        auto v=split(line);if(v.size()!=7)throw std::runtime_error("Malformed frame CSV");
        Frame f{std::stod(v[0]),v[1],v[2]};
        if(!std::isfinite(f.t) || (!frames.empty() && f.t<=frames.back().t))throw std::runtime_error("Invalid image timestamps");
        if(inertial && (f.t<=imu.front().t || f.t>=imu.back().t))continue;
        frames.push_back(f);
    }
    if(frames.size()<2)throw std::runtime_error("Not enough frames with IMU coverage");
    if(frameLimit && frames.size()>frameLimit)frames.resize(frameLimit);
    if(monocular&&sparseMap)throw std::runtime_error("Sparse-map export currently requires stereo replay");
    const fs::path out=output.empty()?root/(inertial?"slam_inertial":"slam_stereo"):output;
    if(fs::exists(out))throw std::runtime_error("SLAM output already exists: "+out.string());
    fs::create_directories(out);
    auto sensor=monocular?(inertial?ORB_SLAM3::System::IMU_MONOCULAR:ORB_SLAM3::System::MONOCULAR):(inertial?ORB_SLAM3::System::IMU_STEREO:ORB_SLAM3::System::STEREO);
    ORB_SLAM3::System slam(vocabulary.string(),settings.string(),sensor,viewer);
    std::unique_ptr<bhu::SparseMapRecorder> mapExport;
    if(sparseMap)mapExport=std::make_unique<bhu::SparseMapRecorder>(out,frames.front().t,inertial);
    std::ofstream status(out/"tracking.csv");status<<"timestamp_s,state,tracking_ms,imu_initialized,pipeline_ms\n"<<std::setprecision(15);
    std::ofstream history(out/"pose_history.csv");history<<"timestamp_s,x,y,z,qx,qy,qz,qw,state,imu_initialized,segment\n"<<std::setprecision(15);
    unsigned poseSegment=0;int previousState=-1;bool previousInitialized=false;
    size_t j=0,tracked=0;bool imuInitialized=false;auto epoch=std::chrono::steady_clock::now();
    std::vector<double> pipelineTimes;size_t initializedTracked=0;double firstInit=-1;
    try {
        for(const auto& f:frames) {
            if(realtime)std::this_thread::sleep_until(epoch+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(f.t-frames.front().t)));
            auto pipelineStart=std::chrono::steady_clock::now();
            auto left=cv::imread((root/f.left).string(),cv::IMREAD_GRAYSCALE);
            auto right=monocular?cv::Mat():cv::imread((root/f.right).string(),cv::IMREAD_GRAYSCALE);
            if(left.empty() || (!monocular&&right.empty()))throw std::runtime_error("Missing input image");
            std::vector<ORB_SLAM3::IMU::Point> batch;
            if(inertial) {
                // Include a sample at/after image time so preintegration can
                // interpolate its endpoint. Do not submit that sample twice.
                while(j<imu.size() && imu[j].t<=f.t) {
                    auto& m=imu[j++];batch.emplace_back(m.a[0],m.a[1],m.a[2],m.w[0],m.w[1],m.w[2],m.t);
                }
                if(j<imu.size()) {
                    auto& m=imu[j++];batch.emplace_back(m.a[0],m.a[1],m.a[2],m.w[0],m.w[1],m.w[2],m.t);
                }
            }
            auto start=std::chrono::steady_clock::now();
            auto cameraFromWorld=monocular?slam.TrackMonocular(left,f.t,batch):slam.TrackStereo(left,right,f.t,batch);
            double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
            int state=slam.GetTrackingState();if(state==2)++tracked;
            bool initialized=inertial && slam.GetTimeFromIMUInit()>0;
            double pipelineMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-pipelineStart).count();pipelineTimes.push_back(pipelineMs);
            if(initialized&&state==2){++initializedTracked;if(firstInit<0)firstInit=f.t-frames.front().t;}
            if(mapExport)mapExport->capture(slam,left,f.t,state,initialized,cameraFromWorld,&f==&frames.back());
            if(slam.MapChanged() || state!=2 || state!=previousState || initialized!=previousInitialized)++poseSegment;
            previousState=state;previousInitialized=initialized;
            if(state==2){auto worldFromCamera=cameraFromWorld.inverse();auto p=worldFromCamera.translation();auto q=worldFromCamera.unit_quaternion();
                if(p.allFinite() && q.coeffs().allFinite())history<<f.t<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<','<<state<<','<<initialized<<','<<poseSegment<<'\n';}
            if(!history)throw std::runtime_error("Cannot write replay pose history");
            imuInitialized=imuInitialized || initialized;
            status<<f.t<<','<<state<<','<<ms<<','<<initialized<<','<<pipelineMs<<'\n';
        }
    }catch(...){slam.Shutdown();throw;}
    slam.Shutdown();
    struct rusage usage{};getrusage(RUSAGE_SELF,&usage);std::sort(pipelineTimes.begin(),pipelineTimes.end());
    auto quantile=[&](double q){return pipelineTimes.empty()?0:pipelineTimes[size_t(q*(pipelineTimes.size()-1))];};
    nlohmann::json replaySummary={{"algorithm","ORB-SLAM3"},{"monocular",monocular},{"inertial",inertial},{"frames",frames.size()},{"tracked_frames",tracked},{"initialized_tracked_frames",initializedTracked},{"first_initialization_elapsed_s",firstInit},{"pipeline_ms",{{"p50",quantile(.5)},{"p95",quantile(.95)},{"p99",quantile(.99)}}},{"wall_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-epoch).count()},{"peak_rss_kib",usage.ru_maxrss}};
    std::ofstream(out/"replay-summary.json")<<replaySummary.dump(2)<<'\n';
    if(mapExport)mapExport->finish();
    // Upstream SaveTrajectoryTUM indexes the first keyframe without checking
    // for an empty map. Save keyframes first and guard initialization failure.
    slam.SaveKeyFrameTrajectoryTUM((out/"keyframes.txt").string());
    if(!tracked || !fs::file_size(out/"keyframes.txt")) {
        std::cerr<<"No usable tracked map; trajectory export skipped. Output: "<<out<<'\n';
        return 1;
    }
    if(!monocular)slam.SaveTrajectoryTUM((out/"trajectory.txt").string());
    // Upstream TUM frame exporter rejects monocular modes; online camera poses
    // remain available in pose_history.csv without pretending to be optimized.
    std::cout<<"Tracked "<<tracked<<" / "<<frames.size()<<" frames. Output: "<<out<<'\n';
    if(inertial)std::cout<<"IMU initialized during replay: "<<imuInitialized<<'\n';
    return tracked && (!inertial || imuInitialized)?0:1;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
