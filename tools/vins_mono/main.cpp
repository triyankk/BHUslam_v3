#include "estimator.h"
#include "frontend.hpp"
#include "bhu/imu.hpp"
#include <json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/resource.h>
#include <glog/logging.h>
using J=nlohmann::json;namespace fs=std::filesystem;
double INIT_DEPTH=5,MIN_PARALLAX=10./460.,ACC_N,ACC_W,GYR_N,GYR_W;
std::vector<Eigen::Matrix3d> RIC;std::vector<Eigen::Vector3d>TIC;
Eigen::Vector3d G(0,0,9.81);
double BIAS_ACC_THRESHOLD=.1,BIAS_GYR_THRESHOLD=.1,SOLVER_TIME=.04,ROW,COL,TD=0,TR=0;
int NUM_ITERATIONS=8,ESTIMATE_EXTRINSIC=0,ESTIMATE_TD=0,ROLLING_SHUTTER=0;
std::string EX_CALIB_RESULT_PATH,VINS_RESULT_PATH,IMU_TOPIC;
static std::vector<std::string> split(const std::string&s){std::stringstream in(s);std::string c;std::vector<std::string>v;while(std::getline(in,c,','))v.push_back(c);return v;}
static std::vector<bhu::Sample> load(const fs::path&p){std::ifstream in(p);if(!in)throw std::runtime_error("Cannot read "+p.string());std::string s;std::getline(in,s);std::vector<bhu::Sample>v;while(std::getline(in,s)){auto c=split(s);if(c.size()!=4)throw std::runtime_error("Malformed IMU CSV");v.push_back({std::stod(c[0]),{std::stod(c[1]),std::stod(c[2]),std::stod(c[3])}});}bhu::validate(v);return v;}
static double quantile(std::vector<double>v,double q){if(v.empty())return 0;std::sort(v.begin(),v.end());return v[size_t(q*(v.size()-1))];}
static Eigen::Vector3d vec(const std::array<double,3>&v){return {v[0],v[1],v[2]};}
int main(int argc,char**argv)try{
    google::InitGoogleLogging(argv[0]);FLAGS_logtostderr=true;
    if(argc<3){std::cerr<<"Usage: vins_mono_replay RECORDING NEW_OUTPUT [--native-noise] [--max-frames N]\n";return 2;}
    fs::path root=argv[1],out=argv[2];bool nativeNoise=false;size_t limit=0;int cvThreads=0;for(int i=3;i<argc;++i){std::string s=argv[i];if(s=="--native-noise")nativeNoise=true;else if(s=="--max-frames"&&i+1<argc)limit=std::stoul(argv[++i]);else if(s=="--opencv-threads"&&i+1<argc)cvThreads=std::stoi(argv[++i]);else throw std::runtime_error("Unknown option "+s);}if(cvThreads>0)cv::setNumThreads(cvThreads);
    if(fs::exists(out))throw std::runtime_error("Output exists");if(fs::exists(root/"INVALID_UNITS.md"))throw std::runtime_error("Invalid IMU unit recording");
    auto imu=bhu::synchronize(load(root/"accel.csv"),load(root/"gyro.csv"));
    cv::FileStorage orb((root/"orb.yaml").string(),cv::FileStorage::READ);if(!orb.isOpened())throw std::runtime_error("Cannot read ORB calibration");
    ROW=int(orb["Camera.height"]);COL=int(orb["Camera.width"]);double hz=double(orb["IMU.Frequency"]),offset=double(orb["IMU.TimeOffsetSeconds"]);if(hz<=0||!std::isfinite(offset))throw std::runtime_error("Invalid IMU configuration");TD=-offset;
    cv::Mat tbc;orb["IMU.T_b_c1"]>>tbc;tbc.convertTo(tbc,CV_64F);if(tbc.rows!=4||tbc.cols!=4)throw std::runtime_error("Invalid extrinsics");Eigen::Matrix3d r;Eigen::Vector3d t;for(int i=0;i<3;++i){t[i]=tbc.at<double>(i,3);for(int j=0;j<3;++j)r(i,j)=tbc.at<double>(i,j);}if((r*r.transpose()-Eigen::Matrix3d::Identity()).norm()>.01||std::abs(r.determinant()-1)>.01)throw std::runtime_error("Invalid rotation");RIC.push_back(r);TIC.push_back(t);
    // ORB takes continuous densities and discretizes at IMU frequency; VINS's
    // midpoint noise Jacobian multiplies sample noise and bias increments by dt.
    // This explicit convention bridge is provisional, not a measured calibration.
    ACC_N=double(orb["IMU.NoiseAcc"])*std::sqrt(hz);GYR_N=double(orb["IMU.NoiseGyro"])*std::sqrt(hz);ACC_W=double(orb["IMU.AccWalk"])*std::sqrt(hz);GYR_W=double(orb["IMU.GyroWalk"])*std::sqrt(hz);
    if(nativeNoise){ACC_N=.08;GYR_N=.004;ACC_W=.00004;GYR_W=.000002;}
    fs::create_directories(out);fs::path settings=out/"vins-config.yaml";
    {cv::FileStorage f(settings.string(),cv::FileStorage::WRITE);f<<"model_type"<<"PINHOLE"<<"camera_name"<<"gemini_rectified_left"<<"image_width"<<int(COL)<<"image_height"<<int(ROW)<<"distortion_parameters"<<"{"<<"k1"<<0.<<"k2"<<0.<<"p1"<<0.<<"p2"<<0.<<"}"<<"projection_parameters"<<"{"<<"fx"<<double(orb["Camera1.fx"])<<"fy"<<double(orb["Camera1.fy"])<<"cx"<<double(orb["Camera1.cx"])<<"cy"<<double(orb["Camera1.cy"])<<"}"<<"extrinsicRotation"<<tbc(cv::Rect(0,0,3,3))<<"extrinsicTranslation"<<tbc(cv::Rect(3,0,1,3))<<"estimate_extrinsic"<<0<<"estimate_td"<<0<<"td"<<TD<<"acc_n"<<ACC_N<<"gyr_n"<<GYR_N<<"acc_w"<<ACC_W<<"gyr_w"<<GYR_W<<"max_cnt"<<150<<"min_dist"<<30<<"equalize"<<1<<"max_solver_time"<<SOLVER_TIME<<"max_num_iterations"<<NUM_ITERATIONS<<"loop_closure"<<0;}
    struct Frame{double t;std::string path;};std::vector<Frame>frames;std::ifstream fin(root/"frames.csv");std::string line;std::getline(fin,line);while(std::getline(fin,line)){auto c=split(line);if(c.size()!=7)throw std::runtime_error("Malformed frames.csv");double time=std::stod(c[0]);if(!std::isfinite(time)||(!frames.empty()&&time<=frames.back().t))throw std::runtime_error("Invalid frame time");if(time+TD>imu.front().t&&time+TD<imu.back().t)frames.push_back({time,c[1]});}if(limit&&frames.size()>limit)frames.resize(limit);if(frames.size()<4)throw std::runtime_error("Insufficient frames");
    VinsFrontend frontend(settings.string());
    // Match upstream ROS node's static-storage initialization: its constructor
    // calls clearState on pointer arrays which must initially be zeroed.
    static Estimator estimator;estimator.setParameter();
    std::ofstream csv(out/"tracking.csv"),poses(out/"pose_history.csv"),bias(out/"bias.csv");csv<<"timestamp_s,solver_initialized,frontend_ms,estimator_ms,pipeline_ms,persistent_features,backend_tracks,visual_support_20,segment\n";poses<<"timestamp_s,x,y,z,qx,qy,qz,qw,segment,visual_support_20\n";bias<<"timestamp_s,bax,bay,baz,bgx,bgy,bgz\n";csv<<std::setprecision(15);poses<<std::setprecision(15);bias<<std::setprecision(15);
    size_t j=0,initialized=0,supported=0,resets=0;unsigned segment=0;bool previous=false;double current=-1,firstInit=-1;std::vector<double>latency;auto start=std::chrono::steady_clock::now();
    for(size_t index=0;index<frames.size();++index){auto&f=frames[index];auto pipelineStart=std::chrono::steady_clock::now();auto image=cv::imread((root/f.path).string(),0);if(image.empty()||image.rows!=ROW||image.cols!=COL)throw std::runtime_error("Missing/wrong-size image");auto begin=std::chrono::steady_clock::now();auto features=frontend.process(image,f.t);double fm=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();begin=std::chrono::steady_clock::now();
        // Skip upstream's two frontend warm-up callbacks plus first feature
        // publication (which the ROS estimator's feature callback discards).
        if(index>=3){double target=f.t+estimator.td;
            if(current<0){current=imu.front().t;estimator.processIMU(0,vec(imu.front().a),vec(imu.front().w));j=1;}
            while(j<imu.size()&&imu[j].t<=target){auto&m=imu[j++];estimator.processIMU(m.t-current,vec(m.a),vec(m.w));current=m.t;}
            if(target>current){if(j==0||j>=imu.size())throw std::runtime_error("No bracketing IMU");auto&a=imu[j-1];auto&b=imu[j];double alpha=(target-a.t)/(b.t-a.t);auto aa=vec(a.a)+(vec(b.a)-vec(a.a))*alpha;auto ww=vec(a.w)+(vec(b.w)-vec(a.w))*alpha;estimator.processIMU(target-current,aa,ww);current=target;}
            std_msgs::Header header;header.stamp.value=f.t;estimator.processImage(features,header);
        }
        double em=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count(),pm=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-pipelineStart).count();latency.push_back(pm);bool ok=estimator.solver_flag==Estimator::NON_LINEAR;if(previous&&!ok){++resets;++segment;}previous=ok;bool visual=ok&&features.size()>=20&&estimator.f_manager.last_track_num>=20;
        if(ok){auto rwc=estimator.Rs[WINDOW_SIZE]*estimator.ric[0];auto p=estimator.Ps[WINDOW_SIZE]+estimator.Rs[WINDOW_SIZE]*estimator.tic[0];Eigen::Quaterniond q(rwc);if(!p.allFinite()||!q.coeffs().allFinite())throw std::runtime_error("Nonfinite VINS pose");++initialized;supported+=visual;if(firstInit<0)firstInit=f.t-frames.front().t;poses<<f.t<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<','<<segment<<','<<visual<<'\n';bias<<f.t;for(double v:estimator.Bas[WINDOW_SIZE])bias<<','<<v;for(double v:estimator.Bgs[WINDOW_SIZE])bias<<','<<v;bias<<'\n';}
        csv<<f.t<<','<<ok<<','<<fm<<','<<em<<','<<pm<<','<<features.size()<<','<<estimator.f_manager.last_track_num<<','<<visual<<','<<segment<<'\n';if(!csv||!poses||!bias)throw std::runtime_error("Output write failed");if(index%300==0)std::cout<<"Processed "<<index<<" initialized="<<ok<<" features="<<features.size()<<std::endl;
    }
    struct rusage usage{};getrusage(RUSAGE_SELF,&usage);J summary={{"dataset",fs::absolute(root).string()},{"algorithm","VINS-Mono upstream core, native offline adapter"},{"frames",frames.size()},{"initialized_frames",initialized},{"visually_supported_frames_heuristic",supported},{"failure_resets",resets},{"first_initialization_elapsed_s",firstInit},{"solver_initialized_at_end",previous},{"loop_closure",false},{"online_extrinsic_calibration",false},{"online_temporal_calibration",false},{"noise_policy",nativeNoise?"upstream EuRoC defaults":"provisional ORB density to per-sample bridge sqrt(frequency)"},{"acc_n",ACC_N},{"gyr_n",GYR_N},{"acc_w",ACC_W},{"gyr_w",GYR_W},{"td",TD},{"pipeline_ms",{{"p50",quantile(latency,.5)},{"p95",quantile(latency,.95)},{"p99",quantile(latency,.99)}}},{"wall_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()},{"peak_rss_kib",usage.ru_maxrss},{"warning","No ground truth; NON_LINEAR is not proof of visual tracking. Visual support threshold 20 is a diagnostic heuristic."}};std::ofstream(out/"summary.json")<<summary.dump(2)<<'\n';std::cout<<summary.dump(2)<<std::endl;return initialized?0:1;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
