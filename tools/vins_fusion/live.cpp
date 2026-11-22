#include "runtime_globals.hpp"
#include "stereo_policy.hpp"
#include "bhu/camera.hpp"
#include "bhu/imu.hpp"
#include "bhu/external_nav.hpp"
#include "bhu/shadow_gate.hpp"
#include "bhu/rolling_samples.hpp"
#include "input_recorder.hpp"
#include "bhu/recording_cue.hpp"
#include "bhu/flight_recording.hpp"
#include "bhu/estimator_watchdog.hpp"
#include "bhu/imu_health.hpp"
#include <json.hpp>
#include <glog/logging.h>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sys/resource.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using J=nlohmann::json; namespace fs=std::filesystem;
using namespace bhu::camera;
static volatile std::sig_atomic_t running=1;
static void stop(int){running=0;}
static uint64_t clock_us(){return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static void need(bool ok,const std::string& s){if(!ok)throw std::runtime_error(s);}
static Eigen::Vector3d vec(const std::array<double,3>&v){return {v[0],v[1],v[2]};}
static double quantile(const bhu::RollingSamples& v,double q){return v.quantile(q);}
struct Capture {
    std::mutex mutex;std::condition_variable ready;
    std::deque<bhu::Sample> acc,gyro;
    std::shared_ptr<ob::FrameSet> latest;
    uint64_t arrival=0,overwritten=0;
    std::string error;
    ob::Pipeline& video;
    std::vector<std::shared_ptr<ob::Sensor>> sensors;
    bool video_started=false;
    std::shared_ptr<FusionInputRecorder> recorder;
    bhu::ImuHealth imuHealth;
    explicit Capture(ob::Pipeline& p):video(p){}
    void finish(){
        if(video_started)try{video.stop();}catch(...){}
        video_started=false;
        for(auto& s:sensors)try{s->stop();}catch(...){}
        sensors.clear();
    }
    ~Capture(){finish();}
    void sample(bool acceleration,const std::shared_ptr<ob::Frame>& f,const cv::Mat& R_A_G,double offset) noexcept {
        try {
            bhu::Sample s;s.t=f->getTimeStampUs()*1e-6+offset;
            if(acceleration) {
                auto v=f->as<ob::AccelFrame>()->getValue();
                imuHealth.observe(true,{v.x,v.y,v.z},clock_us());
                cv::Mat a=R_A_G*(cv::Mat_<double>(3,1)<<v.x,v.y,v.z);
                s.v={a.at<double>(0),a.at<double>(1),a.at<double>(2)};
            } else {auto v=f->as<ob::GyroFrame>()->getValue();imuHealth.observe(false,{v.x,v.y,v.z},clock_us());s.v={v.x,v.y,v.z};}
            if(auto record=std::atomic_load(&recorder))record->imu(acceleration,s.t,s.v);
            std::lock_guard<std::mutex> lock(mutex);
            auto& queue=acceleration?acc:gyro;
            if(!queue.empty() && s.t<=queue.back().t)throw std::runtime_error("Nonmonotonic live IMU");
            queue.push_back(s);while(queue.size()>800)queue.pop_front();ready.notify_all();
        }catch(const std::exception& e){std::lock_guard<std::mutex> lock(mutex);error=e.what();ready.notify_all();}
    }
};

int main(int argc,char**argv)try{
    google::InitGoogleLogging(argv[0]);FLAGS_logtostderr=true;
    if(argc<4){std::cerr<<"Usage: vins_fusion_live CAMERA.yaml NAV.yaml NEW_OUTPUT [--seconds 30 | --continuous] [--record-input --record-cue-port 14602] [--udp-port 14600] [--opencv-threads 1] [--max-features 150] [--feature-distance 30] [--stereo-policy upstream|rectified] [--clahe-clip 2] --allow-provisional\n";return argc==2&&std::string(argv[1])=="--help"?0:2;}
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
    double seconds=30,claheClip=2.;int port=0,cvThreads=1,cuePort=0;bool provisional=false,continuous=false,secondsSet=false,recordInput=false;std::string flightStatus;
    for(int i=4;i<argc;++i){std::string k=argv[i];if(k=="--allow-provisional"){provisional=true;continue;}if(k=="--continuous"){continuous=true;continue;}if(k=="--record-input"){recordInput=true;continue;}need(i+1<argc,"Missing option value");std::string v=argv[++i];if(k=="--max-features")MAX_CNT=std::stoi(v);else if(k=="--feature-distance")MIN_DIST=std::stoi(v);else if(k=="--stereo-policy"){need(v=="upstream"||v=="rectified","Invalid stereo policy");bhu::fusion::rectified_stereo=v=="rectified";}else if(k=="--opencv-threads")cvThreads=std::stoi(v);else if(k=="--clahe-clip")claheClip=std::stod(v);else if(k=="--seconds"){seconds=std::stod(v);secondsSet=true;}else if(k=="--udp-port")port=std::stoi(v);else if(k=="--record-cue-port")cuePort=std::stoi(v);else if(k=="--flight-status")flightStatus=v;else throw std::runtime_error("Unknown option "+k);}
    need(!(continuous&&secondsSet),"Use --continuous or --seconds, not both");
    need(cuePort>=0&&cuePort<=65535&&(!cuePort||recordInput),"Recording cue requires --record-input and a valid port");
    need(std::isfinite(seconds)&&seconds>0&&seconds<=600&&port>=0&&port<=65535,"Invalid limits");
    need(cvThreads>=1&&cvThreads<=8&&std::isfinite(claheClip)&&claheClip>=0&&claheClip<=10,"Invalid thread/CLAHE settings");
    need(MAX_CNT>=150&&MAX_CNT<=250&&MIN_DIST>=15&&MIN_DIST<=30,"Invalid feature selection limits");
    fs::path out=argv[3];need(!fs::exists(out),"Output exists");cv::setNumThreads(cvThreads);
    cv::FileStorage orb(argv[1],cv::FileStorage::READ),nav(argv[2],cv::FileStorage::READ);
    need(orb.isOpened()&&nav.isOpened(),"Cannot read calibration/mount");
    need(int(orb["Calibration.imuValidated"])==1||provisional,"Provisional calibration needs --allow-provisional");
    need(std::string(orb["Camera.type"])=="Rectified"&&int(orb["Camera.fps"])==30,"Requires rectified 30 Hz calibration");
    ROW=int(orb["Camera.height"]);COL=int(orb["Camera.width"]);double hz=double(orb["IMU.Frequency"]),offset=double(orb["IMU.TimeOffsetSeconds"]);if(!std::isfinite(hz)||hz<=0||!std::isfinite(offset))throw std::runtime_error("Invalid IMU configuration");TD=-offset;
    cv::Mat tbc;orb["IMU.T_b_c1"]>>tbc;tbc.convertTo(tbc,CV_64F);if(tbc.rows!=4||tbc.cols!=4)throw std::runtime_error("Invalid extrinsics");Eigen::Matrix3d r;Eigen::Vector3d t;for(int i=0;i<3;++i){t[i]=tbc.at<double>(i,3);for(int j=0;j<3;++j)r(i,j)=tbc.at<double>(i,j);}if((r*r.transpose()-Eigen::Matrix3d::Identity()).norm()>.01||std::abs(r.determinant()-1)>.01)throw std::runtime_error("Invalid rotation");RIC.push_back(r);TIC.push_back(t);
    // ORB takes continuous densities and discretizes at IMU frequency; VINS's
    // midpoint noise Jacobian multiplies sample noise and bias increments by dt.
    // This explicit convention bridge is provisional, not a measured calibration.
    ACC_N=double(orb["IMU.NoiseAcc"])*std::sqrt(hz);GYR_N=double(orb["IMU.NoiseGyro"])*std::sqrt(hz);ACC_W=double(orb["IMU.AccWalk"])*std::sqrt(hz);GYR_W=double(orb["IMU.GyroWalk"])*std::sqrt(hz);
    fs::create_directories(out);fs::path settings=out/"vins-config.yaml";
    {cv::FileStorage f(settings.string(),cv::FileStorage::WRITE);f<<"model_type"<<"PINHOLE"<<"camera_name"<<"gemini_rectified_left"<<"image_width"<<int(COL)<<"image_height"<<int(ROW)<<"distortion_parameters"<<"{"<<"k1"<<0.<<"k2"<<0.<<"p1"<<0.<<"p2"<<0.<<"}"<<"projection_parameters"<<"{"<<"fx"<<double(orb["Camera1.fx"])<<"fy"<<double(orb["Camera1.fy"])<<"cx"<<double(orb["Camera1.cx"])<<"cy"<<double(orb["Camera1.cy"])<<"}"<<"extrinsicRotation"<<tbc(cv::Rect(0,0,3,3))<<"extrinsicTranslation"<<tbc(cv::Rect(3,0,1,3))<<"estimate_extrinsic"<<0<<"estimate_td"<<0<<"td"<<TD<<"acc_n"<<ACC_N<<"gyr_n"<<GYR_N<<"acc_w"<<ACC_W<<"gyr_w"<<GYR_W<<"max_cnt"<<MAX_CNT<<"min_dist"<<MIN_DIST<<"equalize"<<0<<"max_solver_time"<<SOLVER_TIME<<"max_num_iterations"<<NUM_ITERATIONS<<"loop_closure"<<0;}

    double baseline=double(orb["Stereo.b"]);if(!(baseline>0&&baseline<1))throw std::runtime_error("Invalid stereo baseline");
    RIC.push_back(r);TIC.push_back(t+r*Eigen::Vector3d(baseline,0,0));
    // Rectified cameras share intrinsics; right camera centre lies +baseline on left X.
    CAM_NAMES={fs::absolute(settings).string(),fs::absolute(settings).string()};
    {cv::FileStorage f((out/"stereo-extrinsics.yaml").string(),cv::FileStorage::WRITE);cv::Mat right=tbc.clone();for(int k=0;k<3;++k)right.at<double>(k,3)=TIC[1][k];f<<"body_T_cam0"<<tbc<<"body_T_cam1"<<right<<"imu"<<1<<"num_of_cam"<<2<<"multiple_thread"<<0<<"flow_back"<<FLOW_BACK<<"equalize"<<0;}

    const int width=COL,height=ROW;
    ob::Context ctx;ctx.setLoggerSeverity(OB_LOG_SEVERITY_WARN);
    auto devices=ctx.queryDeviceList();need(devices->getCount()==1,"Expected exactly one Orbbec camera");
    auto device=devices->getDevice(0);
    need(std::string(device->getDeviceInfo()->getName()).find("336L")!=std::string::npos,"Expected Gemini 336L");
    Properties properties{device,{},{}};
    if(device->isPropertySupported(OB_PROP_IR_RECTIFY_BOOL,OB_PERMISSION_READ_WRITE))properties.set(OB_PROP_IR_RECTIFY_BOOL,false);
    properties.setInt(OB_PROP_LASER_CONTROL_INT,0);
    auto left=device->getSensor(OB_SENSOR_IR_LEFT)->getStreamProfileList()->getVideoStreamProfile(width,height,OB_FORMAT_Y8,30);
    auto right=device->getSensor(OB_SENSOR_IR_RIGHT)->getStreamProfileList()->getVideoStreamProfile(width,height,OB_FORMAT_Y8,30);
    auto as=device->getSensor(OB_SENSOR_ACCEL),gs=device->getSensor(OB_SENSOR_GYRO);
    auto ap=as->getStreamProfileList()->getAccelStreamProfile(OB_ACCEL_FS_4g,OB_SAMPLE_RATE_200_HZ);
    auto gp=gs->getStreamProfileList()->getGyroStreamProfile(OB_GYRO_FS_1000dps,OB_SAMPLE_RATE_200_HZ);
    auto e=left->getExtrinsicTo(right);
    cv::Mat stereoTranslation=(cv::Mat_<double>(3,1)<<e.trans[0]*.001,e.trans[1]*.001,e.trans[2]*.001);
    cv::Mat r1,r2,p1,p2,q,lx,ly,rx,ry;
    cv::stereoRectify(intrinsic(left),distortion(left),intrinsic(right),distortion(right),{width,height},rotation(e),stereoTranslation,r1,r2,p1,p2,q,cv::CALIB_ZERO_DISPARITY,0);
    need(p2.at<double>(0,3)<0 && std::abs(p2.at<double>(1,3))<1e-6,"Invalid horizontal stereo geometry");
    need(std::abs(cv::norm(stereoTranslation)-double(orb["Stereo.b"]))<1e-5,"Camera baseline differs from settings");
    for(auto field: {std::make_pair("Camera1.fx",std::make_pair(0,0)),{"Camera1.fy",{1,1}},{"Camera1.cx",{0,2}},{"Camera1.cy",{1,2}}})
        need(std::abs(p1.at<double>(field.second.first,field.second.second)-double(orb[field.first]))<.01,"Camera intrinsics differ from settings");
    cv::initUndistortRectifyMap(intrinsic(left),distortion(left),r1,p1,{width,height},CV_32FC1,lx,ly);
    cv::initUndistortRectifyMap(intrinsic(right),distortion(right),r2,p2,{width,height},CV_32FC1,rx,ry);
    cv::Mat rag=rotation(ap->getExtrinsicTo(gp));
    auto lToG=left->getExtrinsicTo(gp);
    cv::Mat R_G_C=rotation(lToG)*r1.t(),configuredT;
    orb["IMU.T_b_c1"]>>configuredT;configuredT.convertTo(configuredT,CV_64F);
    need(configuredT.rows==4 && configuredT.cols==4 && cv::norm(configuredT(cv::Rect(0,0,3,3))-R_G_C)<1e-5,"Camera/IMU rotation differs from settings");
    for(int i=0;i<3;++i)need(std::abs(configuredT.at<double>(i,3)-lToG.trans[i]*.001)<1e-5,"Camera/IMU translation differs from settings");

    bhu::extnav::Alignment alignment;cv::Mat mount;nav["Mount.R_body_optical"]>>mount;mount.convertTo(mount,CV_64F);
    need(mount.rows==3&&mount.cols==3,"Invalid mount");for(int i=0;i<3;++i)for(int j=0;j<3;++j)alignment.R_B_C(i,j)=mount.at<double>(i,j);
    need(bhu::extnav::rotation_valid(alignment.R_B_C),"Invalid mounting rotation");
    // Fusion's feature-support policy is independent of downstream MAVLink
    // quality filtering and the legacy ORB Publisher.quality_min setting.
    int persistentMin=nav["Publisher.persistent_min"].empty()?40:int(nav["Publisher.persistent_min"]);need(persistentMin>=40&&persistentMin<=100,"Invalid persistent-feature threshold");
    static Estimator estimator;estimator.setParameter();
    struct Socket{int fd=-1;~Socket(){if(fd>=0)::close(fd);}} udp;
    sockaddr_in destination{};destination.sin_family=AF_INET;destination.sin_addr.s_addr=htonl(INADDR_LOOPBACK);destination.sin_port=htons(port);
    if(port){udp.fd=::socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);need(udp.fd>=0,"UDP socket failed");}
    std::ofstream csv(out/"tracking.csv"),poses(out/"pose_history.csv");need(bool(csv)&&bool(poses),"Cannot create logs");
    fs::copy_file(argv[1],out/"camera.yaml");fs::copy_file(argv[2],out/"navigation.yaml");
    csv<<"device_s,host_capture_us,initialized,persistent,backend_tracks,stereo_matches,quality,rectify_ms,frontend_ms,backend_ms,pipeline_ms,age_ms,eligible,published,overwritten,preprocess_ms,shadow_time60_500ms,shadow_time40_500ms,hard_reject_mask,velocity_valid,shadow_frame15_feature40,shadow_frame30_feature60,support_reject_mask,reset_counter,recoveries,last_reset_causes,pending_reset_causes,imu_healthy,disarmed_reinitializations\n"<<std::setprecision(15);
    poses<<"timestamp_s,x,y,z,qx,qy,qz,qw,segment,visual_support_20\n"<<std::setprecision(15);
    std::shared_ptr<FusionInputRecorder> recorder;
    std::ofstream comparison(out/"recovery_comparison.csv");need(bool(comparison),"Cannot create gate comparison log");
    comparison<<"host_capture_us,gate_now_us,baseline_eligible,candidate_eligible,candidate_recovery_us,baseline_reset,candidate_reset,baseline_recoveries,candidate_recoveries,hard_mask,support_mask,x,y,z,qx,qy,qz,qw,epoch\n"<<std::setprecision(15);
    if(recordInput&&flightStatus.empty()){recorder=std::make_shared<FusionInputRecorder>(out/"input");fs::copy_file(argv[1],out/"input/orb.yaml");}
    ob::Pipeline video(device);Capture capture(video);std::atomic_store(&capture.recorder,recorder);
    as->start(ap,[&](std::shared_ptr<ob::Frame>f){capture.sample(true,f,rag,0);});capture.sensors.push_back(as);
    gs->start(gp,[&](std::shared_ptr<ob::Frame>f){capture.sample(false,f,rag,0);});capture.sensors.push_back(gs);
    auto config=std::make_shared<ob::Config>();config->enableStream(left);config->enableStream(right);config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);video.enableFrameSync();
    video.start(config,[&](std::shared_ptr<ob::FrameSet> f){auto now=clock_us();std::lock_guard<std::mutex> lock(capture.mutex);if(capture.latest)++capture.overwritten;capture.latest=std::move(f);capture.arrival=now;capture.ready.notify_all();});capture.video_started=true;
    const uint64_t start=clock_us();uint64_t lastDelivery=start;int64_t clockOffset=INT64_MAX;
    double current=-1,lastFrame=-1,previousTime=-1,lastImuReal=-1;bool aligned=false,wasInitialized=false;uint64_t epoch=0,processed=0,initialized=0,eligible=0,published=0,imuWaitSkips=0;
    Eigen::Vector3d previousPosition=Eigen::Vector3d::Zero();bhu::extnav::DurationGate gate(0,0,0,0),baselineShadow(500000),briefRecoveryShadow(500000,100000,200000);bhu::extnav::Gate frame15Feature40(15),frame30Feature60;
    bhu::extnav::TimedShadowGate shadow60(500000),shadow40(500000);uint64_t shadow60Pass=0,shadow40Pass=0,frame15Feature40Pass=0,frame30Feature60Pass=0;cv::Ptr<cv::CLAHE> contrast;if(claheClip>0)contrast=cv::createCLAHE(claheClip,cv::Size(8,8));
    bhu::RollingSamples ages,pipelines,frontends,backends,rectifies,preprocess;
    uint64_t lastStatus=0;size_t lastPersistent=0,lastMatches=0;int lastBackend=0;unsigned lastSupportMask=0,lastHardMask=0;double lastSpeed=0;
    auto recordingCue=std::make_unique<bhu::RecordingCue>(cuePort,clock_us());
    std::unique_ptr<bhu::FlightRecording> flights;if(!flightStatus.empty())flights=std::make_unique<bhu::FlightRecording>(flightStatus);
    bhu::EstimatorWatchdog watchdog;uint64_t automaticResets=0;
    std::ofstream estimatorEvents(out/"estimator_events.jsonl");need(bool(estimatorEvents),"Cannot create estimator event log");
    fs::path flightOut;std::ofstream flightTracking,flightPoses;uint64_t lastFlightPoll=0;
    auto status=[&](bool active,bool accepted,uint64_t captureUs,double age){
        csv.flush();poses.flush();comparison.flush();need(bool(csv)&&bool(poses)&&bool(comparison),"Log flush failed");
        J state={{"algorithm","VINS-Fusion"},{"running",active},{"continuous",continuous},{"host_monotonic_us",clock_us()},{"last_capture_host_us",captureUs},{"last_capture_age_ms",age},{"initialized",wasInitialized},{"gate_eligible",accepted},{"processed",processed},{"eligible",eligible},{"published_udp",published},{"udp_port",port},{"navigation_transport",port?"loopback UDP only":"disabled"},{"calibration_validated",false},{"fusion_validated",false}};
        state["publication_gate"]={{"persistent",lastPersistent},{"backend_tracks",lastBackend},{"stereo_matches",lastMatches},{"support_reject_mask",lastSupportMask},{"reset_counter",gate.reset_counter()},{"recoveries",gate.recoveries()},{"last_reset_causes",gate.last_reset_causes()},{"pending_reset_causes",gate.pending_reset_causes()},{"max_features",MAX_CNT},{"feature_distance",MIN_DIST},{"stereo_policy",bhu::fusion::rectified_stereo?"rectified":"upstream"}};
        state["publication_gate"]["hard_reject_mask"]=lastHardMask;
        state["publication_gate"]["freshness_policy"]="arducopter";state["publication_gate"]["max_age_ms"]=nullptr;state["publication_gate"]["recovery_ms"]=0;state["publication_gate"]["velocity_gap_limit_ms"]=nullptr;
        state["publication_gate"]["pose_speed_m_s"]=std::isfinite(lastSpeed)?J(lastSpeed):J(nullptr);
        state["publication_gate"]["reason"]=!wasInitialized?"initializing":(lastHardMask&64)?"imu_saturation_holdoff":(lastHardMask&8)?"invalid_pose_velocity":lastHardMask?"hard_guard":lastSupportMask?"insufficient_visual_support":accepted?"eligible":"rejected";
        state["brief_recovery_comparison"]={{"shadow_only",true},{"initial_ms",500},{"brief_recovery_ms",100},{"brief_interrupt_limit_ms",200},{"fault_recovery_ms",500},{"reset_counter",briefRecoveryShadow.reset_counter()},{"recoveries",briefRecoveryShadow.recoveries()}};
        state["imu_health"]={{"healthy",capture.imuHealth.healthy(clock_us())},{"acceleration_near_limit_samples",capture.imuHealth.acceleration_clips()},{"gyro_near_limit_samples",capture.imuHealth.gyro_clips()},{"last_clip_host_us",capture.imuHealth.last_clip_host_us()},{"holdoff_ms",200}};
        state["disarmed_estimator_reinitializations"]=automaticResets;
        state["flight_recording_active"]=recorder!=nullptr;state["flight_directory"]=flightOut.string();
        if(recorder){state["input_recording"]=recorder->status();if(active)recordingCue->poll(bool(state["input_recording"]["recording_ready"]),clock_us());}
        state["recording_cue"]={{"port",cuePort},{"requested",recordingCue->requested()},{"cube_owner_acknowledged",recordingCue->acknowledged()},{"audible_confirmation",false}};
        if(flightTracking.is_open()){flightTracking.flush();flightPoses.flush();need(bool(flightTracking)&&bool(flightPoses),"Flight log flush failed");}
        {std::ofstream file(out/"status.json.tmp");file<<state.dump(2)<<'\n';file.flush();need(bool(file),"Status write failed");}
        fs::rename(out/"status.json.tmp",out/"status.json");lastStatus=clock_us();
    };
    status(true,false,0,0);
    std::cout<<"FUSION_LIVE_READY continuous="<<continuous<<" seconds="<<seconds<<" UDP="<<port<<std::endl;
    while(running&&(continuous||(clock_us()-start)*1e-6<seconds)){
        if(flights&&clock_us()-lastFlightPoll>=200000){
            lastFlightPoll=clock_us();auto event=flights->poll(lastFlightPoll);
            if(event==bhu::FlightRecording::Start){
                flightOut=out/"flights"/("flight-"+std::to_string(lastFlightPoll));fs::create_directories(flightOut);
                recorder=std::make_shared<FusionInputRecorder>(flightOut/"input");fs::copy_file(argv[1],flightOut/"input/orb.yaml");std::atomic_store(&capture.recorder,recorder);
                flightTracking.open(flightOut/"tracking.csv");flightPoses.open(flightOut/"pose_history.csv");
                flightTracking<<"device_s,age_ms,eligible\n"<<std::setprecision(15);flightPoses<<"timestamp_s,x,y,z,qx,qy,qz,qw,segment,visual_support_20\n"<<std::setprecision(15);
                std::ofstream(flightOut/"flight.json")<<J{{"recording_start_host_us",lastFlightPoll},{"finalized",false},{"trigger","Cube armed"}}.dump(2)<<'\n';
                recordingCue=std::make_unique<bhu::RecordingCue>(cuePort,lastFlightPoll);std::cout<<"FLIGHT_RECORDING_STARTED "<<flightOut<<std::endl;
            }else if(event==bhu::FlightRecording::Stop){
                std::atomic_store(&capture.recorder,std::shared_ptr<FusionInputRecorder>{});recorder->finish();recorder.reset();flightTracking.close();flightPoses.close();
                auto metadata=J::parse(std::ifstream(flightOut/"flight.json"));metadata["recording_stop_host_us"]=clock_us();metadata["finalized"]=true;metadata["stop_reason"]="fresh Cube disarm plus 3 s tail";std::ofstream(flightOut/"flight.json")<<metadata.dump(2)<<'\n';
                std::cout<<"FLIGHT_RECORDING_FINALIZED "<<flightOut<<std::endl;
            }
        }
        std::shared_ptr<ob::FrameSet> frames;uint64_t arrival=0,overwritten=0;
        {std::unique_lock<std::mutex> lock(capture.mutex);capture.ready.wait_for(lock,std::chrono::milliseconds(200),[&]{return capture.latest||!capture.error.empty();});need(capture.error.empty(),capture.error);if(!capture.latest){need(clock_us()-lastDelivery<2000000,"Camera stalled");continue;}frames=std::move(capture.latest);arrival=capture.arrival;overwritten=capture.overwritten;}
        lastDelivery=clock_us();auto begin=clock_us();auto lf=frames->getFrame(OB_FRAME_IR_LEFT),rf=frames->getFrame(OB_FRAME_IR_RIGHT);need(lf&&rf,"Missing stereo frame");
        uint64_t stamp=lf->getTimeStampUs();double time=stamp*1e-6,target=time+TD;
        need(stamp&&time>lastFrame&&std::abs(double(stamp)-double(rf->getTimeStampUs()))<=500,"Invalid stereo timestamp");lastFrame=time;
        // Device integration time is retained. This host-time estimate excludes
        // an independently measured exposure/USB delay; never use send time.
        clockOffset=std::min(clockOffset,int64_t(arrival)-int64_t(stamp));uint64_t hostCapture=uint64_t(int64_t(stamp)+clockOffset);
        std::vector<bhu::Sample> acc,gyro;
        {std::unique_lock<std::mutex> lock(capture.mutex);capture.ready.wait_for(lock,std::chrono::milliseconds(25),[&]{return !capture.error.empty()||(!capture.acc.empty()&&!capture.gyro.empty()&&capture.acc.back().t>target+.006&&capture.gyro.back().t>target+.005);});need(capture.error.empty(),capture.error);acc.assign(capture.acc.begin(),capture.acc.end());gyro.assign(capture.gyro.begin(),capture.gyro.end());}
        if(acc.size()<2||gyro.size()<2){++imuWaitSkips;continue;}auto imu=bhu::synchronize(acc,gyro);
        if(imu.size()<2||imu.front().t>=target||imu.back().t<=target){++imuWaitSkips;continue;}
        need(current<0||imu.front().t<=current,"IMU buffer overrun");
        auto l=lf->as<ob::VideoFrame>(),rr=rf->as<ob::VideoFrame>();
        need(lf->getFormat()==OB_FORMAT_Y8&&rf->getFormat()==OB_FORMAT_Y8&&l->getWidth()==unsigned(width)&&l->getHeight()==unsigned(height)&&rr->getWidth()==unsigned(width)&&rr->getHeight()==unsigned(height)&&lf->getDataSize()==size_t(width)*height&&rf->getDataSize()==size_t(width)*height,"Invalid image buffer");
        auto rectStart=clock_us();cv::Mat image,rightImage;cv::remap(cv::Mat(height,width,CV_8UC1,lf->getData()),image,lx,ly,cv::INTER_LINEAR);cv::remap(cv::Mat(height,width,CV_8UC1,rf->getData()),rightImage,rx,ry,cv::INTER_LINEAR);double rectMs=(clock_us()-rectStart)*.001;
        if(recorder)recorder->stereo(time,stamp,rf->getTimeStampUs(),hostCapture,image,rightImage);
        auto contrastStart=clock_us();if(contrast){contrast->apply(image,image);contrast->apply(rightImage,rightImage);}double contrastMs=contrast?(clock_us()-contrastStart)*.001:0;preprocess.push_back(contrastMs);
        auto frontStart=clock_us();auto features=estimator.featureTracker.trackImage(time,image,rightImage);double frontMs=(clock_us()-frontStart)*.001;auto backStart=clock_us();
        // Integrate every retained IMU sample between selected images, including
        // an interpolated endpoint. Discarding video must not discard IMU.
        if(!estimator.initFirstPoseFlag){std::vector<std::pair<double,Eigen::Vector3d>> init;for(const auto&m:imu)if(m.t<=target&&m.t>=target-.1)init.push_back({m.t,vec(m.a)});need(!init.empty(),"No gravity initialization samples");estimator.initFirstIMUPose(init);}
        if(current<0){size_t k=0;while(k+1<imu.size()&&imu[k+1].t<target-.05)++k;current=imu[k].t;lastImuReal=current;estimator.processIMU(current,0,vec(imu[k].a),vec(imu[k].w));}
        size_t j=0;while(j<imu.size()&&imu[j].t<=current)++j;
        for(;j<imu.size()&&imu[j].t<=target;++j){const auto&m=imu[j];need(m.t-lastImuReal<=.02,"Missing retained IMU samples");estimator.processIMU(m.t,m.t-current,vec(m.a),vec(m.w));current=m.t;lastImuReal=m.t;}
        need(j>0&&j<imu.size(),"No IMU endpoint bracket");const auto endpoint=bhu::interpolate_endpoint(imu[j-1],imu[j],target);Eigen::Vector3d angular=vec(endpoint.w);
        if(target>current){estimator.processIMU(target,target-current,vec(endpoint.a),angular);current=target;}
        estimator.processImage(features,time);double backMs=(clock_us()-backStart)*.001;
        bool ok=estimator.solver_flag==Estimator::NON_LINEAR;if(wasInitialized&&!ok){++epoch;aligned=false;previousTime=-1;}wasInitialized=ok;
        size_t persistent=std::count_if(estimator.featureTracker.track_cnt.begin(),estimator.featureTracker.track_cnt.end(),[](int n){return n>=2;}),matches=0;for(auto&kv:features)matches+=kv.second.size()==2;
        // Bench-only support heuristic, not covariance or a calibrated failure
        // detector. Upstream's failureDetection returns false unconditionally.
        int quality=int(std::min<size_t>(100,persistent));bool supported=ok&&persistent>=size_t(persistentMin)&&estimator.f_manager.last_track_num>=20&&matches>=20;
        bhu::extnav::Sample sample;sample.capture_host_us=hostCapture;sample.reset_epoch=epoch;sample.imu_initialized=ok;sample.imu_healthy=capture.imuHealth.healthy(clock_us());sample.quality=quality;sample.tracking_ok=supported;
        if(ok){++initialized;sample.T_W_C.linear()=estimator.Rs[WINDOW_SIZE]*estimator.ric[0];sample.T_W_C.translation()=estimator.Ps[WINDOW_SIZE]+estimator.Rs[WINDOW_SIZE]*estimator.tic[0];need(sample.T_W_C.matrix().allFinite(),"Nonfinite pose");
            if(!aligned){Eigen::Vector3d forward=sample.T_W_C.rotation()*alignment.R_B_C.transpose()*Eigen::Vector3d::UnitX();forward.z()=0;need(forward.norm()>.1,"Camera heading is vertical");forward.normalize();Eigen::Vector3d down(0,0,-1),right=down.cross(forward);alignment.T_L_W.linear().row(0)=forward.transpose();alignment.T_L_W.linear().row(1)=right.transpose();alignment.T_L_W.linear().row(2)=down.transpose();alignment.T_L_W.translation()=-alignment.T_L_W.rotation()*sample.T_W_C.translation();aligned=true;}
            sample.frame_alignment_valid=aligned;sample.angular_velocity_C=estimator.ric[0].transpose()*angular;
            if(previousTime>0&&time>previousTime){sample.camera_velocity_W=(sample.T_W_C.translation()-previousPosition)/(time-previousTime);sample.velocity_valid=sample.camera_velocity_W.norm()<10;}
            previousPosition=sample.T_W_C.translation();previousTime=time;
            Eigen::Quaterniond quat(sample.T_W_C.rotation());const auto&p=sample.T_W_C.translation();poses<<time<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<quat.x()<<','<<quat.y()<<','<<quat.z()<<','<<quat.w()<<','<<epoch<<','<<supported<<'\n';
            if(flightPoses.is_open())flightPoses<<time<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<quat.x()<<','<<quat.y()<<','<<quat.z()<<','<<quat.w()<<','<<epoch<<','<<supported<<'\n';
        }
        const auto gateNow=clock_us();auto accepted=gate.accept(sample,gateNow);auto baseline=baselineShadow.accept(sample,gateNow);auto candidate=briefRecoveryShadow.accept(sample,gateNow);auto strong=sample;strong.tracking_ok=ok&&persistent>=60&&estimator.f_manager.last_track_num>=20&&matches>=20;const bool pass60=shadow60.accept(strong,gateNow),frame30Pass=bool(frame30Feature60.accept(strong,gateNow));frame30Feature60Pass+=frame30Pass;auto relaxed=sample;relaxed.tracking_ok=ok&&persistent>=40&&estimator.f_manager.last_track_num>=20&&matches>=20;const bool pass40=shadow40.accept(relaxed,gateNow),frame15Pass=bool(frame15Feature40.accept(relaxed,gateNow));frame15Feature40Pass+=frame15Pass;shadow60Pass+=pass60;shadow40Pass+=pass40;lastHardMask=bhu::extnav::hard_rejection_mask(sample,gateNow,0);lastSpeed=sample.camera_velocity_W.norm();lastPersistent=persistent;lastMatches=matches;lastBackend=estimator.f_manager.last_track_num;lastSupportMask=(persistent<size_t(persistentMin)?1u:0u)|(lastBackend<20?2u:0u)|(matches<20?4u:0u);bool sent=false;if(accepted){++eligible;if(port){auto pose=bhu::extnav::convert(sample,alignment);auto message=bhu::extnav::encode(sample,pose,*accepted);std::array<uint8_t,MAVLINK_MAX_PACKET_LEN> wire{};auto n=mavlink_msg_to_send_buffer(wire.data(),&message);need(::sendto(udp.fd,wire.data(),n,0,reinterpret_cast<sockaddr*>(&destination),sizeof(destination))==n,"UDP send failed");++published;sent=true;}}
        {Eigen::Quaterniond q(sample.T_W_C.rotation());const auto& p=sample.T_W_C.translation();comparison<<hostCapture<<','<<gateNow<<','<<bool(baseline)<<','<<bool(candidate)<<','<<briefRecoveryShadow.qualification_duration_us()<<','<<unsigned(baselineShadow.reset_counter())<<','<<unsigned(briefRecoveryShadow.reset_counter())<<','<<baselineShadow.recoveries()<<','<<briefRecoveryShadow.recoveries()<<','<<bhu::extnav::hard_rejection_mask(sample,gateNow)<<','<<lastSupportMask<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<','<<epoch<<'\n';need(bool(comparison),"Comparison log write failed");}
        double pipelineMs=(clock_us()-begin)*.001,age=(clock_us()-hostCapture)*.001;ages.push_back(age);pipelines.push_back(pipelineMs);frontends.push_back(frontMs);backends.push_back(backMs);rectifies.push_back(rectMs);++processed;
        csv<<time<<','<<hostCapture<<','<<ok<<','<<persistent<<','<<estimator.f_manager.last_track_num<<','<<matches<<','<<quality<<','<<rectMs<<','<<frontMs<<','<<backMs<<','<<pipelineMs<<','<<age<<','<<bool(accepted)<<','<<sent<<','<<overwritten<<','<<contrastMs<<','<<pass60<<','<<pass40<<','<<lastHardMask<<','<<sample.velocity_valid<<','<<frame15Pass<<','<<frame30Pass<<','<<lastSupportMask<<','<<unsigned(gate.reset_counter())<<','<<gate.recoveries()<<','<<gate.last_reset_causes()<<','<<gate.pending_reset_causes()<<','<<sample.imu_healthy<<','<<automaticResets<<'\n';need(bool(csv)&&bool(poses),"Log write failed");
        if(flightTracking.is_open())flightTracking<<time<<','<<age<<','<<bool(accepted)<<'\n';
        if(clock_us()-lastStatus>=1000000)status(true,bool(accepted),hostCapture,age);
        if(watchdog.poll(clock_us(),ok,sample.velocity_valid,supported&&sample.imu_healthy,
                         flights&&flights->fresh_disarmed(clock_us()),recorder!=nullptr)){
            estimatorEvents<<J{{"event","disarmed_estimator_reinitialize"},{"host_monotonic_us",clock_us()},
                {"device_s",time},{"reason","invalid pose velocity for 2 s with adequate support"},
                {"previous_epoch",epoch},{"fresh_disarmed",true},{"recording_active",false}}.dump()<<'\n';
            estimatorEvents.flush();need(bool(estimatorEvents),"Estimator event write failed");
            estimator.clearState();estimator.featureTracker=FeatureTracker();estimator.setParameter();
            current=-1;lastImuReal=-1;previousTime=-1;aligned=false;wasInitialized=false;++epoch;++automaticResets;
            std::cout<<"DISARMED_ESTIMATOR_REINITIALIZED epoch="<<epoch<<std::endl;
        }
        if(processed%100==0)std::cout<<"frames="<<processed<<" init="<<ok<<" age_ms="<<age<<" eligible="<<eligible<<std::endl;
    }
    capture.finish();if(recorder)recorder->finish();double elapsed=(clock_us()-start)*1e-6;struct rusage ru{};getrusage(RUSAGE_SELF,&ru);uint64_t overwritten;{std::lock_guard<std::mutex> lock(capture.mutex);overwritten=capture.overwritten;}
    status(false,false,0,0);
    J summary={{"imu_endpoint_owned",true},{"algorithm","VINS-Fusion live stereo + IMU"},{"opencv_threads",cv::getNumThreads()},{"clahe_clip_limit",claheClip},{"preprocess_ms_median",quantile(preprocess,.5)},{"shadow_time60_500ms",shadow60Pass},{"shadow_time40_500ms",shadow40Pass},{"publication_gate",{{"freshness_policy","arducopter"},{"recovery_ms",0},{"persistent_min",persistentMin},{"backend_min",20},{"stereo_min",20},{"max_age_ms",nullptr}}},{"shadow_frame30_feature60",frame30Feature60Pass},{"shadow_gates_publish",false},{"shadow_frame15_feature40",frame15Feature40Pass},{"seconds",elapsed},{"processed",processed},{"initialized",initialized},{"eligible",eligible},{"published_udp",published},{"overwritten_pending_stereo",overwritten},{"imu_wait_skips",imuWaitSkips},{"processed_fps",processed/elapsed},{"age_ms_median",quantile(ages,.5)},{"age_ms_p95",quantile(ages,.95)},{"age_ms_max",quantile(ages,1)},{"pipeline_ms_median",quantile(pipelines,.5)},{"pipeline_ms_p95",quantile(pipelines,.95)},{"frontend_ms_median",quantile(frontends,.5)},{"backend_ms_median",quantile(backends,.5)},{"rectify_ms_median",quantile(rectifies,.5)},{"rss_mib",ru.ru_maxrss/1024.},{"calibration_validated",false},{"clock_mapping","lower envelope device-to-host delivery; exposure/USB latency not independently measured"},{"failure_detector_disabled_upstream",true},{"fusion_validated",false},{"loop_closure",false}};
    summary["disarmed_estimator_reinitializations"]=automaticResets;summary["imu_health"]={{"acceleration_near_limit_samples",capture.imuHealth.acceleration_clips()},{"gyro_near_limit_samples",capture.imuHealth.gyro_clips()},{"holdoff_ms",200}};summary["frontend"]={{"max_features",MAX_CNT},{"feature_distance",MIN_DIST},{"stereo_policy",bhu::fusion::rectified_stereo?"rectified":"upstream"}};summary["gate_recoveries"]=gate.recoveries();summary["reset_counter"]=gate.reset_counter();
    summary["continuous"]=continuous;summary["statistics_scope"]="most recent up to 8192 processed frames";summary["statistics_samples"]=ages.size();
    if(recorder)summary["input_recording"]=recorder->status();
    std::ofstream(out/"summary.json")<<summary.dump(2)<<'\n';std::cout<<summary.dump(2)<<std::endl;return processed?0:1;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
