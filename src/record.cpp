#include "bhu/camera.hpp"
#include "bhu/qr_trigger.hpp"
#include "bhu/cube_feedback.hpp"
#include "bhu/benchmark.hpp"
#include <json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <vector>
#include <atomic>
#include <deque>
#include <thread>
#include <fcntl.h>
#include <unistd.h>

namespace fs=std::filesystem;
static volatile std::sig_atomic_t running=1;
static void stop(int) { running=0; }
static int feedbackPort=0;
static uint64_t hostUs(){return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static void pgm(const fs::path& path,const cv::Mat& image){
    std::ofstream f(path,std::ios::binary);f<<"P5\n"<<image.cols<<' '<<image.rows<<"\n255\n";
    for(int row=0;row<image.rows;++row)f.write(reinterpret_cast<const char*>(image.ptr(row)),image.cols);
    if(!f)throw std::runtime_error("Image write failed: "+path.string());
}
struct RecordingStatus {
    fs::path output;nlohmann::json data;bool complete=false;
    void save(){std::ofstream f(output/"recording.json");f<<data.dump(2)<<'\n';if(!f)throw std::runtime_error("Cannot write recording status");}
    ~RecordingStatus(){if(!complete)try{data["phase"]="ERROR";data["complete"]=false;save();}catch(...){}}
};

using namespace bhu::camera;

static void writeSettings(const fs::path& path,int w,int h,int fps,const cv::Mat& p,double baseline,const cv::Mat& tbc) {
    std::ofstream s(path);
    // ORB-SLAM3's settings reader rejects integer nodes for floating-point
    // parameters, even when a calibrated value is mathematically integral.
    s << std::setprecision(12) << std::showpoint << "%YAML:1.0\nFile.version: \"1.0\"\nCamera.type: \"Rectified\"\n"
      << "Camera1.fx: " << p.at<double>(0,0) << "\nCamera1.fy: " << p.at<double>(1,1)
      << "\nCamera1.cx: " << p.at<double>(0,2) << "\nCamera1.cy: " << p.at<double>(1,2)
      << "\nCamera.width: "<<w<<"\nCamera.height: "<<h<<"\nCamera.fps: "<<fps
      << "\nCamera.RGB: 0\nStereo.b: "<<baseline<<"\nStereo.ThDepth: 40.0\n"
      << "# Factory extrinsics, with left-camera rectification rotation applied.\n"
      << "# T_b_c1 maps rectified left-camera coordinates into gyro/body coordinates.\n"
      << "IMU.T_b_c1: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: f\n   data: [";
    for(int i=0;i<16;++i) s << (i?", ":"") << tbc.at<double>(i/4,i%4);
    s << "]\nIMU.Frequency: 200.0\nIMU.InsertKFsWhenLost: 0\n"
      << "# Adapter convention: camera-clock time = IMU timestamp + offset.\nIMU.TimeOffsetSeconds: 0.0\n"
      << "# Provisional noise parameters: MUST validate by stationary recording/Allan analysis.\n"
      << "Calibration.imuValidated: 0\nIMU.NoiseGyro: 0.001\nIMU.NoiseAcc: 0.01\n"
      << "IMU.GyroWalk: 0.000001\nIMU.AccWalk: 0.0001\n"
      << "ORBextractor.nFeatures: 1000\nORBextractor.scaleFactor: 1.2\nORBextractor.nLevels: 8\n"
      << "ORBextractor.iniThFAST: 20\nORBextractor.minThFAST: 7\n"
      << "Viewer.KeyFrameSize: 0.05\nViewer.KeyFrameLineWidth: 1.0\nViewer.GraphLineWidth: 0.9\n"
      << "Viewer.PointSize: 2.0\nViewer.CameraSize: 0.08\nViewer.CameraLineWidth: 3.0\n"
      << "Viewer.ViewpointX: 0.0\nViewer.ViewpointY: -0.7\nViewer.ViewpointZ: -3.5\nViewer.ViewpointF: 500.0\n";
    if(!s) throw std::runtime_error("Could not write ORB settings");
}

int main(int argc,char**argv) try {
    if(argc<3) {
        std::cerr<<"Usage: gemini_record OUTPUT_DIRECTORY SECONDS [WIDTH HEIGHT] [--diagnostic] [--boot-timed | --qr-trigger] [--power-profile 15W --cube-feedback-port 14601 --idle-seconds 600]\n"
                 <<"Records rectified stereo + 200 Hz IMU; output directory must not exist.\n";
        return 2;
    }
    cv::setNumThreads(1);
    fs::path output=argv[1]; double seconds=std::stod(argv[2]);
    int width=640,height=400,index=3;bool diagnostic=false,qrTriggered=false,bootTimed=false;double idleSeconds=600;
    std::string requiredProfile="any";
    if(index<argc && std::string(argv[index]).rfind("--",0)!=0){if(index+1>=argc)throw std::runtime_error("Specify both dimensions");width=std::stoi(argv[index++]);height=std::stoi(argv[index++]);}
    for(;index<argc;++index){std::string key=argv[index];if(key=="--diagnostic"){diagnostic=true;continue;}if(key=="--qr-trigger"){qrTriggered=true;continue;}if(key=="--boot-timed"){bootTimed=true;continue;}
        if(index+1>=argc)throw std::runtime_error("Missing option value");std::string value=argv[++index];
        if(key=="--cube-feedback-port")feedbackPort=std::stoi(value);else if(key=="--power-profile")requiredProfile=value;
        else if(key=="--idle-seconds")idleSeconds=std::stod(value);else if(key=="--seconds")seconds=std::stod(value);else throw std::runtime_error("Unknown option "+key);}
    if(!std::isfinite(seconds) || seconds<=0 || width<=0 || height<=0)
        throw std::runtime_error("Finite positive duration and dimensions required");
    if(fs::exists(output)) throw std::runtime_error("Output already exists; refusing to overwrite it");
    if(bootTimed && qrTriggered)throw std::runtime_error("Timed boot capture cannot use QR triggering");
    if((diagnostic && seconds>180) || !std::isfinite(idleSeconds) || idleSeconds<=0 || idleSeconds>3600 || feedbackPort<0 || feedbackPort>65535)
        throw std::runtime_error("Invalid diagnostic duration/idle/feedback limit");
    auto profile=bhu::bench::power_profile();
    if(requiredProfile!="any" && profile!=requiredProfile)throw std::runtime_error("Power profile is "+profile+"; expected "+requiredProfile);
    auto ancestor=fs::absolute(output).parent_path();while(!fs::exists(ancestor))ancestor=ancestor.parent_path();
    uint64_t estimate=uint64_t(seconds*30*width*height*(diagnostic?4:2));
    if(diagnostic && fs::space(ancestor).available<estimate+1024ull*1024*1024)throw std::runtime_error("Insufficient free space for lossless diagnostic recording plus 1 GiB reserve");
    bhu::cube::Feedback feedback(feedbackPort);
    std::unique_ptr<bhu::bench::QrScanner> scanner;if(qrTriggered)scanner=std::make_unique<bhu::bench::QrScanner>();
    bhu::bench::QrTrigger trigger;
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
    ob::Context ctx;ctx.setLoggerSeverity(OB_LOG_SEVERITY_WARN);
    auto devices=ctx.queryDeviceList();
    if(devices->getCount()!=1) throw std::runtime_error("Expected exactly one Orbbec camera");
    auto device=devices->getDevice(0);
    if(std::string(device->getDeviceInfo()->getName()).find("336L")==std::string::npos)
        throw std::runtime_error("Expected Gemini 336L");
    Properties props{device,{},{}};
    // Use calibration bound to the actual native profiles. On the tested
    // firmware these are already rectified; the normalization map is identity.
    if(device->isPropertySupported(OB_PROP_IR_RECTIFY_BOOL,OB_PERMISSION_READ_WRITE))
        props.set(OB_PROP_IR_RECTIFY_BOOL,false);
    props.setInt(OB_PROP_LASER_CONTROL_INT,0);
    auto leftSensor=device->getSensor(OB_SENSOR_IR_LEFT);
    auto rightSensor=device->getSensor(OB_SENSOR_IR_RIGHT);
    auto left=leftSensor->getStreamProfileList()->getVideoStreamProfile(width,height,OB_FORMAT_Y8,30);
    auto right=rightSensor->getStreamProfileList()->getVideoStreamProfile(width,height,OB_FORMAT_Y8,30);
    auto accSensor=device->getSensor(OB_SENSOR_ACCEL);
    auto gyroSensor=device->getSensor(OB_SENSOR_GYRO);
    auto acc=accSensor->getStreamProfileList()->getAccelStreamProfile(OB_ACCEL_FS_4g,OB_SAMPLE_RATE_200_HZ);
    auto gyro=gyroSensor->getStreamProfileList()->getGyroStreamProfile(OB_GYRO_FS_1000dps,OB_SAMPLE_RATE_200_HZ);
    auto kl=intrinsic(left), kr=intrinsic(right), dl=distortion(left), dr=distortion(right);
    auto e=left->getExtrinsicTo(right);
    cv::Mat r=rotation(e), t=(cv::Mat_<double>(3,1)<<e.trans[0]*.001,e.trans[1]*.001,e.trans[2]*.001);
    double baseline=cv::norm(t);
    if(baseline<.085 || baseline>.105) throw std::runtime_error("Unexpected stereo baseline");
    cv::Mat r1,r2,p1,p2,q,maplx,maply,maprx,mapry;
    cv::stereoRectify(kl,dl,kr,dr,{width,height},r,t,r1,r2,p1,p2,q,cv::CALIB_ZERO_DISPARITY,0);
    if(p2.at<double>(0,3)>=0 || std::abs(p2.at<double>(1,3))>1e-6)
        throw std::runtime_error("Expected horizontal stereo with positive left-minus-right disparity");
    cv::initUndistortRectifyMap(kl,dl,r1,p1,{width,height},CV_32FC1,maplx,maply);
    cv::initUndistortRectifyMap(kr,dr,r2,p2,{width,height},CV_32FC1,maprx,mapry);
    auto lToG=left->getExtrinsicTo(gyro);
    cv::Mat tbc=cv::Mat::eye(4,4,CV_64F),rbc=rotation(lToG)*r1.t();
    rbc.copyTo(tbc(cv::Rect(0,0,3,3)));
    for(int i=0;i<3;++i) tbc.at<double>(i,3)=lToG.trans[i]*.001;
    cv::Mat rag=rotation(acc->getExtrinsicTo(gyro));
    fs::create_directories(output/"left");fs::create_directory(output/"right");
    if(diagnostic){fs::create_directory(output/"native_left");fs::create_directory(output/"native_right");}
    RecordingStatus status{output,{{"schema",1},{"phase","STARTING"},{"complete",false},{"diagnostic",diagnostic},{"qr_trigger",qrTriggered},
        {"power_profile_start",profile},{"navigation_transmission",false},{"slam_running",false},{"native_imu_units","SDK SI, native sensor axes; not raw ADC"},
        {"width",width},{"height",height},{"fps_requested",30},{"imu_hz_requested",200},{"maximum_seconds",seconds},
        {"mode",bootTimed?"boot_timed":"diagnostic"},
        {"protocol",bootTimed?"Automatic fixed duration; no QR or phase prompts":"10s still, 60s moving, 10s still; QR may stop early"}}};status.save();
    writeSettings(output/"orb.yaml",width,height,30,p1,baseline,tbc);
    cv::FileStorage cal((output/"factory.yaml").string(),cv::FileStorage::WRITE);
    cal<<"K_left"<<kl<<"D_left"<<dl<<"K_right"<<kr<<"D_right"<<dr<<"R_left_to_right"<<r
       <<"t_left_to_right_m"<<t<<"R1"<<r1<<"R2"<<r2<<"P1"<<p1<<"P2"<<p2
       <<"T_body_rectified_left"<<tbc<<"R_accel_to_gyro"<<rag;
    cal<<"serial"<<device->getDeviceInfo()->getSerialNumber()<<"firmware"<<device->getDeviceInfo()->getFirmwareVersion()
       <<"accel_csv_units"<<"m/s^2"<<"gyro_csv_units"<<"rad/s"
       <<"sdk_accel_to_si"<<1.0<<"sdk_gyro_to_si"<<1.0;
    if(diagnostic){
        auto ai=acc->getIntrinsic();auto gi=gyro->getIntrinsic();
        cal<<"R_native_left_to_gyro"<<rotation(lToG)
           <<"accel_scale_misalignment"<<cv::Mat(3,3,CV_64F,ai.scaleMisalignment)
           <<"gyro_scale_misalignment"<<cv::Mat(3,3,CV_64F,gi.scaleMisalignment)
           <<"accel_bias"<<cv::Mat(3,1,CV_64F,ai.bias)<<"gyro_bias"<<cv::Mat(3,1,CV_64F,gi.bias)
           <<"accel_reference_temperature"<<ai.referenceTemp<<"gyro_reference_temperature"<<gi.referenceTemp
           <<"accel_sdk_noise_density"<<ai.noiseDensity<<"gyro_sdk_noise_density"<<gi.noiseDensity
           <<"accel_sdk_random_walk"<<ai.randomWalk<<"gyro_sdk_random_walk"<<gi.randomWalk;
        nlohmann::json properties=nlohmann::json::object();
        for(auto id:{OB_PROP_IR_EXPOSURE_INT,OB_PROP_IR_GAIN_INT,OB_PROP_LASER_CONTROL_INT})
            if(device->isPropertySupported(id,OB_PERMISSION_READ))properties[std::to_string(id)]=device->getIntProperty(id);
        status.data["camera_properties_at_start"]=properties;
        status.data["serial"]=device->getDeviceInfo()->getSerialNumber();status.data["firmware"]=device->getDeviceInfo()->getFirmwareVersion();status.save();
    }
    cal.release();
    std::ofstream accelCsv(output/"accel.csv"),gyroCsv(output/"gyro.csv"),frames(output/"frames.csv"),nativeAccel,nativeGyro,timing,events(output/"events.jsonl");
    if(diagnostic){nativeAccel.open(output/"accel_native.csv");nativeGyro.open(output/"gyro_native.csv");timing.open(output/"capture.csv");
        nativeAccel<<"device_us,system_us,host_arrival_us,index,x,y,z\n"<<std::setprecision(15);
        nativeGyro<<"device_us,system_us,host_arrival_us,index,x,y,z\n"<<std::setprecision(15);
        timing<<"device_s,left_system_us,right_system_us,host_received_us,left_index,right_index,write_ms\n"<<std::setprecision(15);}
    if(!accelCsv || !gyroCsv || !frames || !events || (diagnostic && (!nativeAccel || !nativeGyro || !timing)))throw std::runtime_error("Cannot create recording files");
    auto event=[&](const std::string& name){events<<nlohmann::json({{"event",name},{"host_us",hostUs()}}).dump()<<'\n';events.flush();if(!events)throw std::runtime_error("Cannot write recording event");};
    accelCsv<<"timestamp_s,x,y,z\n"<<std::setprecision(15);
    gyroCsv<<"timestamp_s,x,y,z\n"<<std::setprecision(15);
    frames<<"timestamp_s,left,right,left_us,right_us,left_index,right_index\n"<<std::setprecision(15);
    std::mutex am,gm;bool callbackFailed=false;std::mutex errorMutex;std::atomic<bool> active{!qrTriggered && !bootTimed};
    struct Measurement{uint64_t device,system,host,index;double x,y,z;};
    std::deque<Measurement> accPreroll,gyroPreroll;uint64_t accCount=0,gyroCount=0;
    auto append=[&](const Measurement& m,bool acceleration){
        auto& file=acceleration?accelCsv:gyroCsv;
        cv::Mat value=(cv::Mat_<double>(3,1)<<m.x,m.y,m.z);if(acceleration)value=rag*value;
        file<<m.device*1e-6<<','<<value.at<double>(0)<<','<<value.at<double>(1)<<','<<value.at<double>(2)<<'\n';
        if(diagnostic){auto& raw=acceleration?nativeAccel:nativeGyro;raw<<m.device<<','<<m.system<<','<<m.host<<','<<m.index<<','<<m.x<<','<<m.y<<','<<m.z<<'\n';if(!raw)throw std::runtime_error("Native IMU write failed");}
        if(!file)throw std::runtime_error("IMU write failed");if(acceleration)++accCount;else ++gyroCount;
    };
    auto onAccel=[&](std::shared_ptr<ob::Frame> f) {
        try {
            auto v=f->as<ob::AccelFrame>()->getValue();
            // The public SDK API supplies SI units (see bundled IMU README).
            // Internal decoder units are not the public frame-value contract.
            Measurement m{f->getTimeStampUs(),f->getSystemTimeStampUs(),hostUs(),f->getIndex(),v.x,v.y,v.z};
            std::lock_guard<std::mutex> lock(am);
            if(!active){accPreroll.push_back(m);while(accPreroll.size()>100)accPreroll.pop_front();return;}
            for(const auto& previous:accPreroll)append(previous,true);accPreroll.clear();append(m,true);
        } catch(...) {std::lock_guard<std::mutex> lock(errorMutex);callbackFailed=true;}
    };
    auto onGyro=[&](std::shared_ptr<ob::Frame> f) {
        try {
            auto v=f->as<ob::GyroFrame>()->getValue();
            Measurement m{f->getTimeStampUs(),f->getSystemTimeStampUs(),hostUs(),f->getIndex(),v.x,v.y,v.z};
            std::lock_guard<std::mutex> lock(gm);
            if(!active){gyroPreroll.push_back(m);while(gyroPreroll.size()>100)gyroPreroll.pop_front();return;}
            for(const auto& previous:gyroPreroll)append(previous,false);gyroPreroll.clear();append(m,false);
        } catch(...) {std::lock_guard<std::mutex> lock(errorMutex);callbackFailed=true;}
    };
    // Sensors outlive callbacks/files; stop them explicitly on every exit path.
    struct SensorStop {
        std::vector<std::shared_ptr<ob::Sensor>> started;
        ~SensorStop(){for(auto& s:started)try{s->stop();}catch(...) {}}
    } sensorStop;
    accSensor->start(acc,onAccel);sensorStop.started.push_back(accSensor);
    gyroSensor->start(gyro,onGyro);sensorStop.started.push_back(gyroSensor);
    auto config=std::make_shared<ob::Config>();config->enableStream(left);config->enableStream(right);
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    ob::Pipeline video(device);video.enableFrameSync();video.start(config);
    struct VideoStop{ob::Pipeline& pipeline;bool on=true;void finish(){if(on){pipeline.stop();on=false;}}~VideoStop(){if(on)try{pipeline.stop();}catch(...){}}} videoStop{video};
    uint64_t armedHost=hostUs(),startedHost=(qrTriggered || bootTimed)?0:armedHost,nextQr=0,lastDevice=0,lastLeftIndex=0,lastRightIndex=0,indexGaps=0;
    size_t count=0,rejected=0;bool moveCue=false,settleCue=false;std::string reason="duration_limit";
    if(qrTriggered){status.data["phase"]="ARMED";status.save();event("ARMED");feedback.send("ready");std::cout<<"ARMED: show test_vio QR. Start beep means remain stationary for 10 seconds."<<std::endl;}
    else if(!bootTimed){status.data["phase"]="RECORDING";status.save();event("TEST_START");}
    else {status.data["phase"]="WAITING_FIRST_FRAME";status.save();}
    while(running) {
        auto now=hostUs();
        if(startedHost && (now-startedHost)*1e-6>=seconds)break;
        if(!startedHost && (now-armedHost)*1e-6>=idleSeconds){reason="idle_timeout";break;}
        if(diagnostic && startedHost){
            double elapsed=(now-startedHost)*1e-6;
            if(!bootTimed && elapsed>=10 && !moveCue){moveCue=true;event("MOVE");feedback.send("move");std::cout<<"MOVE: smooth translations and gentle rotations for 60 seconds."<<std::endl;}
            if(!bootTimed && elapsed>=70 && !settleCue){settleCue=true;event("SETTLE");feedback.send("settle");std::cout<<"SETTLE: place camera still for final 10 seconds."<<std::endl;}
            if(count%30==0 && fs::space(output).available<256ull*1024*1024)throw std::runtime_error("Recording stopped: less than 256 MiB free space");
        }
        {std::lock_guard<std::mutex> lock(errorMutex);if(callbackFailed) throw std::runtime_error("IMU callback failed");}
        auto set=video.waitForFrameset(200);if(!set)continue;
        auto received=hostUs();
        auto lf=set->getFrame(OB_FRAME_IR_LEFT),rf=set->getFrame(OB_FRAME_IR_RIGHT);
        if(!lf||!rf)continue;
        auto lt=lf->getTimeStampUs(),rt=rf->getTimeStampUs();
        if(!lt || !rt || std::abs(static_cast<double>(lt)-static_cast<double>(rt))>500) {++rejected;continue;}
        auto lv=lf->as<ob::VideoFrame>(),rv=rf->as<ob::VideoFrame>();
        if(lf->getFormat()!=OB_FORMAT_Y8 || rf->getFormat()!=OB_FORMAT_Y8
           || lv->getWidth()!=static_cast<uint32_t>(width) || lv->getHeight()!=static_cast<uint32_t>(height)
           || rv->getWidth()!=lv->getWidth() || rv->getHeight()!=lv->getHeight()
           || lf->getDataSize()!=static_cast<size_t>(width)*height || rf->getDataSize()!=lf->getDataSize())
            throw std::runtime_error("Unexpected Y8 format or buffer size/stride");
        cv::Mat li(height,width,CV_8UC1,lf->getData()),ri(height,width,CV_8UC1,rf->getData()),lo,ro;
        cv::remap(li,lo,maplx,maply,cv::INTER_LINEAR);cv::remap(ri,ro,maprx,mapry,cv::INTER_LINEAR);
        if(bootTimed && !startedHost){startedHost=hostUs();active=true;status.data.update({{"phase","RECORDING"},{"start_host_us",startedHost},{"start_device_s",lt*1e-6}});status.save();event("TEST_START");feedback.send("record_start");std::cout<<"TEST_START: automatic timed recording, "<<seconds<<" seconds."<<std::endl;}
        if(qrTriggered && hostUs()>=nextQr){bool present=scanner->matches(lo);nextQr=hostUs()+500000;auto triggered=trigger.observe(present,hostUs()/1000);
            if(triggered==bhu::bench::TriggerEvent::Start){startedHost=hostUs();active=true;status.data.update({{"phase","RECORDING"},{"start_host_us",startedHost},{"start_device_s",lt*1e-6}});status.save();event("TEST_START");feedback.send("start");std::cout<<"TEST_START: hold still for 10 seconds."<<std::endl;}
            if(triggered==bhu::bench::TriggerEvent::Stop){reason="qr_stop";break;}}
        if(!startedHost)continue;
        if(lastDevice && lt<=lastDevice)throw std::runtime_error("Nonmonotonic image device timestamp");
        if(count){if(lf->getIndex()>lastLeftIndex+1)indexGaps+=lf->getIndex()-lastLeftIndex-1;if(rf->getIndex()>lastRightIndex+1)indexGaps+=rf->getIndex()-lastRightIndex-1;}
        lastDevice=lt;lastLeftIndex=lf->getIndex();lastRightIndex=rf->getIndex();
        if(!count){status.data["first_frame_device_s"]=lt*1e-6;status.save();}
        auto name=std::to_string(count)+(diagnostic?".pgm":".png");auto writeStart=hostUs();
        if(diagnostic){pgm(output/"native_left"/name,li);pgm(output/"native_right"/name,ri);pgm(output/"left"/name,lo);pgm(output/"right"/name,ro);
            timing<<lt*1e-6<<','<<lf->getSystemTimeStampUs()<<','<<rf->getSystemTimeStampUs()<<','<<received<<','<<lf->getIndex()<<','<<rf->getIndex()<<','<<(hostUs()-writeStart)*.001<<'\n';}
        else if(!cv::imwrite((output/"left"/name).string(),lo) || !cv::imwrite((output/"right"/name).string(),ro))throw std::runtime_error("Image write failed");
        frames<<lt*1e-6<<",left/"<<name<<",right/"<<name<<','<<lt<<','<<rt<<','<<lf->getIndex()<<','<<rf->getIndex()<<'\n';
        ++count;
    }
    auto stopHost=hostUs();if(!running)reason="signal";
    videoStop.finish();
    if(startedHost){event("TEST_STOP");if(!bootTimed)feedback.send("stop");std::this_thread::sleep_for(std::chrono::milliseconds(200));}
    for(auto& s:sensorStop.started)s->stop();sensorStop.started.clear();
    accelCsv.flush();gyroCsv.flush();frames.flush();if(diagnostic){nativeAccel.flush();nativeGyro.flush();timing.flush();}
    if(!accelCsv || !gyroCsv || !frames || (diagnostic && (!nativeAccel || !nativeGyro || !timing))) throw std::runtime_error("Recording write failed");
    double elapsed=startedHost?(stopHost-startedHost)*1e-6:0;
    status.data.update({{"phase",count>=2?"SAVED":"INCOMPLETE"},{"complete",count>=2},{"stop_reason",reason},{"duration_s",elapsed},
        {"stereo_pairs",count},{"stereo_timestamp_rejections",rejected},{"combined_left_right_index_gaps",indexGaps},
        {"accel_samples",accCount},{"gyro_samples",gyroCount},{"last_frame_device_s",lastDevice*1e-6},
        {"protocol_duration_complete",!bootTimed && elapsed>=80},{"timed_duration_complete",bootTimed && reason=="duration_limit" && elapsed>=seconds},{"power_profile_end",bhu::bench::power_profile()}});
    status.save();
    std::cout<<"Recorded "<<count<<" stereo pairs; rejected "<<rejected<<" timestamp mismatches.\n";
    if(count<2){status.complete=true;throw std::runtime_error("Insufficient stereo pairs");}
    event("SAVED");
    if(bootTimed){int fd=::open(output.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC);if(fd<0)throw std::runtime_error("Cannot open recording directory for sync");int synced=::syncfs(fd);::close(fd);if(synced!=0)throw std::runtime_error("Recording filesystem sync failed");status.data["filesystem_synced_before_stop_cue"]=true;status.save();int metaFd=::open((output/"recording.json").c_str(),O_RDONLY|O_CLOEXEC);if(metaFd<0)throw std::runtime_error("Cannot sync final recording metadata");int saved=::fsync(metaFd);::close(metaFd);if(saved!=0)throw std::runtime_error("Final metadata sync failed");feedback.send("record_stop");}
    else feedback.send("saved");
    status.complete=true;
    std::cout<<"SAVED: "<<fs::absolute(output)<<std::endl;
    return 0;
}catch(const std::exception& e){try{bhu::cube::Feedback(feedbackPort).send("error");}catch(...){}std::cerr<<e.what()<<'\n';return 1;}
