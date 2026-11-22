#include "System.h"
#include "bhu/camera.hpp"
#include "bhu/external_nav.hpp"
#include "bhu/imu.hpp"
#include "bhu/benchmark.hpp"
#include "bhu/qr_trigger.hpp"
#include "bhu/cube_feedback.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace fs=std::filesystem;
using namespace bhu::camera;
static volatile std::sig_atomic_t running=1;
static void stop(int){running=0;}
static uint64_t clock_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static void need(bool condition,const std::string& message) {if(!condition)throw std::runtime_error(message);}
static int feedback_port=0;

// Own every callback buffer until its sensor/pipeline has stopped, including
// on exceptions. The SDK callbacks never run ORB or perform disk writes.
struct Capture {
    std::mutex mutex;std::condition_variable ready;
    std::deque<bhu::Sample> acc,gyro;
    std::shared_ptr<ob::FrameSet> latest;
    uint64_t arrival=0,overwritten=0;
    std::string error;
    ob::Pipeline& video;
    std::vector<std::shared_ptr<ob::Sensor>> sensors;
    bool video_started=false;
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
                cv::Mat a=R_A_G*(cv::Mat_<double>(3,1)<<v.x,v.y,v.z);
                s.v={a.at<double>(0),a.at<double>(1),a.at<double>(2)};
            } else {auto v=f->as<ob::GyroFrame>()->getValue();s.v={v.x,v.y,v.z};}
            std::lock_guard<std::mutex> lock(mutex);
            auto& queue=acceleration?acc:gyro;
            if(!queue.empty() && s.t<=queue.back().t)throw std::runtime_error("Nonmonotonic live IMU");
            queue.push_back(s);while(queue.size()>800)queue.pop_front();ready.notify_all();
        }catch(const std::exception& e){std::lock_guard<std::mutex> lock(mutex);error=e.what();ready.notify_all();}
    }
};

int main(int argc,char** argv) try {
    if(argc<5) {
        std::cerr<<"Usage: gemini_live VOCABULARY CAMERA.yaml NAV.yaml NEW_OUTPUT [--seconds 10] [--udp-port 14600] [--allow-provisional]\n"
                 <<"[--qr-trigger --power-profile 15W --dashboard-port 8766 --idle-seconds 600]\n"
                 <<"[--cube-test --udp-port 14600 --cube-feedback-port 14601] (explicit QR integration)\n"
                 <<"Default: dry run, CSV and gated MAVLink log only; no UART access.\n";return 2;
    }
    // Install handlers before camera properties change or vocabulary loading
    // begins, so a startup interruption still unwinds reversible settings.
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
    std::signal(SIGPIPE,SIG_IGN);
    double seconds=10,idle_seconds=600;int port=0,dashboard_port=8766;bool provisional=false,qr_trigger=false,cube_test=false;
    std::string required_profile="15W";
    for(int i=5;i<argc;++i) {
        std::string key=argv[i];
        if(key=="--allow-provisional"){provisional=true;continue;}
        if(key=="--qr-trigger"){qr_trigger=true;continue;}
        if(key=="--cube-test"){cube_test=true;continue;}
        need(i+1<argc,"Missing option value");std::string value=argv[++i];
        if(key=="--seconds")seconds=std::stod(value);
        else if(key=="--udp-port")port=std::stoi(value);
        else if(key=="--power-profile")required_profile=value;
        else if(key=="--dashboard-port")dashboard_port=std::stoi(value);
        else if(key=="--idle-seconds")idle_seconds=std::stod(value);
        else if(key=="--cube-feedback-port")feedback_port=std::stoi(value);
        else throw std::runtime_error("Unknown option "+key);
    }
    need(std::isfinite(seconds) && seconds>0 && port>=0 && port<=65535,"Invalid live limits");
    need(!qr_trigger || ((port==0 || cube_test) && seconds<=3600 && std::isfinite(idle_seconds) && idle_seconds>0 && idle_seconds<=3600),"QR navigation requires explicit --cube-test; limits must be <=3600 seconds");
    need(!cube_test || (qr_trigger && port>0 && feedback_port>0),"Cube test requires QR, navigation and feedback ports");
    need(feedback_port>=0 && feedback_port<=65535,"Invalid feedback port");
    bhu::cube::Feedback feedback(feedback_port);
    need(fs::exists(argv[1]),"Missing vocabulary");
    fs::path output=argv[4];need(!fs::exists(output),"Output already exists");
    cv::setNumThreads(1);
    cv::FileStorage settings(argv[2],cv::FileStorage::READ),nav(argv[3],cv::FileStorage::READ);
    need(settings.isOpened() && nav.isOpened(),"Cannot open live settings");
    need(int(settings["Calibration.imuValidated"])==1 || provisional,"IMU calibration is provisional; bench testing requires --allow-provisional");
    need(std::string(settings["Camera.type"])=="Rectified","Live mode requires rectified stereo settings");
    const int width=int(settings["Camera.width"]),height=int(settings["Camera.height"]);
    need(int(settings["Camera.fps"])==30,"Live camera adapter requires 30 FPS profile");
    const double offset=double(settings["IMU.TimeOffsetSeconds"]);
    need(std::isfinite(offset),"Invalid IMU time offset");
    cv::Mat mount,lever;nav["Mount.R_body_optical"]>>mount;nav["Mount.camera_position_body_m"]>>lever;
    need(mount.rows==3 && mount.cols==3 && lever.total()==3,"Missing vehicle mounting transform");
    mount.convertTo(mount,CV_64F);lever.convertTo(lever,CV_64F);
    need(cv::checkRange(lever),"Nonfinite camera lever arm");
    bhu::extnav::Alignment alignment;
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)alignment.R_B_C(i,j)=mount.at<double>(i,j);
    need(bhu::extnav::rotation_valid(alignment.R_B_C),"Invalid camera mount rotation");
    int quality_min=int(nav["Publisher.quality_min"]);
    need(quality_min>=60 && quality_min<=100,"Live quality threshold must be 60..100");
    std::unique_ptr<bhu::bench::Benchmark> benchmark;
    std::unique_ptr<bhu::bench::QrScanner> qr;
    bhu::bench::QrTrigger trigger;
    if(qr_trigger) {
        need(!settings["loopClosing"].empty() && int(settings["loopClosing"])==1,"Benchmark settings must explicitly enable loopClosing");
        benchmark=std::make_unique<bhu::bench::Benchmark>(output,dashboard_port,required_profile);
        benchmark->integration(cube_test,feedback_port>0);
        qr=std::make_unique<bhu::bench::QrScanner>();
        auto loop_events=fs::absolute(output/"loop-events.csv").string();
        need(::setenv("BHU_VIO_LOOP_EVENTS",loop_events.c_str(),1)==0,"Cannot configure loop event log");
    }
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
    cv::Mat t=(cv::Mat_<double>(3,1)<<e.trans[0]*.001,e.trans[1]*.001,e.trans[2]*.001);
    cv::Mat r1,r2,p1,p2,q,lx,ly,rx,ry;
    cv::stereoRectify(intrinsic(left),distortion(left),intrinsic(right),distortion(right),{width,height},rotation(e),t,r1,r2,p1,p2,q,cv::CALIB_ZERO_DISPARITY,0);
    need(p2.at<double>(0,3)<0 && std::abs(p2.at<double>(1,3))<1e-6,"Invalid horizontal stereo geometry");
    need(std::abs(cv::norm(t)-double(settings["Stereo.b"]))<1e-5,"Camera baseline differs from settings");
    for(auto field: {std::make_pair("Camera1.fx",std::make_pair(0,0)),{"Camera1.fy",{1,1}},{"Camera1.cx",{0,2}},{"Camera1.cy",{1,2}}})
        need(std::abs(p1.at<double>(field.second.first,field.second.second)-double(settings[field.first]))<.01,"Camera intrinsics differ from settings");
    cv::initUndistortRectifyMap(intrinsic(left),distortion(left),r1,p1,{width,height},CV_32FC1,lx,ly);
    cv::initUndistortRectifyMap(intrinsic(right),distortion(right),r2,p2,{width,height},CV_32FC1,rx,ry);
    cv::Mat rag=rotation(ap->getExtrinsicTo(gp));
    auto lToG=left->getExtrinsicTo(gp);
    cv::Mat R_G_C=rotation(lToG)*r1.t(),configuredT;
    settings["IMU.T_b_c1"]>>configuredT;configuredT.convertTo(configuredT,CV_64F);
    need(configuredT.rows==4 && configuredT.cols==4 && cv::norm(configuredT(cv::Rect(0,0,3,3))-R_G_C)<1e-5,"Camera/IMU rotation differs from settings");
    for(int i=0;i<3;++i)need(std::abs(configuredT.at<double>(i,3)-lToG.trans[i]*.001)<1e-5,"Camera/IMU translation differs from settings");
    Eigen::Matrix3d R_C_G;
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)R_C_G(i,j)=R_G_C.at<double>(j,i);
    fs::create_directories(output);
    fs::copy_file(argv[2],output/"camera.yaml");fs::copy_file(argv[3],output/"navigation.yaml");
    std::ofstream csv(output/"tracking.csv"),wire(output/"odometry.bin",std::ios::binary);
    need(bool(csv) && bool(wire),"Cannot create live logs");
    csv<<"device_s,host_capture_us,state,imu_initialized,quality,tracking_ms,age_ms,eligible,x,y,z,qw,qx,qy,qz\n"<<std::setprecision(15);
    struct Socket {int fd=-1;~Socket(){if(fd>=0)::close(fd);}} udp;
    sockaddr_in destination{};destination.sin_family=AF_INET;destination.sin_port=htons(uint16_t(port));destination.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    if(port){udp.fd=::socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);need(udp.fd>=0,"Cannot open publisher UDP socket");}
    ORB_SLAM3::System slam(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO,false);
    struct Shutdown {ORB_SLAM3::System& slam;bool done=false;void finish(){if(!done){slam.Shutdown();done=true;}}~Shutdown(){finish();}} shutdown{slam};
    ob::Pipeline video(device);Capture capture(video);
    as->start(ap,[&](std::shared_ptr<ob::Frame> f){capture.sample(true,f,rag,offset);});capture.sensors.push_back(as);
    gs->start(gp,[&](std::shared_ptr<ob::Frame> f){capture.sample(false,f,rag,offset);});capture.sensors.push_back(gs);
    auto config=std::make_shared<ob::Config>();config->enableStream(left);config->enableStream(right);
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);video.enableFrameSync();
    video.start(config,[&](std::shared_ptr<ob::FrameSet> frames){
        auto arrival=clock_us();std::lock_guard<std::mutex> lock(capture.mutex);
        if(capture.latest)++capture.overwritten;
        capture.latest=std::move(frames);capture.arrival=arrival;capture.ready.notify_all();
    });capture.video_started=true;
    bhu::extnav::Gate gate;double start=clock_us()*1e-6,last_frame=0,last_imu=0,previous_pose_time=0;
    int64_t clock_offset=std::numeric_limits<int64_t>::max();
    Eigen::Vector3d previous_position=Eigen::Vector3d::Zero();
    bool aligned=false;uint64_t epoch=0,frames=0,tracked=0,eligible=0;
    uint64_t next_qr=0,overwrites_at_start=0,plot_segment=0;
    int last_plot_state=-1;bool last_plot_initialized=false;
    std::string stop_reason="duration_limit";
    if(benchmark){benchmark->status("ARMED");feedback.send("ready");std::cout<<"ARMED: present test_vio QR to start; dashboard http://127.0.0.1:"<<dashboard_port<<std::endl;}
    bool navigation_active=false;uint64_t last_navigation_cue=0;
    std::cout<<"Live stereo-inertial capture started; "<<(port?"bench UDP enabled":"dry run, no navigation transmission")<<std::endl;
    while(running && (benchmark ? (benchmark->started()?benchmark->elapsed(clock_us())<seconds:clock_us()*1e-6-start<idle_seconds) : clock_us()*1e-6-start<seconds)) {
        std::shared_ptr<ob::FrameSet> set;uint64_t arrival=0;
        {
            std::unique_lock<std::mutex> lock(capture.mutex);
            capture.ready.wait_for(lock,std::chrono::milliseconds(200),[&]{return capture.latest || !capture.error.empty();});
            need(capture.error.empty(),capture.error);if(!capture.latest)continue;
            set=std::move(capture.latest);arrival=capture.arrival;
        }
        const auto iteration_start=clock_us();
        auto lf=set->getFrame(OB_FRAME_IR_LEFT),rf=set->getFrame(OB_FRAME_IR_RIGHT);
        if(!lf || !rf)continue;
        const auto stamp=lf->getTimeStampUs();const double time=stamp*1e-6;
        need(stamp && (!last_frame || time>last_frame),"Camera device clock reset/nonmonotonic frame");
        need(std::abs(double(stamp)-double(rf->getTimeStampUs()))<=500,"Stereo timestamp skew exceeds limit");
        last_frame=time;
        // Lower envelope of delivery offsets maps the device clock into host
        // monotonic time. USB/exposure delay remains provisional and is logged.
        clock_offset=std::min(clock_offset,int64_t(arrival)-int64_t(stamp));
        const uint64_t host_capture=uint64_t(int64_t(stamp)+clock_offset);
        // While armed, only rectify/decode at the QR cadence. VIO has not
        // started yet; avoid paying the full 30-FPS image-processing cost.
        if(benchmark && !benchmark->started() && clock_us()<next_qr)continue;
        auto l=lf->as<ob::VideoFrame>(),r=rf->as<ob::VideoFrame>();
        need(lf->getFormat()==OB_FORMAT_Y8 && rf->getFormat()==OB_FORMAT_Y8
             && l->getWidth()==unsigned(width) && l->getHeight()==unsigned(height)
             && r->getWidth()==unsigned(width) && r->getHeight()==unsigned(height)
             && lf->getDataSize()==size_t(width)*height && rf->getDataSize()==size_t(width)*height,"Invalid stereo buffer");
        cv::Mat li(height,width,CV_8UC1,lf->getData()),ri(height,width,CV_8UC1,rf->getData()),left_image,right_image;
        cv::remap(li,left_image,lx,ly,cv::INTER_LINEAR);cv::remap(ri,right_image,rx,ry,cv::INTER_LINEAR);
        double qr_ms=0;
        if(benchmark && clock_us()>=next_qr) {
            auto scan_start=clock_us();bool present=qr->matches(left_image);
            qr_ms=(clock_us()-scan_start)*.001;next_qr=clock_us()+500000;
            auto event=trigger.observe(present,clock_us()/1000);
            if(event==bhu::bench::TriggerEvent::Start) {
                benchmark->start(time,clock_us());last_imu=time-.1;
                {std::lock_guard<std::mutex> lock(capture.mutex);overwrites_at_start=capture.overwritten;}
                feedback.send("start");
                std::cout<<"TEST_START: remove QR and begin smooth motion."<<std::endl;
            } else if(event==bhu::bench::TriggerEvent::Stop) {
                feedback.send("stop");stop_reason="qr_stop";std::cout<<"TEST_STOP: capture finished. You can stop moving."<<std::endl;break;
            }
        }
        if(benchmark && !benchmark->started())continue;
        std::vector<bhu::Sample> acc,gyro;
        {
            std::unique_lock<std::mutex> lock(capture.mutex);
            capture.ready.wait_for(lock,std::chrono::milliseconds(20),[&]{return !capture.error.empty() ||
                (capture.acc.size()>1 && capture.gyro.size()>1 && capture.acc.back().t>time+.006 && capture.gyro.back().t>time+.005);});
            need(capture.error.empty(),capture.error);
            acc.assign(capture.acc.begin(),capture.acc.end());gyro.assign(capture.gyro.begin(),capture.gyro.end());
        }
        if(acc.size()<2 || gyro.size()<2)continue;
        auto imu=bhu::synchronize(acc,gyro);
        if(imu.size()<2 || imu.front().t>=time || imu.back().t<=time)continue;
        std::vector<ORB_SLAM3::IMU::Point> batch;
        Eigen::Vector3d camera_gyro=Eigen::Vector3d::Zero();
        for(const auto& m:imu) {
            if(m.t<=last_imu)continue;
            batch.emplace_back(m.a[0],m.a[1],m.a[2],m.w[0],m.w[1],m.w[2],m.t);
            last_imu=m.t;camera_gyro=R_C_G*Eigen::Vector3d(m.w[0],m.w[1],m.w[2]);
            if(m.t>=time)break;
        }
        need(!batch.empty(),"Missing IMU integration batch");
        auto begin=clock_us();auto T_C_W=slam.TrackStereo(left_image,right_image,time,batch);
        double ms=(clock_us()-begin)*.001;int state=slam.GetTrackingState();bool initialized=slam.GetTimeFromIMUInit()>0;
        auto T_W_C=T_C_W.inverse();
        bool changed=slam.MapChanged();if(changed){++epoch;aligned=false;previous_pose_time=0;}
        if(state!=2 || !initialized){aligned=false;previous_pose_time=0;}
        bhu::extnav::Sample sample;sample.capture_host_us=host_capture;sample.reset_epoch=epoch;
        sample.T_W_C.matrix()=T_W_C.matrix().cast<double>();sample.angular_velocity_C=camera_gyro;
        auto matches=slam.GetTrackedMapPoints();
        auto count=std::count_if(matches.begin(),matches.end(),[](auto* point){return point!=nullptr;});
        // Experimental support score: 200 associated map points -> 100. This
        // is not calibrated uncertainty and cannot establish flight readiness.
        sample.quality=std::min(100,int(count/2));sample.tracking_ok=state==2 && sample.quality>=quality_min;
        sample.imu_initialized=initialized;
        if(state==2 && initialized && !aligned) {
            // ORB inertial initialization aligns gravity to -world-Z. Flip to
            // down-positive while preserving a proper rotation, then remove
            // the initial vehicle yaw. ArduPilot VOXL may align this local yaw.
            Eigen::Matrix3d down=Eigen::Vector3d(1,-1,-1).asDiagonal();
            Eigen::Matrix3d body=down*sample.T_W_C.rotation()*alignment.R_B_C.transpose();
            double yaw=std::atan2(body(1,0),body(0,0));
            alignment.T_L_W.linear()=Eigen::AngleAxisd(-yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix()*down;
            alignment.T_L_W.translation()=-alignment.T_L_W.rotation()*sample.T_W_C.translation();
            aligned=true;++epoch;sample.reset_epoch=epoch;
        }
        sample.frame_alignment_valid=aligned;
        if(state==2 && initialized) {
            double dt=time-previous_pose_time;
            if(previous_pose_time && dt>0 && dt<=.15) {
                sample.camera_velocity_W=(sample.T_W_C.translation()-previous_position)/dt;
                sample.velocity_valid=sample.camera_velocity_W.allFinite();
            }
            previous_position=sample.T_W_C.translation();previous_pose_time=time;
        }
        auto reset=benchmark && !cube_test?std::optional<uint8_t>{}:gate.accept(sample,clock_us());
        if(cube_test && bool(reset)!=navigation_active && clock_us()-last_navigation_cue>2000000) {
            feedback.send(reset?"valid":"lost");last_navigation_cue=clock_us();
            navigation_active=bool(reset);
        }
        if(reset) {
            auto pose=bhu::extnav::convert(sample,alignment);
            auto message=bhu::extnav::encode(sample,pose,*reset);
            std::array<uint8_t,MAVLINK_MAX_PACKET_LEN> bytes{};auto n=mavlink_msg_to_send_buffer(bytes.data(),&message);
            wire.write(reinterpret_cast<const char*>(bytes.data()),n);need(bool(wire),"Odometry log write failed");
            if(port)need(::sendto(udp.fd,bytes.data(),n,0,reinterpret_cast<sockaddr*>(&destination),sizeof(destination))==n,"UDP odometry send failed");
            ++eligible;
        }
        Eigen::Quaterniond orientation(sample.T_W_C.rotation());auto position=sample.T_W_C.translation();
        csv<<time<<','<<host_capture<<','<<state<<','<<initialized<<','<<sample.quality<<','<<ms<<','
           <<(clock_us()-host_capture)*.001<<','<<bool(reset)<<','<<position.x()<<','<<position.y()<<','<<position.z()<<','
           <<orientation.w()<<','<<orientation.x()<<','<<orientation.y()<<','<<orientation.z()<<'\n';
        need(bool(csv),"Tracking log write failed");++frames;if(state==2)++tracked;
        if(benchmark) {
            if(changed || state!=2 || state!=last_plot_state || initialized!=last_plot_initialized)++plot_segment;
            last_plot_state=state;last_plot_initialized=initialized;
            bhu::bench::Frame frame;
            frame.device_s=time;frame.elapsed_s=benchmark->elapsed(clock_us());frame.tracking_ms=ms;
            frame.age_ms=(clock_us()-host_capture)*.001;frame.qr_ms=qr_ms;frame.wall_ms=(clock_us()-iteration_start)*.001;
            frame.x=position.x();frame.y=position.y();frame.z=position.z();frame.qw=orientation.w();frame.qx=orientation.x();frame.qy=orientation.y();frame.qz=orientation.z();
            frame.state=state;frame.initialized=initialized;frame.quality=sample.quality;frame.segment=plot_segment;frame.index=lf->getIndex();
            benchmark->frame(frame);
        }
        if(frames%100==0)std::cout<<"Processed "<<frames<<"; tracked "<<tracked<<"; initialized "<<initialized<<"; eligible "<<eligible<<std::endl;
    }
    const auto stopped=clock_us();
    capture.finish();
    if(benchmark) {
        if(!running)stop_reason="signal";
        else if(!benchmark->started())stop_reason="idle_timeout";
        if(stop_reason!="qr_stop")feedback.send(benchmark->started()?"stop":"error");
        benchmark->stop(stop_reason,stopped,capture.overwritten-overwrites_at_start);
        // Let pending loop candidates finish while capture is already stopped.
        if(frames)std::this_thread::sleep_for(std::chrono::seconds(2));
        shutdown.finish();
        if(tracked) {
            slam.SaveKeyFrameTrajectoryTUM((output/"keyframes_optimized.txt").string());
            if(fs::file_size(output/"keyframes_optimized.txt"))slam.SaveTrajectoryTUM((output/"trajectory_optimized.txt").string());
        }
        std::cout.flush();csv.flush();wire.flush();
        bhu::bench::create_report(output);benchmark->status("COMPLETE");
        if(frames)feedback.send("saved");
        std::cout<<"Report: "<<fs::absolute(output/"report.html")<<std::endl;
    }
    std::ofstream summary(output/"summary.txt");
    {std::lock_guard<std::mutex> lock(capture.mutex);summary<<"overwritten_video_frames="<<capture.overwritten<<'\n';}
    summary<<"processed="<<frames<<"\ntracked="<<tracked<<"\neligible="<<eligible<<"\nudp_enabled="<<bool(port)<<"\nprovisional="<<provisional<<'\n';
    std::cout<<"Live capture finished. Processed "<<frames<<"; tracked "<<tracked<<"; gated odometry "<<eligible<<'\n';
    return frames?0:1;
}catch(const std::exception& e){try{bhu::cube::Feedback(feedback_port).send("error");}catch(...){}std::cerr<<e.what()<<'\n';return 1;}
