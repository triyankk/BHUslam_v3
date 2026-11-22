#pragma once
#include <opencv2/core.hpp>
#include <json.hpp>
#include <array>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <thread>
#include <vector>
#include <stdexcept>
#include <chrono>

// A bounded disk writer keeps recording latency out of sensor callbacks and
// estimator work. Overflow is explicit and does not block VIO.
class FusionInputRecorder {
    struct Item {
        int type=0,width=0,height=0;
        double t=0;
        uint64_t stamp=0,rightStamp=0,host=0;
        std::array<double,3> value{};
        std::vector<uint8_t> left,right;
    };
    std::filesystem::path root_;
    std::ofstream acc_,gyro_,frames_,capture_;
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<Item> queue_;
    std::thread worker_;
    bool stopping_=false;
    unsigned queuedFrames_=0;
    std::string error_;
    std::atomic<uint64_t> droppedFrames_{0},droppedImu_{0},writtenFrames_{0},writtenImu_{0};
    bool recordingReady_=false;
    bool haveAccel_=false,haveGyro_=false;
    double firstAccel_=-1,firstGyro_=-1;
    static void pgm(const std::filesystem::path& path,const std::vector<uint8_t>& data,int width,int height) {
        std::ofstream file(path,std::ios::binary);file<<"P5\n"<<width<<' '<<height<<"\n255\n";
        file.write(reinterpret_cast<const char*>(data.data()),data.size());file.flush();
        if(!file)throw std::runtime_error("Image write failed");
    }
    void flush(){acc_.flush();gyro_.flush();frames_.flush();capture_.flush();if(!acc_||!gyro_||!frames_||!capture_)throw std::runtime_error("Input CSV write failed");}
    void work() noexcept {
        try {
            auto nextSpaceCheck=std::chrono::steady_clock::now();
            for(;;){
                std::deque<Item> batch;
                {std::unique_lock<std::mutex> lock(mutex_);ready_.wait_for(lock,std::chrono::milliseconds(250),[&]{return stopping_||!queue_.empty();});
                 if(stopping_&&queue_.empty())break;batch.swap(queue_);queuedFrames_=0;}
                if(std::chrono::steady_clock::now()>=nextSpaceCheck){
                    if(std::filesystem::space(root_).available<2ull*1024*1024*1024)throw std::runtime_error("Input recording stopped: less than 2 GiB free");
                    nextSpaceCheck=std::chrono::steady_clock::now()+std::chrono::seconds(1);
                }
                for(const auto& item:batch){
                    if(item.type<2){auto& file=item.type==0?acc_:gyro_;file<<item.t;for(double v:item.value)file<<','<<v;file<<'\n';++writtenImu_;if(item.type==0)haveAccel_=true;else haveGyro_=true;}
                    else{auto name=std::to_string(item.stamp)+".pgm";pgm(root_/"left"/name,item.left,item.width,item.height);pgm(root_/"right"/name,item.right,item.width,item.height);
                         frames_<<item.t<<",left/"<<name<<",right/"<<name<<','<<item.stamp<<','<<item.rightStamp<<"\n";capture_<<item.t<<','<<item.host<<'\n';++writtenFrames_;}
                }
                flush();
                {std::lock_guard<std::mutex> lock(mutex_);recordingReady_=haveAccel_&&haveGyro_&&writtenFrames_>0;}
            }
            flush();
        }catch(const std::exception& e){std::lock_guard<std::mutex> lock(mutex_);error_=e.what();}
    }
public:
    explicit FusionInputRecorder(const std::filesystem::path& root):root_(root){
        std::filesystem::create_directories(root_/"left");std::filesystem::create_directories(root_/"right");
        acc_.open(root_/"accel.csv");gyro_.open(root_/"gyro.csv");frames_.open(root_/"frames.csv");capture_.open(root_/"capture.csv");
        acc_<<"timestamp_s,x,y,z\n"<<std::setprecision(16);gyro_<<"timestamp_s,x,y,z\n"<<std::setprecision(16);
        frames_<<"timestamp_s,left,right,left_us,right_us\n"<<std::setprecision(16);capture_<<"device_s,host_capture_us\n"<<std::setprecision(16);flush();
        std::ofstream(root_/"recording.json")<<"{\"finalized\":false,\"images\":\"selected processed stereo pairs, rectified before CLAHE\"}\n";
        worker_=std::thread([this]{work();});
    }
    ~FusionInputRecorder(){finish();}
    void imu(bool acceleration,double t,const std::array<double,3>& value){
        std::lock_guard<std::mutex> lock(mutex_);if(stopping_||!error_.empty()||queue_.size()>=8192){++droppedImu_;return;}
        auto& first=acceleration?firstAccel_:firstGyro_;if(first<0)first=t;
        Item item;item.type=acceleration?0:1;item.t=t;item.value=value;queue_.push_back(std::move(item));ready_.notify_one();
    }
    void stereo(double t,uint64_t stamp,uint64_t rightStamp,uint64_t host,const cv::Mat& left,const cv::Mat& right){
        if(left.type()!=CV_8UC1||right.type()!=CV_8UC1||left.size()!=right.size()||!left.isContinuous()||!right.isContinuous())throw std::runtime_error("Recorder requires continuous stereo Y8");
        std::lock_guard<std::mutex> lock(mutex_);if(stopping_||!error_.empty()||queuedFrames_>=8||queue_.size()>=8192){++droppedFrames_;return;}
        // Do not save a pending pre-arm image before the recording's IMU start.
        if(firstAccel_<0||firstGyro_<0||t<=std::max(firstAccel_,firstGyro_)+.005)return;
        Item item;item.type=2;item.t=t;item.stamp=stamp;item.rightStamp=rightStamp;item.host=host;item.width=left.cols;item.height=left.rows;
        item.left.assign(left.data,left.data+left.total());item.right.assign(right.data,right.data+right.total());queue_.push_back(std::move(item));++queuedFrames_;ready_.notify_one();
    }
    nlohmann::json status(){std::lock_guard<std::mutex> lock(mutex_);return {{"recording_ready",recordingReady_&&error_.empty()},{"frames_written",writtenFrames_.load()},{"imu_written",writtenImu_.load()},{"dropped_frames",droppedFrames_.load()},{"dropped_imu",droppedImu_.load()},{"error",error_},{"queue_items",queue_.size()}};}
    void finish() noexcept {
        if(!worker_.joinable())return;
        {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}ready_.notify_all();worker_.join();
        try{auto state=status();state["finalized"]=state["error"]=="";state["images"]="selected processed stereo pairs, rectified before CLAHE; overwritten camera pairs are absent";state["imu_units"]="m/s^2 and rad/s, acceleration rotated to gyro frame";std::ofstream(root_/"recording.json")<<state.dump(2)<<'\n';}catch(...){}
    }
};
