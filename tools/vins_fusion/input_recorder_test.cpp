#include "input_recorder.hpp"
#include <opencv2/imgcodecs.hpp>
#include <unistd.h>
#include <iostream>
static void check(bool value){if(!value)throw std::runtime_error("Recording check failed");}
static unsigned lines(const std::filesystem::path& p){std::ifstream f(p);unsigned n=0;std::string s;while(std::getline(f,s))++n;return n;}
int main(){
 auto root=std::filesystem::temp_directory_path()/("fusion-record-test-"+std::to_string(getpid()));std::filesystem::remove_all(root);
 FusionInputRecorder recorder(root);cv::Mat left(4,6,CV_8UC1,cv::Scalar(37)),right(4,6,CV_8UC1,cv::Scalar(91));
 check(!bool(recorder.status()["recording_ready"]));
 for(unsigned i=0;i<40;i++){recorder.imu(true,.95+i*.005,{1,2,3});recorder.imu(false,.95+i*.005,{.1,.2,.3});}
 for(unsigned i=0;i<4;i++)recorder.stereo(1+i*.03,1000000+i*30000,1000100+i*30000,2000000+i*30000,left,right);
 left.setTo(0);right.setTo(0);recorder.finish();auto state=recorder.status();
 check(state["frames_written"]==4&&state["imu_written"]==80&&state["dropped_frames"]==0&&state["dropped_imu"]==0&&state["error"]=="");
 check(state["recording_ready"]==true);
 check(lines(root/"accel.csv")==41&&lines(root/"gyro.csv")==41&&lines(root/"frames.csv")==5);
 {std::ifstream f(root/"frames.csv");std::string text((std::istreambuf_iterator<char>(f)),{});check(text.find(",1000000,1000100\n")!=std::string::npos);}
 auto l=cv::imread((root/"left/1000000.pgm").string(),cv::IMREAD_GRAYSCALE),r=cv::imread((root/"right/1000000.pgm").string(),cv::IMREAD_GRAYSCALE);
 check(l.size()==cv::Size(6,4)&&r.size()==l.size()&&cv::countNonZero(l!=37)==0&&cv::countNonZero(r!=91)==0);
 check(nlohmann::json::parse(std::ifstream(root/"recording.json"))["finalized"]==true);
 std::filesystem::remove_all(root);std::cout<<"PASS: asynchronous frame ownership, full IMU CSV, drain/finalization and replay image format\n";
}
