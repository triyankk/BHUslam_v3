#include "bhu/qr_trigger.hpp"
#include "bhu/benchmark.hpp"
#include <json.hpp>
#include <qrencode.h>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>

using namespace bhu::bench;
static void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
static cv::Mat fixture(const char* payload){
    auto* qr=QRcode_encodeString8bit(payload,0,QR_ECLEVEL_M);require(qr,"Encode QR fixture");
    cv::Mat gray(qr->width+8,qr->width+8,CV_8UC1,cv::Scalar(255));
    for(int y=0;y<qr->width;++y)for(int x=0;x<qr->width;++x)if(qr->data[y*qr->width+x]&1)gray.at<uint8_t>(y+4,x+4)=0;
    QRcode_free(qr);cv::resize(gray,gray,{},7,7,cv::INTER_NEAREST);return gray;
}
int main()try {
    QrScanner scanner;auto correct=fixture("test_vio");
    require(scanner.matches(correct),"Decode exact QR payload");
    cv::Mat rotated;cv::rotate(correct,rotated,cv::ROTATE_90_CLOCKWISE);
    require(scanner.matches(rotated),"Decode rotated QR");
    require(!scanner.matches(fixture("TEST_VIO")),"Reject wrong case");
    require(!scanner.matches(fixture("test_vio ")),"Reject trailing payload");
    require(!scanner.matches(cv::Mat(400,640,CV_8UC1,cv::Scalar(127))),"Reject no QR");
    QrTrigger trigger;
    require(trigger.observe(true,1000)==TriggerEvent::None,"Single detection cannot start");
    trigger.observe(false,1500);
    require(trigger.observe(true,2000)==TriggerEvent::None,"Lost confirmation resets streak");
    require(trigger.observe(true,2500)==TriggerEvent::Start,"Two detections start");
    for(uint64_t t=3000;t<7500;t+=500)require(trigger.observe(true,t)==TriggerEvent::None,"Held QR cannot stop");
    trigger.observe(false,7500);trigger.observe(false,9500);
    require(trigger.observe(true,10000)==TriggerEvent::None,"Single return does not stop");
    require(trigger.observe(true,10500)==TriggerEvent::Stop,"Remove then re-present stops");
    require(trigger.observe(true,11000)==TriggerEvent::None,"Completed run cannot restart");
    QrTrigger early;early.observe(true,1000);early.observe(true,1500);early.observe(false,1600);early.observe(false,3600);
    early.observe(true,4000);require(early.observe(true,4500)==TriggerEvent::None,"Minimum run time enforced");
    auto directory=std::filesystem::temp_directory_path()/("bhu-qr-test-"+std::to_string(::getpid()));
    require(std::filesystem::create_directory(directory),"Unique report fixture directory");
    std::ofstream(directory/"run.json")<<R"({"power_profile_start":"FIXTURE","duration_s":2,"stop_reason":"qr_stop"})";
    std::ofstream f(directory/"frames.csv");f<<"device_s,elapsed_s,state,imu_initialized,quality,tracking_ms,age_ms,qr_ms,wall_ms,segment,frame_index,x,y,z,qw,qx,qy,qz\n"
       <<"10,0,2,0,80,10,20,2,12,0,1,0,0,0,1,0,0,0\n"
       <<"11,1,2,1,80,30,40,0,31,1,2,1,2,3,1,0,0,0\n"
       <<"12,2,1,0,0,20,30,0,21,2,3,0,0,0,1,0,0,0\n";f.close();
    std::ofstream(directory/"resources.csv")<<"elapsed_s,process_cpu_pct,rss_mb,peak_rss_mb,input_w,cpu_khz,temperature_max_c\n0,100,300,400,8,1000000,40\n1,200,320,400,10,1000000,41\n";
    std::ofstream(directory/"console.log")<<"*Loop detected\nBAD LOOP!!!\nLM: Active map reset, Done!!!\n";
    std::ofstream(directory/"trajectory_optimized.txt")<<"10 0 0 0 0 0 0 1\n11 1 2 3 0 0 0 1\n12 0 0 0 0 0 0 1\n";
    create_report(directory);nlohmann::json m;std::ifstream(directory/"metrics.json")>>m;
    require(m["frames"]==3 && m["tracked"]==2 && m["initialized_frames"]==1,"Report coverage");
    require(m["tracking_ms"]["p50"]==20 && m["tracking_ms"]["p95"]==29,"Timing percentiles");
    require(m["accepted_loop_closures"]==0 && m["active_map_resets"]==1,"Rejected loop candidate must not count");
    require(m["sampled_input_energy_j"]==9 && m["energy_sample_coverage_s"]==1,"Rail energy integration and coverage");
    require(!m["imu_initialized_at_end"].get<bool>(),"Do not hide failed final initialization");
    nlohmann::json graph;std::ifstream(directory/"plot_data.json")>>graph;
    require(graph["optimized_points"].size()==2,"Optimized graph omits lost frames");
    require(!graph["optimized_points"][0][5].get<bool>() && graph["optimized_points"][1][5].get<bool>(),"Optimized graph preserves IMU validity");
    require(graph["optimized_points"][0][4]!=graph["optimized_points"][1][4],"Optimized graph preserves segment gaps");
    std::ofstream(directory/"loop-events.csv")<<"11,4,1\n";create_report(directory);
    std::ifstream(directory/"metrics.json")>>m;require(m["accepted_loop_closures"]==1,"Confirmed loop event counts");
    require(std::filesystem::file_size(directory/"report.html")>2000,"Standalone plot report");
    auto telemetry=directory/"telemetry";
    auto host_us=[] {return uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());};
    {
        Benchmark benchmark(telemetry,0,power_profile());
        benchmark.start(20,host_us());
        Frame frame;frame.device_s=20;frame.state=2;frame.initialized=true;frame.tracking_ms=5;benchmark.frame(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(1300));
        benchmark.stop("fixture",host_us(),0);
    }
    create_report(telemetry);
    std::ifstream(telemetry/"metrics.json")>>m;
    require(m["rss_mb"]["p50"].is_number() && m["rss_mb"]["p50"].get<double>()>0,"Resource thread records actual process memory");
    require(std::filesystem::exists(telemetry/"trajectory.png"),"Native static trajectory PNG");
    std::filesystem::remove_all(directory);
    std::cout<<"PASS: native QR decode, strict payload, start/stop debounce, report coverage, rejected-loop handling and resource metrics\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
