// Offline image diagnostics only; never changes capture or VINS settings.
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <set>
#include <array>
#include <algorithm>
#include <vector>
using J=nlohmann::json;namespace fs=std::filesystem;
static std::vector<std::string> split(const std::string&s){std::stringstream in(s);std::string x;std::vector<std::string> a;while(std::getline(in,x,','))a.push_back(x);return a;}
static double percentile(std::vector<double>v,double q){if(v.empty())return 0;std::sort(v.begin(),v.end());return v[size_t((v.size()-1)*q)];}
static J pixels(const cv::Mat& im){
    if(im.type()!=CV_8UC1||im.empty())throw std::runtime_error("Expected nonempty grayscale image");
    cv::Scalar mean,stddev;cv::meanStdDev(im,mean,stddev);cv::Mat lap;cv::Laplacian(im,lap,CV_64F);cv::Scalar lm,ls;cv::meanStdDev(lap,lm,ls);
    std::array<size_t,256> hist{};size_t dark=0,white=0;for(int y=0;y<im.rows;y++)for(int x=0;x<im.cols;x++){int v=im.at<uchar>(y,x);hist[v]++;dark+=v<=5;white+=v>=250;}
    auto q=[&](double fraction){size_t sum=0;for(int i=0;i<256;i++){sum+=hist[i];if(sum>=size_t(fraction*im.total()))return i;}return 255;};
    return {{"mean",mean[0]},{"stddev",stddev[0]},{"p05",q(.05)},{"p95",q(.95)},{"dark_le5_fraction",double(dark)/im.total()},{"bright_ge250_fraction",double(white)/im.total()},{"laplacian_variance",ls[0]*ls[0]}};
}
static J snapshot(const cv::Mat& l,const cv::Mat&r,cv::Mat* overlay){
    std::vector<cv::Point2f> left,right,back;cv::goodFeaturesToTrack(l,left,150,.01,30);std::set<int> cells;for(const auto&p:left)cells.insert(std::min(2,int(p.y*3/l.rows))*4+std::min(3,int(p.x*4/l.cols)));
    if(overlay){cv::hconcat(l,r,*overlay);cv::cvtColor(*overlay,*overlay,cv::COLOR_GRAY2BGR);}
    size_t accepted=0,geometry=0,forwardFailed=0,reverseFailed=0,borderFailed=0,roundTripFailed=0;std::vector<double> dy,disparity;
    if(!left.empty()){
        std::vector<uchar> status,backStatus;std::vector<float> error;cv::calcOpticalFlowPyrLK(l,r,left,right,status,error,cv::Size(21,21),3);back=left;
        cv::calcOpticalFlowPyrLK(r,l,right,back,backStatus,error,cv::Size(21,21),3,cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,30,.01),cv::OPTFLOW_USE_INITIAL_FLOW);
        for(size_t i=0;i<left.size();i++){
            if(!status[i])++forwardFailed;
            else if(!backStatus[i])++reverseFailed;
            else if(right[i].x<1||right[i].x>=r.cols-1||right[i].y<1||right[i].y>=r.rows-1)++borderFailed;
            else if(cv::norm(left[i]-back[i])>.5)++roundTripFailed;
            bool ok=status[i]&&backStatus[i]&&cv::norm(left[i]-back[i])<=.5&&right[i].x>=1&&right[i].x<r.cols-1&&right[i].y>=1&&right[i].y<r.rows-1;
            if(ok){++accepted;double y=std::abs(left[i].y-right[i].y),d=left[i].x-right[i].x;dy.push_back(y);disparity.push_back(d);geometry+=y<1&&d>.5;}
            if(overlay){cv::circle(*overlay,left[i],3,ok?cv::Scalar(0,220,0):cv::Scalar(0,180,255),1);if(ok)cv::circle(*overlay,right[i]+cv::Point2f(float(l.cols),0),3,{255,180,0},1);}
        }
    }
    return {{"detected_corners",left.size()},{"occupied_grid_cells_4x3",cells.size()},{"forward_lk_failed",forwardFailed},{"reverse_lk_failed",reverseFailed},{"right_border_failed",borderFailed},{"round_trip_gt_half_pixel",roundTripFailed},{"stereo_lk_fb_matches",accepted},{"positive_disparity_epipolar_lt1px",geometry},{"stereo_abs_dy_median_px",percentile(dy,.5)},{"stereo_abs_dy_p95_px",percentile(dy,.95)},{"disparity_median_px",percentile(disparity,.5)}};
}
int main(int argc,char**argv)try{
    if(argc!=3&&argc!=5)throw std::runtime_error("Usage: fusion_image_quality RECORDING NEW_OUTPUT [--samples N]");size_t maxSamples=0;if(argc==5){if(std::string(argv[3])!="--samples")throw std::runtime_error("Unknown sampling option");maxSamples=std::stoul(argv[4]);if(maxSamples<2||maxSamples>10000)throw std::runtime_error("Samples must be 2..10000");}fs::path root=argv[1],out=argv[2];if(fs::exists(out))throw std::runtime_error("Output exists");fs::create_directories(out);cv::setNumThreads(1);
    std::ifstream in(root/"frames.csv");if(!in)throw std::runtime_error("Missing frames.csv");std::string line;std::getline(in,line);std::vector<std::string> frameLines;while(std::getline(in,line))if(!line.empty())frameLines.push_back(line);std::set<size_t> selected;if(maxSamples){size_t n=std::min(maxSamples,frameLines.size());for(size_t i=0;i<n;i++)selected.insert(n==1?0:i*(frameLines.size()-1)/(n-1));}size_t index=0;double first=-1;J rows=J::array();cv::Mat contact;
    for(const auto&frameLine:frameLines){auto c=split(frameLine);if(c.size()!=7)throw std::runtime_error("Invalid frames.csv");if(first<0)first=std::stod(c[0]);const size_t sourceIndex=index++;if(maxSamples?!selected.count(sourceIndex):sourceIndex%15!=0)continue;
        cv::Mat l=cv::imread((root/c[1]).string(),0),r=cv::imread((root/c[2]).string(),0);if(l.empty()||r.empty())throw std::runtime_error("Missing image");cv::Mat overlay,le,re,lm,rm;auto enhanced=cv::createCLAHE(3,cv::Size(8,8));enhanced->apply(l,le);enhanced->apply(r,re);auto mild=cv::createCLAHE(2,cv::Size(8,8));mild->apply(l,lm);mild->apply(r,rm);
        J row={{"source_index",index-1},{"elapsed_s",std::stod(c[0])-first},{"left",pixels(l)},{"right",pixels(r)},{"original_snapshot",snapshot(l,r,&overlay)},{"clahe_snapshot",snapshot(le,re,nullptr)},{"clahe_mild_snapshot",snapshot(lm,rm,nullptr)}};
        rows.push_back(row);
        if((maxSamples&&(rows.size()-1)%std::max<size_t>(1,selected.size()/8)==0)||(!maxSamples&&(index-1)%150==0)){std::string name="stereo-"+std::to_string(index-1)+".jpg";cv::putText(overlay,"t="+std::to_string(row["elapsed_s"].get<double>())+"s  original IR; green=stereo matched, yellow=left only",{10,22},cv::FONT_HERSHEY_SIMPLEX,.55,{0,200,255},1);cv::imwrite((out/name).string(),overlay);cv::Mat thumb;cv::resize(overlay,thumb,{960,300});if(contact.empty())contact=thumb;else cv::vconcat(contact,thumb,contact);}
    }
    if(rows.empty())throw std::runtime_error("No diagnostic samples");if(!contact.empty())cv::imwrite((out/"contact-sheet.jpg").string(),contact);
    J summary={{"recording",fs::absolute(root).string()},{"sampled_pairs",rows.size()},{"sample_stride",maxSamples?0:15},{"sampling",maxSamples?"evenly spaced indices including first/last":"every fifteenth pair"},{"samples_requested",maxSamples},{"clahe_clip_limits",J::array({2,3})},{"snapshot_only",true},{"clahe_is_offline_candidate_only",true}};
    for(auto eye:{"left","right"})for(auto field:{"mean","stddev","p05","p95","dark_le5_fraction","bright_ge250_fraction","laplacian_variance"}){std::vector<double> v;for(const auto&row:rows)v.push_back(row[eye][field]);summary[eye][field]={{"min",percentile(v,0)},{"median",percentile(v,.5)},{"p95",percentile(v,.95)},{"max",percentile(v,1)}};}
    for(auto method:{"original_snapshot","clahe_mild_snapshot","clahe_snapshot"})for(auto field:{"detected_corners","occupied_grid_cells_4x3","forward_lk_failed","reverse_lk_failed","right_border_failed","round_trip_gt_half_pixel","stereo_lk_fb_matches","positive_disparity_epipolar_lt1px","stereo_abs_dy_median_px","stereo_abs_dy_p95_px","disparity_median_px"}){std::vector<double>v;for(const auto&row:rows)v.push_back(row[method][field]);summary[method][field]={{"median",percentile(v,.5)},{"p95",percentile(v,.95)}};}
    std::ofstream(out/"samples.json")<<rows.dump(2)<<'\n';std::ofstream(out/"summary.json")<<summary.dump(2)<<'\n';std::cout<<summary.dump(2)<<'\n';
    return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
