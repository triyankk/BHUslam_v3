#include "frontend.hpp"
#include "feature_tracker.h"
// The ROS project keeps frontend/backend globals in separate processes. Rename
// only the conflicting frontend globals at compile time in our single process.
int ROW=400,COL=640,FOCAL_LENGTH=460,WINDOW_SIZE=20;
std::string IMAGE_TOPIC,FISHEYE_MASK;std::vector<std::string> CAM_NAMES;
int MAX_CNT=150,MIN_DIST=30,FREQ=30,SHOW_TRACK=0,STEREO_TRACK=0,EQUALIZE=1,FISHEYE=0;
double F_THRESHOLD=1.;bool PUB_THIS_FRAME=true;
struct VinsFrontend::Impl{FeatureTracker tracker;size_t images=0;};
VinsFrontend::VinsFrontend(const std::string& config):impl_(new Impl{}){
    cv::FileStorage f(config,cv::FileStorage::READ);ROW=int(f["image_height"]);COL=int(f["image_width"]);
    if(ROW<=0||COL<=0)throw std::runtime_error("Invalid VINS image dimensions");
    impl_->tracker.prev_time=0;impl_->tracker.cur_time=0;
    impl_->tracker.readIntrinsicParameter(config);
}
VinsFrontend::~VinsFrontend()=default;
VinsFeatures VinsFrontend::process(const cv::Mat&image,double time){
    VinsFeatures out;if(impl_->images++==0)return out; // Upstream skips first callback.
    auto&t=impl_->tracker;PUB_THIS_FRAME=true;t.readImage(image,time);
    for(unsigned i=0;t.updateID(i);++i){}
    for(size_t i=0;i<t.ids.size();++i)if(t.track_cnt[i]>1){Eigen::Matrix<double,7,1> v;v<<t.cur_un_pts[i].x,t.cur_un_pts[i].y,1.,t.cur_pts[i].x,t.cur_pts[i].y,t.pts_velocity[i].x,t.pts_velocity[i].y;out[t.ids[i]].emplace_back(0,v);}
    return out;
}
