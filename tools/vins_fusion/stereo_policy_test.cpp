#include "runtime_globals.hpp"
#include "stereo_policy.hpp"
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <iostream>
#include <unistd.h>

static void need(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main()try{
    cv::setNumThreads(1);ROW=240;COL=400;MAX_CNT=200;MIN_DIST=20;
    auto path=std::filesystem::temp_directory_path()/("bhu-stereo-policy-"+std::to_string(getpid())+".yaml");
    struct Cleanup{std::filesystem::path p;~Cleanup(){std::filesystem::remove(p);}}cleanup{path};
    {cv::FileStorage f(path.string(),cv::FileStorage::WRITE);
     f<<"model_type"<<"PINHOLE"<<"camera_name"<<"synthetic"<<"image_width"<<COL<<"image_height"<<ROW
      <<"distortion_parameters"<<"{"<<"k1"<<0.<<"k2"<<0.<<"p1"<<0.<<"p2"<<0.<<"}"
      <<"projection_parameters"<<"{"<<"fx"<<300.<<"fy"<<300.<<"cx"<<200.<<"cy"<<120.<<"}";}
    cv::Mat left(ROW,COL,CV_8UC1);cv::RNG rng(718);
    rng.fill(left,cv::RNG::UNIFORM,0,255);cv::GaussianBlur(left,left,{5,5},.8);
    bhu::fusion::rectified_stereo=true;
    auto matches=[&](double dx,double dy,bool blank){
        cv::Mat right;cv::Mat transform=(cv::Mat_<double>(2,3)<<1,0,dx,0,1,dy);
        cv::warpAffine(left,right,transform,left.size());if(blank)right.setTo(0);
        FeatureTracker tracker;tracker.readIntrinsicParameter({path.string(),path.string()});
        auto features=tracker.trackImage(1.,left,right);size_t count=0;
        for(auto& f:features)if(f.second.size()==2){++count;
            const auto&a=f.second[0].second;const auto&b=f.second[1].second;
            need(a[3]>b[3]&&std::abs(a[4]-b[4])<=1.,"All accepted stereo points must satisfy rectified geometry");
            if(dx==-8&&dy==0)need(std::abs(a[3]-b[3]-8)<.5,"Known synthetic stereo disparity");
        }
        return count;
    };
    auto good=matches(-8,0,false);need(good>=30,"Retain useful stereo support on known textured plane");
    need(matches(8,0,false)==0,"Reject reversed baseline sign");
    need(matches(-8,6,false)==0,"Reject vertical epipolar mismatch");
    need(matches(-8,0,true)==0,"Blank image must not fabricate stereo support");
    // Exercise the actual backend/frontend reset sequence used by the grounded
    // watchdog, not just its disarm predicate. It must reload camera models and
    // discard both IMU integration and old temporal feature state.
    RIC.assign(2,Eigen::Matrix3d::Identity());TIC.assign(2,Eigen::Vector3d::Zero());
    TIC[1].x()=.1;CAM_NAMES={path.string(),path.string()};
    ACC_N=.1;GYR_N=.01;ACC_W=.001;GYR_W=.0001;
    static Estimator estimator;estimator.setParameter();
    for(unsigned attempt=0;attempt<3;attempt++){
        std::vector<std::pair<double,Eigen::Vector3d>> initialAcceleration{{1.,Eigen::Vector3d(0,0,9.81)}};
        estimator.initFirstIMUPose(initialAcceleration);
        estimator.processIMU(1.,.005,Eigen::Vector3d(0,0,9.81),Eigen::Vector3d::Zero());
        estimator.featureTracker.trackImage(1.,left,left);
        need(!estimator.featureTracker.ids.empty(),"Reset fixture has real temporal features");
        estimator.clearState();estimator.featureTracker=FeatureTracker();estimator.setParameter();
        need(estimator.solver_flag==Estimator::INITIAL&&!estimator.initFirstPoseFlag&&estimator.frame_count==0,"Backend returns to uninitialized state");
        need(estimator.featureTracker.ids.empty()&&estimator.featureTracker.m_camera.size()==2,"Frontend clears tracks and reloads both camera models");
        for(auto p:estimator.pre_integrations)need(p==nullptr,"No retained IMU integration after reset");
    }
    std::cout<<"PASS: actual tracker retains "<<good<<" known-disparity correspondences and rejects reversed/vertical/blank pairs\n";
    std::cout<<"PASS: repeated native estimator/frontend reinitialization clears state and reloads cameras\n";
    return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
