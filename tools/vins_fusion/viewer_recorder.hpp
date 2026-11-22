#pragma once
#include "estimator.h"
#include <json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>
#include <map>

// Display-only snapshots from the native estimator; no feedback into optimization.
class FusionViewerRecorder {
    using J=nlohmann::json;
    std::filesystem::path out_;
    J data_;
    double start_;
public:
    FusionViewerRecorder(const std::filesystem::path& out,const std::filesystem::path& recording,double start,int stride):out_(out),start_(start){
        std::filesystem::create_directory(out_/"viewer-images");
        data_={{"schema",1},{"algorithm","VINS-Fusion stereo + IMU"},{"recording",recording.filename().string()},{"width",COL},{"height",ROW},{"sample_stride",stride},{"depth_limit_m",80},{"frames",J::array()},{"note","Sampled online sliding-window landmarks and frozen history; no global map optimization or loop closure. Native failure detection is disabled. Coordinates in local metres."}};
    }
    void capture(const Estimator&e,const cv::Mat&left,const cv::Mat&right,
                 const std::map<int,std::vector<std::pair<int,Eigen::Matrix<double,7,1>>>>&features,
                 const std::map<int,int>&ages,double timestamp,size_t sourceIndex,unsigned epoch,bool initialized,bool supported){
        const auto n=data_["frames"].size();const auto file="viewer-images/"+std::to_string(n)+".jpg";
        cv::Mat joined;cv::hconcat(left,right,joined);
        if(!cv::imwrite((out_/file).string(),joined,{cv::IMWRITE_JPEG_QUALITY,78}))throw std::runtime_error("Cannot write viewer stereo image");
        J f={{"t",timestamp-start_},{"timestamp_s",timestamp},{"source_index",sourceIndex},{"epoch",epoch},{"initialized",initialized},{"supported",supported},{"image",file},{"camera",nullptr},{"features",J::array()},{"points",J::array()}};
        auto rounded=[](double v,double scale){return std::round(v*scale)/scale;};
        for(const auto&[id,observations]:features){J a=J::array({id,nullptr,nullptr,nullptr,nullptr,ages.count(id)?ages.at(id):1});for(const auto&[eye,p]:observations){if(eye<0||eye>1||!p.allFinite())continue;a[1+eye*2]=rounded(p[3],10);a[2+eye*2]=rounded(p[4],10);}if(!a[1].is_null())f["features"].push_back(a);}
        if(initialized){
            Eigen::Matrix3d r=e.Rs[WINDOW_SIZE]*e.ric[0];Eigen::Vector3d p=e.Ps[WINDOW_SIZE]+e.Rs[WINDOW_SIZE]*e.tic[0];Eigen::Quaterniond q(r);
            if(p.allFinite()&&q.coeffs().allFinite())f["camera"]={p.x(),p.y(),p.z(),q.x(),q.y(),q.z(),q.w()};
            // Same anchor-to-world reconstruction and maturity criteria as upstream pubPointCloud.
            for(const auto&track:e.f_manager.feature){int anchor=track.start_frame;
                if(track.feature_per_frame.size()<2||anchor<0||anchor>=WINDOW_SIZE-2||track.solve_flag!=1||!std::isfinite(track.estimated_depth)||track.estimated_depth<=0||track.estimated_depth>80)continue;
                Eigen::Vector3d pc=track.feature_per_frame.front().point*track.estimated_depth;
                Eigen::Vector3d pw=e.Rs[anchor]*(e.ric[0]*pc+e.tic[0])+e.Ps[anchor];
                if(pw.allFinite())f["points"].push_back({track.feature_id,rounded(pw.x(),1000),rounded(pw.y(),1000),rounded(pw.z(),1000)});
            }
        }
        data_["frames"].push_back(std::move(f));
    }
    void finish(){
        std::ofstream json(out_/"viewer-data.json");json<<data_.dump();if(!json)throw std::runtime_error("Cannot write viewer data");json.close();
        std::ifstream in(BHU_FUSION_VIEWER_PAGE);std::string html{std::istreambuf_iterator<char>(in),{}};const std::string marker="/*FUSION_VIEWER_DATA*/null";auto at=html.find(marker);if(at==std::string::npos)throw std::runtime_error("Missing Fusion viewer template");
        auto serialized=data_.dump(); // Prevent a recording name from terminating the embedded script.
        size_t pos=0;while((pos=serialized.find('<',pos))!=std::string::npos){serialized.replace(pos,1,"\\u003c");pos+=6;}
        html.replace(at,marker.size(),serialized);std::ofstream out(out_/"viewer.html");out<<html;if(!out)throw std::runtime_error("Cannot write Fusion viewer");
        std::map<std::pair<unsigned,int>,J> points;for(const auto&f:data_["frames"])for(const auto&p:f["points"])points[{f["epoch"].get<unsigned>(),p[0].get<int>()}]=p;
        std::map<unsigned,std::vector<J>> epochs;for(const auto&[key,p]:points)epochs[key.first].push_back(p);
        for(const auto&[epoch,cloud]:epochs){std::ofstream ply(out_/("fusion-epoch-"+std::to_string(epoch)+".ply"));ply<<"ply\nformat ascii 1.0\ncomment Frozen last-observed VIO landmark history; not globally optimized\nelement vertex "<<cloud.size()<<"\nproperty float x\nproperty float y\nproperty float z\nend_header\n";for(const auto&p:cloud)ply<<p[1]<<' '<<p[2]<<' '<<p[3]<<'\n';if(!ply)throw std::runtime_error("Cannot write Fusion PLY");}
    }
};
