#pragma once
#include "System.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <algorithm>
#include <tuple>

namespace bhu {
// Snapshots copy geometry while the map-update mutex is held. Never retain map
// pointers across TrackStereo: a subsequent reset can clear their contents.
class SparseMapRecorder {
    using J=nlohmann::json;
    std::filesystem::path out_;
    J data_;
    double start_,next_=0;
    std::map<std::string,size_t> last_;
    std::vector<double> lost_;
    static double round(double x){return std::round(x*1000.)/1000.;}
    static J pose(const Sophus::SE3f& twc){auto p=twc.translation();auto q=twc.unit_quaternion();return J::array({round(p.x()),round(p.y()),round(p.z()),q.x(),q.y(),q.z(),q.w()});}
public:
    SparseMapRecorder(const std::filesystem::path& out,double start,bool inertial):out_(out),start_(start){
        std::filesystem::create_directory(out_/"images");
        data_={{"schema",1},{"mode",inertial?"Stereo + IMU":"Stereo"},{"inertial",inertial},{"interval_s",2},{"point_limit",12000},{"geometry_rounding_m",.001},{"snapshots",J::array()},{"maps",J::object()},{"notes","Online map snapshots, not ground truth. Reset maps have independent coordinates. Geometry may be refined between snapshots."}};
    }
    void capture(ORB_SLAM3::System& slam,const cv::Mat& image,double time,int state,bool initialized,const Sophus::SE3f& tcw,bool force=false){
        double t=time-start_;if(state!=2)lost_.push_back(time);
        if(!force&&t<next_)return;next_=t+2;
        const auto points=slam.GetTrackedMapPoints();const auto keys=slam.GetTrackedKeyPointsUn();
        cv::Mat thumbnail;
        cv::cvtColor(image,thumbnail,cv::COLOR_GRAY2BGR);
        for(size_t i=0;i<std::min(keys.size(),points.size());++i)if(points[i]&&!points[i]->isBad())cv::rectangle(thumbnail,keys[i].pt-cv::Point2f(2,2),keys[i].pt+cv::Point2f(2,2),cv::Scalar(50,240,70),1);
        std::string file="images/"+std::to_string(data_["snapshots"].size())+".jpg";if(!cv::imwrite((out_/file).string(),thumbnail,{cv::IMWRITE_JPEG_QUALITY,78}))throw std::runtime_error("Cannot save map preview");
        J snapshot={{"t",t},{"state",state},{"initialized",initialized},{"image",file},{"map",nullptr},{"points",J::array()},{"keys",J::array()},{"edges",J::array()},{"route",J::array()}};
        ORB_SLAM3::Map* map=nullptr;if(state==2)for(auto* p:points)if(p&&!p->isBad()){map=p->GetMap();if(map)break;}
        if(map&&tcw.matrix().allFinite()){
            std::unique_lock<std::mutex> lock(map->mMutexMapUpdate);
            const auto id=std::to_string(map->GetId())+":"+std::to_string(map->GetInitKFid());snapshot["map"]=id;snapshot["camera"]=pose(tcw.inverse());
            auto all=map->GetAllMapPoints();std::sort(all.begin(),all.end(),[](auto*a,auto*b){return a->mnId<b->mnId;});
            const size_t stride=std::max<size_t>(1,(all.size()+11999)/12000);size_t valid=0;
            for(size_t i=0;i<all.size();++i){auto*p=all[i];if(!p||p->isBad())continue;auto pos=p->GetWorldPos();if(!pos.allFinite())continue;++valid;if(i%stride)continue;snapshot["points"].push_back({round(pos.x()),round(pos.y()),round(pos.z()),p->Observations()});}
            snapshot["valid_landmarks"]=valid;
            auto kfs=map->GetAllKeyFrames();std::sort(kfs.begin(),kfs.end(),[](auto*a,auto*b){return a->mTimeStamp<b->mTimeStamp;});std::set<unsigned long> ids;
            for(auto*k:kfs)if(k&&!k->isBad()){auto twc=k->GetPoseInverse();if(!twc.matrix().allFinite())continue;ids.insert(k->mnId);snapshot["keys"].push_back({{"id",k->mnId},{"t",k->mTimeStamp-start_},{"pose",pose(twc)}});}
            std::set<std::tuple<unsigned long,unsigned long,int>> edges;
            auto edge=[&](auto*k,auto*j,int type){if(j&&ids.count(j->mnId)&&j!=k){auto a=k->mnId,b=j->mnId;if(a>b)std::swap(a,b);edges.emplace(a,b,type);}};
            ORB_SLAM3::KeyFrame* prev=nullptr;
            for(auto*k:kfs)if(k&&ids.count(k->mnId)){edge(k,k->GetParent(),0);for(auto*j:k->GetCovisiblesByWeight(100))edge(k,j,1);for(auto*j:k->GetLoopEdges())edge(k,j,2);
                if(prev){auto it=std::lower_bound(lost_.begin(),lost_.end(),prev->mTimeStamp);if(it==lost_.end()||*it>k->mTimeStamp)snapshot["route"].push_back({prev->mnId,k->mnId});}prev=k;}
            for(auto [a,b,type]:edges)snapshot["edges"].push_back({a,b,type});
            data_["maps"][id]={{"label","Map "+id},{"last_s",t},{"last_snapshot",data_["snapshots"].size()}};last_[id]=data_["snapshots"].size();
        }
        data_["snapshots"].push_back(std::move(snapshot));
    }
    void finish(){
        for(auto&[id,index]:last_){auto& s=data_["snapshots"][index];std::string name=id;std::replace(name.begin(),name.end(),':','-');std::ofstream ply(out_/("map-"+name+".ply"));ply<<"ply\nformat ascii 1.0\ncomment ORB-SLAM3 online snapshot in local metres\nelement vertex "<<s["points"].size()<<"\nproperty float x\nproperty float y\nproperty float z\nend_header\n";for(auto&p:s["points"])ply<<p[0]<<' '<<p[1]<<' '<<p[2]<<'\n';if(!ply)throw std::runtime_error("Cannot write map PLY");}
        std::ofstream json(out_/"sparse-map.json");json<<data_.dump();if(!json)throw std::runtime_error("Cannot save sparse map JSON");json.close();
        std::ifstream input(BHU_SPARSE_PAGE);std::string html{std::istreambuf_iterator<char>(input),{}};std::string marker="/*SPARSE_MAP_DATA*/null";auto at=html.find(marker);if(at==std::string::npos)throw std::runtime_error("Missing sparse map template");html.replace(at,marker.size(),data_.dump());std::ofstream page(out_/"sparse-map.html");page<<html;if(!page)throw std::runtime_error("Cannot save map viewer");
        // Standalone preview of the richest saved snapshot, for quick review.
        const J* richest=nullptr;for(const auto&s:data_["snapshots"])if(!s["points"].empty()&&(!richest||s["points"].size()>(*richest)["points"].size()))richest=&s;
        if(richest){const auto&s=*richest;const bool inertial=data_["inertial"];auto xyz=[&](const J&p){cv::Vec3d v{double(p[0]),double(p[1]),double(p[2])};return inertial?v:cv::Vec3d{v[0],v[2],-v[1]};};
            std::vector<double> axes[3];for(auto&p:s["points"]){auto v=xyz(p);for(int k=0;k<3;++k)axes[k].push_back(v[k]);}cv::Vec3d lo,hi;for(int k=0;k<3;++k){std::sort(axes[k].begin(),axes[k].end());lo[k]=axes[k][size_t(.02*(axes[k].size()-1))];hi[k]=axes[k][size_t(.98*(axes[k].size()-1))];}for(auto&key:s["keys"]){auto p=xyz(key["pose"]);for(int k=0;k<3;++k){lo[k]=std::min(lo[k],p[k]);hi[k]=std::max(hi[k],p[k]);}}auto center=(lo+hi)*.5;double extent=std::max({.2,hi[0]-lo[0],hi[1]-lo[1],hi[2]-lo[2]});
            cv::Mat preview(850,1200,CV_8UC3,cv::Scalar(26,18,10));auto project=[&](cv::Vec3d p){p-=center;double x=.8253*p[0]+.5646*p[1],y=-.5646*p[0]+.8253*p[1];return cv::Point(int(600+x*650/extent),int(450-(.7174*y+.6967*p[2])*650/extent));};
            for(auto&p:s["points"]){auto v=xyz(p);double color=std::clamp((v[2]-lo[2])/std::max(.01,hi[2]-lo[2]),0.,1.);auto q=project(v);if(q.x>=0&&q.y>=100&&q.x<1200&&q.y<790)cv::circle(preview,q,1,{35,60+180*color,250},cv::FILLED);}
            std::map<unsigned long,cv::Vec3d> keys;for(auto&k:s["keys"])keys[k["id"]]=xyz(k["pose"]);for(auto&e:s["route"])if(keys.count(e[0])&&keys.count(e[1]))cv::line(preview,project(keys[e[0]]),project(keys[e[1]]),{240,210,50},2,cv::LINE_AA);for(auto&[id,p]:keys)cv::circle(preview,project(p),3,{230,240,250},1,cv::LINE_AA);
            cv::putText(preview,std::string(data_["mode"])+" | sparse 3D landmarks and keyframes",{25,40},cv::FONT_HERSHEY_SIMPLEX,.8,{240,240,240},1,cv::LINE_AA);cv::putText(preview,"Map "+std::string(s["map"])+" at "+std::to_string(double(s["t"]))+" s | "+std::to_string(s["points"].size())+" displayed points",{25,80},cv::FONT_HERSHEY_SIMPLEX,.55,{190,200,210},1,cv::LINE_AA);cv::putText(preview,"Richest online snapshot; independent local map. Color: height. No ground truth.",{25,825},cv::FONT_HERSHEY_SIMPLEX,.55,{190,200,210},1,cv::LINE_AA);if(!cv::imwrite((out_/"sparse-map-preview.png").string(),preview))throw std::runtime_error("Cannot write map preview");
        }
    }
};
}
