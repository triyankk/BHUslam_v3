#pragma once
#include <libobsensor/ObSensor.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace bhu::camera {
struct Properties {
    std::shared_ptr<ob::Device> device;
    std::vector<std::pair<OBPropertyID,bool>> saved;
    std::vector<std::pair<OBPropertyID,int>> savedInt;
    void set(OBPropertyID id,bool value) {
        if(!device->isPropertySupported(id,OB_PERMISSION_READ_WRITE))
            throw std::runtime_error("Required reversible camera property unavailable: "+std::to_string(id));
        bool old=device->getBoolProperty(id);
        saved.emplace_back(id,old);
        device->setBoolProperty(id,value);
        if(device->getBoolProperty(id)!=value) throw std::runtime_error("Camera property did not take effect");
    }
    void setInt(OBPropertyID id,int value) {
        if(!device->isPropertySupported(id,OB_PERMISSION_READ_WRITE))
            throw std::runtime_error("Required reversible integer property unavailable");
        savedInt.emplace_back(id,device->getIntProperty(id));
        device->setIntProperty(id,value);
        if(device->getIntProperty(id)!=value)throw std::runtime_error("Integer property did not take effect");
    }
    ~Properties() {
        for(auto it=savedInt.rbegin();it!=savedInt.rend();++it)try {
            device->setIntProperty(it->first,it->second);
        }catch(const std::exception& e){std::cerr<<"Setting restoration failed: "<<e.what()<<'\n';}
        for(auto it=saved.rbegin();it!=saved.rend();++it) try {
            device->setBoolProperty(it->first,it->second);
        } catch(const std::exception& e) {std::cerr<<"Setting restoration failed: "<<e.what()<<'\n';}
    }
};

static cv::Mat rotation(const OBExtrinsic& e) {
    cv::Mat r(3,3,CV_64F);
    for(int i=0;i<9;++i) r.at<double>(i/3,i%3)=e.rot[i];
    if(cv::norm(r*r.t()-cv::Mat::eye(3,3,CV_64F))>0.01 || std::abs(cv::determinant(r)-1)>0.01)
        throw std::runtime_error("Invalid SDK extrinsic rotation");
    return r;
}
static cv::Mat intrinsic(std::shared_ptr<ob::VideoStreamProfile> p) {
    auto k=p->getIntrinsic();
    if(k.fx<=0 || k.fy<=0) throw std::runtime_error("Invalid camera intrinsics");
    return (cv::Mat_<double>(3,3)<<k.fx,0,k.cx,0,k.fy,k.cy,0,0,1);
}
static cv::Mat distortion(std::shared_ptr<ob::VideoStreamProfile> p) {
    auto d=p->getDistortion();
    if(d.model==OB_DISTORTION_NONE) return cv::Mat::zeros(1,8,CV_64F);
    if(d.model!=OB_DISTORTION_BROWN_CONRADY && d.model!=OB_DISTORTION_BROWN_CONRADY_K6)
        throw std::runtime_error("Unsupported distortion model; refusing an approximate conversion");
    return (cv::Mat_<double>(1,8)<<d.k1,d.k2,d.p1,d.p2,d.k3,d.k4,d.k5,d.k6);
}

} // namespace bhu::camera
