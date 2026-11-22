#pragma once
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <map>
#include <vector>
#include <memory>
using VinsFeatures=std::map<int,std::vector<std::pair<int,Eigen::Matrix<double,7,1>>>>;
class VinsFrontend {
    struct Impl;std::unique_ptr<Impl> impl_;
public:
    explicit VinsFrontend(const std::string&config);
    ~VinsFrontend();
    VinsFeatures process(const cv::Mat&,double);
};
