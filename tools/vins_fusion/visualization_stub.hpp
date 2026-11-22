#pragma once
// ROS output hooks only. Numerical estimator and tracker code are unchanged.
#include "estimator.h"
inline void pubTrackImage(const cv::Mat&,double){}
inline void pubLatestOdometry(const Eigen::Vector3d&,const Eigen::Quaterniond&,const Eigen::Vector3d&,double){}
inline void printStatistics(const Estimator&,double){}
inline void pubOdometry(const Estimator&,const std_msgs::Header&){}
inline void pubKeyPoses(const Estimator&,const std_msgs::Header&){}
inline void pubCameraPose(const Estimator&,const std_msgs::Header&){}
inline void pubPointCloud(const Estimator&,const std_msgs::Header&){}
inline void pubKeyframe(const Estimator&){}
inline void pubTF(const Estimator&,const std_msgs::Header&){}
