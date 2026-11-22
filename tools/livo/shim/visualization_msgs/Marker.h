#pragma once
#include <ros/ros.h>
namespace visualization_msgs {struct Marker {
enum{CYLINDER=3,ADD=0};std_msgs::Header header;std::string ns;int id=0,type=0,action=0;
geometry_msgs::Pose pose;geometry_msgs::Vector3 scale;
struct {double r=0,g=0,b=0,a=0;}color;ros::Duration lifetime;
};}
