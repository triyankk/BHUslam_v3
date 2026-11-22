#pragma once
#include <ros/ros.h>
#include <cmath>
namespace tf {inline geometry_msgs::Quaternion createQuaternionMsgFromRollPitchYaw(double r,double p,double y){
double cr=std::cos(r/2),sr=std::sin(r/2),cp=std::cos(p/2),sp=std::sin(p/2),cy=std::cos(y/2),sy=std::sin(y/2);
return {sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy};}}
