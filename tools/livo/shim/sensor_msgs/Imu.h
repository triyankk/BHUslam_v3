#pragma once
#include <ros/ros.h>
#include <memory>
namespace sensor_msgs {struct Imu {
using Ptr=std::shared_ptr<Imu>;using ConstPtr=std::shared_ptr<const Imu>;
std_msgs::Header header;geometry_msgs::Vector3 angular_velocity,linear_acceleration;
};using ImuConstPtr=Imu::ConstPtr;}
