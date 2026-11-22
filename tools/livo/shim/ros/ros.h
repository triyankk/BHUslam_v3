#pragma once
// Offline numerical-core compatibility only. No ROS transport exists here.
#include <cassert>
#include <cstdio>
#include <string>
#include <stdexcept>
#define ROS_INFO(...) std::fprintf(stderr,__VA_ARGS__)
#define ROS_WARN(...) std::fprintf(stderr,__VA_ARGS__)
#define ROS_ERROR(...) std::fprintf(stderr,__VA_ARGS__)
#define ROS_ASSERT(x) assert(x)
namespace ros {
struct Time {double value=0;Time& fromSec(double x){value=x;return *this;}double toSec()const{return value;}};
struct Duration {};
struct Publisher {template<class T>void publish(const T&)const{throw std::runtime_error("Offline replay cannot publish");}};
struct Rate {explicit Rate(int){} void sleep()const{}};
struct NodeHandle {template<class T>void param(const std::string&,T& out,const T& value)const{out=value;}};
}
namespace geometry_msgs {
struct Vector3{double x=0,y=0,z=0;};
struct Quaternion{double x=0,y=0,z=0,w=1;};
struct Pose{Vector3 position;Quaternion orientation;};
}
namespace std_msgs {struct Header{ros::Time stamp;std::string frame_id;};}
