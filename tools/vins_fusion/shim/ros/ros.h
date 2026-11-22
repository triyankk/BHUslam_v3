#pragma once
// Logging and timestamp compatibility only: no ROS runtime, subscriptions or IO.
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <sstream>
namespace ros { class NodeHandle {}; struct Time { Time()=default; explicit Time(double t):value(t){} double value=0; double toSec() const{return value;} }; }
#define ROS_DEBUG(...) do {} while(0)
#define ROS_DEBUG_STREAM(x) do {} while(0)
#define ROS_INFO(...) do {std::fprintf(stderr,__VA_ARGS__);std::fputc('\n',stderr);} while(0)
#define ROS_WARN(...) ROS_INFO(__VA_ARGS__)
#define ROS_ERROR(...) ROS_INFO(__VA_ARGS__)
#define ROS_INFO_STREAM(x) do {std::ostringstream bhu_log;bhu_log<<x;ROS_INFO("%s",bhu_log.str().c_str());} while(0)
#define ROS_WARN_STREAM(x) ROS_INFO_STREAM(x)
#define ROS_ERROR_STREAM(x) ROS_INFO_STREAM(x)
#define ROS_ASSERT(x) do {if(!(x)){std::fprintf(stderr,"VINS assertion failed: %s\n",#x);std::abort();}} while(0)
#define ROS_ASSERT_MSG(x,...) do {if(!(x)){ROS_ERROR(__VA_ARGS__);std::abort();}} while(0)
#define ROS_BREAK() std::abort()
