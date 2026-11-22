#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <stdexcept>
int main(){
 sensor_msgs::Imu m;m.header.stamp.fromSec(12.345);
 if(std::abs(m.header.stamp.toSec()-12.345)>1e-12)return 1;
 auto q=tf::createQuaternionMsgFromRollPitchYaw(0,0,3.141592653589793/2);
 if(std::abs(q.z-std::sqrt(.5))>1e-12||std::abs(q.w-std::sqrt(.5))>1e-12)return 2;
 try{ros::Publisher{}.publish(m);}catch(const std::runtime_error&){return 0;}
 return 3;
}
