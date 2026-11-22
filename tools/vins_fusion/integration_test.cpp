#include "factor/integration_base.h"
#include "bhu/imu.hpp"
#include <iostream>
#include <stdexcept>
double ACC_N=.01*std::sqrt(200.),GYR_N=.001*std::sqrt(200.);
double ACC_W=.0001*std::sqrt(200.),GYR_W=.000001*std::sqrt(200.);
static void check(bool good){if(!good)throw std::runtime_error("VINS integration check failed");}
int main(){const Eigen::Vector3d z=Eigen::Vector3d::Zero(),g(0,0,9.81);IntegrationBase p(g,z,z,z);for(int i=0;i<200;++i)p.push_back(.005,g,z);check(std::abs(p.sum_dt-1)<1e-12);check((p.delta_v-g).norm()<1e-9);check((p.delta_p-.5*g).norm()<1e-9);check(std::abs(p.delta_q.w()-1)<1e-12);check(p.covariance.allFinite());
 IntegrationBase noise(g,z,z,z);noise.push_back(.005,g,z);check(std::abs(noise.covariance(9,9)-.0001*.0001*.005)<1e-20);check(std::abs(noise.covariance(12,12)-.000001*.000001*.005)<1e-24);
 const Eigen::Vector3d w(0,0,.2);IntegrationBase spin(g,w,z,z);for(int i=0;i<200;++i)spin.push_back(.005,g,w);check(std::abs(Eigen::AngleAxisd(spin.delta_q).angle()-.2)<1e-6);
 bhu::Imu left{1.,{1,2,9.81},{.1,.2,.3}},right{1.01,{5,6,9.81},{.5,.6,.7}};
 const auto endpoint=bhu::interpolate_endpoint(left,right,1.005);
 left.a.fill(-100);right.a.fill(100);left.w.fill(-100);right.w.fill(100);
 check(std::abs(endpoint.a[0]-3)<1e-10&&std::abs(endpoint.a[1]-4)<1e-10&&std::abs(endpoint.w[2]-.5)<1e-10);
 bool rejected=false;try{bhu::interpolate_endpoint(left,right,1.02);}catch(const std::runtime_error&){rejected=true;}check(rejected);
 IntegrationBase owned(Eigen::Vector3d(1,2,9.81),Eigen::Vector3d(.1,.2,.3),z,z),reference(Eigen::Vector3d(1,2,9.81),Eigen::Vector3d(.1,.2,.3),z,z);
 owned.push_back(.005,Eigen::Vector3d(endpoint.a[0],endpoint.a[1],endpoint.a[2]),Eigen::Vector3d(endpoint.w[0],endpoint.w[1],endpoint.w[2]));reference.push_back(.005,Eigen::Vector3d(3,4,9.81),Eigen::Vector3d(.3,.4,.5));check((owned.delta_v-reference.delta_v).norm()<1e-10&&owned.delta_q.angularDistance(reference.delta_q)<1e-10);
 std::cout<<"PASS: SI acceleration, gyro rotation, time/bias covariance, owning endpoint interpolation and native preintegration\n";
}
