#include "factor/projectionOneFrameTwoCamFactor.h"
#include <stdexcept>
#include <iostream>
int main(){
 const double b=.0947195968628;double left[7]={-.007886,-.001086,.014248,0,0,0,1};double right[7]={left[0]+b,left[1],left[2],0,0,0,1},invDepth=.5,td=0;
 const double* args[]={left,right,&invDepth,&td};double residual[2];
 ProjectionOneFrameTwoCamFactor::sqrt_info=Eigen::Matrix2d::Identity();
 ProjectionOneFrameTwoCamFactor factor({.2,.1,1},{.2-b/2,.1,1},Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero(),0,0);
 factor.Evaluate(args,residual,nullptr);
 if(std::hypot(residual[0],residual[1])>1e-12)throw std::runtime_error("Stereo extrinsics direction or metre units incorrect");
 right[0]=left[0]-b;factor.Evaluate(args,residual,nullptr);
 if(std::abs(residual[0])<.09)throw std::runtime_error("Wrong baseline sign was not detected");
 std::cout<<"PASS: upstream stereo projection, positive right-camera centre baseline, metre-scale depth\n";
}
