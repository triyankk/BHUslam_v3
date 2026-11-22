#include "bhu/imu.hpp"
#include <iostream>
#include <functional>
void check(bool yes) { if(!yes) throw std::runtime_error("test failed"); }
void throws(const std::function<void()>& f) { bool caught=false; try{f();}catch(const std::runtime_error&){caught=true;} check(caught); }
int main() {
    using bhu::Sample;
    std::vector<Sample> a{{0,{0,0,9.81}},{.01,{2,4,9.81}},{.02,{4,8,9.81}}};
    std::vector<Sample> g{{-.005,{0,0,0}},{.005,{1,2,3}},{.015,{4,5,6}},{.025,{7,8,9}}};
    auto out=bhu::synchronize(a,g);
    check(out.size()==2); check(std::abs(out[0].a[0]-1)<1e-9);
    check(std::abs(out[1].a[1]-6)<1e-9); check(out[0].w[2]==3);
    auto bad=a;bad[1].t=0;throws([&]{bhu::synchronize(bad,g);});
    bad=a;bad[1].v[0]=NAN;throws([&]{bhu::synchronize(bad,g);});
    throws([&]{bhu::synchronize(a,g,.002);});
    std::cout << "IMU interpolation, boundary, gap, duplicate and nonfinite checks passed\n";
}
