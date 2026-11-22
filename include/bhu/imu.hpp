#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace bhu {
struct Sample { double t; std::array<double,3> v; };
struct Imu { double t; std::array<double,3> a,w; };

// Own the interpolated values. Returning/storing a lazy Eigen expression over
// temporary vector conversions would leave dangling references at an endpoint.
inline Imu interpolate_endpoint(const Imu& a,const Imu& b,double target,double maxGap=.02) {
    if(!std::isfinite(a.t)||!std::isfinite(b.t)||!std::isfinite(target)
       ||!std::isfinite(maxGap)||maxGap<=0||b.t<=a.t||b.t-a.t>maxGap
       ||target<a.t||target>b.t)
        throw std::runtime_error("Invalid/bracketless IMU endpoint");
    Imu out{target,{},{}};const double alpha=(target-a.t)/(b.t-a.t);
    for(size_t k=0;k<3;++k){
        if(!std::isfinite(a.a[k])||!std::isfinite(b.a[k])||!std::isfinite(a.w[k])||!std::isfinite(b.w[k]))
            throw std::runtime_error("Nonfinite IMU endpoint input");
        out.a[k]=a.a[k]+alpha*(b.a[k]-a.a[k]);
        out.w[k]=a.w[k]+alpha*(b.w[k]-a.w[k]);
    }
    return out;
}

inline void validate(const std::vector<Sample>& s) {
    if(s.size()<2) throw std::runtime_error("Insufficient IMU samples");
    for(size_t i=0;i<s.size();++i) {
        if(!std::isfinite(s[i].t) || (i && s[i].t<=s[i-1].t))
            throw std::runtime_error("IMU timestamps must be finite and strictly increasing");
        for(double v:s[i].v) if(!std::isfinite(v)) throw std::runtime_error("Nonfinite IMU value");
    }
}

// Gyroscope timestamps define the integration timeline. Require bracketing
// acceleration samples; never extrapolate across the start/end of a recording.
inline std::vector<Imu> synchronize(const std::vector<Sample>& acc,
                                    const std::vector<Sample>& gyro, double maxGap=0.02) {
    validate(acc); validate(gyro);
    std::vector<Imu> out;
    size_t j=0;
    for(size_t i=0;i<gyro.size();++i) {
        const auto& g=gyro[i];
        if(i && g.t-gyro[i-1].t>maxGap) throw std::runtime_error("Gyroscope sample gap exceeds limit");
        if(g.t<acc.front().t || g.t>acc.back().t) continue;
        while(j+1<acc.size()-1 && acc[j+1].t<g.t) ++j;
        const auto& a=acc[j]; const auto& b=acc[j+1];
        if(b.t-a.t>maxGap) throw std::runtime_error("Accelerometer sample gap exceeds limit");
        double f=(g.t-a.t)/(b.t-a.t);
        Imu m{g.t,{},g.v};
        for(int k=0;k<3;++k) m.a[k]=a.v[k]+f*(b.v[k]-a.v[k]);
        out.push_back(m);
    }
    return out;
}
}
