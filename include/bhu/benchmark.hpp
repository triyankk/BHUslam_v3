#pragma once
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace bhu::bench {
struct Frame {
    double device_s=0,elapsed_s=0,tracking_ms=0,age_ms=0,qr_ms=0,wall_ms=0;
    double x=0,y=0,z=0,qw=1,qx=0,qy=0,qz=0;
    uint64_t segment=0,index=0;
    int state=0,quality=0;
    bool initialized=false;
};
std::string power_profile();
void create_report(const std::filesystem::path& directory);
class Benchmark {
    struct Impl;std::unique_ptr<Impl> impl_;
public:
    Benchmark(const std::filesystem::path& output,int port,const std::string& required_profile);
    ~Benchmark();
    void start(double device_s,uint64_t host_us);
    void frame(const Frame& frame);
    void stop(const std::string& reason,uint64_t host_us,uint64_t overwritten);
    void status(const std::string& phase);
    void integration(bool navigation,bool buzzer);
    double elapsed(uint64_t host_us)const;
    bool started()const;
};
} // namespace bhu::bench
