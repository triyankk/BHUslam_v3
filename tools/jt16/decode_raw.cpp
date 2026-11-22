// Offline JT16 replay through the same pinned SDK decoder as acquisition.
// No serial, network, ROS or flight output. Output uses SDK sensor axes/time.
#include "udp1_8_parser.h"
#include "logger.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <memory>

namespace fs=std::filesystem;
using namespace hesai::lidar;
#pragma pack(push,1)
struct SavedPoint { float x,y,z; double timestamp; uint16_t ring; uint8_t intensity,confidence; };
#pragma pack(pop)
static_assert(sizeof(SavedPoint)==24);

int main(int argc,char** argv) try {
    if(argc!=4) throw std::runtime_error("Usage: jt16_decode_raw INPUT.raw CORRECTION.csv NEW_OUTPUT");
    std::ifstream input(argv[1],std::ios::binary);
    if(!input)throw std::runtime_error("Cannot read raw capture");
    if(fs::exists(argv[3]))throw std::runtime_error("Output already exists");
    const fs::path out=argv[3];
    const std::vector<uint8_t> raw{std::istreambuf_iterator<char>(input),{}};
    Logger::GetInstance().setLogTargetRule(HESAI_LOG_TARGET_NONE);
    // SDK trig tables occupy about 74 MB; keep the parser off the stack.
    auto parserStorage=std::make_unique<Udp1_8Parser<LidarPointXYZICRT>>();
    auto& parser=*parserStorage;
    parser.SetFrameAzimuth(0);
    parser.LoadCorrectionFile(argv[2]);
    if(!parser.isSetCorrectionSucc())throw std::runtime_error("Correction table failed to load");
    LidarDecodedFrame<LidarPointXYZICRT> frame(1800,16);
    DriverParam params;
    frame.fParam.Init(params);
    fs::create_directories(out/"scans");
    std::ofstream index(out/"scans.csv");
    index<<"scan,sensor_begin_s,sensor_end_s,packets,points,nonzero_points,file\n"<<std::setprecision(17);
    size_t offset=0,valid=0,invalid=0,skipped=0,imu=0,saved=0,dropped=0,points=0;
    bool first=true,broken=false; int lastSequence=-1;
    auto finish=[&] {
        double span=frame.frame_end_timestamp-frame.frame_start_timestamp;
        if(first||broken||span<.10||span>.30||frame.packet_num<32) {++dropped;}
        else {
            const std::string name="scans/"+std::to_string(saved)+".bin";
            std::ofstream cloud(out/name,std::ios::binary);size_t count=0,nonzero=0;
            for(uint32_t k=0;k<frame.packet_num;++k)for(uint32_t j=0;j<frame.valid_points[k];++j) {
                const auto& p=frame.points[k*frame.per_points_num+j];
                if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(p.timestamp)||p.ring>=16)
                    throw std::runtime_error("Invalid decoded point");
                SavedPoint q{p.x,p.y,p.z,p.timestamp,p.ring,p.intensity,p.confidence};
                cloud.write(reinterpret_cast<const char*>(&q),sizeof(q));++count;
                if(p.x!=0||p.y!=0||p.z!=0)++nonzero;
            }
            cloud.flush();if(!cloud)throw std::runtime_error("Point write failed");
            index<<saved<<','<<frame.frame_start_timestamp<<','<<frame.frame_end_timestamp<<','
                 <<frame.packet_num<<','<<count<<','<<nonzero<<','<<name<<'\n';
            ++saved;points+=count;
        }
        first=false;broken=false;frame.Update();
    };
    while(offset+16<=raw.size()) {
        const auto* p=raw.data()+offset;
        if(p[0]!=0xee||p[1]!=0xff||p[2]!=1||p[3]!=8||p[5]>1){++offset;++skipped;broken=true;continue;}
        const size_t length=p[5]==0?80:34;
        if(offset+length>raw.size())break;
        UdpPacket packet(p,length);
        if(frame.packet_num>=frame.maxPacketPerFrame)throw std::runtime_error("Scan exceeds bounded SDK buffer");
        int status=parser.DecodePacket(frame,packet);
        if(status<0){++invalid;++offset;broken=true;continue;}
        if(status==1){++imu;offset+=length;continue;}
        if(frame.scan_complete){finish();status=parser.DecodePacket(frame,packet);}
        if(status!=0||frame.scan_complete||frame.packet_num==0)throw std::runtime_error("Boundary re-decode failed");
        const int sequence=p[length-6]|(int(p[length-5])<<8);
        if(lastSequence>=0&&((sequence-lastSequence)&65535)!=1)broken=true;
        lastSequence=sequence;
        if(parser.ComputeXYZI(frame,frame.packet_num-1)!=0)throw std::runtime_error("SDK XYZ conversion failed");
        ++valid;offset+=length;
    }
    // The final scan lacks a following boundary; never label it complete.
    if(frame.packet_num)++dropped;
    index.flush();if(!index)throw std::runtime_error("Index write failed");
    std::ofstream summary(out/"decode-summary.json");
    summary<<"{\n\"sdk_decoder\":\"Udp1_8Parser\",\n\"valid_point_packets\":"<<valid
           <<",\n\"imu_packets\":"<<imu<<",\n\"invalid_candidates\":"<<invalid
           <<",\n\"skipped_bytes\":"<<skipped<<",\n\"trailing_bytes\":"<<raw.size()-offset
           <<",\n\"complete_scans_saved\":"<<saved<<",\n\"partial_or_invalid_scans_dropped\":"<<dropped
           <<",\n\"points_saved\":"<<points
           <<",\n\"point_record_bytes\":24,\n\"point_layout\":\"little-endian float32 x,y,z; float64 sensor_time_s; uint16 ring; uint8 intensity,confidence\","
             "\n\"deskewed\":false,\n\"camera_aligned\":false,\n\"sensor_time_is_verified_utc\":false\n}\n";
    summary.flush();if(!summary)throw std::runtime_error("Summary write failed");
    std::cout<<"Saved "<<saved<<" complete scans, "<<points<<" points; dropped "<<dropped<<" partial/invalid scans\n";
    return saved?0:3;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
