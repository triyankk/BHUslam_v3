// Observe the existing optFlow JT16 loopback feed; never open a sensor port.
#include <json.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>
#include <unistd.h>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>
using Clock=std::chrono::steady_clock;
#pragma pack(push,1)
struct Header { char magic[8];uint64_t host_ns,index;uint32_t count,original;uint16_t chunk,chunks; };
struct Point {float x,y,z,intensity;};
struct Imu {char magic[8];uint64_t host_ns,index;double values[7];};
#pragma pack(pop)
static_assert(sizeof(Header)==36&&sizeof(Point)==16&&sizeof(Imu)==80);
static double quantile(std::vector<double> v,double q){if(v.empty())return 0;std::sort(v.begin(),v.end());return v[size_t(q*(v.size()-1))];}
int main(int argc,char**argv)try{
 if(argc>2)throw std::runtime_error("Usage: jt16_inspect_feed [seconds=20]");
 const double seconds=argc==2?std::stod(argv[1]):20.;
 if(!std::isfinite(seconds)||seconds<=0||seconds>60)throw std::runtime_error("Seconds must be >0 and <=60");
 const int fd=socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);if(fd<0)throw std::runtime_error("socket failed");
 struct Close{int fd;~Close(){close(fd);}} close_socket{fd};
 int bytes=4*1024*1024;setsockopt(fd,SOL_SOCKET,SO_RCVBUF,&bytes,sizeof(bytes));
 sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(18776);
 if(bind(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a)))throw std::runtime_error("Feed port 18776 is occupied; leave the existing listener running");
 const auto start=Clock::now();uint64_t datagrams=0,malformed=0,clouds=0,imus=0,incomplete=0,cloud_skips=0,imu_skips=0,current=0,last_cloud=0,last_imu=0;
 Header active{};std::vector<bool> seen;std::vector<double> cloud_times,imu_times,ages,point_counts,original_counts,acc_norms,gyro_norms,imu_sensor_times;
 uint64_t valid_points=0,nonfinite_points=0;std::array<double,3> lo{1e9,1e9,1e9},hi{-1e9,-1e9,-1e9};
 std::array<char,65536> buf{};
 while(std::chrono::duration<double>(Clock::now()-start).count()<seconds){
  pollfd wait{fd,POLLIN,0};if(poll(&wait,1,50)<=0)continue;
  const auto n=recv(fd,buf.data(),buf.size(),0);if(n<8){++malformed;continue;}++datagrams;
  const double arrival=std::chrono::duration<double>(Clock::now()-start).count();
  if(!std::memcmp(buf.data(),"JT16VIM1",8)){
   if(n!=sizeof(Imu)){++malformed;continue;}Imu imu;std::memcpy(&imu,buf.data(),sizeof(imu));
   if(!std::all_of(std::begin(imu.values),std::end(imu.values),[](double v){return std::isfinite(v);})){++malformed;continue;}
   if(last_imu&&imu.index>last_imu+1)imu_skips+=imu.index-last_imu-1;
   if(last_imu&&imu.index<=last_imu){++malformed;continue;}last_imu=imu.index;
   ++imus;imu_times.push_back(arrival);imu_sensor_times.push_back(imu.values[0]);
   acc_norms.push_back(std::hypot(std::hypot(imu.values[1],imu.values[2]),imu.values[3]));gyro_norms.push_back(std::hypot(std::hypot(imu.values[4],imu.values[5]),imu.values[6]));
  }else if(!std::memcmp(buf.data(),"JT16VPC1",8)){
   if(n<sizeof(Header)){++malformed;continue;}Header h;std::memcpy(&h,buf.data(),sizeof(h));
   if(!h.count||h.count>12000||h.chunks!=(h.count+511)/512||h.chunk>=h.chunks){++malformed;continue;}
   const uint32_t count=std::min(uint32_t{512},h.count-h.chunk*512);
   if(n!=sizeof(h)+count*sizeof(Point)){++malformed;continue;}
   if(h.index!=current){if(!seen.empty()&&!std::all_of(seen.begin(),seen.end(),[](bool v){return v;}))++incomplete;current=h.index;active=h;seen.assign(h.chunks,false);}
   if(h.count!=active.count||h.host_ns!=active.host_ns||seen[h.chunk]){++malformed;continue;}seen[h.chunk]=true;
   for(uint32_t j=0;j<count;j++){Point p;std::memcpy(&p,buf.data()+sizeof(h)+j*sizeof(Point),sizeof(p));if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)){++nonfinite_points;continue;}++valid_points;const std::array<double,3> v{p.x,p.y,p.z};for(int k=0;k<3;k++){lo[k]=std::min(lo[k],v[k]);hi[k]=std::max(hi[k],v[k]);}}
   if(std::all_of(seen.begin(),seen.end(),[](bool v){return v;})){
    if(last_cloud&&h.index>last_cloud+1)cloud_skips+=h.index-last_cloud-1;last_cloud=h.index;
    ++clouds;cloud_times.push_back(arrival);point_counts.push_back(h.count);original_counts.push_back(h.original);
    const double now_ns=std::chrono::duration<double,std::nano>(Clock::now().time_since_epoch()).count();ages.push_back((now_ns-double(h.host_ns))*1e-6);
   }
  }else ++malformed;
 }
 auto rate=[](const auto& v){return v.size()>1?(v.size()-1)/(v.back()-v.front()):0.;};
 auto gaps=[](const auto& v){std::vector<double> out;for(size_t i=1;i<v.size();i++)out.push_back((v[i]-v[i-1])*1000);return out;};
 const auto cg=gaps(cloud_times),ig=gaps(imu_times);
 nlohmann::json out={{"source","existing read-only JT16 viewer feed on 127.0.0.1:18776"},{"seconds",std::chrono::duration<double>(Clock::now()-start).count()},{"datagrams",datagrams},{"malformed",malformed},{"clouds",clouds},{"cloud_hz",rate(cloud_times)},{"cloud_gap_ms_p95",quantile(cg,.95)},{"cloud_gap_ms_max",quantile(cg,1)},{"points_per_cloud_median",quantile(point_counts,.5)},{"original_points_per_cloud_median",quantile(original_counts,.5)},{"incomplete_clouds",incomplete},{"cloud_index_gaps",cloud_skips},{"imu_callbacks",imus},{"imu_callback_hz",rate(imu_times)},{"imu_callback_gap_ms_p95",quantile(ig,.95)},{"imu_callback_gap_ms_max",quantile(ig,1)},{"imu_callback_index_gaps",imu_skips},{"imu_accel_norm_g_median",quantile(acc_norms,.5)},{"imu_gyro_norm_deg_s_median",quantile(gyro_norms,.5)},{"side_feed_delivery_age_ms_p95",quantile(ages,.95)},{"finite_points",valid_points},{"nonfinite_points",nonfinite_points},{"point_bounds_m",{{"min",lo},{"max",hi}}},{"sensor_timestamp_verified",false},{"raw_sensor_packet_loss_audited",false},{"warning","Best-effort local viewer feed, not raw packet audit. Point timestamps/rings absent. SDK may suppress identical IMU values. IMU units copied from existing SDK adapter, not calibrated here."}};
 std::cout<<out.dump(2)<<'\n';return clouds&&imus?0:1;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
