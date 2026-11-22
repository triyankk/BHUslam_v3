// Isolated JT16-only mapping adapter. No camera, Cube or MAVLink interface.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <json.hpp>
#include "bhu/flight_recording.hpp"
#include <arpa/inet.h>
#include <poll.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <fcntl.h>
#include <iostream>
#include <unordered_map>
#include <vector>
namespace fs=std::filesystem;using J=nlohmann::json;using Clock=std::chrono::steady_clock;
#pragma pack(push,1)
struct Header{char magic[8];uint64_t host_ns,index;uint32_t count,original;uint16_t chunk,chunks;};
struct Point{float x,y,z;double t;uint16_t ring;uint8_t intensity,confidence;};
struct Imu{char magic[8];uint64_t host_ns,index;double v[7];};
#pragma pack(pop)
static_assert(sizeof(Header)==36&&sizeof(Point)==24&&sizeof(Imu)==80);
struct Cell{int x,y,z;bool operator==(const Cell&c)const{return x==c.x&&y==c.y&&z==c.z;}};
struct Hash{size_t operator()(const Cell&c)const{return size_t(c.x)*73856093u^size_t(c.y)*19349663u^size_t(c.z)*83492791u;}};
struct Average{double x=0,y=0,z=0;uint64_t n=0;};
static builtin_interfaces::msg::Time stamp(double t){if(!std::isfinite(t)||t<=0||t>=2147483647.)throw std::runtime_error("Invalid sensor clock");int64_t ns=std::llround(t*1e9);builtin_interfaces::msg::Time s;s.sec=ns/1000000000;s.nanosec=ns%1000000000;return s;}
static sensor_msgs::msg::PointField field(const std::string&n,unsigned o,unsigned type){sensor_msgs::msg::PointField f;f.name=n;f.offset=o;f.datatype=type;f.count=1;return f;}
int main(int argc,char**argv)try{
 const bool flightMode=argc==6&&std::string(argv[4])=="--flight-status";
 if((argc!=4&&!flightMode)||std::string(argv[3])!="--allow-provisional")throw std::runtime_error("Usage: jt16_map_bridge NEW_OUTPUT SECONDS|continuous --allow-provisional [--flight-status FILE]");
 const bool continuous=std::string(argv[2])=="continuous";
 fs::path out=argv[1];double seconds=continuous?0:std::stod(argv[2]);if(fs::exists(out)||(!continuous&&(!std::isfinite(seconds)||seconds<5||seconds>600)))throw std::runtime_error("New output required; duration must be 5..600 s or continuous");
 fs::create_directories(out);std::ofstream imuLog,poses,cloudLog;fs::path sessionOut=out;
 bhu::FlightRecording flights(flightMode?argv[5]:"");bool recording=!flightMode,hadFlight=false;
 auto openLogs=[&](const fs::path& directory){fs::create_directories(directory);imuLog.open(directory/"imu.csv");poses.open(directory/"trajectory.csv");cloudLog.open(directory/"clouds.csv");
  imuLog<<"sensor_s,host_ns,ax_m_s2,ay_m_s2,az_m_s2,gx_rad_s,gy_rad_s,gz_rad_s\n"<<std::setprecision(16);
  poses<<"sensor_s,x,y,z,qx,qy,qz,qw,cov_x,cov_y,cov_z\n"<<std::setprecision(16);
  cloudLog<<"sensor_begin_s,sensor_end_s,host_ns,input_points,accepted_points\n"<<std::setprecision(16);};
 if(recording)openLogs(out);
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("bhu_jt16_map_bridge");
 auto cloudPub=node->create_publisher<sensor_msgs::msg::PointCloud2>("/bhu/jt16/points",rclcpp::SensorDataQoS());
 auto imuPub=node->create_publisher<sensor_msgs::msg::Imu>("/bhu/jt16/imu",rclcpp::SensorDataQoS().keep_last(400));
 std::unordered_map<Cell,Average,Hash> voxels;uint64_t poseCount=0,registered=0,mapOverflow=0,malformed=0,invalidScans=0,incomplete=0,cloudCount=0,imuCount=0;J trajectory=J::array();
 auto odomSub=node->create_subscription<nav_msgs::msg::Odometry>("/Odometry",10,[&](nav_msgs::msg::Odometry::ConstSharedPtr m){
  if(!recording)return;
  const auto&p=m->pose.pose.position;const auto&q=m->pose.pose.orientation;for(double v:{p.x,p.y,p.z,q.x,q.y,q.z,q.w})if(!std::isfinite(v))throw std::runtime_error("Nonfinite LIO odometry");
  double t=m->header.stamp.sec+m->header.stamp.nanosec*1e-9;poses<<t<<','<<p.x<<','<<p.y<<','<<p.z<<','<<q.x<<','<<q.y<<','<<q.z<<','<<q.w<<','<<m->pose.covariance[0]<<','<<m->pose.covariance[7]<<','<<m->pose.covariance[14]<<'\n';
  if(trajectory.size()<50000)trajectory.push_back({t,p.x,p.y,p.z});if(++poseCount==1)std::cout<<"JT16_MAP_TRACKING_STARTED (provisional calibration)\n"<<std::flush;
 });
 auto mapSub=node->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_registered",10,[&](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){
  if(!recording)return;
  if(m->header.frame_id!="camera_init")throw std::runtime_error("Unexpected registered-cloud frame");
  std::array<int,3> offsets{-1,-1,-1};for(const auto&f:m->fields)for(int k=0;k<3;k++)if(f.name==std::string(1,"xyz"[k])&&f.datatype==7)offsets[k]=f.offset;
  if(m->is_bigendian||std::any_of(offsets.begin(),offsets.end(),[](int v){return v<0;})||!m->point_step||m->row_step<uint64_t(m->width)*m->point_step||m->data.size()<uint64_t(m->row_step)*m->height)throw std::runtime_error("Malformed registered cloud");
  for(int o:offsets)if(unsigned(o)+4>m->point_step)throw std::runtime_error("Invalid registered point field offset");
  for(unsigned y=0;y<m->height;y++)for(unsigned x=0;x<m->width;x++){
   std::array<float,3> p{};for(int k=0;k<3;k++)std::memcpy(&p[k],m->data.data()+y*m->row_step+x*m->point_step+offsets[k],4);
   if(!std::all_of(p.begin(),p.end(),[](float v){return std::isfinite(v)&&std::abs(v)<10000;}))continue;
   Cell key{int(std::floor(p[0]/.2)),int(std::floor(p[1]/.2)),int(std::floor(p[2]/.2))};auto it=voxels.find(key);
   if(it==voxels.end()){if(voxels.size()>=250000){++mapOverflow;continue;}it=voxels.emplace(key,Average{}).first;}
   auto&a=it->second;a.x+=p[0];a.y+=p[1];a.z+=p[2];++a.n;
  }++registered;
 });
 const int fd=socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);if(fd<0)throw std::runtime_error("socket failed");struct Close{int fd;~Close(){close(fd);}} closer{fd};
 int bytes=4*1024*1024;setsockopt(fd,SOL_SOCKET,SO_RCVBUF,&bytes,sizeof(bytes));sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);addr.sin_port=htons(18777);
 if(bind(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr)))throw std::runtime_error("Full mapping feed port 18777 occupied");
 Header active{};std::vector<bool> seen;std::vector<Point> points;uint64_t index=0;double lastImu=-1,lastCloud=-1;auto start=Clock::now(),lastArrival=start,sessionStart=start;
 auto save=[&](const fs::path& target,bool final){
  fs::create_directories(target);J map=J::array();std::ofstream ply(target/"map.ply");ply<<"ply\nformat ascii 1.0\nelement vertex "<<voxels.size()<<"\nproperty float x\nproperty float y\nproperty float z\nend_header\n"<<std::setprecision(7);
  for(const auto&[k,a]:voxels){double x=a.x/a.n,y=a.y/a.n,z=a.z/a.n;map.push_back({x,y,z});ply<<x<<' '<<y<<' '<<z<<'\n';}
  ply.flush();if(!ply)throw std::runtime_error("Map write failed");
  J summary={{"algorithm","Hesai FAST-LIO2 C++ with JT16 built-in IMU"},{"camera_used",false},{"cube_interface",false},{"calibration_validated",false},{"imu_units","m/s^2 and rad/s"},{"time_basis","unmodified shared device sensor clock; not verified UTC"},{"clouds_published",cloudCount},{"imu_published",imuCount},{"odometry_samples",poseCount},{"trajectory_export_samples",trajectory.size()},{"trajectory_export_limit",50000},{"registered_clouds",registered},{"map_voxels",voxels.size()},{"voxel_m",.2},{"map_voxel_limit",250000},{"map_overflow_points",mapOverflow},{"malformed_feed",malformed},{"invalid_scans",invalidScans},{"incomplete_scans",incomplete},{"seconds",std::chrono::duration<double>(Clock::now()-sessionStart).count()},{"recording_start_host_us",std::chrono::duration_cast<std::chrono::microseconds>(sessionStart.time_since_epoch()).count()},{"continuous",continuous},{"finalized",final},{"loop_closure",false},{"warning","Provisional identity lidar/IMU extrinsics and unmeasured noise; shadow mapping only."}};
  {std::ofstream f(target/"summary.json");f<<summary.dump(2)<<'\n';f.flush();if(!f)throw std::runtime_error("Summary write failed");}
  {std::ofstream f(target/"map-data.json");f<<J{{"summary",summary},{"points",map},{"trajectory",trajectory}}.dump()<<'\n';f.flush();if(!f)throw std::runtime_error("Map JSON write failed");}
  for(const auto&name:{"map.ply","summary.json","map-data.json"}){int f=::open((target/name).c_str(),O_RDONLY|O_CLOEXEC);if(f<0||::fsync(f)){if(f>=0)::close(f);throw std::runtime_error("Checkpoint sync failed");}::close(f);}
  return summary;
 };
 auto lastFlush=start,lastCheckpoint=start,lastFlightPoll=start-std::chrono::seconds(1);unsigned checkpoint=0;
 std::array<char,65536> buffer{};
 std::cout<<"JT16_MAP_LISTENING: keep sensor still during initialization\n"<<std::flush;
 while(rclcpp::ok()&&(continuous||std::chrono::duration<double>(Clock::now()-start).count()<seconds)){
  if(flightMode&&Clock::now()-lastFlightPoll>=std::chrono::milliseconds(200)){
   lastFlightPoll=Clock::now();auto now=std::chrono::duration_cast<std::chrono::microseconds>(lastFlightPoll.time_since_epoch()).count();auto event=flights.poll(now);
   if(event==bhu::FlightRecording::Start){sessionOut=out/"flights"/("flight-"+std::to_string(now));openLogs(sessionOut);voxels.clear();trajectory=J::array();poseCount=registered=mapOverflow=malformed=invalidScans=incomplete=cloudCount=imuCount=0;recording=true;hadFlight=true;checkpoint=0;sessionStart=Clock::now();lastCheckpoint=sessionStart;std::cout<<"MAPPING_RECORDING_STARTED "<<sessionOut<<std::endl;}
   else if(event==bhu::FlightRecording::Stop){imuLog.flush();poses.flush();cloudLog.flush();save(sessionOut,true);imuLog.close();poses.close();cloudLog.close();recording=false;std::cout<<"MAPPING_RECORDING_FINALIZED "<<sessionOut<<std::endl;}
  }
  if(recording&&Clock::now()-lastFlush>std::chrono::seconds(1)){imuLog.flush();poses.flush();cloudLog.flush();if(!imuLog||!poses||!cloudLog)throw std::runtime_error("CSV write failed");lastFlush=Clock::now();}
  if(recording&&continuous&&poseCount&&Clock::now()-lastCheckpoint>std::chrono::seconds(checkpoint?30:5)){
   auto parent=sessionOut/"checkpoints";auto name=std::to_string(++checkpoint);auto stage=parent/(name+".tmp");save(stage,false);fs::rename(stage,parent/name);
   auto temp=sessionOut/"latest-checkpoint.tmp";fs::remove(temp);fs::create_directory_symlink(fs::path("checkpoints")/name,temp);fs::rename(temp,sessionOut/"latest-checkpoint");lastCheckpoint=Clock::now();
  }
  rclcpp::spin_some(node);pollfd wait{fd,POLLIN,0};if(poll(&wait,1,2)<=0){if(Clock::now()-lastArrival>std::chrono::seconds(10))throw std::runtime_error("No mapping feed for 10 seconds");continue;}
  auto n=recv(fd,buffer.data(),buffer.size(),0);if(n<8){++malformed;continue;}lastArrival=Clock::now();
  if(!std::memcmp(buffer.data(),"JT16VIM1",8)){
   if(n!=sizeof(Imu)){++malformed;continue;}Imu m;std::memcpy(&m,buffer.data(),sizeof(m));
   if(!std::all_of(std::begin(m.v),std::end(m.v),[](double v){return std::isfinite(v);})||m.v[0]<=lastImu){++malformed;continue;}lastImu=m.v[0];
   sensor_msgs::msg::Imu msg;msg.header.stamp=stamp(m.v[0]);msg.header.frame_id="jt16_imu";msg.orientation_covariance[0]=-1;
   msg.linear_acceleration.x=m.v[1]*9.80665;msg.linear_acceleration.y=m.v[2]*9.80665;msg.linear_acceleration.z=m.v[3]*9.80665;
   msg.angular_velocity.x=m.v[4]*M_PI/180.;msg.angular_velocity.y=m.v[5]*M_PI/180.;msg.angular_velocity.z=m.v[6]*M_PI/180.;imuPub->publish(msg);
   if(recording){imuLog<<m.v[0]<<','<<m.host_ns;for(int k=1;k<=6;k++)imuLog<<','<<m.v[k]*(k<=3?9.80665:M_PI/180.);imuLog<<'\n';++imuCount;}
  }else if(!std::memcmp(buffer.data(),"JT16MPC1",8)){
   if(n<sizeof(Header)){++malformed;continue;}Header h;std::memcpy(&h,buffer.data(),sizeof(h));
   if(!h.count||h.count>20000||h.count!=h.original||h.chunks!=(h.count+511)/512||h.chunk>=h.chunks){++malformed;continue;}
   unsigned count=std::min(512u,h.count-h.chunk*512);if(n!=sizeof(h)+count*sizeof(Point)){++malformed;continue;}
   if(h.index!=index){if(!seen.empty()&&!std::all_of(seen.begin(),seen.end(),[](bool v){return v;}))++incomplete;index=h.index;active=h;seen.assign(h.chunks,false);points.resize(h.count);}
   if(h.count!=active.count||h.host_ns!=active.host_ns||seen[h.chunk]){++malformed;continue;}seen[h.chunk]=true;std::memcpy(points.data()+h.chunk*512,buffer.data()+sizeof(h),count*sizeof(Point));
   if(!std::all_of(seen.begin(),seen.end(),[](bool v){return v;}))continue;
   bool valid=true;double begin=points.front().t,end=points.back().t;
   for(size_t j=0;j<points.size();j++)if(!std::isfinite(points[j].t)||(j&&points[j].t<points[j-1].t))valid=false;
   if(!valid||end-begin<.10||end-begin>.30||begin<=lastCloud){++invalidScans;continue;}lastCloud=begin;
   sensor_msgs::msg::PointCloud2 msg;msg.header.stamp=stamp(begin);msg.header.frame_id="jt16_lidar";msg.height=1;msg.is_dense=true;msg.point_step=32;
   msg.fields={field("x",0,7),field("y",4,7),field("z",8,7),field("intensity",16,7),field("ring",20,4),field("timestamp",24,8)};
   double acceptedBegin=0;
   for(const auto&p:points){double norm=double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z;if(!std::isfinite(norm)||norm<.09||norm>900||p.ring>=16)continue;
    if(!msg.width)acceptedBegin=p.t;
    size_t offset=msg.data.size();msg.data.resize(offset+32);std::memcpy(msg.data.data()+offset,&p.x,12);float intensity=p.intensity;std::memcpy(msg.data.data()+offset+16,&intensity,4);std::memcpy(msg.data.data()+offset+20,&p.ring,2);std::memcpy(msg.data.data()+offset+24,&p.t,8);++msg.width;
   }
   if(msg.width<32){++invalidScans;continue;}msg.header.stamp=stamp(acceptedBegin);msg.row_step=msg.point_step*msg.width;cloudPub->publish(msg);if(recording){cloudLog<<begin<<','<<end<<','<<h.host_ns<<','<<h.count<<','<<msg.width<<'\n';++cloudCount;}
  }else ++malformed;
 }
 for(int i=0;i<10&&rclcpp::ok();i++){rclcpp::spin_some(node);usleep(10000);}
 if(recording){auto summary=save(sessionOut,true);std::cout<<summary.dump(2)<<'\n';}
 rclcpp::shutdown();return flightMode?0:(poseCount&&voxels.size()?0:1);
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 1;}
