// Offline adapter around FAST-LIVO2 numerical modules. No ROS/serial output.
#include "vio.h"
// Upstream defines an unused non-inline comparator in this header; avoid an
// ODR collision with its numerical implementation translation unit.
#define time_list bhu_offline_unused_time_list
#include "IMU_Processing.h"
#undef time_list
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <stdexcept>
#include <sstream>
#include <memory>

namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
#pragma pack(push,1)
struct RawPoint {float x,y,z;double time;uint16_t ring;uint8_t intensity,confidence;};
#pragma pack(pop)
static_assert(sizeof(RawPoint)==24);
struct TimedPoint {PointType point;double time;};
struct Sample {double t;V3D v;};
struct Image {double t;fs::path path;};
static std::vector<std::string> split(const std::string& s){std::stringstream in(s);std::vector<std::string>v;std::string x;while(std::getline(in,x,',')){if(!x.empty()&&x.back()=='\r')x.pop_back();v.push_back(x);}return v;}
static std::vector<Sample> readImu(const fs::path& p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing IMU CSV");std::string s;std::getline(f,s);std::vector<Sample>v;while(std::getline(f,s)){auto x=split(s);if(x.size()!=4)throw std::runtime_error("Bad IMU CSV");v.push_back({std::stod(x[0]),{std::stod(x[1]),std::stod(x[2]),std::stod(x[3])}});}return v;}
static M3D eigen3(const cv::Mat& m){if(m.rows!=3||m.cols!=3)throw std::runtime_error("Bad rotation");M3D r;for(int i=0;i<3;i++)for(int j=0;j<3;j++)r(i,j)=m.at<double>(i,j);return r;}
static V3D vector3(const cv::Mat& m){if(m.total()!=3)throw std::runtime_error("Bad translation");return {m.at<double>(0),m.at<double>(1),m.at<double>(2)};}

int main(int argc,char** argv)try{
 cv::setNumThreads(2);omp_set_num_threads(1);
 if(argc<3||argc>4)throw std::runtime_error("Usage: livo_replay CONFIG.yaml NEW_OUTPUT [MAX_IMAGES]");
 if(fs::exists(argv[2]))throw std::runtime_error("Output already exists");
 cv::FileStorage config(argv[1],cv::FileStorage::READ);if(!config.isOpened())throw std::runtime_error("Missing config");
 std::string recording,decoded;config["recording"]>>recording;config["decoded"]>>decoded;
 double offset=double(config["lidar_to_camera_time_offset_s"]);
 cv::Mat rotation,translation;config["R_camera_lidar"]>>rotation;config["t_camera_lidar_m"]>>translation;
 M3D Rcl=eigen3(rotation);V3D tcl=vector3(translation);
 if(!Rcl.allFinite()||!tcl.allFinite()||(Rcl.transpose()*Rcl-M3D::Identity()).norm()>1e-6||std::abs(Rcl.determinant()-1)>1e-6)throw std::runtime_error("Invalid transform");
 fs::path input=recording,cloudRoot=decoded,out=argv[2];
 std::ifstream scans(cloudRoot/"scans.csv");if(!scans)throw std::runtime_error("Missing decoded scans");
 std::string line;std::getline(scans,line);std::vector<TimedPoint>points;
 double begin=0,end=0;while(std::getline(scans,line)){
  auto x=split(line);if(x.size()!=7)throw std::runtime_error("Bad scan index");
  if(points.empty())begin=std::stod(x[1])+offset;end=std::stod(x[2])+offset;
  std::ifstream file(cloudRoot/x[6],std::ios::binary);if(!file)throw std::runtime_error("Missing scan binary");
  RawPoint p;size_t n=0;while(file.read(reinterpret_cast<char*>(&p),sizeof(p))){++n;double range=std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);if(range<.3||range>60)continue;
   if(!std::isfinite(range)||!std::isfinite(p.time)||p.ring>=16)throw std::runtime_error("Invalid point");
   PointType q{};q.x=p.x;q.y=p.y;q.z=p.z;q.intensity=p.intensity;points.push_back({q,p.time+offset});}
  if(n!=std::stoull(x[4]))throw std::runtime_error("Scan size mismatch");
 }
 if(points.empty())throw std::runtime_error("No valid points");
 for(size_t i=1;i<points.size();++i)if(points[i].time<points[i-1].time)throw std::runtime_error("Nonmonotonic points");
 auto acc=readImu(input/"accel.csv"),gyro=readImu(input/"gyro.csv");
 std::vector<sensor_msgs::Imu::ConstPtr>imus;size_t ai=0;
 for(auto& g:gyro){if(g.t<begin-.5||g.t>end+.05)continue;while(ai+1<acc.size()&&acc[ai+1].t<g.t)++ai;if(ai+1>=acc.size()||g.t<acc[ai].t)continue;
  if(acc[ai+1].t-acc[ai].t>.03)throw std::runtime_error("Acceleration interpolation gap");
  double fraction=(g.t-acc[ai].t)/(acc[ai+1].t-acc[ai].t);V3D a=acc[ai].v*(1-fraction)+acc[ai+1].v*fraction;
  auto m=std::make_shared<sensor_msgs::Imu>();m->header.stamp.fromSec(g.t-begin);m->linear_acceleration={a.x(),a.y(),a.z()};m->angular_velocity={g.v.x(),g.v.y(),g.v.z()};imus.push_back(m);
 }
 std::vector<Image>images;std::ifstream ff(input/"frames.csv");std::getline(ff,line);while(std::getline(ff,line)){auto x=split(line);double t=std::stod(x.at(0));if(t>begin+.1&&t<end-.05)images.push_back({t,input/x.at(1)});}
 if(images.empty()||imus.empty())throw std::runtime_error("No overlapping images/IMU");
 fs::create_directories(out);fs::create_directories(fs::path(ROOT_DIR)/"Log");
 fs::copy_file(argv[1],out/"config.yaml");
 std::ofstream poses(out/"trajectory.csv"),diag(out/"diagnostics.csv");poses<<"timestamp_s,x,y,z,qx,qy,qz,qw\n"<<std::setprecision(16);
 diag<<"timestamp_s,points,downsampled,lidar_constraints,visual_points,imu_initializing,processing_ms,map_voxels\n"<<std::setprecision(16);
 cv::FileStorage cameraCalibration((input/"orb.yaml").string(),cv::FileStorage::READ);
 cv::Mat TIC;cameraCalibration["IMU.T_b_c1"]>>TIC;TIC.convertTo(TIC,CV_64F);
 if(TIC.rows!=4||TIC.cols!=4)throw std::runtime_error("Missing camera/IMU transform");
 M3D Ric=eigen3(TIC(cv::Rect(0,0,3,3)));V3D tic(TIC.at<double>(0,3),TIC.at<double>(1,3),TIC.at<double>(2,3));
 StatesGroup state,propagated;M3D Ril=Ric*Rcl;V3D til=Ric*tcl+tic;
 ImuProcess imu;imu.set_extrinsic(til,Ril);imu.set_imu_init_frame_num(80);
 imu.set_gyr_cov_scale(V3D::Constant(.01));imu.set_acc_cov_scale(V3D::Constant(.1));imu.set_gyr_bias_cov(V3D::Constant(.0001));imu.set_acc_bias_cov(V3D::Constant(.0001));imu.set_inv_expo_cov(.1);
 VoxelMapConfig vc{};ros::NodeHandle nh;loadVoxelConfig(nh,vc);vc.max_layer_=2;vc.max_voxel_size_=.5;vc.planner_threshold_=.0025;vc.beam_err_=.05;vc.dept_err_=.02;
 if(!config["voxel_size_m"].empty())vc.max_voxel_size_=double(config["voxel_size_m"]);
 if(!config["plane_eigen_threshold"].empty())vc.planner_threshold_=double(config["plane_eigen_threshold"]);
 if(vc.max_voxel_size_<=0||vc.planner_threshold_<=0)throw std::runtime_error("Invalid voxel/plane settings");
 std::unordered_map<VOXEL_LOCATION,VoxelOctoTree*> map;VoxelMapManager manager(vc,map);manager.extR_=Ril;manager.extT_=til;
 vk::PinholeCamera camera{double(cameraCalibration["Camera.width"]),double(cameraCalibration["Camera.height"]),1.,double(cameraCalibration["Camera1.fx"]),double(cameraCalibration["Camera1.fy"]),double(cameraCalibration["Camera1.cx"]),double(cameraCalibration["Camera1.cy"]),0,0,0,0,0};
 VIOManager visual;visual.cam=&camera;visual.state=&state;visual.state_propagat=&propagated;visual.grid_size=20;visual.grid_n_height=20;visual.grid_n_width=32;
 visual.patch_size=8;visual.patch_pyrimid_level=3;visual.max_iterations=5;visual.img_point_cov=100;visual.outlier_threshold=1000;
 visual.normal_en=true;visual.inverse_composition_en=false;visual.raycast_en=false;visual.exposure_estimate_en=true;visual.colmap_output_en=false;visual.plot_flag=false;
 visual.setImuToLidarExtrinsic(til,Ril);std::vector<double>r(9),t{tcl.x(),tcl.y(),tcl.z()};for(int i=0;i<3;i++)for(int j=0;j<3;j++)r[3*i+j]=Rcl(i,j);visual.setLidarToCameraExtrinsic(r,t);visual.initializeVIO();
 LidarMeasureGroup measures;measures.last_lio_update_time=0;PointCloudXYZI::Ptr deskew(new PointCloudXYZI()),down(new PointCloudXYZI());pcl::VoxelGrid<PointType>filter;filter.setLeafSize(.15,.15,.15);
 bool built=false;size_t pi=0,ii=0,processed=0;const size_t limit=argc==4?std::stoull(argv[3]):images.size();double previous=0;auto start=Clock::now();
 for(const auto& image:images){if(processed>=limit)break;auto tick=Clock::now();const double target=image.t-begin;
  MeasureGroup group;group.lio_time=target;
  while(ii<imus.size()&&imus[ii]->header.stamp.toSec()<=target){if(imus[ii]->header.stamp.toSec()>previous)group.imu.push_back(imus[ii]);++ii;}
  if(group.imu.empty())throw std::runtime_error("No fresh IMU for image");
  measures.measures.clear();measures.measures.push_back(group);measures.lio_vio_flg=LIO;measures.pcl_proc_cur->clear();
  while(pi<points.size()&&points[pi].time<image.t){auto q=points[pi].point;double rel=points[pi].time-begin;if(rel>=previous){q.curvature=(rel-measures.last_lio_update_time)*1000.;measures.pcl_proc_cur->push_back(q);}++pi;}
  deskew->clear();imu.Process2(measures,state,deskew);propagated=state;manager.state_=state;manager.feats_undistort_=deskew;
  int constraints=0;size_t downsampled=0;visual.total_points=0;
  if(!imu.imu_need_init&&!deskew->empty()){
   filter.setInputCloud(deskew);filter.filter(*down);downsampled=down->size();manager.feats_down_body_=down;manager.feats_down_size_=down->size();
   manager.feats_down_world_->clear();
   for(const auto& p:down->points){auto q=p;V3D w=state.rot_end*(Ril*V3D(p.x,p.y,p.z)+til)+state.pos_end;q.x=w.x();q.y=w.y();q.z=w.z();manager.feats_down_world_->push_back(q);}
   if(!built){manager.BuildVoxelMap();built=true;}
   manager.StateEstimation(propagated);state=manager.state_;constraints=manager.effct_feat_num_;
   for(size_t i=0;i<down->size();++i){const auto& p=down->points[i];auto& pv=manager.pv_list_[i];pv.point_w=state.rot_end*(Ril*V3D(p.x,p.y,p.z)+til)+state.pos_end;M3D cross=manager.cross_mat_list_[i];
    pv.var=(state.rot_end*Ril)*manager.body_cov_list_[i]*(state.rot_end*Ril).transpose()+(-cross)*state.cov.block<3,3>(0,0)*(-cross).transpose()+state.cov.block<3,3>(3,3);}
   manager.UpdateVoxelMap(manager.pv_list_);
  }
  // Camera updates remain independent of whether this LiDAR slice had returns.
  if(!imu.imu_need_init&&built){
   cv::Mat frame=cv::imread(image.path.string(),cv::IMREAD_COLOR);if(frame.empty())throw std::runtime_error("Missing camera image");
   visual.processFrame(frame,manager.pv_list_,manager.voxel_map_,target);
   if(!state.pos_end.allFinite()||!state.rot_end.allFinite()||state.pos_end.norm()>10000||state.vel_end.norm()>100)throw std::runtime_error("Estimator diverged");
   // Output the left camera origin, matching the baseline's pose reference.
   const V3D cameraPosition=state.pos_end+state.rot_end*tic;Eigen::Quaterniond q(state.rot_end*Ric);
   poses<<image.t<<','<<cameraPosition.x()<<','<<cameraPosition.y()<<','<<cameraPosition.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<'\n';
  }
  double ms=std::chrono::duration<double,std::milli>(Clock::now()-tick).count();diag<<image.t<<','<<deskew->size()<<','<<downsampled<<','<<constraints<<','<<(built?visual.total_points:0)<<','<<imu.imu_need_init<<','<<ms<<','<<manager.voxel_map_.size()<<'\n';
  previous=target;++processed;if(processed%100==0){poses.flush();diag.flush();std::cerr<<"REPLAY_IMAGES "<<processed<<" / "<<std::min(limit,images.size())<<'\n';}
 }
 poses.flush();diag.flush();if(!poses||!diag)throw std::runtime_error("Replay write failed");
 std::ofstream status(out/"summary.json");status<<"{\"processed_images\":"<<processed<<",\"seconds\":"<<std::chrono::duration<double>(Clock::now()-start).count()<<",\"calibration_validated\":false,\"accuracy_validated\":false,\"flight_output\":false}\n";
 return 0;
}catch(const std::exception& e){std::cerr<<"LIVO_REPLAY_ERROR "<<e.what()<<'\n';return 2;}
