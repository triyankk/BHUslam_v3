// Offline stationary-recording assessment. No device access or calibration writes.
#include "bhu/imu.hpp"
#include <json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
namespace fs=std::filesystem;
using J=nlohmann::json;using V=cv::Vec3d;
static std::vector<std::string> split(std::string s){std::vector<std::string> v;std::stringstream in(s);while(std::getline(in,s,','))v.push_back(s);return v;}
static std::vector<bhu::Sample> read(const fs::path&p){std::ifstream in(p);if(!in)throw std::runtime_error("Cannot read "+p.string());std::string s;std::getline(in,s);std::vector<bhu::Sample> v;while(std::getline(in,s)){auto c=split(s);if(c.size()!=4)throw std::runtime_error("Malformed IMU CSV");v.push_back({std::stod(c[0]),{std::stod(c[1]),std::stod(c[2]),std::stod(c[3])}});}bhu::validate(v);return v;}
static V xyz(const bhu::Sample&s){return {s.v[0],s.v[1],s.v[2]};}
static J json(V x){return J::array({x[0],x[1],x[2]});}
static double percentile(std::vector<double>x,double p){if(x.empty())throw std::runtime_error("No samples for statistic");std::sort(x.begin(),x.end());return x[std::min(x.size()-1,size_t(p*(x.size()-1)))];}
struct Stats{size_t n=0;V mean{0,0,0},sd{0,0,0};double normMean=0;};
static Stats stats(const std::vector<bhu::Sample>&s,double a,double b){Stats r;for(auto&v:s)if(v.t>=a&&v.t<b){++r.n;r.mean+=xyz(v);r.normMean+=cv::norm(xyz(v));}if(r.n<2)throw std::runtime_error("Insufficient samples in window");r.mean*=1./r.n;r.normMean/=r.n;for(auto&v:s)if(v.t>=a&&v.t<b){V d=xyz(v)-r.mean;for(int i=0;i<3;++i)r.sd[i]+=d[i]*d[i];}for(int i=0;i<3;++i)r.sd[i]=std::sqrt(r.sd[i]/(r.n-1));return r;}
static J json(Stats s){return {{"n",s.n},{"mean",json(s.mean)},{"stddev",json(s.sd)},{"mean_vector_norm",cv::norm(s.mean)},{"mean_sample_norm",s.normMean}};}
static V allan(const std::vector<bhu::Sample>&s,size_t m){if(!m||s.size()<2*m)throw std::runtime_error("Invalid Allan cluster size");std::vector<V> prefix(s.size()+1,V(0,0,0));for(size_t i=0;i<s.size();++i)prefix[i+1]=prefix[i]+xyz(s[i]);V sum(0,0,0);size_t n=s.size()-2*m+1;for(size_t i=0;i<n;++i){V d=(prefix[i+2*m]-2*prefix[i+m]+prefix[i])/double(m);for(int k=0;k<3;++k)sum[k]+=d[k]*d[k];}for(int k=0;k<3;++k)sum[k]=std::sqrt(sum[k]/(2*n));return sum;}
int main(int argc,char**argv)try{
    if(argc==2&&std::string(argv[1])=="--self-test"){
        std::vector<bhu::Sample>s;for(int i=0;i<1000;++i)s.push_back({i*.01,{double(i),2.,-3.}});auto a=allan(s,10);auto b=stats(s,0,10);if(std::abs(a[0]-10/std::sqrt(2.))>1e-10||a[1]!=0||a[2]!=0||std::abs(b.mean[0]-499.5)>1e-10)throw std::runtime_error("Statistics/Allan test failed");std::cout<<"Constant-channel Allan deviation, linear ramp and mean checks passed\n";return 0;
    }
    if(argc!=3){std::cerr<<"Usage: gemini_imu_stationary RECORDING NEW_OUTPUT\n";return 2;}
    fs::path root=argv[1],out=argv[2];if(fs::exists(out))throw std::runtime_error("Output exists");auto g=read(root/"gyro.csv"),a=read(root/"accel.csv");std::ifstream framesFile(root/"frames.csv");std::string line;std::getline(framesFile,line);struct F{double t;fs::path l;};std::vector<F> frames;while(std::getline(framesFile,line)){auto c=split(line);frames.push_back({std::stod(c.at(0)),root/c.at(1)});}if(frames.size()<2)throw std::runtime_error("No frames");double t0=frames.front().t,end=frames.back().t;fs::create_directories(out);
    J report={{"dataset",fs::absolute(root).string()},{"units",{{"gyro","rad/s"},{"accel","m/s^2"}}},{"duration_s",end-t0},{"warning","Stationarity supported by image consistency is not ground truth. Short-run statistics do not calibrate random walk or all accelerometer biases."}};
    std::ifstream meta(root/"recording.json");if(meta)meta>>report["recording"];
    report["full"]={{"gyro",json(stats(g,t0,end))},{"accel",json(stats(a,t0,end))}};
    report["settled_after_10s"]={{"gyro",json(stats(g,t0+10,end))},{"accel",json(stats(a,t0+10,end))}};
    auto start=stats(g,t0+10,t0+30),finish=stats(g,end-20,end);report["gyro_last20_minus_early20_rad_s"]=json(finish.mean-start.mean);
    J windows=J::array();std::ofstream win(out/"windows.csv");win<<"start_elapsed_s,gyro_mean_x,gyro_mean_y,gyro_mean_z,gyro_std_x,gyro_std_y,gyro_std_z,acc_mean_norm,acc_std_x,acc_std_y,acc_std_z\n";
    cv::Mat plot(850,1200,CV_8UC3,cv::Scalar(255,255,255));std::vector<std::vector<cv::Point>> tracks(4);V baseline=stats(g,t0+10,end).mean;
    for(double t=t0;t+9.9<end;t+=10){auto gs=stats(g,t,std::min(t+10,end)),as=stats(a,t,std::min(t+10,end));windows.push_back({{"elapsed_s",t-t0},{"gyro",json(gs)},{"accel",json(as)}});win<<t-t0;for(double v:gs.mean.val)win<<','<<v;for(double v:gs.sd.val)win<<','<<v;win<<','<<cv::norm(as.mean);for(double v:as.sd.val)win<<','<<v;win<<'\n';int x=100+int((t-t0+5)/(end-t0)*1000);for(int k=0;k<3;++k)tracks[k].push_back({x,150+k*170-int((gs.mean[k]-baseline[k])*1e5)});tracks[3].push_back({x,720-int((cv::norm(as.mean)-9.77)*1000)});}
    report["windows_10s"]=windows;
    for(int k=0;k<4;++k){int y=k<3?150+k*170:720;cv::line(plot,{100,y},{1100,y},{180,180,180});std::string title=k<3?"Gyro "+std::string(1,"XYZ"[k])+": 10s mean minus settled mean (100 px = 0.001 rad/s)":"Acceleration mean norm: 100 px = 0.1 m/s^2, baseline 9.77";cv::putText(plot,title,{30,y-65},cv::FONT_HERSHEY_SIMPLEX,.55,{0,0,0},1);cv::polylines(plot,tracks[k],false,{180,60,20},2);for(auto p:tracks[k])cv::circle(plot,p,4,{180,60,20},-1);}
    cv::putText(plot,"Elapsed seconds:   0                    30                    60                    90                   120",{90,825},cv::FONT_HERSHEY_SIMPLEX,.5,{0,0,0});cv::imwrite((out/"imu-window-means.png").string(),plot);
    J adev=J::object();std::ofstream af(out/"allan.csv");af<<"sensor,tau_s,cluster_samples,overlapping_differences,x,y,z\n";for(auto sensor:{std::string("gyro"),std::string("accel")}){auto&s=sensor=="gyro"?g:a;std::vector<bhu::Sample> settled;std::vector<double> dt;for(size_t i=1;i<s.size();++i)dt.push_back(s[i].t-s[i-1].t);double step=percentile(dt,.5);for(auto v:s)if(v.t>=t0+10&&v.t<end)settled.push_back(v);J points=J::array();for(double tau:{.01,.02,.05,.1,.2,.5,1.,2.,5.,10.}){size_t m=std::max(size_t(1),size_t(llround(tau/step)));V d=allan(settled,m);points.push_back({{"tau_s",m*step},{"deviation",json(d)},{"overlapping_differences",settled.size()-2*m+1}});af<<sensor<<','<<m*step<<','<<m<<','<<settled.size()-2*m+1;for(double v:d.val)af<<','<<v;af<<'\n';}adev[sensor]=points;}report["overlapping_allan_short_record"]=adev;
    // Match a sampled image against the first image. This detects scene motion
    // independently of IMU bias; the pixel displacement includes feature noise.
    auto anchorFrame=std::lower_bound(frames.begin(),frames.end(),t0+10,[](const F&f,double t){return f.t<t;});if(anchorFrame==frames.end())throw std::runtime_error("Recording shorter than settling interval");
    auto orb=cv::ORB::create(1000);auto anchor=cv::imread(anchorFrame->l.string(),0);if(anchor.empty())throw std::runtime_error("Missing anchor image");cv::Mat enhancedAnchor;cv::normalize(anchor,enhancedAnchor,0,255,cv::NORM_MINMAX);std::vector<cv::KeyPoint> ka;cv::Mat da;orb->detectAndCompute(enhancedAnchor,cv::noArray(),ka,da);std::vector<double> shifts,differences,counts,means;std::ofstream im(out/"visual-stability.csv");im<<"elapsed_s,ratio_matches,homography_inliers,median_anchor_displacement_px,mean_abs_pixel_difference\n";cv::Mat contact(400,3*320,CV_8UC3,cv::Scalar(0,0,0));double last=-2;int tile=0;
    for(auto&f:frames){double elapsed=f.t-t0;if(elapsed-last<.99)continue;last=elapsed;auto image=cv::imread(f.l.string(),0);if(image.empty())throw std::runtime_error("Missing sampled image");cv::Mat diff,enhanced;cv::normalize(image,enhanced,0,255,cv::NORM_MINMAX);cv::absdiff(image,anchor,diff);double mad=cv::mean(diff)[0];differences.push_back(mad);means.push_back(cv::mean(image)[0]);std::vector<cv::KeyPoint> kb;cv::Mat db;orb->detectAndCompute(enhanced,cv::noArray(),kb,db);std::vector<cv::Point2f> pa,pb;if(!da.empty()&&!db.empty()){std::vector<std::vector<cv::DMatch>> matches;cv::BFMatcher(cv::NORM_HAMMING).knnMatch(da,db,matches,2);for(auto&m:matches)if(m.size()==2&&m[0].distance<.7*m[1].distance){pa.push_back(ka[m[0].queryIdx].pt);pb.push_back(kb[m[0].trainIdx].pt);}}std::vector<double> displacement;cv::Mat mask;if(pa.size()>=8)cv::findHomography(pa,pb,cv::RANSAC,2,mask);for(size_t i=0;i<pa.size();++i)if(!mask.empty()&&mask.at<uchar>(int(i)))displacement.push_back(cv::norm(pa[i]-pb[i]));double shift=displacement.size()<20?-1:percentile(displacement,.5);if(shift>=0&&elapsed>=10&&std::abs(f.t-anchorFrame->t)>.01)shifts.push_back(shift);counts.push_back(displacement.size());im<<elapsed<<','<<pa.size()<<','<<displacement.size()<<','<<shift<<','<<mad<<'\n';
        if(tile<6&&elapsed>=tile*(end-t0-1)/5){cv::Mat thumb;cv::resize(enhanced,thumb,{320,200});cv::cvtColor(thumb,thumb,cv::COLOR_GRAY2BGR);cv::putText(thumb,cv::format("%.1f s (contrast enhanced)",elapsed),{8,22},cv::FONT_HERSHEY_SIMPLEX,.45,{0,0,255},1);thumb.copyTo(contact(cv::Rect(tile%3*320,tile/3*200,320,200)));++tile;}}
    report["visual_stability"]={{"sampled_images",counts.size()},{"images_with_match_estimate",shifts.size()},{"anchor_displacement_median_px",shifts.empty()?J():J(percentile(shifts,.5))},{"anchor_displacement_p95_px",shifts.empty()?J():J(percentile(shifts,.95))},{"anchor_displacement_max_px",shifts.empty()?J():J(percentile(shifts,1))},{"inlier_count_median",percentile(counts,.5)},{"pixel_absolute_difference_p95",percentile(differences,.95)}};
    report["visual_stability"]["minimum_inliers_for_displacement"]=20;report["visual_stability"]["anchor_elapsed_s"]=anchorFrame->t-t0;report["visual_stability"]["raw_image_mean_median_0_to_255"]=percentile(means,.5);report["visual_stability"]["diagnostic_contrast_enhancement_only"]=true;
    cv::imwrite((out/"stationary-contact.png").string(),contact);std::ofstream(out/"assessment.json")<<std::setw(2)<<report<<'\n';std::cout<<std::setw(2)<<report<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
