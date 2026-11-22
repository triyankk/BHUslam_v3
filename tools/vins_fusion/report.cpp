#include <json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <opencv2/calib3d.hpp>
using J=nlohmann::json;namespace fs=std::filesystem;
static std::vector<double> csv(const std::string&s){std::stringstream in(s);std::string c;std::vector<double>v;while(std::getline(in,c,','))v.push_back(std::stod(c));return v;}
static double q(std::vector<double>v,double p){if(v.empty())return 0;std::sort(v.begin(),v.end());return v[size_t((v.size()-1)*p)];}
static cv::Matx33d rotation(const J&q){double x=q[0],y=q[1],z=q[2],w=q[3],n=std::sqrt(x*x+y*y+z*z+w*w);x/=n;y/=n;z/=n;w/=n;return {1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)};}
static cv::Vec3d position(const J&p){return {double(p[1]),double(p[2]),double(p[3])};}
int main(int argc,char**argv)try{
    if(argc!=2)throw std::runtime_error("Usage: vins_mono_report BENCHMARK_ROOT");fs::path root=argv[1];J result={{"warning","No ground truth. Native initialization/health flags differ. Visual support is a diagnostic threshold, not accuracy."},{"runs",J::array()}};
    for(auto name:{"two-minute-vins-fusion","two-minute-vins-fusion-native-noise","two-minute-vins","two-minute-orb-mono","two-minute-vins-native-noise","two-minute-orb-stereo","eighty-second-vins-fusion","eighty-second-vins","eighty-second-orb-mono"}){
        fs::path dir=root/name;if(!fs::exists(dir/"tracking.csv"))continue;bool vins=std::string(name).find("-vins")!=std::string::npos;J summary;std::ifstream meta(dir/(vins?"summary.json":"replay-summary.json"));if(!meta)throw std::runtime_error(std::string("Incomplete summary: ")+name);meta>>summary;
        J run={{"name",name},{"vins",vins},{"summary",summary},{"poses",J::array()},{"timeline",J::array()}};std::map<long long,std::pair<bool,bool>>valid;std::ifstream tracking(dir/"tracking.csv");std::string s;std::getline(tracking,s);double t0=-1,last=-1,gap=0,longest=0,first=-1;std::vector<double>activeLatency,stereoMatches,frontendMs,backendMs;double weak=0,longestWeak=0;size_t count=0,visionPoor=0;
        while(std::getline(tracking,s)){auto c=csv(s);double t=c.at(0);if(t0<0)t0=t;last=t;bool ok=vins?c.at(1)==1:c.at(1)==2&&c.at(3)==1;bool supported=vins?c.at(7)==1:ok;double ms=c.at(4);if(vins){frontendMs.push_back(c[2]);backendMs.push_back(c[3]);}if(c.size()>9)stereoMatches.push_back(c[9]);if(ok&&!supported){weak+=1./29.964335;longestWeak=std::max(longestWeak,weak);}else weak=0;valid[llround(t*1e6)]={ok,supported};++count;if(ok){if(first<0)first=t-t0;longest=std::max(longest,gap);gap=0;activeLatency.push_back(ms);visionPoor+=!supported;}else if(first>=0)gap+=1./29.964335;
            run["timeline"].push_back({t-t0,ok,supported,ms,vins?c.at(5):-1.});}
        longest=std::max(longest,gap);if(vins)run["phase_ms"]={{"frontend_p50",q(frontendMs,.5)},{"frontend_p95",q(frontendMs,.95)},{"backend_p50",q(backendMs,.5)},{"backend_p95",q(backendMs,.95)}};run["longest_initialized_weak_visual_s"]=longestWeak;if(!stereoMatches.empty())run["stereo_matches"]={{"p05",q(stereoMatches,.05)},{"p50",q(stereoMatches,.5)},{"min",*std::min_element(stereoMatches.begin(),stereoMatches.end())}};run["frame_count"]=count;run["duration_s"]=last-t0;run["first_native_initialized_pose_s"]=first;run["longest_post_init_native_gap_s"]=longest;run["active_pipeline_p95_ms"]=q(activeLatency,.95);run["initialized_low_visual_support_frames"]=visionPoor;run["native_available_frames"]=activeLatency.size();
        std::ifstream history(dir/"pose_history.csv");std::getline(history,s);std::map<int,std::vector<std::vector<double>>>quiet;
        while(std::getline(history,s)){auto c=csv(s);auto it=valid.find(llround(c.at(0)*1e6));if(it==valid.end()||!it->second.first)continue;int segment=int(c.at(vins?8:10));run["poses"].push_back({c[0]-t0,c[1],c[2],c[3],segment,it->second.second,J::array({c[4],c[5],c[6],c[7]})});if(std::string(name).find("two-minute")==0&&c[0]-t0>=91&&c[0]-t0<=115)quiet[segment].push_back(c);}
        run["quiet_tail"]=nullptr;size_t most=0;for(auto&[segment,points]:quiet)if(points.size()>most){most=points.size();auto&a=points.front();auto&b=points.back();double span=b[0]-a[0];if(span>=20){double drift=0,max=0;for(int k=1;k<=3;++k)drift+=(b[k]-a[k])*(b[k]-a[k]);for(auto&p:points){double e=0;for(int k=1;k<=3;++k)e+=(p[k]-a[k])*(p[k]-a[k]);max=std::max(max,std::sqrt(e));}run["quiet_tail"]={{"span_s",span},{"endpoint_displacement_m",std::sqrt(drift)},{"maximum_displacement_m",max},{"segment",segment}};}}
        result["runs"].push_back(run);
    }
    if(result["runs"].empty())throw std::runtime_error("No completed runs");
    const J*reference=nullptr;for(const auto&r:result["runs"])if(r["name"]=="two-minute-orb-stereo")reference=&r;
    if(reference) for(auto&r:result["runs"]) if(r["name"]!="two-minute-orb-stereo"&&std::string(r["name"]).find("two-minute")==0){
      auto agreement=[&](double lo,double hi)->J{
        std::map<long long,const J*> byTime;for(const auto&p:(*reference)["poses"])if(double(p[0])>=lo&&double(p[0])<=hi)byTime[llround(double(p[0])*1e6)]=&p;
        using Pair=std::pair<const J*,const J*>;std::map<std::pair<int,int>,std::vector<Pair>> groups;
        for(const auto&p:r["poses"]){auto it=byTime.find(llround(double(p[0])*1e6));if(it!=byTime.end())groups[{int(p[4]),int((*it->second)[4])}].push_back({&p,it->second});}
        const std::vector<Pair>*best=nullptr;for(auto&[id,g]:groups)if(!best||g.size()>best->size())best=&g;if(!best||best->size()<30)return nullptr;
        cv::Matx33d sum=cv::Matx33d::zeros();for(auto[a,b]:*best)sum+=rotation((*b)[6])*rotation((*a)[6]).t();cv::SVD svd(sum);cv::Mat d=cv::Mat::eye(3,3,CV_64F);d.at<double>(2,2)=cv::determinant(svd.u*svd.vt);cv::Mat rr=svd.u*d*svd.vt;cv::Matx33d R;rr.copyTo(cv::Mat(R,false));cv::Vec3d am(0,0,0),bm(0,0,0);
        for(auto[a,b]:*best){am+=R*position(*a);bm+=position(*b);}am*=1./best->size();bm*=1./best->size();double error=0,num=0,den=0;for(auto[a,b]:*best){auto x=R*position(*a)-am,y=position(*b)-bm,delta=x-y;error+=delta.dot(delta);num+=x.dot(y);den+=x.dot(x);}
        return {{"matched_poses",best->size()},{"first_s",double((*best->front().first)[0])},{"last_s",double((*best->back().first)[0])},{"unit_scale_rigid_position_rms_m",std::sqrt(error/best->size())},{"diagnostic_reference_over_run_scale",den>0?J(num/den):J(nullptr)},{"warning","Estimator agreement only; stereo is not ground truth. Each interval has its own rigid alignment. Scale was not fitted for RMS."}};
      };
      r["agreement_with_stereo_reference"]=agreement(5,60);r["agreement_intervals"]=J::array();for(auto window:std::vector<std::pair<double,double>>{{5,60},{60,70},{70,89}}){auto value=agreement(window.first,window.second);r["agreement_intervals"].push_back({{"requested_first_s",window.first},{"requested_last_s",window.second},{"agreement",value}});}
    }
    // Offline image overview: independent local frames, common metre labels.
    cv::Mat overview(960,1680,CV_8UC3,cv::Scalar(35,25,17));
    std::vector<std::string> selected={"two-minute-vins-fusion","two-minute-vins","two-minute-orb-stereo","eighty-second-vins-fusion","eighty-second-vins","eighty-second-orb-mono"};
    for(size_t panel=0;panel<selected.size();++panel){const J*run=nullptr;for(const auto&r:result["runs"])if(r["name"]==selected[panel])run=&r;if(!run)continue;int ox=(panel%3)*560,oy=(panel/3)*480;cv::putText(overview,selected[panel],{ox+20,oy+32},cv::FONT_HERSHEY_SIMPLEX,.55,{240,230,220},1,cv::LINE_AA);
      const auto&pp=(*run)["poses"];if(pp.empty())continue;double xmin=1e100,xmax=-1e100,ymin=1e100,ymax=-1e100;
      auto project=[](const J&p){double x=p[1],y=p[2],z=p[3];return cv::Point2d(.825*x+.565*y,z+.32*(-.565*x+.825*y));};
      for(const auto&p:pp){auto q=project(p);xmin=std::min(xmin,q.x);xmax=std::max(xmax,q.x);ymin=std::min(ymin,q.y);ymax=std::max(ymax,q.y);}double scale=360/std::max({.2,xmax-xmin,ymax-ymin});
      auto pix=[&](const cv::Point2d&q){return cv::Point(int(ox+280+(q.x-(xmin+xmax)/2)*scale),int(oy+235-(q.y-(ymin+ymax)/2)*scale));};
      for(size_t i=1;i<pp.size();++i){const auto&a=pp[i-1];const auto&b=pp[i];if(a[4]!=b[4]||double(b[0])-double(a[0])>.05)continue;cv::line(overview,pix(project(a)),pix(project(b)),bool(a[5])&&bool(b[5])?cv::Scalar(181,226,93):cv::Scalar(107,193,255),2,cv::LINE_AA);}
      cv::circle(overview,pix(project(pp.front())),5,{255,255,255},2);cv::circle(overview,pix(project(pp.back())),5,{80,80,255},2);
      std::ostringstream label;label<<std::fixed<<std::setprecision(1)<<"Projected extent "<<xmax-xmin<<" x "<<ymax-ymin<<" m";cv::putText(overview,label.str(),{ox+20,oy+435},cv::FONT_HERSHEY_SIMPLEX,.5,{210,190,170},1,cv::LINE_AA);
    }
    cv::putText(overview,"Independent local coordinates; each panel auto-fits. White=start, red=end. Amber=weak visual support. No ground truth.",{20,945},cv::FONT_HERSHEY_SIMPLEX,.6,{240,230,220},1,cv::LINE_AA);cv::imwrite((root/"trajectory-overview.png").string(),overview);
    std::ofstream(root/"comparison.json")<<result.dump(2)<<'\n';
    std::ifstream templateFile(BHU_VINS_PAGE);std::string html{std::istreambuf_iterator<char>(templateFile),{}};std::string marker="/*VINS_COMPARISON*/null";auto at=html.find(marker);if(at==std::string::npos)throw std::runtime_error("Missing report template marker");html.replace(at,marker.size(),result.dump());std::ofstream(root/"comparison.html")<<html;
    std::ofstream md(root/"metrics.md");md<<"# Recorded-data comparison\n\nNo ground truth; these are availability, runtime and stationary-tail consistency metrics. Native health flags differ.\n\n| Run | Frames | Native initialized poses | Init (s) | Longest later native gap (s) | Active pipeline p95 (ms) | Peak RSS (MiB) | Quiet-tail drift (m) |\n|---|---:|---:|---:|---:|---:|---:|---:|\n";for(auto&r:result["runs"]){md<<"| "<<std::string(r["name"])<<" | "<<r["frame_count"]<<" | "<<r["native_available_frames"]<<" | "<<r["first_native_initialized_pose_s"]<<" | "<<r["longest_post_init_native_gap_s"]<<" | "<<r["active_pipeline_p95_ms"]<<" | "<<double(r["summary"]["peak_rss_kib"])/1024<<" | ";if(r["quiet_tail"].is_null())md<<"unavailable";else md<<r["quiet_tail"]["endpoint_displacement_m"];md<<" |\n";}std::cout<<root/"comparison.html"<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
