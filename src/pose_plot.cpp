#include <json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <map>
#include <algorithm>
namespace fs=std::filesystem;using Json=nlohmann::json;
struct Status{int state=0,segment=0;bool initialized=false;};
struct Pose{double t;cv::Vec3d p;cv::Matx33d R;int segment;bool initialized;};
struct Graph{std::string name,kind="optimized export";std::vector<Pose> poses,keys;size_t processed=0,tracked=0;int resets=0;double first=0,last=0;};
static cv::Matx33d rotation(double x,double y,double z,double w){double n=std::sqrt(x*x+y*y+z*z+w*w);if(!std::isfinite(n)||n<.99||n>1.01)throw std::runtime_error("Invalid exported quaternion");x/=n;y/=n;z/=n;w/=n;return {1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)};}
static Graph load(const fs::path& root,const std::string& name,double start){
    Graph graph;graph.name=name;std::unordered_map<int64_t,Status> statuses;std::ifstream tracking(root/"tracking.csv");if(!tracking)throw std::runtime_error("Missing replay tracking.csv");std::string line;std::getline(tracking,line);int segment=0;bool previous=false;
    while(std::getline(tracking,line)){for(char& c:line)if(c==',')c=' ';std::istringstream row(line);double t,ms;int state,initialized;if(!(row>>t>>state>>ms>>initialized))throw std::runtime_error("Malformed tracking row");bool good=state==2;if(good && !previous)++segment;previous=good;statuses[std::llround(t*1e6)]={state,segment,bool(initialized)};++graph.processed;graph.tracked+=good;}
    std::ifstream log(root.string()+".log");while(std::getline(log,line))if(line.find("LM: Active map reset, Done!!!")!=std::string::npos)++graph.resets;
    auto read=[&](const fs::path& path){std::vector<Pose> poses;std::ifstream input(path);if(!input)return poses;double t,x,y,z,qx,qy,qz,qw;while(input>>t>>x>>y>>z>>qx>>qy>>qz>>qw){if(!std::isfinite(t+x+y+z))throw std::runtime_error("Nonfinite pose");auto it=statuses.find(std::llround(t*1e6));if(it==statuses.end() || it->second.state!=2)continue;poses.push_back({t-start,{x,y,z},rotation(qx,qy,qz,qw),it->second.segment,it->second.initialized});}return poses;};
    graph.poses=read(root/"trajectory.txt");graph.keys=read(root/"keyframes.txt");
    // TUM frame export is recentered at the first atlas keyframe; the keyframe
    // export is not. Recover that transform from its first raw row, not from a
    // filtered keyframe list, and apply it once to the keyframe nodes.
    std::ifstream raw(root/"keyframes.txt");double t,x,y,z,qx,qy,qz,qw;if(raw>>t>>x>>y>>z>>qx>>qy>>qz>>qw){auto R=rotation(qx,qy,qz,qw).t();cv::Vec3d origin{x,y,z};for(auto& key:graph.keys){key.p=R*(key.p-origin);key.R=R*key.R;}}else if(!graph.keys.empty())throw std::runtime_error("Invalid keyframe reference");
    if(!graph.poses.empty()){graph.first=graph.poses.front().t;graph.last=graph.poses.back().t;}return graph;
}
static Graph online(const fs::path& root,const Graph& summary,double start){Graph graph=summary;graph.poses.clear();graph.keys.clear();graph.name="VIO - online history across resets";graph.kind="online history; segments have separate reset frames";std::ifstream input(root/"pose_history.csv");std::string line;std::getline(input,line);while(std::getline(input,line)){for(auto& c:line)if(c==',')c=' ';std::istringstream row(line);double t,x,y,z,qx,qy,qz,qw;int state,initialized,segment;if(!(row>>t>>x>>y>>z>>qx>>qy>>qz>>qw>>state>>initialized>>segment))throw std::runtime_error("Malformed pose history");if(state==2)graph.poses.push_back({t-start,{x,y,z},rotation(qx,qy,qz,qw),segment,bool(initialized)});}if(!graph.poses.empty()){graph.first=graph.poses.front().t;graph.last=graph.poses.back().t;}return graph;}
static Json alignStereo(Graph& stereo,const Graph& vio){
    std::unordered_map<int64_t,const Pose*> byTime;for(const auto& p:stereo.poses)byTime[std::llround(p.t*1e6)]=&p;
    using Pair=std::pair<const Pose*,const Pose*>;std::map<std::pair<int,int>,std::vector<Pair>> groups;
    for(const auto& v:vio.poses){auto it=byTime.find(std::llround(v.t*1e6));if(it!=byTime.end() && v.initialized)groups[{it->second->segment,v.segment}].push_back({it->second,&v});}
    auto best=groups.end();for(auto it=groups.begin();it!=groups.end();++it)if(best==groups.end() || it->second.size()>best->second.size())best=it;
    if(best==groups.end() || best->second.size()<30)throw std::runtime_error("Insufficient matched initialized poses for frame alignment");
    cv::Matx33d sum=cv::Matx33d::zeros();for(auto [s,v]:best->second)sum+=v->R*s->R.t();cv::SVD svd(cv::Mat(sum),cv::SVD::FULL_UV);cv::Mat correction=cv::Mat::eye(3,3,CV_64F);correction.at<double>(2,2)=cv::determinant(svd.u*svd.vt)<0?-1:1;cv::Mat result=svd.u*correction*svd.vt;cv::Matx33d R;for(int i=0;i<3;++i)for(int j=0;j<3;++j)R(i,j)=result.at<double>(i,j);
    cv::Vec3d translation{0,0,0};for(auto [s,v]:best->second)translation+=v->p-R*s->p;translation*=1.0/best->second.size();
    double positionSquared=0,angleSquared=0;for(auto [s,v]:best->second){auto delta=R*s->p+translation-v->p;positionSquared+=delta.dot(delta);auto d=v->R*(R*s->R).t();double angle=std::acos(std::clamp((cv::trace(cv::Mat(d))[0]-1)*.5,-1.0,1.0));angleSquared+=angle*angle;}
    Json matrix=Json::array();for(int i=0;i<3;++i)matrix.push_back({R(i,0),R(i,1),R(i,2)});
    Json info={{"method","mean matched camera orientation on largest shared initialized segment; mean translation; unit scale"},{"R_vio_stereo",matrix},{"translation_m",{translation[0],translation[1],translation[2]}},{"determinant",cv::determinant(cv::Mat(R))},{"scale",1.0},{"stereo_segment",best->first.first},{"vio_segment",best->first.second},{"matched_poses",best->second.size()},{"first_match_s",best->second.front().first->t},{"last_match_s",best->second.back().first->t},{"orientation_rms_deg",std::sqrt(angleSquared/best->second.size())*180/CV_PI},{"position_rms_m",std::sqrt(positionSquared/best->second.size())},{"display_only",true}};
    for(auto* poses:{&stereo.poses,&stereo.keys})for(auto& p:*poses){p.p=R*p.p+translation;p.R=R*p.R;}
    stereo.name="Stereo - VIO reference axes";stereo.kind="optimized export; display aligned to a reference VIO segment";return info;
}
static Json json(const Graph& graph){Json result={{"name",graph.name},{"kind",graph.kind},{"processed",graph.processed},{"tracked",graph.tracked},{"resets",graph.resets},{"first_s",graph.first},{"last_s",graph.last},{"poses",Json::array()},{"keys",Json::array()}};
    auto append=[](Json& out,const Pose& p){Json axes=Json::array();for(int i=0;i<3;++i)axes.push_back({p.R(0,i),p.R(1,i),p.R(2,i)});out.push_back({p.t,p.p[0],p.p[1],p.p[2],p.segment,p.initialized,axes});};for(auto& p:graph.poses)append(result["poses"],p);for(auto& p:graph.keys)append(result["keys"],p);return result;}
static void panel(cv::Mat image,const Graph& graph,double duration){
    auto text=[&](const std::string& value,int x,int y,double scale=.65){cv::putText(image,value,{x,y},cv::FONT_HERSHEY_SIMPLEX,scale,{220,230,240},1,cv::LINE_AA);};
    text(graph.name,30,43,.85);std::ostringstream timing;timing<<std::fixed<<std::setprecision(2)<<"Pose coverage: "<<graph.first<<" - "<<graph.last<<" s of "<<duration<<" s";text(timing.str(),30,80);
    text(std::to_string(graph.poses.size())+" tracked exported poses; "+std::to_string(graph.keys.size())+" keyframe nodes",30,111);
    text("Replay tracked "+std::to_string(graph.tracked)+" / "+std::to_string(graph.processed)+"; resets "+std::to_string(graph.resets),30,142);
    if(graph.poses.empty()){text("No tracked poses saved.",30,400);return;}
    cv::Vec3d lo=graph.poses[0].p,hi=lo;for(auto& p:graph.poses)for(int i=0;i<3;++i){lo[i]=std::min(lo[i],p.p[i]);hi[i]=std::max(hi[i],p.p[i]);}auto center=(lo+hi)*.5;double extent=.15;for(int i=0;i<3;++i)extent=std::max(extent,hi[i]-lo[i]);double scale=360/extent;
    auto project=[&](cv::Vec3d p){p-=center;double a=.8253356*p[0]+.5646425*p[1],b=-.5646425*p[0]+.8253356*p[1];return cv::Point(int(image.cols*.5+a*scale),int(440-(.7960838*p[2]-.6051864*b)*scale));};
    cv::Scalar axes[3]={{90,100,240},{240,180,90},{230,160,200}};
    for(int i=0;i<3;++i){auto end=center;end[i]+=extent*.27;auto p=project(end);cv::arrowedLine(image,project(center),p,axes[i],1,cv::LINE_AA,0,.1);text(std::string(1,"XYZ"[i])+" (m)",p.x+5,p.y);}
    for(size_t i=1;i<graph.poses.size();++i){auto& a=graph.poses[i-1];auto& b=graph.poses[i];if(a.segment!=b.segment || b.t-a.t>.051)continue;cv::line(image,project(a.p),project(b.p),a.initialized && b.initialized?cv::Scalar(182,223,66):cv::Scalar(97,183,239),2,cv::LINE_AA);}
    size_t stride=std::max<size_t>(1,graph.keys.size()/18);for(size_t i=0;i<graph.keys.size();++i){auto& k=graph.keys[i];cv::circle(image,project(k.p),3,{235,235,235},1,cv::LINE_AA);if(i%stride==0)for(int axis=0;axis<3;++axis){cv::Vec3d direction{k.R(0,axis),k.R(1,axis),k.R(2,axis)};cv::line(image,project(k.p),project(k.p+direction*extent*.035),axes[axis],1,cv::LINE_AA);}}
    cv::circle(image,project(graph.poses.front().p),7,{255,255,255},2,cv::LINE_AA);auto end=project(graph.poses.back().p);cv::rectangle(image,end-cv::Point(5,5),end+cv::Point(5,5),{255,255,255},cv::FILLED);
    std::ostringstream spans;spans<<std::fixed<<std::setprecision(2)<<"Extent X/Y/Z: "<<hi[0]-lo[0]<<" / "<<hi[1]-lo[1]<<" / "<<hi[2]-lo[2]<<" m";text(spans.str(),30,738);
    text("White circles: keyframes. Small RGB axes: camera orientation.",30,775,.52);
    if(graph.kind.rfind("online",0)==0)text("Online history: segments use separate reset frames; no KF overlay.",30,180,.52);
}
int main(int argc,char** argv)try{
    if(argc!=5 && argc!=6)throw std::runtime_error("Usage: gemini_pose_plot RECORDING STEREO_REPLAY VIO_REPLAY NEW_OUTPUT [--align-stereo-to-vio]");bool aligned=argc==6;if(aligned && std::string(argv[5])!="--align-stereo-to-vio")throw std::runtime_error("Unknown plot option");fs::path output=argv[4];if(fs::exists(output))throw std::runtime_error("Output exists");std::ifstream metadata(fs::path(argv[1])/"recording.json");Json meta;metadata>>meta;double start=meta.at("first_frame_device_s");auto stereo=load(argv[2],"Stereo only",start),vio=load(argv[3],"VIO - surviving final map only",start);
    auto vioOnline=fs::exists(fs::path(argv[3])/"pose_history.csv")?online(argv[3],vio,start):vio;double duration=meta.at("duration_s");
    Json alignment;if(aligned)alignment=alignStereo(stereo,vioOnline);
    Json result={{"recording_duration_s",duration},{"power_profile",meta.at("power_profile_start")},{"graphs",{json(stereo),json(vioOnline)}},{"edges","Temporal adjacency; loop/covisibility constraints were not exported"},{"frames","Separate local camera frames, not aligned for overlay"}};
    if(vioOnline.kind!=vio.kind)result["graphs"].push_back(json(vio));
    if(aligned){result["display_alignment"]=alignment;result["frames"]="Stereo display transformed to reference VIO segment; other reset segments retain independent frames";}
    fs::create_directories(output);std::ofstream data(output/"poses.json");data<<result.dump(2)<<'\n';
    cv::Mat image(890,1920,CV_8UC3,cv::Scalar(31,22,16));panel(image(cv::Rect(0,0,960,800)),stereo,duration);panel(image(cv::Rect(960,0,960,800)),vioOnline,duration);
    cv::putText(image,aligned?"Stereo display aligned to dominant VIO segment; unit scale. Amber: visual. Green: initialized. Gaps separate segments.":"Amber: visual tracking. Green: IMU initialized. Paths break at tracking loss. Separate coordinate frames / scales.",{30,830},cv::FONT_HERSHEY_SIMPLEX,.6,{225,230,235},1,cv::LINE_AA);
    cv::putText(image,"Stereo: optimized export. VIO: online history when available. Reset segments are not a continuous world trajectory.",{30,865},cv::FONT_HERSHEY_SIMPLEX,.6,{180,190,205},1,cv::LINE_AA);
    if(!cv::imwrite((output/"pose-graph.png").string(),image))throw std::runtime_error("Cannot write pose plot");
    std::ifstream page(BHU_POSE_PAGE);std::string html{std::istreambuf_iterator<char>(page),{}};auto pos=html.find("/*POSE_DATA*/null");if(pos==std::string::npos)throw std::runtime_error("Missing pose page marker");html.replace(pos,17,result.dump());std::ofstream report(output/"pose-graph.html");report<<html;if(!report)throw std::runtime_error("Cannot write graph page");std::cout<<output/"pose-graph.html"<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
