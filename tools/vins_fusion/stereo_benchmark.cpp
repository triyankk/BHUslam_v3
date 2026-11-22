// Offline actual VINS tracker A/B; no camera, sockets or Cube access.
#include "runtime_globals.hpp"
#include "stereo_policy.hpp"
#include <json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
using J=nlohmann::json;namespace fs=std::filesystem;
static double q(std::vector<double> v,double p){if(v.empty())return 0;std::sort(v.begin(),v.end());return v[size_t((v.size()-1)*p)];}
int main(int argc,char**argv)try{
    if(argc!=4&&argc!=5)throw std::runtime_error("Usage: fusion_stereo_benchmark RECORDING NEW_OUTPUT MAX_FRAMES (0=all) [PROFILE_IDS e.g. 4,5]");
    fs::path root=argv[1],out=argv[2];size_t limit=std::stoul(argv[3]);
    if(fs::exists(out))throw std::runtime_error("Output exists");fs::create_directories(out);cv::setNumThreads(4);
    cv::FileStorage orb((root/"orb.yaml").string(),cv::FileStorage::READ);if(!orb.isOpened()||std::string(orb["Camera.type"])!="Rectified")throw std::runtime_error("Requires rectified capture");
    ROW=int(orb["Camera.height"]);COL=int(orb["Camera.width"]);auto settings=out/"camera.yaml";
    {cv::FileStorage f(settings.string(),cv::FileStorage::WRITE);f<<"model_type"<<"PINHOLE"<<"camera_name"<<"rectified"<<"image_width"<<COL<<"image_height"<<ROW
      <<"distortion_parameters"<<"{"<<"k1"<<0.<<"k2"<<0.<<"p1"<<0.<<"p2"<<0.<<"}"
      <<"projection_parameters"<<"{"<<"fx"<<double(orb["Camera1.fx"])<<"fy"<<double(orb["Camera1.fy"])<<"cx"<<double(orb["Camera1.cx"])<<"cy"<<double(orb["Camera1.cy"])<<"}";}
    struct Frame{double t;std::string l,r;};std::vector<Frame>frames;std::ifstream input(root/"frames.csv");std::string line;std::getline(input,line);
    while(std::getline(input,line)){std::stringstream s(line);std::vector<std::string>c;std::string v;while(std::getline(s,v,','))c.push_back(v);if(c.size()!=5&&c.size()!=7)throw std::runtime_error("Invalid frames CSV");frames.push_back({std::stod(c[0]),c[1],c[2]});if(limit&&frames.size()>=limit)break;}
    if(frames.empty())throw std::runtime_error("No frames");J results=J::array();
    std::vector<int> profiles={0,1,2,3};if(argc==5){profiles.clear();std::stringstream list(argv[4]);std::string item;while(std::getline(list,item,',')){int p=std::stoi(item);if(p<0||p>6)throw std::runtime_error("Invalid profile");profiles.push_back(p);}}
    const int maxCounts[]={150,150,150,200,200,200,250},distances[]={30,30,20,20,30,25,30};
    for(int profile:profiles){
        bhu::fusion::rectified_stereo=profile>0;MAX_CNT=maxCounts[profile];MIN_DIST=distances[profile];
        FeatureTracker tracker;tracker.readIntrinsicParameter({settings.string(),settings.string()});auto clahe=cv::createCLAHE(2,cv::Size(8,8));
        std::ofstream csv(out/("profile-"+std::to_string(profile)+".csv"));csv<<"timestamp_s,persistent,stereo_matches,geometry_matches,frontend_ms\n";
        std::vector<double>counts,geometry,timing;size_t supported=0;double longest=0,gapStart=-1;
        for(const auto& f:frames){auto l=cv::imread((root/f.l).string(),0),r=cv::imread((root/f.r).string(),0);if(l.empty()||r.empty())throw std::runtime_error("Missing stereo image");clahe->apply(l,l);clahe->apply(r,r);
            auto start=std::chrono::steady_clock::now();auto features=tracker.trackImage(f.t,l,r);double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
            size_t n=0,g=0,p=std::count_if(tracker.track_cnt.begin(),tracker.track_cnt.end(),[](int age){return age>=2;});
            for(const auto& item:features)if(item.second.size()==2){++n;const auto&a=item.second[0].second;const auto&b=item.second[1].second;g+=a[3]>b[3]&&std::abs(a[4]-b[4])<=1.;}
            bool good=p>=40&&n>=20;supported+=good;if(!good&&gapStart<0)gapStart=f.t;if(good&&gapStart>=0){longest=std::max(longest,f.t-gapStart);gapStart=-1;}
            counts.push_back(n);geometry.push_back(g);timing.push_back(ms);csv<<std::setprecision(15)<<f.t<<','<<p<<','<<n<<','<<g<<','<<ms<<'\n';
        }
        if(gapStart>=0)longest=std::max(longest,frames.back().t-gapStart);
        J row={{"profile",profile},{"stereo_policy",profile?"rectified":"upstream"},{"max_features",MAX_CNT},{"feature_distance",MIN_DIST},{"frames",frames.size()},{"supported_frames",supported},{"supported_fraction",double(supported)/frames.size()},{"longest_unsupported_s",longest},{"stereo_p50",q(counts,.5)},{"geometry_p50",q(geometry,.5)},{"frontend_ms_p50",q(timing,.5)},{"frontend_ms_p95",q(timing,.95)}};results.push_back(row);std::cout<<row.dump()<<std::endl;
    }
    std::ofstream(out/"summary.json")<<J{{"recording",fs::absolute(root).string()},{"frontend_only",true},{"ground_truth",false},{"profiles",results}}.dump(2)<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
