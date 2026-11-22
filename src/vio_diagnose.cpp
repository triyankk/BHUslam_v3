// Offline diagnostics only. Fits are consistency checks against stereo odometry,
// not a calibrated replacement for ground truth or a stationary Allan recording.
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <json.hpp>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>
#include <vector>
namespace fs=std::filesystem;
using J=nlohmann::json;
using V=cv::Vec3d;
using M=cv::Matx33d;
struct S{double t;V v;};
struct Pose{double t;V p;M r;};
struct Frame{double t;std::string l,r;int stereo=-1,vio=-1;};
struct Pair{double a,b;V w;};
static std::vector<std::string> cells(std::string s){std::replace(s.begin(),s.end(),',',' ');std::istringstream in(s);std::vector<std::string> v;while(in>>s)v.push_back(s);return v;}
static std::vector<S> samples(const fs::path&p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing "+p.string());std::vector<S> v;std::string s;std::getline(f,s);while(std::getline(f,s)){auto c=cells(s);v.push_back({std::stod(c.at(0)),{std::stod(c.at(1)),std::stod(c.at(2)),std::stod(c.at(3))}});}return v;}
static V at(const std::vector<S>&s,double t){auto it=std::lower_bound(s.begin(),s.end(),t,[](const S&a,double b){return a.t<b;});if(it==s.begin()||it==s.end())throw std::runtime_error("IMU coverage");const auto& a=*(it-1);double f=(t-a.t)/(it->t-a.t);return a.v*(1-f)+it->v*f;}
static V mean(const std::vector<S>&s,double a,double b){V out(0,0,0);V prev=at(s,a);double t=a;auto it=std::upper_bound(s.begin(),s.end(),a,[](double t,const S&s){return t<s.t;});while(it!=s.end()&&it->t<b){out+=(prev+it->v)*(.5*(it->t-t));prev=it->v;t=it->t;++it;}out+=(prev+at(s,b))*(.5*(b-t));return out/(b-a);}
static J vec(V v){return J::array({v[0],v[1],v[2]});}
static J matrix(M r){J v=J::array();for(double x:r.val)v.push_back(x);return v;}
static double percentile(std::vector<double> x,double p){if(x.empty())return 0;std::sort(x.begin(),x.end());return x[std::min(x.size()-1,size_t(p*(x.size()-1)))];}
static J stats(const std::vector<double>&x){return {{"n",x.size()},{"median",percentile(x,.5)},{"p95",percentile(x,.95)},{"max",percentile(x,1)}};}
static M quat(double x,double y,double z,double w){double n=std::sqrt(x*x+y*y+z*z+w*w);x/=n;y/=n;z/=n;w/=n;return M(1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y));}
struct Fit{double offset=0,rms=1e9,scale=0;V bias{};};
static double splitTime=0;
// camera-clock time = gyro time + offset; gyro camera axes = R_bc^T * gyro body axes.
static Fit fit(const std::vector<Pair>& pairs,const std::vector<S>&g,M rbc,double offset,int part){
    std::vector<V> a,b;V am(0,0,0),bm(0,0,0);for(auto&p:pairs){if(part==1&&p.a>=splitTime)continue;if(part==2&&p.a<splitTime)continue;if(p.a-offset<=g.front().t||p.b-offset>=g.back().t)continue;V x=rbc.t()*mean(g,p.a-offset,p.b-offset);a.push_back(x);b.push_back(p.w);am+=x;bm+=p.w;}
    Fit f;f.offset=offset;if(a.size()<20)return f;am*=1./a.size();bm*=1./a.size();f.bias=am-bm;double e=0,num=0,den=0;for(size_t i=0;i<a.size();++i){V d=a[i]-b[i]-f.bias;e+=d.dot(d);num+=(a[i]-am).dot(b[i]-bm);den+=(a[i]-am).dot(a[i]-am);}f.rms=std::sqrt(e/a.size());f.scale=num/den;return f;
}
static J fitjson(Fit f){return {{"offset_seconds_added_to_imu",f.offset},{"rms_rad_s",f.rms},{"gyro_minus_visual_bias_camera_rad_s",vec(f.bias)},{"visual_over_gyro_scale",f.scale}};}
int main(int argc,char**argv)try{
    if(argc==2&&std::string(argv[1])=="--self-test"){
        std::vector<S> g;for(int i=-200;i<=24200;++i){double t=i*.005;g.push_back({t,{std::sin(.17*t*t),.7*std::cos(.11*t*t),.4*std::sin(.09*t*t)}});}
        V bias(.03,-.01,.008);std::vector<Pair> p;for(int i=2;i<1190;++i){double a=i*.1,b=a+.1;p.push_back({a,b,mean(g,a-.012,b-.012)-bias});}
        splitTime=60;Fit best;for(int ms=-30;ms<=30;++ms){auto f=fit(p,g,M::eye(),ms*.001,1);if(f.rms<best.rms)best=f;}
        auto held=fit(p,g,M::eye(),best.offset,2);
        if(std::abs(best.offset-.012)>1e-9||cv::norm(best.bias-bias)>1e-9||held.rms>1e-9||std::abs(best.scale-1)>1e-9||fit(p,g,M::eye(),0,1).rms<.01)throw std::runtime_error("Timing/bias self-test failed");
        std::vector<S> constant{{0,{1,2,3}},{1,{1,2,3}},{2,{1,2,3}}};if(cv::norm(mean(constant,.2,1.8)-V(1,2,3))>1e-12)throw std::runtime_error("Integration test failed");
        std::cout<<"Synthetic timing sign, held-out fit, gyro bias, scale and integration checks passed\n";return 0;
    }
    if(argc!=5){std::cerr<<"Usage: gemini_vio_diagnose RECORDING STEREO_REPLAY VIO_REPLAY NEW_OUTPUT\n";return 2;}
    fs::path root=argv[1],stereo=argv[2],vio=argv[3],out=argv[4];if(fs::exists(out))throw std::runtime_error("Output exists");fs::create_directories(out);
    auto g=samples(root/"gyro.csv"),acc=samples(root/"accel.csv");std::map<long long,Frame> frames;std::ifstream ff(root/"frames.csv");std::string s;std::getline(ff,s);while(std::getline(ff,s)){auto c=cells(s);double t=std::stod(c.at(0));frames[llround(t*1e6)]={t,c.at(1),c.at(2)};}if(frames.empty())throw std::runtime_error("No frames");double t0=frames.begin()->second.t;
    J result={{"dataset",fs::absolute(root).string()},{"warning","Stereo is a diagnostic reference, not ground truth. No settings are applied."}};
    std::vector<double> losses;
    for(int mode=0;mode<2;++mode){std::ifstream f((mode?vio:stereo)/"tracking.csv");if(!f)throw std::runtime_error("Missing tracking CSV");std::getline(f,s);int prev=-1;J transitions=J::array();size_t good=0;std::vector<double> latency;while(std::getline(f,s)){auto c=cells(s);double t=std::stod(c[0]);int state=std::stoi(c[1]);auto it=frames.find(llround(t*1e6));if(it==frames.end())continue;(mode?it->second.vio:it->second.stereo)=state;latency.push_back(std::stod(c[2]));good+=state==2;if(state!=prev){transitions.push_back({{"elapsed_s",t-t0},{"from",prev},{"to",state}});if(mode&&prev==2&&state!=2)losses.push_back(t-t0);prev=state;}}result[mode?"vio":"stereo"]={{"tracked",good},{"tracking_ms",stats(latency)},{"transitions",transitions}};}
    std::vector<Pose> poses;std::ifstream pf(stereo/"trajectory.txt");while(std::getline(pf,s)){std::istringstream in(s);double t,x,y,z,qx,qy,qz,qw;if(!(in>>t>>x>>y>>z>>qx>>qy>>qz>>qw))continue;auto it=frames.find(llround(t*1e6));if(it!=frames.end()&&it->second.stereo==2)poses.push_back({t,{x,y,z},quat(qx,qy,qz,qw)});}
    cv::FileStorage config((root/"orb.yaml").string(),cv::FileStorage::READ);cv::Mat tbc;config["IMU.T_b_c1"]>>tbc;tbc.convertTo(tbc,CV_64F);M rbc;for(int i=0;i<3;++i)for(int j=0;j<3;++j)rbc(i,j)=tbc.at<double>(i,j);
    std::vector<Pair> pairs;for(size_t i=0;i+3<poses.size();i+=3){auto&a=poses[i];auto&b=poses[i+3];double dt=b.t-a.t;if(dt<.09||dt>.11)continue;V w;cv::Rodrigues(a.r.t()*b.r,w);w/=dt;if(cv::norm(w)>3)continue;pairs.push_back({a.t,b.t,w});}
    if(pairs.size()<40)throw std::runtime_error("Insufficient visual rotation reference");
    splitTime=t0+(frames.rbegin()->second.t-t0)*.5;
    std::ofstream sweep(out/"timing-sweep.csv");sweep<<"offset_s,train_rms_rad_s,holdout_rms_rad_s,scale\n";Fit best;for(int ms=-100;ms<=100;++ms){auto f=fit(pairs,g,rbc,ms*.001,1),h=fit(pairs,g,rbc,ms*.001,2);sweep<<f.offset<<','<<f.rms<<','<<h.rms<<','<<f.scale<<'\n';if(f.rms<best.rms)best=f;}
    result["rotation_check"]={{"pairs",pairs.size()},{"reference_interval_seconds",.1},{"split_elapsed_seconds",splitTime-t0},{"zero_offset_train",fitjson(fit(pairs,g,rbc,0,1))},{"best_train",fitjson(best)},{"best_holdout",fitjson(fit(pairs,g,rbc,best.offset,2))},{"zero_offset_holdout",fitjson(fit(pairs,g,rbc,0,2))}};
    for(auto offset:{0.,best.offset}){auto trained=fit(pairs,g,rbc,offset,1);double e=0;size_t n=0;for(auto&p:pairs)if(p.a>=splitTime&&p.a-offset>g.front().t&&p.b-offset<g.back().t){V d=rbc.t()*mean(g,p.a-offset,p.b-offset)-p.w-trained.bias;e+=d.dot(d);++n;}result["rotation_check"][offset==0?"zero_holdout_with_training_bias":"best_holdout_with_training_bias"]={{"n",n},{"rms_rad_s",std::sqrt(e/n)}};}
    // Translational acceleration is a noisy independent consistency check:
    // quadratic visual-position fits over 0.47 s versus rotated accelerometer,
    // smoothed with the approximate continuous quadratic-fit acceleration kernel.
    // Camera/IMU lever-arm acceleration is not corrected here (centimetre offset).
    double firstStereoLoss=frames.rbegin()->second.t;for(auto&t:result["stereo"]["transitions"])if(t["from"]==2&&t["to"]!=2){firstStereoLoss=t0+double(t["elapsed_s"]);break;}
    std::vector<V> worldAcc,visualAcc;
    for(size_t i=7;i+7<poses.size();i+=3){if(poses[i+7].t-poses[i-7].t>.48||poses[i+7].t>=firstStereoLoss)continue;cv::Mat a(15,3,CV_64F),b(15,3,CV_64F),coef;V wa(0,0,0);double weights=0;for(int k=-7;k<=7;++k){double dt=poses[i+k].t-poses[i].t;a.at<double>(k+7,0)=1;a.at<double>(k+7,1)=dt;a.at<double>(k+7,2)=dt*dt;for(int axis=0;axis<3;++axis)b.at<double>(k+7,axis)=poses[i+k].p[axis];double x=dt/.25,w=std::pow(std::max(0.,1-x*x),2);wa+=w*(poses[i+k].r*rbc.t()*mean(acc,poses[i+k].t-best.offset-.005,poses[i+k].t-best.offset+.005));weights+=w;}cv::solve(a,b,coef,cv::DECOMP_SVD);V va(2*coef.at<double>(2,0),2*coef.at<double>(2,1),2*coef.at<double>(2,2));if(cv::norm(va)>20)continue;worldAcc.push_back(wa/weights);visualAcc.push_back(va);}
    if(worldAcc.size()>30){V am(0,0,0),vm(0,0,0);for(size_t i=0;i<worldAcc.size();++i){am+=worldAcc[i];vm+=visualAcc[i];}am*=1./worldAcc.size();vm*=1./worldAcc.size();double num=0,den=0,positive=0,negative=0;for(size_t i=0;i<worldAcc.size();++i){V a=worldAcc[i]-am,v=visualAcc[i]-vm;num+=a.dot(v);den+=a.dot(a);positive+=(a-v).dot(a-v);negative+=(a+v).dot(a+v);}result["accelerometer_visual_check"]={{"n",worldAcc.size()},{"fitted_scale",num/den},{"gravity_vector_at_unit_scale",vec(am-vm)},{"gravity_norm_at_unit_scale",cv::norm(am-vm)},{"positive_polarity_rms_m_s2",std::sqrt(positive/worldAcc.size())},{"negative_polarity_rms_m_s2",std::sqrt(negative/worldAcc.size())},{"limitation","Visual second derivatives are noisy; lever-arm acceleration uncorrected; not a bias or noise calibration."}};}
    V gm(0,0,0),vm(0,0,0);for(auto&p:pairs){gm+=mean(g,p.a-best.offset,p.b-best.offset);vm+=p.w;}gm*=1./pairs.size();vm*=1./pairs.size();M h=M::zeros();for(auto&p:pairs){V a=mean(g,p.a-best.offset,p.b-best.offset)-gm,b=p.w-vm;h+=M(a[0]*b[0],a[0]*b[1],a[0]*b[2],a[1]*b[0],a[1]*b[1],a[1]*b[2],a[2]*b[0],a[2]*b[1],a[2]*b[2]);}cv::SVD svd(h);cv::Mat d=cv::Mat::eye(3,3,CV_64F);d.at<double>(2,2)=cv::determinant(svd.vt.t()*svd.u.t());cv::Mat rr=svd.vt.t()*d*svd.u.t();M rfit;rr.copyTo(cv::Mat(rfit,false));V delta;cv::Rodrigues(rfit*rbc,delta);result["rotation_check"]["fitted_gyro_to_camera_rotation"]=matrix(rfit);result["rotation_check"]["deviation_from_factory_degrees"]=cv::norm(delta)*180/CV_PI;result["rotation_check"]["excitation_singular_values"]=J::array({svd.w.at<double>(0),svd.w.at<double>(1),svd.w.at<double>(2)});
    std::ofstream residual(out/"rotation-residuals.csv");residual<<"elapsed_s,visual_wx,visual_wy,visual_wz,gyro_wx,gyro_wy,gyro_wz,error_rad_s,acc_norm\n";std::vector<double> gn,an;for(auto&p:pairs){V w=rbc.t()*mean(g,p.a,p.b);residual<<(p.a+p.b)*.5-t0;for(double v:p.w.val)residual<<','<<v;for(double v:w.val)residual<<','<<v;residual<<','<<cv::norm(w-p.w)<<','<<cv::norm(mean(acc,p.a,p.b))<<'\n';gn.push_back(cv::norm(w));an.push_back(cv::norm(mean(acc,p.a,p.b)));}result["gyro_norm_rad_s"]=stats(gn);result["accel_norm_m_s2"]=stats(an);
    // Low-motion candidates are deliberately strict; they do not prove that the
    // device was mechanically stationary and cannot identify all accel biases.
    J still=J::array();for(double a=t0+1;a+2<frames.rbegin()->second.t;a+=2){std::vector<V> av,gv;for(auto&m:g)if(m.t>=a&&m.t<a+2)gv.push_back(m.v);for(auto&m:acc)if(m.t>=a&&m.t<a+2)av.push_back(m.v);if(gv.size()<350||av.size()<350)continue;V am(0,0,0),gm(0,0,0);for(auto v:av)am+=v;for(auto v:gv)gm+=v;am*=1./av.size();gm*=1./gv.size();double ar=0,gr=0;for(auto v:av)ar+=(v-am).dot(v-am);for(auto v:gv)gr+=(v-gm).dot(v-gm);ar=std::sqrt(ar/av.size());gr=std::sqrt(gr/gv.size());if(cv::norm(gm)<.08&&gr<.015&&ar<.12)still.push_back({{"start_elapsed_s",a-t0},{"accel_mean",vec(am)},{"accel_norm",cv::norm(am)},{"accel_rms",ar},{"gyro_mean",vec(gm)},{"gyro_rms",gr}});}result["low_motion_2s_candidates"]=still;
    auto orb=cv::ORB::create(1000);std::ofstream images(out/"image-quality.csv");images<<"elapsed_s,stereo_state,vio_state,mean,stddev,laplacian_variance,dark_fraction,bright_fraction,orb_left,stereo_ratio_matches,median_vertical_error_px,gyro_norm,accel_norm\n";
    cv::Mat contact(int(losses.size())*200,4*320,CV_8UC3,cv::Scalar(0,0,0));size_t n=0;
    for(auto&kv:frames){auto&f=kv.second;double elapsed=f.t-t0;bool selected=(n++%15==0);for(double loss:losses)if(std::abs(elapsed-loss)<.15)selected=true;if(!selected)continue;auto l=cv::imread((root/f.l).string(),0),r=cv::imread((root/f.r).string(),0);if(l.empty()||r.empty())throw std::runtime_error("Missing image");cv::Scalar m,sd;cv::meanStdDev(l,m,sd);cv::Mat lap;cv::Laplacian(l,lap,CV_64F);cv::Scalar lm,ls;cv::meanStdDev(lap,lm,ls);std::vector<cv::KeyPoint> kl,kr;cv::Mat dl,dr;orb->detectAndCompute(l,cv::noArray(),kl,dl);orb->detectAndCompute(r,cv::noArray(),kr,dr);std::vector<double> dy;if(!dl.empty()&&!dr.empty()){std::vector<std::vector<cv::DMatch>> matches;cv::BFMatcher(cv::NORM_HAMMING).knnMatch(dl,dr,matches,2);for(auto&x:matches)if(x.size()==2&&x[0].distance<.7*x[1].distance){auto a=kl[x[0].queryIdx].pt,b=kr[x[0].trainIdx].pt;if(a.x>b.x)dy.push_back(std::abs(a.y-b.y));}}
        images<<elapsed<<','<<f.stereo<<','<<f.vio<<','<<m[0]<<','<<sd[0]<<','<<ls[0]*ls[0]<<','<<double(cv::countNonZero(l<5))/l.total()<<','<<double(cv::countNonZero(l>250))/l.total()<<','<<kl.size()<<','<<dy.size()<<','<<percentile(dy,.5)<<','<<cv::norm(at(g,f.t))<<','<<cv::norm(at(acc,f.t))<<'\n';}
    for(size_t row=0;row<losses.size();++row)for(int col=0;col<4;++col){double target=t0+losses[row]+(col-2)*.5;auto it=frames.lower_bound(llround(target*1e6));if(it==frames.end())continue;auto image=cv::imread((root/it->second.l).string());cv::resize(image,image,{320,200});cv::putText(image,cv::format("%.2fs S%d V%d",it->second.t-t0,it->second.stereo,it->second.vio),{5,20},cv::FONT_HERSHEY_SIMPLEX,.5,{0,0,255},1);image.copyTo(contact(cv::Rect(col*320,int(row)*200,320,200)));}
    if(!contact.empty())cv::imwrite((out/"tracking-loss-contact.png").string(),contact);
    std::ofstream(out/"diagnostics.json")<<std::setw(2)<<result<<'\n';std::cout<<std::setw(2)<<result<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
