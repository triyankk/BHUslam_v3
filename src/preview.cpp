// Read-only LAN camera preview: no recording, QR triggers, Cube or SLAM.
#include "bhu/camera.hpp"
#include "bhu/qr_trigger.hpp"
#include <json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <atomic>
#include <chrono>
#include <csignal>
#include <mutex>
#include <thread>
#include <poll.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace bhu::camera;
static volatile std::sig_atomic_t running=1;
static void stop(int){running=0;}
static double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static void need(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
static const char* page=R"HTML(<!doctype html><meta name="viewport" content="width=device-width,initial-scale=1"><title>Gemini camera preview</title>
<style>body{background:#111922;color:#e5eef7;font:17px system-ui;margin:20px}img{width:100%;max-width:1280px;image-rendering:auto;border:2px solid #536478}#state{font-size:22px;padding:12px;background:#243446}p{max-width:1000px}</style>
<h2>Gemini 336L infrared view — preview only</h2><p>Left: rectified left IR image used by the QR scanner. Right: right IR image. Each is 640 × 400; no additional software ROI. This view does not start recording or send navigation.</p>
<div id="state">Connecting…</div><p>Present test_vio with a white border, fully inside the left image. If the phone is visible but its QR pattern has little contrast in infrared, try the printed code. Distance and focus also affect decoding.</p>
<img id="view" alt="Live stereo infrared camera frames"><p id="details"></p>
<script>const im=document.getElementById('view'),s=document.getElementById('state'),d=document.getElementById('details');async function tick(){try{const r=await fetch('/state.json',{cache:'no-store'});if(!r.ok)throw Error(r.status);const j=await r.json();s.textContent=j.qr_match?'✓ test_vio decoded':(j.decoded&&j.decoded.length?'Different QR payload: '+JSON.stringify(j.decoded):'No QR decoded');s.style.color=j.qr_match?'#62e2a2':'#ffc66e';d.textContent='Frame '+j.index+' · '+j.width+'×'+j.height+' per eye · '+j.scan_ms.toFixed(1)+' ms scan · rectification samples outside input: '+j.map_outside_pixels;im.src='/frame.jpg?t='+Date.now()}catch(e){s.textContent='Preview unavailable: '+e.message}setTimeout(tick,250)}tick()</script>)HTML";
struct Server {
    int fd=-1;std::atomic<bool> done{false};std::mutex mutex;std::thread worker;
    std::vector<uchar> jpeg;nlohmann::json state={{"qr_match",false},{"index",0},{"width",640},{"height",400},{"scan_ms",0},{"map_outside_pixels",0}};
    Server(const std::string& host,int port){
        fd=::socket(AF_INET,SOCK_STREAM|SOCK_CLOEXEC,0);need(fd>=0,"Cannot create preview socket");int one=1;::setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
        sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(port);
        if(::inet_pton(AF_INET,host.c_str(),&address.sin_addr)!=1 || ::bind(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0 || ::listen(fd,8)<0){::close(fd);fd=-1;throw std::runtime_error("Cannot bind preview address/port");}
        try{worker=std::thread([this]{while(!done)serve();});}catch(...){::close(fd);throw;}
    }
    ~Server(){done=true;if(worker.joinable())worker.join();if(fd>=0)::close(fd);}
    void publish(const cv::Mat& image,const nlohmann::json& value){std::vector<uchar> encoded;need(cv::imencode(".jpg",image,encoded,{cv::IMWRITE_JPEG_QUALITY,90}),"JPEG encode failed");std::lock_guard<std::mutex> lock(mutex);jpeg=std::move(encoded);state=value;}
    void serve(){
        pollfd event{fd,POLLIN,0};if(::poll(&event,1,100)<=0)return;int client=::accept4(fd,nullptr,nullptr,SOCK_CLOEXEC);if(client<0)return;
        timeval timeout{0,300000};::setsockopt(client,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));::setsockopt(client,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
        char buffer[2048];auto n=::recv(client,buffer,sizeof(buffer),0);std::string request=n>0?std::string(buffer,n):"",body,type="text/plain",status="404 Not Found";
        if(request.rfind("GET / ",0)==0){body=page;type="text/html; charset=utf-8";status="200 OK";}
        else if(request.rfind("GET /state.json ",0)==0){std::lock_guard<std::mutex> lock(mutex);body=state.dump();type="application/json";status="200 OK";}
        else if(request.rfind("GET /frame.jpg",0)==0){std::lock_guard<std::mutex> lock(mutex);body.assign(jpeg.begin(),jpeg.end());type="image/jpeg";status=jpeg.empty()?"503 Service Unavailable":"200 OK";}
        std::string response="HTTP/1.1 "+status+"\r\nContent-Type: "+type+"\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
        size_t sent=0;while(sent<response.size()){auto count=::send(client,response.data()+sent,response.size()-sent,MSG_NOSIGNAL);if(count<=0)break;sent+=count;}::close(client);
    }
};
int main(int argc,char** argv)try{
    int port=8767;double duration=1800;std::string host="127.0.0.1";
    for(int i=1;i<argc;++i){std::string key=argv[i];need(i+1<argc,"Missing preview option value");std::string value=argv[++i];if(key=="--port")port=std::stoi(value);else if(key=="--listen")host=value;else if(key=="--seconds")duration=std::stod(value);else throw std::runtime_error("Unknown preview option");}
    need(port>0 && port<=65535 && std::isfinite(duration) && duration>0 && duration<=7200,"Invalid preview limits");
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);std::signal(SIGPIPE,SIG_IGN);cv::setNumThreads(1);Server server(host,port);
    ob::Context context;context.setLoggerSeverity(OB_LOG_SEVERITY_WARN);auto devices=context.queryDeviceList();need(devices->getCount()==1,"Expected one Orbbec camera");auto device=devices->getDevice(0);need(std::string(device->getDeviceInfo()->getSerialNumber())=="CPC2963000JJ","Unexpected camera serial");
    Properties properties{device,{},{}};if(device->isPropertySupported(OB_PROP_IR_RECTIFY_BOOL,OB_PERMISSION_READ_WRITE))properties.set(OB_PROP_IR_RECTIFY_BOOL,false);properties.setInt(OB_PROP_LASER_CONTROL_INT,0);
    auto left=device->getSensor(OB_SENSOR_IR_LEFT)->getStreamProfileList()->getVideoStreamProfile(640,400,OB_FORMAT_Y8,30),right=device->getSensor(OB_SENSOR_IR_RIGHT)->getStreamProfileList()->getVideoStreamProfile(640,400,OB_FORMAT_Y8,30);
    auto e=left->getExtrinsicTo(right);cv::Mat t=(cv::Mat_<double>(3,1)<<e.trans[0]*.001,e.trans[1]*.001,e.trans[2]*.001),r1,r2,p1,p2,q,lx,ly,rx,ry;
    cv::stereoRectify(intrinsic(left),distortion(left),intrinsic(right),distortion(right),{640,400},rotation(e),t,r1,r2,p1,p2,q,cv::CALIB_ZERO_DISPARITY,0);
    cv::initUndistortRectifyMap(intrinsic(left),distortion(left),r1,p1,{640,400},CV_32FC1,lx,ly);cv::initUndistortRectifyMap(intrinsic(right),distortion(right),r2,p2,{640,400},CV_32FC1,rx,ry);
    size_t outside=0;for(int y=0;y<400;++y)for(int x=0;x<640;++x)if(lx.at<float>(y,x)<-.01 || lx.at<float>(y,x)>639.01 || ly.at<float>(y,x)<-.01 || ly.at<float>(y,x)>399.01)++outside;
    ob::Pipeline pipeline(device);auto config=std::make_shared<ob::Config>();config->enableStream(left);config->enableStream(right);config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);pipeline.enableFrameSync();pipeline.start(config);
    struct Cleanup{ob::Pipeline& p;~Cleanup(){try{p.stop();}catch(...){}}}cleanup{pipeline};
    bhu::bench::QrScanner scanner;double start=now(),next=0;std::cout<<"Preview http://"<<host<<':'<<port<<"; no recording or Cube output"<<std::endl;
    while(running && now()-start<duration){auto frames=pipeline.waitForFrameset(200);if(!frames || now()<next)continue;next=now()+.2;
        auto l=frames->getFrame(OB_FRAME_IR_LEFT),r=frames->getFrame(OB_FRAME_IR_RIGHT);if(!l || !r)continue;need(l->getDataSize()==640*400 && r->getDataSize()==640*400,"Unexpected preview image size");
        cv::Mat a(400,640,CV_8UC1,l->getData()),b(400,640,CV_8UC1,r->getData()),lo,ro,joined;cv::remap(a,lo,lx,ly,cv::INTER_LINEAR);cv::remap(b,ro,rx,ry,cv::INTER_LINEAR);double began=now();bool match=scanner.matches(lo);double scan=(now()-began)*1000;
        cv::hconcat(lo,ro,joined);cv::cvtColor(joined,joined,cv::COLOR_GRAY2BGR);cv::rectangle(joined,{0,0,640,400},match?cv::Scalar(80,230,80):cv::Scalar(0,160,255),2);cv::putText(joined,match?"LEFT IR: test_vio decoded":"LEFT IR: scanning entire frame",{10,25},cv::FONT_HERSHEY_SIMPLEX,.65,{0,200,255},2);cv::putText(joined,"RIGHT IR",{650,25},cv::FONT_HERSHEY_SIMPLEX,.65,{0,200,255},2);
        server.publish(joined,{{"qr_match",match},{"decoded",scanner.decoded},{"index",l->getIndex()},{"width",640},{"height",400},{"scan_ms",scan},{"map_outside_pixels",outside}});
    }return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
