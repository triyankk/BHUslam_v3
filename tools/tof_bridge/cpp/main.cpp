#include "native.hpp"
#include "cube_udp.hpp"
#include "bhu/extnav_relay.hpp"
#include <csignal>
#include <deque>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <sys/socket.h>

#ifndef TOF_ASSET_DIR
#define TOF_ASSET_DIR "tools/tof_bridge"
#endif
using namespace tof;
static volatile std::sig_atomic_t interrupted=0;
static void signal_handler(int) { interrupted=1; }

struct Options {
    fs::path assets=TOF_ASSET_DIR, calibration, output;
    std::string uart="/dev/ttyTHS1", orientation="rear";
    int port=8765,bus=7,address=0x41,baud=0,system=1,slot=2,confidence=51,vio_port=0;
    int mux_address=0,mux_channel=0;
    bool sensor_only=false;
    std::string aggregation="zone-mean";
    int minimum_active_zones=1;
    int cube_udp_port=0,cube_feedback_port=14606;
    double duration=0,hz=15;
    uint8_t rotation() const {
        if (orientation=="front") return 0;
        if (orientation=="right") return 2;
        if (orientation=="rear") return 4;
        if (orientation=="left") return 6;
        if (orientation=="up") return 24;
        fail("Unknown orientation");return 0;
    }
    std::string prefix() const {return "RNGFND"+(slot==10?std::string("A"):std::to_string(slot))+"_";}
};

// Cube proximity telemetry shares a global max across front and long-range rear.
static bool cube_distance_in_tof_range(unsigned cm){return cm>=1&&cm<=500;}

struct State {
    mutable std::mutex mutex;
    const Options& options;
    double start=now(),heartbeat=-1;
    std::deque<Json> sensor,cube,messages;
    Json latest=nullptr,parameters=Json::object();
    std::string status="Starting";
    Json error=nullptr;
    int baud=0;
    uint64_t vio_received=0,vio_rejected=0,vio_forwarded=0;
    uint64_t sensor_count=0,cube_count=0,bad_packets=0,reconnects=0,unavailable_cube_reports=0;
    explicit State(const Options& config):options(config) {}
    Json snapshot() const {
        std::lock_guard<std::mutex> guard(mutex);
        double stamp=now(),elapsed=stamp-start;
        Json sensor_points=Json::array(),cube_points=Json::array();
        for (const auto& row:sensor) if (row.at("t").get<double>()>elapsed-65) sensor_points.push_back(row);
        for (const auto& row:cube) if (row.at("t").get<double>()>elapsed-65) cube_points.push_back(row);
        double first=0,last=0;size_t count=0;
        for (const auto& row:sensor) {
            double t=row.at("t");
            if (t>elapsed-5) {if (!count) first=t;last=t;++count;}
        }
        double hz=count>1 && last>first?double(count-1)/(last-first):0;
        return {{"elapsed_s",elapsed},{"baud",baud?Json(baud):Json(nullptr)},
            {"orientation",options.orientation},{"uart",(options.sensor_only || options.cube_udp_port)?Json(nullptr):Json(options.uart)},{"usb",nullptr},
            {"sensor_only",options.sensor_only},
            {"transport",options.sensor_only?"sensor-only":options.cube_udp_port?"shared-usb":"serial"},
            {"cube_udp_port",options.cube_udp_port},{"cube_feedback_port",options.cube_feedback_port},
            {"aggregation",options.aggregation},{"minimum_active_zones",options.minimum_active_zones},
            {"confidence_threshold",options.confidence},
            {"backend","C++17"},{"rangefinder_slot",options.slot},
            {"cube_report_kind",cube.empty()?Json(nullptr):cube.back().at("report_kind")},
            {"mux_address",options.mux_address?Json(options.mux_address):Json(nullptr)},
            {"mux_channel",options.mux_address?Json(options.mux_channel):Json(nullptr)},
            {"vio_udp_port",options.vio_port},{"vio_received",vio_received},
            {"vio_rejected",vio_rejected},{"vio_forwarded",vio_forwarded},
            {"sensor_serial",latest.is_null()?Json(nullptr):latest.at("sensor_serial")},
            {"bridge_status",status},{"bridge_error",error},{"cube_error",nullptr},
            {"sensor_age_s",latest.is_null()?Json(nullptr):Json(stamp-latest.at("monotonic_s").get<double>())},
            {"cube_age_s",cube.empty()?Json(nullptr):Json(stamp-cube.back().at("monotonic_s").get<double>())},
            {"heartbeat_age_s",heartbeat<0?Json(nullptr):Json(stamp-heartbeat)},
            {"hz",hz},{"sensor_count",sensor_count},{"cube_count",cube_count},
            {"bad_usb_packets",0},{"unavailable_cube_reports",unavailable_cube_reports},{"uart_bad_packets",bad_packets},{"reconnects",reconnects},
            {"latest",latest},{"cube_parameters",parameters},{"telemetry_messages",messages},
            {"sensor",sensor_points},{"cube",cube_points},{"output",options.output.string()}};
    }
};
template<class T> static void bounded_push(std::deque<T>& target,T value,size_t maximum) {
    target.push_back(std::move(value));while(target.size()>maximum)target.pop_front();
}
struct Session {
    fs::path path;
    std::ofstream measurements,tx,rx,peers;
    Json summary={{"frames",0},{"sent_distances",0},{"rejected",0},{"duplicates",0},
        {"result_counter_gaps",0},{"mavlink_packets",0},{"mavlink_bytes",0},
        {"uart_rx_bytes",0},{"uart_rx_messages",Json::object()},
        {"calibration_status_counts",Json::object()},{"autopilot_heartbeats",0},{"error",nullptr}};
    double started=now(),previous=-1,mean=0,m2=0,minimum=1e30,maximum=0;
    size_t intervals=0;
    explicit Session(const fs::path& directory):path(directory) {
        check(fs::create_directory(path),"Capture directory already exists");
        measurements.open(path/"measurements.jsonl");tx.open(path/"mavlink.bin",std::ios::binary);
        rx.open(path/"uart-rx.bin",std::ios::binary);peers.open(path/"cube-reports.jsonl");
        check(measurements && tx && rx && peers,"Cannot open recording files");
        measurements.exceptions(std::ios::badbit|std::ios::failbit);
        tx.exceptions(std::ios::badbit|std::ios::failbit);
        rx.exceptions(std::ios::badbit|std::ios::failbit);
        peers.exceptions(std::ios::badbit|std::ios::failbit);
    }
    static void increment(Json& object,const std::string& key,size_t amount=1) {
        object[key]=object.value(key,uint64_t(0))+amount;
    }
    template<class Link> void send(Link& link,const mavlink_message_t& message) {
        auto bytes=packet(message);link.write(bytes);
        tx.write(reinterpret_cast<const char*>(bytes.data()),std::streamsize(bytes.size()));tx.flush();
        increment(summary,"mavlink_packets");increment(summary,"mavlink_bytes",bytes.size());
    }
    void event(const Json& row) {peers<<row.dump()<<'\n';peers.flush();}
    void frame(const Json& row) {
        measurements<<row.dump()<<'\n';measurements.flush();
        double stamp=row.at("monotonic_s");
        if (previous>=0) {
            double delta=stamp-previous;
            ++intervals;double diff=delta-mean;mean+=diff/double(intervals);m2+=diff*(delta-mean);
            minimum=std::min(minimum,delta);maximum=std::max(maximum,delta);
        }
        previous=stamp;
    }
    void finish(const std::string& error, uint64_t bad) {
        summary["error"]=error.empty()?Json(nullptr):Json(error);
        summary["elapsed_s"]=now()-started;summary["uart_bad_packets"]=bad;
        if (intervals) {
            summary["observed_hz"]=1/mean;summary["interval_min_ms"]=minimum*1000;
            summary["interval_max_ms"]=maximum*1000;
            summary["interval_stddev_ms"]=std::sqrt(m2/double(intervals))*1000;
        }
        save_json(path/"summary.json",summary);
    }
};

static void run_session(const Options& options,State& state,std::atomic<bool>& stop,Session& capture) {
    auto calibration=Json::parse(read_text(options.calibration));
    Fd lock(::open((options.assets/(".i2c-"+std::to_string(options.bus)+"-"+
        [&]{std::ostringstream s;s<<std::hex<<options.address;return s.str();}()+".lock")).c_str(),O_RDWR|O_CREAT|O_CLOEXEC,0664));
    check(lock.get()>=0 && ::flock(lock.get(),LOCK_EX|LOCK_NB)==0,"Sensor is owned by another bridge");
    // A mux is shared by every downstream address: own it for the full session.
    fs::path mux_lock_path=options.assets/(".i2c-"+std::to_string(options.bus)+"-mux-"+
        std::to_string(options.mux_address)+".lock");
    Fd mux_lock(options.mux_address?::open(mux_lock_path.c_str(),O_RDWR|O_CREAT|O_CLOEXEC,0664):-1);
    if (options.mux_address)
        check(mux_lock.get()>=0 && ::flock(mux_lock.get(),LOCK_EX|LOCK_NB)==0,"Mux is owned by another bridge");
    capture.summary.update({{"mux_address",options.mux_address?Json(options.mux_address):Json(nullptr)},
        {"mux_channel",options.mux_address?Json(options.mux_channel):Json(nullptr)}});
    int baud=0;
    std::optional<Serial> serial;
    std::optional<CubeUdp> shared_usb;
    if (options.cube_udp_port) shared_usb.emplace(options.cube_udp_port,options.cube_feedback_port);
    if (!options.sensor_only && !shared_usb) {
        baud=options.baud?options.baud:detect_baud(options.uart,uint8_t(options.system),stop);
        serial.emplace(options.uart,baud);
    }
    check(!stop.load(),"Stopping");
    bhu::extnav::RelaySocket vio(unsigned(options.vio_port),uint8_t(options.system));
    Parser parser;
    std::optional<I2cMux> mux;
    if (options.mux_address) {
        mux.emplace(options.bus,uint16_t(options.mux_address));
        mux->select(options.mux_channel);
    }
    Sensor sensor(options.bus,uint16_t(options.address),stop);
    sensor.initialize(options.assets,calibration);
    capture.summary.update({{"bus",options.bus},{"address",options.address},{"orientation",options.orientation},
        {"baud",baud},{"uart",(options.sensor_only || options.cube_udp_port)?Json(nullptr):Json(options.uart)},
        {"sensor_only",options.sensor_only},{"sensor_serial",sensor.serial},{"rangefinder_slot",options.slot},
        {"transport",options.sensor_only?"sensor-only":shared_usb?"shared-usb":"serial"},
        {"cube_udp_port",options.cube_udp_port},{"cube_feedback_port",options.cube_feedback_port},
        {"calibration_file",options.calibration.string()},{"backend","C++17"},
        {"aggregation",options.aggregation},{"minimum_active_zones",options.minimum_active_zones},
        {"confidence_threshold",options.confidence}});
    {
        std::lock_guard<std::mutex> guard(state.mutex);
        state.status="Running";state.error=nullptr;state.baud=baud;
    }
    const std::vector<std::string> config_names={options.prefix()+"TYPE",options.prefix()+"ORIENT",
        options.prefix()+"MIN",options.prefix()+"MAX","PRX1_TYPE"};
    size_t config_index=0;
    std::optional<uint8_t> previous;
    double next=now(),last_heartbeat=0,last_request=0,last_config=0;
    while (!stop.load()) {
        // Reassert one channel after a mux reset; never enable two 0x41 sensors.
        if (mux) mux->select(options.mux_channel);
        if(serial) if(auto message=vio.poll([] {return uint64_t(now()*1e6);}))capture.send(*serial,*message);
        {
            std::lock_guard<std::mutex> guard(state.mutex);
            state.vio_received=vio.filter.received;state.vio_rejected=vio.filter.rejected;
            state.vio_forwarded=vio.filter.forwarded;
        }
        auto measurement=sensor.measure(options.confidence);
        double stamp=now();
        bool duplicate=previous && *previous==measurement.number;
        if (duplicate) Session::increment(capture.summary,"duplicates");
        else if (previous) Session::increment(capture.summary,"result_counter_gaps",uint8_t(measurement.number-*previous)-1);
        previous=measurement.number;
        auto selected=selected_distance(measurement,options.aggregation=="zone-mean",options.minimum_active_zones);
        bool valid=measurement.row.at("calibration_status")==0 && selected && !duplicate;
        Json row=measurement.row;
        row.update({{"elapsed_s",stamp-capture.started},{"monotonic_s",stamp},
                    {"sensor_serial",sensor.serial},{"valid",valid},{"transmitted",valid && (serial || shared_usb)},
                    {"selected_mm",selected?Json(*selected):Json(nullptr)}});
        if (valid) {
            if (serial) {
                capture.send(*serial,distance_message(uint8_t(options.system),options.rotation(),*selected,stamp));
            } else if (shared_usb) {
                capture.send(*shared_usb,distance_message(uint8_t(options.system),options.rotation(),*selected,stamp));
            }
            if (serial || shared_usb) Session::increment(capture.summary,"sent_distances");
            int distance=*selected;
            capture.summary["selected_min_mm"]=std::min(capture.summary.value("selected_min_mm",distance),distance);
            capture.summary["selected_max_mm"]=std::max(capture.summary.value("selected_max_mm",distance),distance);
            if (measurement.nearest) {
                const int nearest=*measurement.nearest;
                capture.summary["nearest_min_mm"]=std::min(capture.summary.value("nearest_min_mm",nearest),nearest);
                capture.summary["nearest_max_mm"]=std::max(capture.summary.value("nearest_max_mm",nearest),nearest);
            }
        } else Session::increment(capture.summary,"rejected");
        Session::increment(capture.summary,"frames");
        Session::increment(capture.summary["calibration_status_counts"],std::to_string(row.at("calibration_status").get<int>()));
        capture.frame(row);
        {
            std::lock_guard<std::mutex> guard(state.mutex);
            state.latest=row;++state.sensor_count;
            bounded_push(state.sensor,Json{{"t",stamp-state.start},
                {"m",valid?Json(double(*selected/10)/100):Json(nullptr)}},1800);
        }
        if (serial && stamp-last_heartbeat>=1) {
            mavlink_message_t message{};
            mavlink_msg_heartbeat_pack(uint8_t(options.system),191,&message,MAV_TYPE_ONBOARD_CONTROLLER,
                MAV_AUTOPILOT_INVALID,0,0,MAV_STATE_ACTIVE);
            capture.send(*serial,message);last_heartbeat=stamp;
        }
        if (serial && stamp-last_request>=.2) {capture.send(*serial,distance_request(uint8_t(options.system)));last_request=stamp;}
        if (serial && stamp-last_config>=.4) {
            // ArduPilot has a small pending PARAM_VALUE queue: request one at a time.
            capture.send(*serial,parameter_request(uint8_t(options.system),config_names[config_index]));
            config_index=(config_index+1)%config_names.size();last_config=stamp;
        }
        // Limit each drain so a busy UART cannot starve fresh sensor sampling.
        for (int i=0;(serial || shared_usb) && i<4;++i) {
            auto incoming=serial?serial->read():shared_usb->read();if (incoming.empty()) break;
            capture.rx.write(reinterpret_cast<const char*>(incoming.data()),std::streamsize(incoming.size()));capture.rx.flush();
            Session::increment(capture.summary,"uart_rx_bytes",incoming.size());
            parser.feed(incoming.data(),incoming.size(),[&](const mavlink_message_t& message){
                auto entry=mavlink_get_msg_entry(message.msgid);
                // IDs are used in the summary; packets retain the full official dialect.
                (void)entry;
                Session::increment(capture.summary["uart_rx_messages"],std::to_string(message.msgid));
                if (message.sysid!=uint32_t(options.system) || message.compid!=1) return;
                double received=now();Json event;
                std::lock_guard<std::mutex> guard(state.mutex);
                if (message.msgid==MAVLINK_MSG_ID_HEARTBEAT) {
                    state.heartbeat=received;
                    Session::increment(capture.summary,"autopilot_heartbeats");
                    event={{"type","heartbeat"},{"monotonic_s",received}};
                } else if (message.msgid==MAVLINK_MSG_ID_DISTANCE_SENSOR) {
                    mavlink_distance_sensor_t value{};mavlink_msg_distance_sensor_decode(&message,&value);
                    if (value.orientation!=options.rotation()) return;
                    const char* kind=cube_distance_kind(options.slot,value.orientation,value.id);
                    if (!kind) return;
                    if(!cube_distance_in_tof_range(value.current_distance)){++state.unavailable_cube_reports;Session::increment(capture.summary,"unavailable_cube_reports");return;}
                    event={{"type","distance"},{"monotonic_s",received},{"t",received-state.start},
                        {"m",double(value.current_distance)/100},{"sensor_id",value.id},{"report_kind",kind},
                        {"cube_boot_ms",uint32_t(value.time_boot_ms)},{"source_system",message.sysid},{"source_component",message.compid},
                        {"orientation",value.orientation},{"min_cm",unsigned(value.min_distance)},{"max_cm",unsigned(value.max_distance)}};
                    bounded_push(state.cube,event,1200);++state.cube_count;
                } else if (message.msgid==MAVLINK_MSG_ID_PARAM_VALUE) {
                    mavlink_param_value_t value{};mavlink_msg_param_value_decode(&message,&value);
                    std::string name(value.param_id,strnlen(value.param_id,16));
                    if ((name.rfind(options.prefix(),0)!=0 && name!="PRX1_TYPE") || !std::isfinite(value.param_value)) return;
                    state.parameters[name]=float(value.param_value);
                    event={{"type","parameter"},{"monotonic_s",received},{"name",name},{"value",float(value.param_value)}};
                } else if (message.msgid==MAVLINK_MSG_ID_STATUSTEXT) {
                    mavlink_statustext_t value{};mavlink_msg_statustext_decode(&message,&value);
                    event={{"type","status_text"},{"monotonic_s",received},{"severity",value.severity},
                        {"text",std::string(value.text,strnlen(value.text,50))}};
                    bounded_push(state.messages,event,30);
                }
                if (!event.is_null()) capture.event(event);
            });
        }
        {std::lock_guard<std::mutex> guard(state.mutex);state.bad_packets=parser.bad;}
        next+=1/options.hz;
        double remaining=next-now();
        if (remaining>0) std::this_thread::sleep_for(std::chrono::duration<double>(remaining));
        else next=now();
    }
    capture.summary["uart_bad_packets"]=parser.bad;
    capture.summary["vio_received"]=vio.filter.received;
    capture.summary["vio_rejected"]=vio.filter.rejected;
    capture.summary["vio_forwarded"]=vio.filter.forwarded;
    capture.summary["vio_superseded"]=vio.filter.superseded;
}

static void worker(const Options& options,State& state,std::atomic<bool>& stop) {
    unsigned index=0,backoff=2;
    while (!stop.load()) {
        std::ostringstream name;name<<"bridge-"<<std::setfill('0')<<std::setw(3)<<++index;
        double started=now();std::string error;
        try {
            Session capture(options.output/name.str());
            try {run_session(options,state,stop,capture);}
            catch (const std::exception& e) {if (!stop.load()) error=e.what();}
            uint64_t bad;
            {std::lock_guard<std::mutex> guard(state.mutex);bad=state.bad_packets;}
            capture.finish(error,bad);
        } catch (const std::exception& e) {if (!stop.load()) error=e.what();}
        if (stop.load()) break;
        {
            std::lock_guard<std::mutex> guard(state.mutex);
            state.status="Reconnecting sensor";state.error=error;++state.reconnects;
        }
        std::cerr<<"Capture paused: "<<error<<"; retrying\n";
        if (now()-started>10) backoff=2;
        for (unsigned i=0;i<backoff*20 && !stop.load();++i) pause_ms(50);
        backoff=std::min(30u,backoff*2);
    }
    std::lock_guard<std::mutex> guard(state.mutex);state.status="Stopped";
}

class Server {
    Fd socket_;
    const Options& options_;
    const std::string page_;
public:
    explicit Server(const Options& options):socket_(::socket(AF_INET,SOCK_STREAM|SOCK_CLOEXEC,0)),
        options_(options),page_(read_text(options.assets/"viewer.html")) {
        check(socket_.get()>=0,os_error("Create HTTP socket"));
        int enabled=1;::setsockopt(socket_.get(),SOL_SOCKET,SO_REUSEADDR,&enabled,sizeof(enabled));
        sockaddr_in address{};address.sin_family=AF_INET;
        address.sin_port=htons(uint16_t(options.port));address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        check(::bind(socket_.get(),reinterpret_cast<sockaddr*>(&address),sizeof(address))==0,os_error("Bind viewer port"));
        check(::listen(socket_.get(),8)==0,os_error("Listen on viewer port"));
    }
    void tick(State& state) {
        pollfd waiting{socket_.get(),POLLIN,0};
        if (::poll(&waiting,1,100)<=0) return;
        Fd client(::accept4(socket_.get(),nullptr,nullptr,SOCK_CLOEXEC));if (client.get()<0)return;
        timeval timeout{0,500000};
        ::setsockopt(client.get(),SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
        ::setsockopt(client.get(),SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
        std::string request;std::array<char,2048> buffer{};
        while (request.find("\r\n\r\n")==std::string::npos && request.size()<8192) {
            auto size=::recv(client.get(),buffer.data(),buffer.size(),0);if (size<=0)return;
            request.append(buffer.data(),size_t(size));
        }
        std::istringstream first(request.substr(0,request.find("\r\n")));
        std::string method,path,version;first>>method>>path>>version;
        std::string body,type="text/plain; charset=utf-8",status="200 OK";
        path=path.substr(0,path.find('?'));
        if (request.size()>=8192) {status="431 Request Header Fields Too Large";body="Request too large";}
        else if (method!="GET") {status="405 Method Not Allowed";body="GET required";}
        else if (path=="/") {body=page_;type="text/html; charset=utf-8";}
        else if (path=="/api/state") {body=state.snapshot().dump();type="application/json";}
        else {status="404 Not Found";body="Not found";}
        std::string response="HTTP/1.1 "+status+"\r\nContent-Type: "+type+"\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
        size_t offset=0;
        while (offset<response.size()) {
            auto size=::send(client.get(),response.data()+offset,response.size()-offset,MSG_NOSIGNAL);
            if (size<=0) break;
            offset+=size_t(size);
        }
    }
};

static void self_test(const Options& options) {
    check(cube_distance_in_tof_range(95)&&cube_distance_in_tof_range(500)&&!cube_distance_in_tof_range(0)&&!cube_distance_in_tof_range(18000),"Rear global-max sentinel must not count as front ToF reception");
    std::vector<uint8_t> raw(132);raw[0]=0x10;raw[2]=128;raw[4]=255;raw[5]=uint8_t(-5);
    raw[24]=19;raw[25]=100;raw[78]=100;raw[79]=uint8_t(743&255);raw[80]=uint8_t(743>>8);
    auto measurement=decode_measurement(raw,0,20);
    check(measurement.nearest==743,"Secondary/threshold parsing failed");
    check(measurement.row.at("temperature_c")==-5,"Signed sensor temperature failed");
    check(!decode_measurement(raw,0,101).nearest,"Confidence rejection failed");
    std::vector<uint8_t> zones(132);zones[0]=0x10;zones[2]=128;
    auto set_zone=[&](int i,int mm,int quality) {
        zones[24+3*i]=uint8_t(quality);zones[25+3*i]=uint8_t(mm&255);zones[26+3*i]=uint8_t(mm>>8);
    };
    for (int i=0;i<3;++i)set_zone(i,1000*(i+1),20);
    zones[78]=255;zones[79]=100; // A secondary return is not a fourth zone.
    auto three=decode_measurement(zones,0,20);
    check(three.active_zones==3 && !selected_distance(three,true,4),"Three-zone gate or secondary exclusion failed");
    set_zone(3,4000,20);
    check(selected_distance(decode_measurement(zones,0,20),true,4)==2500,"Four-zone arithmetic mean failed");
    set_zone(4,5000,20);
    check(selected_distance(decode_measurement(zones,0,20),true,4)==3000,"All active zones must enter the mean");
    set_zone(5,1,255);set_zone(6,5001,255);set_zone(7,500,19);
    check(decode_measurement(zones,0,20).active_zones==5,"Invalid/weak zones counted as active");
    std::fill(zones.begin()+4,zones.end(),0);
    set_zone(0,1000,50);set_zone(1,2000,51);set_zone(2,3000,255);
    auto above50=decode_measurement(zones,0,51);
    check(above50.active_zones==2 && selected_distance(above50,true,1)==2500,
        "Confidence must be strictly above 50; average every qualifying zone");
    set_zone(1,2000,50);set_zone(2,3000,50);
    check(!selected_distance(decode_measurement(zones,0,51),true,1),"No confident zones must suppress output");
    check(std::string(cube_distance_kind(2,4,1))=="rangefinder","Rear rangefinder ID failed");
    check(std::string(cube_distance_kind(2,4,14))=="proximity","Rear proximity sector ID failed");
    check(std::string(cube_distance_kind(5,24,19))=="proximity","Upward proximity ID failed");
    check(cube_distance_kind(2,4,12)==nullptr && cube_distance_kind(2,4,0)==nullptr,
        "Unrelated Cube sensor ID accepted");
    auto bytes=packet(distance_message(1,4,743,1234.567));
    Parser parser;int accepted=0;
    for (auto byte:bytes) parser.feed(&byte,1,[&](const mavlink_message_t& message){
        mavlink_distance_sensor_t value{};mavlink_msg_distance_sensor_decode(&message,&value);
        check(message.sysid==1 && message.compid==191,"MAVLink identity mismatch");
        check(value.current_distance==74 && value.orientation==4 && value.min_distance==1 && value.max_distance==500,
              "MAVLink units/orientation mismatch");++accepted;
    });
    check(accepted==1 && parser.bad==0,"Fragmented MAVLink parsing failed");
    bytes[12]^=1;Parser damaged;
    damaged.feed(bytes.data(),bytes.size(),[&](const mavlink_message_t&){fail("Corrupt MAVLink packet accepted");});
    check(damaged.bad==1,"Corrupt MAVLink checksum was not rejected");
    auto calibration=Json::parse(read_text(options.calibration));
    check(sha256(unhex(calibration.at("data_hex")))==calibration.at("sha256"),"Saved calibration integrity failed");
    State state(options);auto snapshot=state.snapshot();
    check(snapshot.at("sensor_age_s").is_null() && snapshot.at("cube_age_s").is_null(),"No-data freshness failed");
    std::cout<<"PASS: native measurement parsing, confidence/secondary selection, units, MAVLink fragmentation/CRC, calibration integrity, empty-state freshness\n";
}

int main(int argc,char** argv) {
    try {
        Options options;bool test=false;
        for (int i=1;i<argc;++i) {
            std::string key=argv[i];
            if (key=="--self-test") {test=true;continue;}
            if (key=="--sensor-only") {options.sensor_only=true;continue;}
            if (key=="--help") {
                std::cout<<"Native ToF/Cube viewer\n--port 8765 --bus 7 --address 0x41 --uart /dev/ttyTHS1\n"
                    <<"--sensor-only (view sensor without opening or sending to the Cube UART)\n"
                    <<"--cube-udp-port 14605 --cube-feedback-port 14606 (shared USB owner, no serial access)\n"
                    <<"--aggregation nearest|zone-mean --min-active-zones 1 (default: zone-mean)\n"
                    <<"--mux-address none|0x70 --mux-channel 0 (0-7; mux disabled by default)\n"
                    <<"--baud auto|57600|115200|921600 --orientation rear --rangefinder-slot 2\n"
                    <<"--system-id 1 --hz 15 --confidence 51 (strictly above 50) --assets PATH --calibration PATH\n"
                    <<"--output NEW_DIRECTORY --duration SECONDS --vio-udp-port 0 --self-test\n";return 0;
            }
            check(i+1<argc,"Missing value for "+key);std::string value=argv[++i];
            if (key=="--port")options.port=std::stoi(value);
            else if (key=="--cube-udp-port")options.cube_udp_port=std::stoi(value);
            else if (key=="--cube-feedback-port")options.cube_feedback_port=std::stoi(value);
            else if (key=="--vio-udp-port")options.vio_port=std::stoi(value);
            else if (key=="--bus")options.bus=std::stoi(value);
            else if (key=="--address")options.address=std::stoi(value,nullptr,0);
            else if (key=="--mux-address")options.mux_address=value=="none"?0:std::stoi(value,nullptr,0);
            else if (key=="--mux-channel")options.mux_channel=std::stoi(value);
            else if (key=="--uart")options.uart=value;
            else if (key=="--baud")options.baud=value=="auto"?0:std::stoi(value);
            else if (key=="--orientation")options.orientation=value;
            else if (key=="--rangefinder-slot")options.slot=std::stoi(value);
            else if (key=="--system-id")options.system=std::stoi(value);
            else if (key=="--hz")options.hz=std::stod(value);
            else if (key=="--duration")options.duration=std::stod(value);
            else if (key=="--confidence")options.confidence=std::stoi(value);
            else if (key=="--aggregation")options.aggregation=value;
            else if (key=="--min-active-zones")options.minimum_active_zones=std::stoi(value);
            else if (key=="--assets")options.assets=fs::absolute(value);
            else if (key=="--calibration")options.calibration=fs::absolute(value);
            else if (key=="--output")options.output=fs::absolute(value);
            else fail("Unknown option "+key);
        }
        check(options.port>0 && options.port<=65535,"Invalid viewer port");
        check(options.vio_port>=0 && options.vio_port<=65535,"Invalid VIO UDP port");
        check(options.cube_udp_port>=0 && options.cube_udp_port<=65535 && options.cube_feedback_port>0
            && options.cube_feedback_port<=65535,"Invalid Cube handoff ports");
        check(!options.cube_udp_port || (!options.sensor_only && options.vio_port==0
            && options.cube_udp_port!=options.cube_feedback_port),"Shared USB requires distinct ports and no sensor-only/VIO relay");
        check(!options.sensor_only || options.vio_port==0,"Sensor-only mode cannot relay VIO");
        check(options.slot>=1 && options.slot<=10 && options.system>=1 && options.system<=255,"Invalid rangefinder slot/system ID");
        check(std::isfinite(options.hz) && options.hz>0 && options.hz<=30,"Invalid sample rate");
        check(std::isfinite(options.duration) && options.duration>=0,"Invalid duration");
        check(options.confidence>=1 && options.confidence<=255,"Invalid confidence threshold");
        check(options.aggregation=="nearest" || options.aggregation=="zone-mean","Unknown aggregation (nearest|zone-mean)");
        check(options.minimum_active_zones>=1 && options.minimum_active_zones<=9,"Minimum active zones must be 1-9");
        check(options.address>=8 && options.address<0x78 && options.bus>=0,"Invalid I2C settings");
        check(options.mux_address==0 || (options.mux_address>=0x70 && options.mux_address<=0x77),"Invalid PCA9548A address (0x70-0x77)");
        check(options.mux_channel>=0 && options.mux_channel<=7,"Invalid mux channel (0-7)");
        check(!options.mux_address || options.address!=options.mux_address,"Sensor and mux addresses must differ");
        check(options.baud==0 || options.baud==57600 || options.baud==115200 || options.baud==921600,"Unsupported baud");
        options.rotation();
        options.assets=fs::weakly_canonical(options.assets);
        if (options.calibration.empty())options.calibration=options.assets/"calibration-23937420-map1.json";
        if (test) {self_test(options);return 0;}
        if (options.output.empty()) {
            auto stamp=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            options.output=options.assets.parent_path().parent_path()/"reports/tof"/("native-live-"+std::to_string(stamp));
        }
        check(!fs::exists(options.output),"Output directory already exists");fs::create_directories(options.output);
        Server server(options);State state(options);std::atomic<bool> stop{false};
        ::signal(SIGINT,signal_handler);::signal(SIGTERM,signal_handler);::signal(SIGPIPE,SIG_IGN);
        std::thread capture(worker,std::cref(options),std::ref(state),std::ref(stop));
        std::cout<<"C++ live viewer: http://127.0.0.1:"<<options.port<<"\nRecordings: "<<options.output<<std::endl;
        try {
            while (!interrupted && (!options.duration || now()-state.start<options.duration)) server.tick(state);
        } catch (...) {stop.store(true);capture.join();throw;}
        stop.store(true);capture.join();save_json(options.output/"viewer-summary.json",state.snapshot());
        return 0;
    } catch (const std::exception& e) {std::cerr<<"Error: "<<e.what()<<'\n';return 1;}
}
