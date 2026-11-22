#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <openssl/evp.h>
#include "json.hpp"
#include "mavlink/ardupilotmega/mavlink.h"

namespace tof {
using Json = nlohmann::json;
namespace fs = std::filesystem;
inline double now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline void pause_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
inline void fail(const std::string& message) { throw std::runtime_error(message); }
inline void check(bool ok, const std::string& message) { if (!ok) fail(message); }
inline std::string os_error(const std::string& where) { return where + ": " + std::strerror(errno); }

class Fd {
    int value_ = -1;
public:
    explicit Fd(int value = -1): value_(value) {}
    ~Fd() { if (value_ >= 0) ::close(value_); }
    Fd(const Fd&) = delete;
    Fd& operator=(const Fd&) = delete;
    int get() const { return value_; }
};
inline std::vector<uint8_t> read_binary(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    check(bool(file), "Cannot open " + path.string());
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
inline std::string read_text(const fs::path& path) {
    auto bytes = read_binary(path);
    return {bytes.begin(), bytes.end()};
}
inline void save_json(const fs::path& path, const Json& value) {
    std::ofstream file(path);
    file << value.dump(2) << '\n';
    check(bool(file), "Cannot write " + path.string());
}
inline std::string sha256(const std::vector<uint8_t>& data) {
    std::array<unsigned char, EVP_MAX_MD_SIZE> hash{};
    unsigned int length = 0;
    check(EVP_Digest(data.data(), data.size(), hash.data(), &length, EVP_sha256(), nullptr) == 1,
          "SHA256 failed");
    std::ostringstream output;
    for (unsigned i=0; i<length; ++i) output << std::hex << std::setw(2) << std::setfill('0') << unsigned(hash[i]);
    return output.str();
}
inline std::vector<uint8_t> unhex(const std::string& text) {
    check(text.size()%2 == 0, "Odd calibration hex length");
    auto digit = [](char c) -> unsigned {
        if (c >= '0' && c <= '9') return unsigned(c-'0');
        if (c >= 'a' && c <= 'f') return unsigned(c-'a'+10);
        if (c >= 'A' && c <= 'F') return unsigned(c-'A'+10);
        fail("Invalid calibration hex"); return 0;
    };
    std::vector<uint8_t> bytes;
    for (size_t i=0; i<text.size(); i+=2) bytes.push_back(uint8_t(digit(text[i])*16+digit(text[i+1])));
    return bytes;
}
inline uint16_t le16(const std::vector<uint8_t>& b, size_t offset) {
    return uint16_t(b.at(offset)) | uint16_t(uint16_t(b.at(offset+1)) << 8);
}
inline uint32_t le32(const std::vector<uint8_t>& b, size_t offset) {
    return uint32_t(le16(b,offset)) | (uint32_t(le16(b,offset+2)) << 16);
}

class I2c {
    Fd fd_;
    uint16_t address_;
public:
    I2c(int bus, uint16_t address): fd_(::open(("/dev/i2c-"+std::to_string(bus)).c_str(), O_RDWR|O_CLOEXEC)), address_(address) {
        check(fd_.get() >= 0, os_error("Open I2C"));
        unsigned long capabilities = 0;
        check(::ioctl(fd_.get(), I2C_FUNCS, &capabilities) >= 0 && (capabilities & I2C_FUNC_I2C),
              "Adapter lacks native I2C transfers");
    }
    std::vector<uint8_t> read(uint8_t reg, uint16_t size) {
        std::vector<uint8_t> output(size);
        i2c_msg messages[2] = {{address_, 0, 1, &reg}, {address_, I2C_M_RD, size, output.data()}};
        i2c_rdwr_ioctl_data transfer{messages,2};
        check(::ioctl(fd_.get(), I2C_RDWR, &transfer) == 2, os_error("I2C read"));
        return output;
    }
    uint8_t byte(uint8_t reg) { return read(reg,1)[0]; }
    // PCA9548A has a single control byte, with no register-address prefix.
    uint8_t read_control() {
        uint8_t value=0;
        i2c_msg message{address_,I2C_M_RD,1,&value};
        i2c_rdwr_ioctl_data transfer{&message,1};
        check(::ioctl(fd_.get(),I2C_RDWR,&transfer)==1,os_error("Read mux control"));
        return value;
    }
    void write_control(uint8_t value) {
        i2c_msg message{address_,0,1,&value};
        i2c_rdwr_ioctl_data transfer{&message,1};
        // A separate transfer ends with STOP, activating the selected channel.
        check(::ioctl(fd_.get(),I2C_RDWR,&transfer)==1,os_error("Write mux control"));
    }
    void write(uint8_t reg, const std::vector<uint8_t>& values) {
        std::vector<uint8_t> payload{reg};
        payload.insert(payload.end(),values.begin(),values.end());
        i2c_msg message{address_,0,uint16_t(payload.size()),payload.data()};
        i2c_rdwr_ioctl_data transfer{&message,1};
        check(::ioctl(fd_.get(), I2C_RDWR, &transfer) == 1, os_error("I2C write"));
    }
    void write(uint8_t reg, uint8_t value) { write(reg,std::vector<uint8_t>{value}); }
};

class I2cMux {
    I2c bus_;
public:
    I2cMux(int bus,uint16_t address):bus_(bus,address) {}
    ~I2cMux() {try {bus_.write_control(0);} catch (...) {}}
    void select(int channel) {
        check(channel>=0 && channel<=7,"Invalid mux channel");
        const uint8_t mask=uint8_t(1u<<channel);
        bus_.write_control(mask);
        check(bus_.read_control()==mask,"Mux channel readback mismatch");
    }
};

struct Measurement {
    Json row;
    std::optional<int> nearest;
    uint8_t number=0;
    int active_zones=0,primary_sum_mm=0;
};

inline std::optional<int> selected_distance(const Measurement& measurement,bool zone_mean,int minimum_zones) {
    if (!zone_mean) return measurement.nearest;
    if (measurement.active_zones<minimum_zones) return std::nullopt;
    // Integer mm truncation preserves floor(mean_mm/10) for MAVLink centimetres.
    return measurement.primary_sum_mm/measurement.active_zones;
}

// ArduPilot sends rangefinders as slot-1, but a RangeFinder proximity backend
// replaces horizontal/upward reports with sector IDs starting at 10 (GCS_Common).
inline const char* cube_distance_kind(int slot,uint8_t orientation,uint8_t id) {
    if (id==slot-1) return "rangefinder";
    if ((orientation<=7 && id==10+orientation) || (orientation==24 && id==19)) return "proximity";
    return nullptr;
}
inline Measurement decode_measurement(const std::vector<uint8_t>& data, uint8_t calibration, int confidence) {
    check(data.size() == 132 && data[0] == 0x10 && le16(data,2) == 128, "Invalid measurement page");
    Measurement result;
    result.number = data[4];
    Json primary=Json::array(), primary_confidence=Json::array();
    Json secondary=Json::array(), secondary_confidence=Json::array();
    for (int y=0; y<3; ++y) {
        Json p=Json::array(), pc=Json::array(), s=Json::array(), sc=Json::array();
        for (int x=0; x<3; ++x) {
            const size_t offset=24+3*(y*3+x), second=offset+54;
            int d=le16(data,offset+1), c=data[offset];
            int sd=le16(data,second+1), cc=data[second];
            if (c>=confidence && d>=10 && d<=5000) {
                ++result.active_zones;result.primary_sum_mm+=d;
            }
            p.push_back(d); pc.push_back(c); s.push_back(sd); sc.push_back(cc);
            for (auto [distance,quality] : {std::pair{d,c},std::pair{sd,cc}})
                if (quality>=confidence && distance>=10 && distance<=5000)
                    if (!result.nearest || distance<*result.nearest) result.nearest=distance;
        }
        primary.push_back(p); primary_confidence.push_back(pc);
        secondary.push_back(s); secondary_confidence.push_back(sc);
    }
    result.row={{"result_number",result.number},{"temperature_c",int(int8_t(data[5]))},
                {"system_tick",le32(data,20)},{"calibration_status",calibration},
                {"distance_mm",primary},{"confidence",primary_confidence},
                {"secondary_mm",secondary},{"secondary_confidence",secondary_confidence},
                {"nearest_mm",result.nearest ? Json(*result.nearest) : Json(nullptr)},
                {"active_zone_count",result.active_zones},
                {"primary_mean_mm",result.active_zones?Json(double(result.primary_sum_mm)/result.active_zones):Json(nullptr)}};
    return result;
}

class Sensor {
    I2c bus_;
    const std::atomic<bool>& stop_;
    void wait(const std::function<bool()>& ready, double timeout, const std::string& description) {
        double end=now()+timeout;
        while (!ready()) {
            check(!stop_.load(), "Stopping");
            check(now()<end, description+" timed out");
            pause_ms(1);
        }
    }
    void boot_command(uint8_t command, const std::vector<uint8_t>& data) {
        std::vector<uint8_t> payload{command,uint8_t(data.size())};
        payload.insert(payload.end(),data.begin(),data.end());
        unsigned sum=0; for (auto b:payload) sum+=b;
        payload.push_back(uint8_t((sum&255)^255));
        bus_.write(8,payload);
        wait([&]{return bus_.read(8,3)[0]==0;},.5,"Bootloader command");
    }
public:
    Sensor(int bus, uint16_t address, const std::atomic<bool>& stop): bus_(bus,address),stop_(stop) {}
    ~Sensor() {
        try { bus_.write(8,0xff); bus_.write(0xe0,0x20); } catch (...) {}
    }
    void command(uint8_t value) {
        bus_.write(8,value);
        uint8_t status=255;
        wait([&]{status=bus_.byte(8);return status<0x10;},3,"Sensor command "+std::to_string(value));
        check(status<=1,"Sensor command failed, status "+std::to_string(status));
    }
    void initialize(const fs::path& assets, const Json& calibration) {
        bus_.write(0xe0,0x21);
        wait([&]{return (bus_.byte(0xe0)&0xcf)==0x41;},.5,"Sensor enable");
        if (bus_.byte(0)==0x80) {
            auto firmware=read_binary(assets/"vendor/tmf882x/firmware.bin");
            check(sha256(firmware)=="0d3304708fdc8e7454347ab2e4da3e70d444159e7f3d5e746fbbf2b2a20599f7", "Firmware checksum mismatch");
            boot_command(0x14,{0x29}); boot_command(0x43,{0,0});
            for (size_t offset=0;offset<firmware.size();offset+=80) {
                check(!stop_.load(),"Stopping");
                boot_command(0x41,{firmware.begin()+offset,firmware.begin()+std::min(offset+80,firmware.size())});
            }
            boot_command(0x11,{}); pause_ms(5);
        }
        check(bus_.byte(0)==3,"Sensor ranging application is not running");
        serial=le32(bus_.read(0x1c,4),0);
        check(calibration.at("sensor_serial").get<uint32_t>()==serial,"Calibration sensor serial mismatch");
        check(calibration.at("spad_map").get<int>()==1,"Only TMF8820 3x3 map 1 is supported");
        command(0xff);
        command(0x16);
        auto header=bus_.read(0x20,4);
        check(header[0]==0x16 && le16(header,2)==188,"Unexpected sensor configuration page");
        bus_.write(0x34,1); command(0x15);
        auto payload=unhex(calibration.at("data_hex").get<std::string>());
        check(payload.size()==188 && sha256(payload)==calibration.at("sha256").get<std::string>(),
              "Calibration checksum/length mismatch");
        command(0x19);
        for (size_t offset=0;offset<payload.size();offset+=32)
            bus_.write(uint8_t(0x24+offset),{payload.begin()+offset,payload.begin()+std::min(offset+32,payload.size())});
        command(0x15);
    }
    uint32_t serial=0;
    Measurement measure(int confidence) {
        bus_.write(0xe1,0xff); command(0x10);
        try {
            wait([&]{return (bus_.byte(0xe1)&2)!=0;},1,"Fresh sensor measurement");
            auto data=bus_.read(0x20,132);
            auto calibration=bus_.byte(7);
            command(0xff);
            return decode_measurement(data,calibration,confidence);
        } catch (...) {
            try {bus_.write(8,0xff);} catch (...) {}
            throw;
        }
    }
};

class Parser {
    mavlink_message_t buffer_{};
    mavlink_status_t status_{};
public:
    uint64_t bad=0;
    void feed(const uint8_t* bytes, size_t length, const std::function<void(const mavlink_message_t&)>& receive) {
        for (size_t i=0;i<length;++i) {
            mavlink_message_t message{}; mavlink_status_t status{};
            auto framing=mavlink_frame_char_buffer(&buffer_,&status_,bytes[i],&message,&status);
            if (framing==MAVLINK_FRAMING_OK) receive(message);
            else if (framing==MAVLINK_FRAMING_BAD_CRC || framing==MAVLINK_FRAMING_BAD_SIGNATURE) ++bad;
        }
    }
};
inline std::vector<uint8_t> packet(const mavlink_message_t& message) {
    std::array<uint8_t,MAVLINK_MAX_PACKET_LEN> buffer{};
    size_t size=mavlink_msg_to_send_buffer(buffer.data(),&message);
    return {buffer.begin(),buffer.begin()+size};
}
inline mavlink_message_t distance_message(uint8_t system, uint8_t orientation, int mm, double stamp) {
    mavlink_message_t message{};
    float quaternion[4]={0,0,0,0};
    mavlink_msg_distance_sensor_pack(system,191,&message,uint32_t(uint64_t(stamp*1000)),
        1,500,uint16_t(mm/10),MAV_DISTANCE_SENSOR_LASER,0,orientation,255,0,0,quaternion,0);
    return message;
}

class Serial {
    Fd fd_;
public:
    Serial(const std::string& path, int baud):fd_(::open(path.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC)) {
        check(fd_.get()>=0,os_error("Open UART"));
        check(::flock(fd_.get(),LOCK_EX|LOCK_NB)==0,os_error("UART already in use"));
        check(::ioctl(fd_.get(),TIOCEXCL)==0,os_error("UART exclusive access"));
        termios config{};
        check(::tcgetattr(fd_.get(),&config)==0,os_error("Read UART settings"));
        ::cfmakeraw(&config);
        config.c_cflag &= ~(CSIZE|PARENB|CSTOPB|CRTSCTS);
        config.c_cflag |= CS8|CLOCAL|CREAD;
        config.c_cc[VMIN]=0; config.c_cc[VTIME]=0;
        speed_t speed;
        switch (baud) {
            case 57600:speed=B57600;break; case 115200:speed=B115200;break;
            case 921600:speed=B921600;break; default:fail("Unsupported UART baud");
        }
        ::cfsetispeed(&config,speed); ::cfsetospeed(&config,speed);
        check(::tcsetattr(fd_.get(),TCSANOW,&config)==0,os_error("Set UART speed"));
    }
    ~Serial() { ::ioctl(fd_.get(),TIOCNXCL); }
    void write(const std::vector<uint8_t>& data) {
        size_t offset=0; double deadline=now()+1;
        while (offset<data.size()) {
            ssize_t n=::write(fd_.get(),data.data()+offset,data.size()-offset);
            if (n>0) {offset+=size_t(n);continue;}
            if (n<0 && errno!=EAGAIN && errno!=EINTR) fail(os_error("UART write"));
            check(now()<deadline,"UART write timed out");
            pollfd waiting{fd_.get(),POLLOUT,0}; ::poll(&waiting,1,20);
        }
    }
    std::vector<uint8_t> read() {
        std::array<uint8_t,4096> buffer{};
        ssize_t n=::read(fd_.get(),buffer.data(),buffer.size());
        if (n<0 && errno!=EAGAIN && errno!=EINTR) fail(os_error("UART read"));
        return n>0?std::vector<uint8_t>(buffer.begin(),buffer.begin()+n):std::vector<uint8_t>{};
    }
};
inline mavlink_message_t parameter_request(uint8_t system,const std::string& name, uint8_t source=1, uint8_t component=191) {
    check(name.size()<=16,"Parameter name too long");
    char field[16]{}; std::memcpy(field,name.data(),name.size());
    mavlink_message_t message{};
    mavlink_msg_param_request_read_pack(source,component,&message,system,1,field,-1);
    return message;
}
inline mavlink_message_t distance_request(uint8_t system) {
    mavlink_message_t message{};
    mavlink_msg_command_long_pack(system,191,&message,system,1,MAV_CMD_REQUEST_MESSAGE,0,132,0,0,0,0,0,0);
    return message;
}
inline int detect_baud(const std::string& path,uint8_t system,const std::atomic<bool>& stop) {
    for (int baud:{57600,115200,921600}) {
        Serial port(path,baud); Parser decoder;
        int heartbeats=0; bool found=false; double end=now()+1.8,next=0;
        while (!stop.load() && now()<end && !found) {
            if (now()>=next) {port.write(packet(parameter_request(system,"MAV_SYSID",254,190)));next=now()+.4;}
            auto data=port.read();
            decoder.feed(data.data(),data.size(),[&](const mavlink_message_t& message){
                if (message.sysid!=system || message.compid!=1) return;
                if (message.msgid==MAVLINK_MSG_ID_PARAM_VALUE) {
                    mavlink_param_value_t value{}; mavlink_msg_param_value_decode(&message,&value);
                    if (std::string(value.param_id,strnlen(value.param_id,16))=="MAV_SYSID") found=true;
                } else if (message.msgid==MAVLINK_MSG_ID_HEARTBEAT) {
                    mavlink_heartbeat_t heartbeat{}; mavlink_msg_heartbeat_decode(&message,&heartbeat);
                    if (heartbeat.autopilot!=MAV_AUTOPILOT_INVALID && ++heartbeats>=2) found=true;
                }
            });
            if (!found) pause_ms(5);
        }
        if (found) return baud;
    }
    fail("No Cube reply at 57600, 115200, or 921600 baud"); return 0;
}
} // namespace tof
