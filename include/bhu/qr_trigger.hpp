#pragma once
#include <opencv2/core.hpp>
#include <zbar.h>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace bhu::bench {
class QrScanner {
    zbar::zbar_image_scanner_t* scanner_=nullptr;
public:
    std::vector<std::string> decoded;
    QrScanner():scanner_(zbar::zbar_image_scanner_create()) {
        if(!scanner_)throw std::runtime_error("Cannot create QR scanner");
        zbar::zbar_image_scanner_set_config(scanner_,zbar::ZBAR_NONE,zbar::ZBAR_CFG_ENABLE,0);
        zbar::zbar_image_scanner_set_config(scanner_,zbar::ZBAR_QRCODE,zbar::ZBAR_CFG_ENABLE,1);
    }
    ~QrScanner(){zbar::zbar_image_scanner_destroy(scanner_);}
    QrScanner(const QrScanner&)=delete;
    bool matches(const cv::Mat& input) {
        decoded.clear();
        if(input.type()!=CV_8UC1 || input.empty())throw std::invalid_argument("QR scan needs grayscale image");
        cv::Mat gray=input.isContinuous()?input:input.clone();
        auto* image=zbar::zbar_image_create();
        if(!image)throw std::runtime_error("Cannot create QR image");
        zbar::zbar_image_set_format(image,0x30303859); // Y800, tightly packed
        zbar::zbar_image_set_size(image,gray.cols,gray.rows);
        zbar::zbar_image_set_data(image,gray.data,gray.total(),nullptr);
        zbar::zbar_scan_image(scanner_,image);bool match=false;
        for(auto* symbol=zbar::zbar_image_first_symbol(image);symbol;symbol=zbar::zbar_symbol_next(symbol))
            if(zbar::zbar_symbol_get_type(symbol)==zbar::ZBAR_QRCODE){
                auto value=std::string(zbar::zbar_symbol_get_data(symbol),zbar::zbar_symbol_get_data_length(symbol));
                decoded.push_back(value);if(value=="test_vio")match=true;
            }
        zbar::zbar_image_destroy(image);return match;
    }
};

enum class TriggerEvent {None,Start,Stop};
// Two positive observations within 750 ms are required. A continuous view can
// never toggle twice. Stop is armed only after 2 s continuously QR-free and a
// minimum 5 s run. A missed observation in a positive streak resets the streak.
class QrTrigger {
    bool active_=false,finished_=false,released_=false;
    unsigned positives_=0;
    uint64_t last_positive_=0,absent_since_=0,started_=0;
public:
    TriggerEvent observe(bool present,uint64_t now_ms) {
        if(finished_)return TriggerEvent::None;
        if(!present) {
            positives_=0;
            if(!absent_since_)absent_since_=now_ms;
            if(active_ && now_ms-absent_since_>=2000)released_=true;
            return TriggerEvent::None;
        }
        absent_since_=0;
        if(last_positive_ && now_ms-last_positive_<=750)++positives_;else positives_=1;
        last_positive_=now_ms;
        if(positives_<2)return TriggerEvent::None;
        if(!active_) {active_=true;started_=now_ms;positives_=0;return TriggerEvent::Start;}
        if(released_ && now_ms-started_>=5000) {finished_=true;return TriggerEvent::Stop;}
        return TriggerEvent::None;
    }
    bool active()const{return active_ && !finished_;}
};
} // namespace bhu::bench
