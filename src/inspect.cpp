#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/TypeHelper.hpp>
#include <fstream>
#include <iostream>

int main() try {
    ob::Context context;
    context.setLoggerSeverity(OB_LOG_SEVERITY_WARN);
    auto devices = context.queryDeviceList();
    if(!devices->getCount()) throw std::runtime_error("No Orbbec device accessible");
    for(uint32_t n=0; n<devices->getCount(); ++n) {
        auto device=devices->getDevice(n);
        auto info=device->getDeviceInfo();
        std::cout << "Device: " << info->getName() << "\nSerial: " << info->getSerialNumber()
                  << "\nFirmware: " << info->getFirmwareVersion() << '\n';
        for(auto id : {OB_PROP_IR_RECTIFY_BOOL, OB_PROP_LASER_BOOL, OB_PROP_IR_AUTO_EXPOSURE_BOOL}) {
            if(device->isPropertySupported(id, OB_PERMISSION_READ))
                std::cout << "Property " << id << ": " << device->getBoolProperty(id) << '\n';
        }
        if(device->isPropertySupported(OB_PROP_LASER_CONTROL_INT,OB_PERMISSION_READ))
            std::cout<<"Laser control: "<<device->getIntProperty(OB_PROP_LASER_CONTROL_INT)<<'\n';
        auto sensors=device->getSensorList();
        for(uint32_t i=0;i<sensors->getCount();++i) {
            auto sensor=sensors->getSensor(i);
            auto type=sensors->getSensorType(i);
            std::cout << "Sensor " << type << " (" << ob::TypeHelper::convertOBSensorTypeToString(type) << ")\n";
            auto profiles=sensor->getStreamProfileList();
            for(uint32_t j=0;j<profiles->getCount();++j) {
                auto p=profiles->getProfile(j);
                if(p->is<ob::VideoStreamProfile>()) {
                    auto v=p->as<ob::VideoStreamProfile>();
                    std::cout << "  " << v->getWidth() << 'x' << v->getHeight()
                              << " fps=" << v->getFps() << " format=" << v->getFormat()
                              << " (" << ob::TypeHelper::convertOBFormatTypeToString(v->getFormat()) << ")\n";
                } else if(p->is<ob::AccelStreamProfile>()) {
                    auto a=p->as<ob::AccelStreamProfile>();
                    std::cout << "  accel range_enum=" << a->getFullScaleRange()
                              << " rate_enum=" << a->getSampleRate() << '\n';
                    if(j==0) {
                        auto k=a->getIntrinsic();std::cout<<"  accel correction scale matrix:";
                        for(double v:k.scaleMisalignment)std::cout<<' '<<v;
                        std::cout<<"\n  accel bias:";for(double v:k.bias)std::cout<<' '<<v;std::cout<<'\n';
                    }
                } else if(p->is<ob::GyroStreamProfile>()) {
                    auto g=p->as<ob::GyroStreamProfile>();
                    std::cout << "  gyro range_enum=" << g->getFullScaleRange()
                              << " rate_enum=" << g->getSampleRate() << '\n';
                    if(j==0) {
                        auto k=g->getIntrinsic();std::cout<<"  gyro correction scale matrix:";
                        for(double v:k.scaleMisalignment)std::cout<<' '<<v;
                        std::cout<<"\n  gyro bias:";for(double v:k.bias)std::cout<<' '<<v;std::cout<<'\n';
                    }
                }
            }
        }
    }
    return 0;
} catch(const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
