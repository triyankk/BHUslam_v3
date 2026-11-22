#include "bhu/qr_trigger.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <json.hpp>
#include <iostream>
int main(int argc,char** argv)try{
    if(argc!=2)throw std::runtime_error("Usage: qr_inspect IMAGE");auto input=cv::imread(argv[1],cv::IMREAD_GRAYSCALE);if(input.empty())throw std::runtime_error("Cannot read image");
    if(input.cols==1280)input=input(cv::Rect(0,0,640,400)).clone();
    bhu::bench::QrScanner scanner;nlohmann::json result=nlohmann::json::array();
    for(int variant=0;variant<4;++variant){cv::Mat image;
        if(variant==0)image=input;else if(variant==1)cv::normalize(input,image,0,255,cv::NORM_MINMAX);else if(variant==2)cv::createCLAHE(3)->apply(input,image);else {cv::createCLAHE(3)->apply(input,image);cv::resize(image,image,{},2,2,cv::INTER_CUBIC);}
        bool match=scanner.matches(image);result.push_back({{"method",variant},{"test_vio",match},{"decoded",scanner.decoded}});
    }
    std::cout<<result.dump(2)<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
