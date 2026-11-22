#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <qrencode.h>
#include <filesystem>
#include <fstream>
#include <iostream>
int main(int argc,char** argv)try {
    if(argc!=2){std::cerr<<"Usage: vio_qr OUTPUT_PREFIX\n";return 2;}
    std::filesystem::path base=argv[1];
    if(!base.parent_path().empty())std::filesystem::create_directories(base.parent_path());
    auto* code=QRcode_encodeString8bit("test_vio",0,QR_ECLEVEL_M);
    if(!code)throw std::runtime_error("QR encoding failed");
    const int border=4,size=code->width+2*border;
    cv::Mat gray(size,size,CV_8UC1,cv::Scalar(255));
    std::ofstream svg(base.string()+".svg");
    svg<<"<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 "<<size<<' '<<size<<"\" width=\"160mm\" height=\"160mm\"><rect width=\"100%\" height=\"100%\" fill=\"white\"/><g fill=\"black\">";
    for(int y=0;y<code->width;++y)for(int x=0;x<code->width;++x)if(code->data[y*code->width+x]&1) {
        gray.at<uint8_t>(y+border,x+border)=0;
        svg<<"<rect x=\""<<x+border<<"\" y=\""<<y+border<<"\" width=\"1\" height=\"1\"/>";
    }
    svg<<"</g></svg>\n";QRcode_free(code);
    cv::resize(gray,gray,{},24,24,cv::INTER_NEAREST);
    if(!svg || !cv::imwrite(base.string()+".png",gray))throw std::runtime_error("Cannot write QR assets");
    std::cout<<base.string()<<".png\n"<<base.string()<<".svg\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
