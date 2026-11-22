#include "bhu/benchmark.hpp"
#include <iostream>
int main(int argc,char** argv)try {
    if(argc!=2){std::cerr<<"Usage: vio_report RUN_DIRECTORY\n";return 2;}
    bhu::bench::create_report(argv[1]);
    std::cout<<std::filesystem::path(argv[1])/"report.html"<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
