// Offline DataFlash reception audit. Never opens a hardware endpoint.
#include "native.hpp"
#include <map>
#include <iostream>
struct Format {size_t length;std::string name,types,columns;};
static size_t field_size(char c){
    switch(c){case 'b':case 'B':case 'M':return 1;case 'h':case 'H':case 'c':case 'C':return 2;
    case 'i':case 'I':case 'e':case 'E':case 'L':case 'f':case 'n':return 4;
    case 'q':case 'Q':case 'd':return 8;case 'N':return 16;case 'Z':case 'a':return 64;default:throw std::runtime_error("Unknown DataFlash field type");}
}
template<class T>static T field(const uint8_t*p){T v;std::memcpy(&v,p,sizeof(v));return v;}
static std::string fixed(const uint8_t*p,size_t n){return std::string(reinterpret_cast<const char*>(p),strnlen(reinterpret_cast<const char*>(p),n));}
int main(int argc,char**argv)try{
    using namespace tof;check(argc==3,"cube_vio_log_review INPUT.BIN NEW_OUTPUT_DIRECTORY");
    fs::path out=argv[2];check(!fs::exists(out),"Output already exists");std::ifstream in(argv[1],std::ios::binary);check(bool(in),"Open Cube BIN");fs::create_directories(out);
    std::ofstream rows(out/"cube-extnav.jsonl");std::map<unsigned,Format> formats;
    uint64_t records=0,visp=0,visv=0,ignoredP=0,ignoredV=0,lowQuality=0,xkfs=0,desync=0;bool truncated=false;std::map<int,uint64_t> sources;
    std::map<std::string,uint64_t> first,last;Json latestParams=Json::object();
    while(true){int a=in.get();if(a==EOF)break;if(a!=0xa3){++desync;continue;}int b=in.get();if(b!=0x95){++desync;if(b!=EOF)in.unget();continue;}int type=in.get();if(type==EOF){truncated=true;break;}
        size_t length=type==128?89:(formats.count(type)?formats.at(type).length:0);
        if(length<3||length>255){++desync;continue;}std::vector<uint8_t> data(length-3);in.read(reinterpret_cast<char*>(data.data()),data.size());if(size_t(in.gcount())!=data.size()){truncated=true;break;}++records;
        if(type==128){check(data.size()==86,"FMT length");formats[data[0]]={data[1],fixed(data.data()+2,4),fixed(data.data()+6,16),fixed(data.data()+22,64)};continue;}
        const auto&fmt=formats.at(type);if(fmt.name!="VISP"&&fmt.name!="VISV"&&fmt.name!="XKFS"&&fmt.name!="PARM")continue;
        Json row={{"message",fmt.name}};std::stringstream columns(fmt.columns);size_t offset=0;
        for(char c:fmt.types){std::string name;std::getline(columns,name,',');auto n=field_size(c);check(offset+n<=data.size(),"DataFlash layout exceeds packet");auto p=data.data()+offset;
            switch(c){case 'b':row[name]=field<int8_t>(p);break;case 'B':case 'M':row[name]=field<uint8_t>(p);break;
            case 'h':case 'c':row[name]=field<int16_t>(p);break;case 'H':case 'C':row[name]=field<uint16_t>(p);break;
            case 'i':case 'e':case 'L':row[name]=field<int32_t>(p);break;case 'I':case 'E':row[name]=field<uint32_t>(p);break;
            case 'q':row[name]=field<int64_t>(p);break;case 'Q':row[name]=field<uint64_t>(p);break;
            case 'f':row[name]=field<float>(p);break;case 'd':row[name]=field<double>(p);break;
            case 'n':case 'N':case 'Z':row[name]=fixed(p,n);break;default:break;}offset+=n;
        }
        if(fmt.name=="PARM"){if(row.contains("Name")&&row.contains("Value")){auto name=row["Name"].get<std::string>();if(name.rfind("EK3_SRC",0)==0||name=="VISO_TYPE")latestParams[name]=row["Value"];}continue;}
        rows<<row.dump()<<'\n';
        if(row.contains("TimeUS")){auto t=row["TimeUS"].get<uint64_t>();if(!first.count(fmt.name))first[fmt.name]=t;last[fmt.name]=t;}
        if(fmt.name=="VISP"){++visp;ignoredP+=row.value("Ign",1)!=0;lowQuality+=row.value("Q",0)<40;}
        if(fmt.name=="VISV"){++visv;ignoredV+=row.value("Ign",1)!=0;}
        if(fmt.name=="XKFS"){++xkfs;if(row.contains("SS"))++sources[row["SS"].get<int>()];}
    }
    check(records&&formats.size(),"No decodable DataFlash formats");Json ss=Json::object();for(auto [source,count]:sources)ss[std::to_string(source)]=count;
    Json summary={{"input",fs::absolute(argv[1]).string()},{"records",records},{"desynchronized_bytes",desync},{"truncated_tail",truncated},
        {"VISP",visp},{"VISV",visv},{"VISP_ignored",ignoredP},{"VISV_ignored",ignoredV},{"VISP_quality_below_40",lowQuality},
        {"cube_reception_proven",visp>0},{"cube_velocity_reception_proven",visv>0},{"XKFS",xkfs},{"source_set_counts_zero_based",ss},
        {"primary_source_observed_only",!sources.empty()&&sources.size()==1&&sources.count(0)>0},{"latest_parameters",latestParams},
        {"fusion_validated",false},{"note","VISP/VISV prove Cube frontend reception. Ign=0 means frontend accepted; it does not prove EKF fusion. XKFS.SS=0 denotes primary source. Timestamps RTimeUS match Jetson capture_host_us."}};
    for(auto [name,t]:first)summary["spans"][name]={{"first_time_us",t},{"last_time_us",last[name]}};
    save_json(out/"summary.json",summary);std::cout<<summary.dump(2)<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
