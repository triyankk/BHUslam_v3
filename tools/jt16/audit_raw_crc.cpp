// Offline capture audit using the pinned SDK's CRC implementation. No device I/O.
#include "inner_com.h"
#include <fstream>
#include <iterator>
#include <cstdint>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    std::ifstream input(argv[1], std::ios::binary);
    if (!input) return 2;
    const std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(input), {}};
    size_t offset=0, valid=0, invalid=0, skipped=0;
    while (offset+16 <= bytes.size()) {
        const auto* p=bytes.data()+offset;
        if (p[0]!=0xee || p[1]!=0xff || p[2]!=1 || p[3]!=8 || p[5]>1) {
            ++offset; ++skipped; continue;
        }
        const size_t length=p[5]==0 ? 80 : 34;
        if (offset+length > bytes.size()) break;
        const auto* tail=p+length-4;
        const uint32_t recorded=uint32_t(tail[0]) | (uint32_t(tail[1])<<8)
            | (uint32_t(tail[2])<<16) | (uint32_t(tail[3])<<24);
        const auto crc=hesai::lidar::Crc32::CRCCalc(p, length-4, (4-length%4)%4);
        if (crc==recorded) {++valid; offset+=length;}
        else {++invalid; ++offset;}
    }
    std::cout << "{\"valid_packets\":" << valid << ",\"invalid_candidates\":" << invalid
              << ",\"skipped_bytes\":" << skipped << ",\"trailing_bytes\":" << bytes.size()-offset
              << "}\n";
    return 0;
}
