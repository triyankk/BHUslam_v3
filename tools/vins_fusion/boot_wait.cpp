#include <json.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

static std::string read(const std::string& path) {
    std::ifstream file(path);
    std::ostringstream out;
    out << file.rdbuf();
    return out.str();
}
static bool cube_ready(const std::string& text) {
    try {
        auto state = Json::parse(text);
        const auto& cube = state.at("cube");
        double age = cube.at("heartbeat_age_ms").get<double>();
        return state.at("running").get<bool>() &&
            cube.at("connected").get<bool>() && !cube.at("armed").get<bool>() &&
            std::isfinite(age) && age >= 0 && age <= 1500;
    } catch (...) { return false; }
}
int main(int argc, char** argv) try {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "fusion_boot_wait STATUS.json TIMEOUT_SECONDS\n"
                     "Require fresh disarmed Cube heartbeat from existing serial owner; never opens serial.\n";
        return 0;
    }
    if (argc != 3) throw std::runtime_error("Expected STATUS.json TIMEOUT_SECONDS");
    std::size_t consumed = 0;
    const double seconds = std::stod(argv[2], &consumed);
    if (consumed != std::string(argv[2]).size() || !std::isfinite(seconds) || seconds < 1 || seconds > 600)
        throw std::runtime_error("Timeout must be 1..600 seconds");
    auto previous = read(argv[1]);
    auto start = Clock::now(), changed = start;
    bool observed_update = false;
    unsigned stable = 0;
    while (std::chrono::duration<double>(Clock::now() - start).count() < seconds) {
        auto text = read(argv[1]);
        auto now = Clock::now();
        if (text != previous) { previous = text; changed = now; observed_update = true; }
        // An old ready=true file from a previous run/boot cannot release startup.
        // The service must update it after this guard begins, and keep it fresh.
        if (observed_update && now - changed < std::chrono::seconds(1) && cube_ready(text)) {
            if (++stable >= 3) {
                std::cout << "CUBE_STARTUP_READY: fresh heartbeat, disarmed; EKF fusion not asserted\n";
                return 0;
            }
        } else stable = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cerr << "Waiting for updated Cube status, connected link and disarmed heartbeat <=1500ms\n";
    return 1;
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
