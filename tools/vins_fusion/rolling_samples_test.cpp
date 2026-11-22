#include "bhu/rolling_samples.hpp"
#include <iostream>
#include <stdexcept>
static void check(bool value) { if (!value) throw std::runtime_error("Rolling sample check failed"); }
int main() {
    bhu::RollingSamples samples(4);
    check(samples.quantile(.5) == 0);
    for (int i = 1; i <= 100000; ++i) samples.push_back(i);
    check(samples.size() == 4);
    check(samples.quantile(0) == 99997 && samples.quantile(1) == 100000);
    check(samples.quantile(.5) == 99998);
    samples.push_back(-5);
    check(samples.quantile(0) == -5 && samples.quantile(1) == 100000);
    std::cout << "PASS: fixed memory, rolling eviction, quantiles\n";
}
