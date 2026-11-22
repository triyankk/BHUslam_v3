#pragma once
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace bhu {
// Fixed memory diagnostics for continuous operation. Values are not used by
// the estimator or publication gate.
class RollingSamples {
    std::vector<double> values_;
    std::size_t limit_, next_ = 0;
public:
    explicit RollingSamples(std::size_t limit = 8192) : limit_(limit) {
        if (!limit) throw std::invalid_argument("Empty rolling sample window");
        values_.reserve(limit);
    }
    void push_back(double value) {
        if (values_.size() < limit_) values_.push_back(value);
        else { values_[next_] = value; next_ = (next_ + 1) % limit_; }
    }
    std::size_t size() const { return values_.size(); }
    double quantile(double q) const {
        if (!(q >= 0 && q <= 1)) throw std::invalid_argument("Invalid quantile");
        if (values_.empty()) return 0;
        auto sorted = values_;
        std::sort(sorted.begin(), sorted.end());
        return sorted[std::size_t(q * (sorted.size() - 1))];
    }
};
}
