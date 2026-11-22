#pragma once
#include <opencv2/core.hpp>
#include <cmath>

namespace bhu::fusion {
// Enabled explicitly by the rectified Gemini adapters, never for arbitrary
// unrectified upstream camera models. Keep the 0.5 px round-trip check intact.
inline bool rectified_stereo=false;
inline bool stereo_geometry(const cv::Point2f& left,const cv::Point2f& right) {
    return !rectified_stereo || (std::isfinite(right.x) && std::isfinite(right.y) &&
        left.x>right.x && std::abs(left.y-right.y)<=1.0f);
}
}
