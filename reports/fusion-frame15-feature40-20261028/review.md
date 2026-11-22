# 15-frame / 40-feature gate comparison

VINS-Fusion stereo + IMU only; corrected owning IMU interpolation. One paced replay of the existing 45-second movement recording at 15 W, four OpenCV threads, mild CLAHE (clip 2). All gates see identical estimator samples. No navigation transmission; the active publication gate and Cube settings remain unchanged.

Processed 691 frames (15.32 fps); 681 initialized. Synthetic playback age p95 104.4 ms, maximum 133.9 ms. Preloaded image benchmark excludes live camera USB and rectification costs.

| Gate | Eligible / initialized | First eligible | Longest gap after initialization* | Closures |
|---|---:|---:|---:|---:|
| 30 frames / 60 features (current) | 98 / 681 (14.4%) | 2.67 s | 12.21 s | 7 |
| 0.5 seconds / 60 features | 314 / 681 (46.1%) | 0.90 s | 3.77 s | 16 |
| 0.5 seconds / 40 features | 556 / 681 (81.6%) | 0.90 s | 1.60 s | 8 |
| 15 frames / 40 features (candidate) | 513 / 681 (75.3%) | 1.50 s | 2.94 s | 6 |

The 15-frame / 40-feature candidate substantially improves availability over the current gate on this movement recording, but still has a 2.94-second gap. The 0.5-second / 40-feature shadow opens sooner and has higher availability here. These results do not establish safe navigation through the remaining gaps.

*Gap is the greatest spacing between eligible samples, including initialization-to-first-eligible and last-eligible-to-recording-end boundaries. It includes normal frame spacing. First eligible is relative to the first processed recording frame. Closures count transitions from eligible to rejected.

The candidate still requires 20 backend tracks and 20 stereo matches, valid initialized/aligned finite poses and velocity, monotonic timestamps, and age <=150 ms. Any failed sample restarts the 15-frame count. Capture gaps above 150 ms and epoch changes also restart it.

Automated checks verify counts against CSV, every accepted candidate has 15 consecutive valid samples, unchanged hard guards, strictly increasing timestamps/source indices, and containment of the strict gate. Native gate tests cover exact sample-15 opening, failed-sample recovery, stale rejection and resets.

Higher eligibility measures availability, not pose accuracy. There is no ground truth. Calibration remains provisional and upstream failure detection is disabled. Cube quality minimum remains 60, so a 40-feature sample may be rejected by the downstream bridge/Cube; this candidate has not been deployed.
