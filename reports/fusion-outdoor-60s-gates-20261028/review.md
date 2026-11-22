# First-minute outdoor gate comparison

VINS-Fusion stereo + IMU only, with corrected owning IMU endpoints. Replayed the first 1,798 stereo pairs (59.971 seconds) from the two-minute outdoor recording, at 15 W, four OpenCV threads and mild CLAHE (clip 2). All four gates evaluated the same estimator output.

Processed 846 frames (14.09 fps); 836 initialized. Synthetic playback age p95 109.1 ms; maximum 144.6 ms. Preloaded images exclude camera USB, live rectification and storage stalls.

| Gate | Eligible / initialized | First eligible | Longest output gap* | Closures |
|---|---:|---:|---:|---:|
| 30 frames / 60 features (current) | 79 / 836 (9.4%) | 2.37 s | 20.69 s | 8 |
| 0.5 seconds / 60 features | 459 / 836 (54.9%) | 0.90 s | 3.10 s | 31 |
| 0.5 seconds / 40 features (candidate) | 668 / 836 (79.9%) | 0.90 s | 1.44 s | 15 |
| 15 frames / 40 features | 581 / 836 (69.5%) | 1.30 s | 1.87 s | 13 |

*Maximum spacing between eligible samples, including initialization-to-first-eligible and last-eligible-to-end boundaries. Includes normal frame spacing. Closures are eligible-to-rejected transitions. Percentages use initialized processed samples, not elapsed-time coverage.

![Gate availability timeline](gate-timeline.svg)

Comparison with the earlier desk movement recording:

| Recording | Current 30 / 60 | 15 frames / 40 | 0.5 s / 40 |
|---|---:|---:|---:|
| 45-second desk movement | 14.4% / 12.21 s | 75.3% / 2.94 s | 81.6% / 1.60 s |
| First minute outdoors | 9.4% / 20.69 s | 69.5% / 1.87 s | 79.9% / 1.44 s |

Each cell gives eligibility / longest gap. Different recordings are different scene and motion workloads; the comparison is descriptive.

The candidate retains initialized/aligned finite pose and velocity checks, 150 ms age and capture-gap limits, 20 backend tracks and 20 stereo matches. Any bad sample, reset or excessive capture gap restarts recovery. Automated checks reconstruct every gate decision from CSV and verify feature/hard-guard thresholds and summary counts.

This first-minute selection is primarily outdoor and excludes the later stairs/indoor transition around 65–85 seconds. Availability does not establish pose accuracy: no ground truth is available and calibration remains provisional. All gates are diagnostic in this replay; no navigation data was transmitted, no active gate or Cube parameter was changed. The downstream quality minimum remains 60, so the 40-feature candidate is not yet an end-to-end accepted navigation configuration.
