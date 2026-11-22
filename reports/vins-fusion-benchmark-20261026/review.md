> **Numerical replay caveat (28 October 2026):** the recorded-data IMU endpoint adapter had a confirmed lifetime bug. Pre-fix VINS numerical replay results below are provisional; image-only and live-camera observations are unaffected by this specific expression. See [issue and corrected runs](../../docs/vins-replay-endpoint-issue.md).

# VINS-Fusion versus VINS-Mono and ORB-SLAM3

VINS-Fusion stereo + IMU is a promising candidate for this Gemini recording: it agrees substantially more closely with ORB stereo-inertial on the shared route and continues producing visually supported poses after ORB loses its initialized track. It is slower than VINS-Mono at 15 W. This does not yet demonstrate accurate recovery after visual loss or readiness for flight.

[Interactive 3D paths and health timelines](comparison/comparison.html) · [Overview PNG](comparison/trajectory-overview.png) · [All metrics](comparison/comparison.json) · [Build and conventions](../../docs/vins-fusion-benchmark.md).

The user's previous route assessment favoured ORB stereo for path fidelity and VINS-Mono for continuity. That is useful qualitative context; no surveyed ground truth is available. The ORB reference named `two-minute-orb-stereo` here is the previous **stereo + IMU** benchmark, not the older pure-stereo replay.

## Results at 15 W

| Recorded run/configuration | Native initialized frames | Frames with diagnostic visual support | Pipeline p95 while initialized | Wall time | Peak RSS |
|---|---:|---:|---:|---:|---:|
| Two-minute Fusion, provisional noise bridge | 3587 / 3597 | 3490 / 3597 (97.0%) | 103.4 ms | 280.6 s | 104.0 MiB |
| Two-minute Fusion, upstream EuRoC noise | 3587 / 3597 | 3478 / 3597 (96.7%) | 102.9 ms | 270.6 s | 103.9 MiB |
| Two-minute VINS-Mono, provisional noise bridge | 3545 / 3597 | 3465 / 3597 (96.3%) | 78.7 ms | 216.2 s | 102.4 MiB |
| Two-minute ORB stereo + IMU | 2196 / 3597 | 2196 / 3597, native OK + IMU initialized | 63.8 ms | 159.0 s | 713.4 MiB |
| Eighty-second Fusion, provisional noise bridge | 2389 / 2399 | 2325 / 2399 (96.9%) | 92.4 ms | 164.7 s | 80.8 MiB |
| Eighty-second VINS-Mono, provisional noise bridge | 2350 / 2399 | 2199 / 2399 (91.7%) | 75.8 ms | 136.2 s | 80.9 MiB |

Native status semantics differ. Fusion's pinned upstream `Estimator::failureDetection()` immediately returns false, disabling its failure detector entirely. Its zero resets and near-continuous initialized flag are therefore **not evidence of reliable visual tracking**. Mono also bypasses its low-feature failure return, although other failure checks remain active. The visual-support heuristic requires at least 20 persistent frontend tracks and 20 backend tracks; it is not a pose-quality certificate. ORB's criterion is different. Do not rank estimator accuracy using these percentages.

Fusion initialized at 0.334 s on both recordings; Mono at 1.735 s and 1.635 s. Some additional available frames therefore come from earlier initialization. Both Fusion configurations remained initialized at the end of the two-minute recording, and the baseline also remained initialized at the end of the eighty-second recording.

## Path fidelity and the weak-vision sections

Over exactly the same 1648 matched camera poses from 5.006–59.971 s, after rigid alignment determined by camera orientations and mean positions, **with metric scale held fixed**:

| Configuration | Position disagreement RMS versus ORB stereo-inertial | Separate diagnostic scale ratio, ORB/run |
|---|---:|---:|
| Fusion, provisional noise bridge | **0.540 m** | 0.990 |
| Fusion, EuRoC noise | 1.209 m | 0.971 |
| VINS-Mono, provisional noise bridge | 2.116 m | 0.995 |

The scale diagnostic is not used to reduce the RMS. These are estimator-to-estimator agreement values, not absolute trajectory errors. Fusion's closer agreement supports keeping it as a candidate given the user's ORB route assessment, but cannot prove that it is more accurate everywhere.

Additional independently aligned windows are saved in `agreement_intervals` in the JSON. For 60–70 s, baseline Fusion/Mono RMS is 0.066/0.283 m. The requested 70–89 s comparison only has a sufficiently long common initialized segment through **75.990 s**: 0.033/0.095 m. Each window has its own alignment; these local shape comparisons do not measure accumulated global drift. They provide **no validated reference for the difficult post-76-second recovery**.

The baseline two-minute Fusion run has 97 initialized frames with weak visual support, including a longest contiguous interval of approximately **1.235 s**. EuRoC noise has 109 such frames and a longest interval of 1.302 s. Mono has 80 and 0.567 s. Stereo matching falls to zero in difficult images; the baseline median is 113 matches. Thus the two-minute run does not show an unqualified visual-continuity win for Fusion despite its native initialized flag. On the eighty-second recording, Fusion's weak-support interval is shorter (0.534 s versus Mono's 1.435 s), and visual support improves more clearly.

The known quiet interval from 91–115 s has endpoint movement of 2.24 mm for baseline Fusion, 12.15 mm for EuRoC-noise Fusion, and 1.99 mm for baseline Mono. Maximum excursions are 2.30, 13.22 and 2.38 mm respectively. These are stationary consistency measurements, **not millimetre positional accuracy**; an estimator can settle at an incorrect position. The change between noise policies illustrates sensitivity, not a measured calibration solution.

## Runtime and practical choice

The baseline Fusion run processes the two-minute recording at an average **12.8 recorded frames/s**, compared with Mono's 16.6. Fusion's median frontend/backend work is 31.0/45.2 ms versus Mono's 18.9/35.2 ms. Neither current synchronous all-frame configuration meets the 33.4 ms budget for 30 Hz live operation. These figures include offline image decoding and are not measured camera-to-Cube latency.

Fusion uses stereo optical flow with backward consistency checking; Mono uses its original CLAHE frontend. The comparison retains these native frontend differences, so it does not isolate the effect of adding stereo depth. Both builds use -O1 and the same local Ceres build, fixed provisional factory extrinsics and time offset, no bias subtraction, no online calibration and **no loop closure**. Optional ROS transport, pose graph and vocabulary are excluded from VINS memory; ORB carries its map/vocabulary. Fifteen-watt mode is verified at the start and end, not a measurement of actual energy consumption. Timing is a single pass per setting with historical Mono/ORB results reused, not a statistical runtime study.

Keep Fusion as the leading additional stereo-inertial candidate for the next calibration/performance stage, alongside the ORB path reference. Before flight selection, measure Gemini timing/extrinsics/noise, benchmark a live scheduling configuration that can bound pose age at 15 W, and add explicit health gating that rejects weak vision, stale poses and implausible estimates independently of Fusion's disabled native failure detector. Validate recovery on a measured route, especially through the blank-wall section. Loop closure should then be evaluated separately, with any global pose corrections kept distinct from the continuous odometry supplied to navigation.

## Reproduction and checks

Official VINS-Fusion revision: `be55a937a57436548ddfb1bd324bc1e9a9e828e0`; upstream checkout remains unmodified. `full/` contains three complete native C++ replays, configurations, logs, source commit, boot ID, executable checksum and 15 W readbacks. Previous Mono/ORB runs are referenced through relative symlinks in `comparison/`; their provenance is preserved. The HTML embeds its plotted data for standalone viewing.

The native IMU integration and stereo-projection sign/unit checks passed. All three outputs passed finite/monotonic timestamp, quaternion, pose/status association, report-count and stereo-support checks. The interactive viewer passed the fixture and all nine real run selections, finite drawing, reset/empty-data handling and no-network checks. The replay executable checksum still matches the manifest after completion.
