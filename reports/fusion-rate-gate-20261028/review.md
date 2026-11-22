# VINS-Fusion tracking rate and gates — 28 October 2026

**Only VINS-Fusion stereo + IMU was used for the estimator tests.** The official numerical core remains unmodified at commit be55a937a57436548ddfb1bd324bc1e9a9e828e0. ORB-SLAM3 and VINS-Mono were not run. The legacy orb.yaml name is the camera/IMU calibration format, not an estimator choice. Source/provenance files are retained here.

Mild enhancement (CLAHE clip limit 2, 8×8 tiles) is retained. Four OpenCV threads gave the best rate of the tested profiles. The live launcher now selects four threads when no explicit override is present; native benchmark arguments remain explicit. This improves throughput, but neither the present gate nor the shadow candidates provide continuous output through the weak transition. Actual publication thresholds and Cube settings remain unchanged.

## Paced movement input, after endpoint correction

The same full 3,597-pair two-minute outdoor/stairwell/indoor recording was released at its recorded timestamps. Each processor chose the newest available pair; all intervening IMU samples were retained with owning interpolated endpoints. Selected source indices, dropped frames, age and gate decisions are recorded per processed frame. Profiles ran sequentially under verified 15 W, with one/two/four OpenCV threads and unchanged Ceres solver settings, calibration and feature cap.

Images were loaded before timing to remove observed multi-second file-read stalls. Timed image access was below 1 ms in these three comparisons. The roughly 1.84 GB input cache and its RSS are **benchmark-only**, not live VIO memory usage. Peak resident memory must not be compared directly to the live adapter. The source/processed frame accounting is checked in run-checks.txt.

| OpenCV threads | Processed frames/s | Playback age median / p95 / max, ms | Existing frame-30 gate | Time-60 shadow | Time-40 shadow |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 14.2 | 86.8 / 115.2 / 163.1 | 31.4% | 53.2% | 71.4% |
| 2 | 16.6 | 76.1 / 108.1 / 179.7 | 40.1% | 58.6% | 74.9% |
| 4 | 17.9 | 70.9 / 106.1 / 203.0 | 40.5% | 60.3% | 78.6% |

Gate fractions use initialized processed samples, not source frames or actual Cube output. The frame-30 gate requires ≥60 persistent features, ≥20 backend tracks and ≥20 stereo matches for 30 consecutive good samples. Time-60 keeps the same feature minimum but requires 0.5 s recovery. Time-40 lowers only that shadow's persistent minimum to 40. All retain the shared finite, velocity, freshness/gap and reset guards; no candidate bypassed a logged hard rejection.

Playback age is **synthetic recorded-source age**, not camera-to-Cube latency. It excludes camera exposure/USB delay and live rectification. File-copy/preprocessing/tracking/estimation work is included, viewer encoding is disabled, and processing is rate-dependent. Different selected image sequences and the fixed wall-time solver budget can affect poses. No ground-truth accuracy or energy claim is made.

## Weak transition, approximately 65–85 seconds

| Threads | Frame-30 eligible / longest gap | Time-60 eligible / longest gap | Time-40 eligible / longest gap |
| --- | ---: | ---: | ---: |
| 1 | 0.0% / 19.88 s | 1.2% / 15.08 s | 18.2% / 6.95 s |
| 2 | 0.0% / 19.93 s | 0.3% / 16.14 s | 19.3% / 6.12 s |
| 4 | 0.0% / 19.91 s | 2.5% / 9.24 s | 25.7% / 4.46 s |

Time-based recovery reduces some outages, but does not create reliable features. Even the relaxed candidate leaves multi-second gaps. The minimum-40 shadow is **not directly deployable**: its support/quality mapping is unvalidated, and the Cube's minimum quality remains 60. It never publishes. A packet-count improvement is not evidence that newly accepted poses are accurate.

## Actual camera checks

Each profile also ran for 20 seconds on the live Gemini at 15 W, with mild enhancement, no requested movement, and UDP disabled. The physical scene/startup feature graph was not controlled identically across these short live runs, so they complement rather than replace the same-recording comparison.

| OpenCV threads | Processed frames/s | Estimated age median / p95 / max, ms | Frame-30 eligible / initialized | Time-60 eligible / initialized |
| --- | ---: | ---: | ---: | ---: |
| 1 | 11.5 | 103.5 / 128.2 / 132.0 | 191 / 221 | 214 / 221 |
| 2 | 14.6 | 83.9 / 108.1 / 118.7 | 253 / 283 | 275 / 283 |
| 4 | 19.4 | 66.9 / 94.3 / 132.1 | 316 / 380 | 355 / 380 |

Live age uses the existing lower-envelope device-to-host delivery mapping; exposure/USB delay is not independently measured. No pose was sent to the Cube. The fixed 150 ms freshness limit stayed active; any observed maximum above it is a rejected sample, not permission to send an older pose. tegrastats files retain resource observations but are not a complete battery/peripheral power measurement.

## Endpoint bug found during testing

An [IMU interpolation lifetime bug](../../docs/vins-replay-endpoint-issue.md) existed in the recorded-data adapter: saved lazy Eigen expressions referenced temporary vectors. A minimal reproduction reports AddressSanitizer stack-use-after-scope. The corrected helper returns owned arrays, validates brackets/finite values and is now shared with live capture. The live path had not used the unsafe saved expression.

The pre-fix exploratory replay showed severe stationary-tail divergence despite initialized state/high feature counts; it was excluded. Current corrected runs have short quiet-tail consistency as recorded in analysis.json. This is not proof of full trajectory accuracy, and changed paced frame selections are a confound in attributing all differences to one cause. Earlier numerical replay comparisons were marked provisional; image-only contrast measurements do not use IMU and remain unaffected.

AddressSanitizer's bad-pattern evidence and clean owning-value test, native endpoint/preintegration regressions and shadow-guard tests are retained. pre-endpoint-fix/ and disk-input/ are historical/audit data, not CPU-rate ranking inputs. live-before-helper-unification/ holds the earlier live checks, which did not use the unsafe saved expression.

## Decision and remaining work

- Keep mild enhancement.
- Use the best tested thread profile for further bench testing; verify it on actual moving live input before relying on its accuracy. More threads do not establish flight readiness.
- Keep the actual publication gate at 30 samples/60 features for now. Timed candidates remain diagnostic only. Validate recovery against pose error and deliberate occlusions before changing it.
- Investigate weak-transition observability, physical camera/IMU calibration and health metrics beyond raw feature count. Faster processing and shorter recovery alone did not remove all gaps.
- Keep Cube source selection/configuration unchanged. Origin/axis checks and EKF fusion are separate, still-unvalidated steps.

[Commands and gate semantics](../../docs/vins-fusion-rate-gates.md) document the explicit test flags. All source/clock/guard checks are in run-checks.txt; native IMU/stereo/shadow tests passed. All runtime work is C++, with Bash/Node orchestration and reporting only. No Python, flight-controller parameter writes, source-switch commands, arming or boot-service changes were introduced.
