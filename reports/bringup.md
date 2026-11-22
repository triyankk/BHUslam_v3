# Gemini 336L bring-up results

Measured on 2026-10-22 using SDK 2.10.6, camera serial CPC2963000JJ,
firmware 1.8.10. Firmware was not changed.

## Capture baseline

- Requested: 640×400 Y8 stereo, 30 FPS; ±4 g / ±1000 degrees/s IMU, 200 Hz.
- Stereo pairs: 895; observed rate 29.96434 FPS.
- IMU samples: 6058 per sensor; observed rate 202.59880 Hz.
- Median image interval: 33.373000 ms; maximum 33.374000 ms.
- Median IMU interval: 4.936000 ms; maximum 4.936000 ms.
- No duplicate/decreasing timestamps or estimated missing samples. No stereo frame-index gaps.
- Maximum left/right capture timestamp difference: 1 µs; no rejected pairs.
- IMU timestamps cover the full image sequence.
- Median acceleration magnitude: 9.75637 m/s².
- Median gyro magnitude: 0.03408 rad/s; this is not a calibrated zero-rate measurement.
- Median sampled left ORB features: 504.5.
- Median positive-disparity stereo matches: 139.0.
- Median sampled vertical matching error: 0 pixels. Scene-dependent diagnostic, not a full calibration residual.
- Laser control restored to its original value 1 after capture.

## Camera geometry

For the selected profile: fx=fy=308.16494751 px; cx=317.25 px; cy=201 px.
Factory rectified stereo baseline: 0.0947195968628 m.
Camera-to-IMU translation: [-0.007886, -0.001086, 0.014248] m, with identity rotation
reported by the SDK for this profile. These are SDK-provided extrinsics, not independently calibrated results.

## Limits

Timing and geometry checks pass for this capture. Full moving VIO accuracy,
camera–IMU time offset and IMU noise/random-walk calibration are not established
by these checks. Nominal IMU frequency remains 200 Hz; actual sample timestamps
are used for integration. No external timing reference was used to establish
absolute timestamp accuracy.

The first diagnostic capture, baseline-01, had an extra erroneous IMU scaling
factor. It is marked INVALID_UNITS.md and is rejected for inertial replay.
The public SDK API already provides m/s² and rad/s, as its bundled IMU example
documents; the adapter preserves these units. baseline-02 uses the corrected
implementation.

Detailed evidence: camera-inspection.txt, camera-after-capture.txt,
baseline-02-capture.log and baseline-02-quality.json in this directory.

## Interruption recovery

A separate capture was interrupted with SIGINT using `timeout`. It wrote 80 stereo
pairs, stopped cleanly, and the following SDK inspection confirmed laser control
was restored to 1. The timeout wrapper returned its expected status 124. The
interrupted recording also passes the timing and SI-unit checks.

## ORB-SLAM3 replay

The ARM64 build and IMU interpolation tests pass. A synthetic stereo smoke
sequence tracked 45/45 frames; this is software integration evidence only.
Real camera stereo replay tracked all 895/895 frames in both profiles:

| Profile | ORB features | Median tracking time | P95 tracking time |
|---|---:|---:|---:|
| Baseline | 1500 | 33.85 ms | 43.77 ms |
| ARM fast | 1000 | 29.36 ms | 33.71 ms |

The fast profile is the recommended starting point on this host. These are
tracking-call times on this recorded scene, excluding PNG decoding and camera
capture. They do not establish a hard real-time guarantee or moving VIO accuracy.

Three minimal upstream patches are preserved under patches/: a C++17 bundle-
adjustment counter type fix and initialization of right-camera settings for
rectified stereo, plus a guard for the IMU-status query before any keyframe
exists. Camera YAML floating-point fields retain decimal syntax even
when values are integral, as required by the upstream settings reader.

## Inertial replay

All 895 frames were processed in stereo-inertial mode using provisional noise
parameters and corrected SI-unit IMU data. The run briefly initialized: 89 frames
were tracked, and the IMU initialization flag was true on 15 frames. The mapper
then reported "Not enough motion for initializing. Reseting...", reset the
active map, and subsequently reported insufficient acceleration. It ended
without a usable VIO map. The adapter exited cleanly with status 1, retained
the tracking CSV, and skipped empty-map trajectory export. This is not a
successful sustained VIO run.

A moving 45-second sequence was subsequently recorded; see the results below.
IMU noise, camera–IMU time offset and independent extrinsic validation remain
provisional.

Fresh camera YAML export was checked to preserve real-valued nodes (including
cy=201.0). Final compilation, shared-library resolution, shell/Python syntax
checks and the IMU interpolation test pass. See [the run instructions](../README.md).

## Moving sequence: motion-01

The 44.85-second recording contains 1,345 stereo pairs and 9,100 samples per
IMU sensor. Observed rates were 29.96434 FPS and 202.59805 Hz. There were no
detected missing samples, frame-index gaps or nonmonotonic timestamps. Maximum
stereo skew was 1 microsecond, and IMU timestamps cover every image. Median
acceleration magnitude was 9.77643 m/s². All capture quality checks passed.

Stereo replay with the default 1,000-feature profile tracked **1,345/1,345**
frames. Tracking-call time was 32.51 ms median and 37.77 ms P95. All exported
poses were finite, with unit quaternions to within 1.1e-7. This is tracking
coverage, not an accuracy measurement against ground truth. Image decoding and
capture are excluded from these timings.

Stereo-inertial replay processed all frames, tracked 391, and reported IMU
initialization on 244 frames. It reset the active map twice with the mapper's
"Not enough motion for initializing" diagnostic. The final map was empty;
replay returned status 1 and correctly skipped trajectory export.

Motion was concentrated in approximately the first 13 seconds. From 15 seconds
onward, gyro standard deviation was mostly about 0.001 rad/s per axis. The
stereo trajectory's full position extent was approximately [0.118, 0.072,
0.173] m. ORB-SLAM3's initialization refinement accumulates time only when
successive keyframe translations sum to more than 5 cm; it can reset when that
sum falls below 2 cm before sufficient initialization motion has accumulated.
These checks remain enabled. The next validation should keep the camera moving
throughout capture, including smooth 30–50 cm translations in several directions
as the workspace permits, plus gentle rotations. This run does not establish
sustained VIO or identify calibration as correct or incorrect.

Evidence: motion-01-quality.json, motion-01-stereo-summary.json,
motion-01-inertial-summary.json and the corresponding replay logs.

## Continuous-motion sequence: motion-02

[Trajectory graph (PNG)](motion-02-trajectory.png) ·
[Vector graph (SVG)](motion-02-trajectory.svg). The graph uses stereo camera
poses, marks tracking recovery, and omits lines across lost-frame intervals.
Reproduce with `python3 scripts/plot_trajectory.py recordings/motion-02/slam_stereo --output reports/motion-02-trajectory`.

The second 45-second capture recorded 1,345 stereo pairs, with motion present
throughout. Capture quality checks passed: 29.96434 FPS stereo, approximately
202.59638 Hz IMU, no detected missing samples, maximum stereo skew 1 microsecond,
and full IMU coverage. The stereo trajectory spans approximately [0.663, 0.558,
1.106] m, so this sequence exercises substantially more translation than motion-01.

| Replay | Frames tracked | Median tracking time | P95 tracking time |
|---|---:|---:|---:|
| Stereo, 1,000 features | 1,253 / 1,345 | 39.24 ms | 52.85 ms |
| Stereo-inertial, 1,000 features | 1,252 / 1,345 | 34.50 ms | 45.20 ms |
| Stereo-inertial, 1,500 features | 1,276 / 1,345 | 45.83 ms | 57.35 ms |

Stereo replay relocalized four times. VIO reported IMU initialization on 352
frames but reset the active map 12 times: six mapper insufficient-motion resets
and six resets after local-map tracking failure. It was not initialized at the
end. Only 74 poses survived in the final inertial trajectory export. Its process
exit status was zero because the adapter checks whether initialization occurred
at any point; that status does not establish sustained VIO. The per-frame flags
and full reset log are the relevant evidence. Timing distributions include
startup/reset frames and cannot be treated as steady-state VIO performance.

An additional diagnostic compared adjacent tracked stereo rotations with gyro
measurements. Axis correlations at zero offset were [0.967, 0.993, 0.971]. A
coarse +/-100 ms offset search at 5 ms spacing selected zero offset after
removing a constant gyro bias from the residual. This supports the existing
axis and clock conventions as a starting point; the visual estimate is not an
independent reference, and this does not validate extrinsics, noise or timing
calibration. No calibration settings were changed based on this diagnostic.

The 1,500-feature comparison initialized on 558 frames and was initialized at
the end, but reset 14 times (three insufficient-motion resets and 11 local-map
tracking failures). Its final trajectory retained only 76 poses. The modest
tracking-coverage improvement does not establish a reliable VIO map and adds
substantial processing time. Keep the 1,000-feature profile as the host's initial
configuration; do not treat the 1,500-feature profile as a VIO fix.

The next useful work is to improve visual tracking under motion (slower angular
motion with continued translation, sufficient scene texture and illumination,
and verified exposure control), followed by measured IMU noise and camera–IMU
calibration. These results do not justify arbitrary axis flips or time-offset
changes. Live capture plus tracking at 30 FPS remains unproven on this host.

Evidence: motion-02-quality.json, motion-02-stereo-summary.json,
motion-02-inertial-summary.json, motion-02-rotation-diagnostic.json and the
corresponding replay logs. The higher-feature comparison is recorded in
motion-02-inertial-1500-summary.json.
