# Flight 213: exploratory FAST-LIVO2 replay

20 November 2026. **No localization improvement established. Both initial
LIVO configurations diverged. The recorded VINS baseline also diverged, so
smaller disagreement or less displacement cannot be called better accuracy.**

All work was offline. No Cube parameters, services, navigation outputs or
depth-avoidance behavior changed.

## Data recovered and initial geometry

- The pinned Hesai SDK decoded 287,758 CRC-valid point packets into **481
  complete scans**, with two boundary scans discarded. Scan spans are
  199.76–200.46 ms. All binary sizes, finite coordinates/timestamps and ring
  values passed checks.
- 4,597,040 point records were saved; 1,299,525 (28.27%) are nonzero returns.
  Zero returns were excluded from estimation. The output retains SDK axes,
  rings and per-point sensor times; scan files themselves are not deskewed.
- The operator confirmed the flight-213 mount: JT16 centre 3.5 cm behind,
  4 cm right, 13 cm above the left lens; rotation axis leaning 20 degrees
  forward from aircraft upright; connector facing rear. The saved camera
  mount is pitched down 22.8 degrees.
- The vendor drawing puts +Y toward the connector and +Z along the upright
  rotation axis. The initial body-to-sensor conversion follows that drawing.
  Physical mounting measurements are still an initial estimate, not a
  calibrated optical/LiDAR-origin transform. See `initial-transform-and-clock.json`.
- Timing uses one saved cloud-end/host timestamp anchor plus the recording's
  camera-device/host mapping, with clock-rate ratio assumed 1. Clock drift and
  capture delay are unvalidated. No timestamp was silently relabeled as UTC.
- A 2,059-image overlap was selected. The LiDAR segment begins around 49 s
  into the camera recording, so this is **mid-flight initialization**, not the
  stationary initialization expected by the estimator.

## What ran

An isolated native C++ adapter was built around FAST-LIVO2's VIO, IMU and
voxel-map numerical modules. It feeds one rectified Gemini image stream,
interpolated Gemini accelerometer/gyro samples and timestamped JT16 points.
Points are cut at image times, deskewed by the upstream IMU module, then used
for sequential geometric and visual updates. Camera processing continues
through LiDAR slices without returns.

No ROS1 runtime was installed. Minimal local message/time types replace ROS
transport; attempting publication throws. Numerical modules are otherwise
upstream, apart from a build-copy fix that initializes propagation timestamps
after IMU initialization. Old Sophus/Eigen compatibility is handled in build
copies. Builds use O1 and bounded thread counts; this is not the upstream
production performance configuration. Source revisions are pinned in the
initial-transform manifest.

The first short smoke run exposed camera processing being incorrectly skipped
on empty LiDAR slices. This was corrected before both trials below. Smoke
results are not included as localization evidence.

## Results

| Configuration | Matched pose interval before stop | LIVO endpoint displacement | Recorded VINS displacement on same timestamps | Result |
| --- | ---: | ---: | ---: | --- |
| 0.5 m map voxels | 51.8 s | 995 m | 11,648 m | Diverged; diagnostic speed guard triggered |
| 2.0 m map voxels | 39.0 s | 1,412 m | 7,103 m | Diverged; diagnostic speed guard triggered |

These are estimator outputs, **not measured aircraft travel or ground-truth
errors**. Both sets of outputs are physically implausible for this flight.
The larger map setting was one explicit sensitivity check for sparse JT16
geometry, not a search for a setting that makes a preferred result.

The 0.5 m trial processed 1,147 images before stopping; 444 frames had LiDAR
plane constraints and 367 had visual candidates. The 2 m trial processed 847;
434 had LiDAR constraints and 123 had visual candidates. Support later fell
away in both runs, leaving inertial propagation to drift. A pose written by
the adapter is not proof of a measurement-supported localization update.
Visual candidate counts are not a validated tracking-health metric.

![Exploratory displacement and measurement support](comparison.png)

The comparison uses exact matching camera timestamps and rigid alignment
without fitted scale. VINS is itself an estimator, not an independent
reference. Its recorded native pose stream contains runaway positions despite
the visual-support flag remaining true. No accuracy ranking is defensible.

## Important input limitations

The recorded replay window contains 19,500 accelerometer samples. **1,455
(7.46%) have at least one component within 98% of the configured ±4 g limit.**
The largest saved component is approximately 39.2 m/s². The initial ~0.45 s
window contains 9 near-limit samples out of 91. Its accelerometer Y standard
deviation is about 20.2 m/s²; this is not a stationary initialization record.
No gyro sample reached the configured ±1000 degrees/s near-limit threshold.
See `imu-input-audit.json` for thresholds and statistics.

Clipping/large inertial disturbance, lack of stationary initialization,
provisional clock/extrinsic calibration and sparse vegetation geometry all
confound the test. These results do not isolate an algorithm limitation or
prove that LIVO cannot improve a properly recorded/calibrated system.

## Adapter checks

- Raw decoder: complete scans accepted; bit-corrupted scan rejected;
  incomplete tail excluded; existing output directories refused.
- Full recovered data: record sizes, finite values, rings and timestamp
  bounds passed across all 481 scans.
- Offline compatibility layer: time/quaternion checks and publication-refusal
  test passed.
- A known stationary synthetic scene checks pose stability and continued
  camera pose output during missing LiDAR returns. Its detailed result is
  recorded separately in `stationary-adapter-test.json`; this is a software
  regression, not validation of real calibration or dynamic-flight accuracy.

## Next experiment

Do not switch flight localization based on these trials. First resolve the
recorded accelerometer saturation/large disturbance, checking mounting,
vibration and sensor configuration rather than merely increasing a range.
Record a continuous synchronized camera/LiDAR/IMU dataset with a stationary
start and finish, timestamp anchors throughout, and a measured carry route.
Refine the optical/LiDAR extrinsics and clock alignment, then repeat the
comparison using an independent reference. Keep Gemini-depth avoidance as
its own pipeline.

## Reproduce

```bash
cd /home/atas/BHUslamV3
bash scripts/build-jt16-raw-decoder.sh
bash scripts/build-livo-offline.sh
python3 tools/livo/prepare_flight213.py
# Use NEW output directories; tools refuse to overwrite existing results.
bin/livo-replay reports/livo-replay-readiness-20261120/flight213-initial.yaml NEW_OUTPUT
python3 tests/livo_stationary_replay_test.py
third_party/sysroot/usr/bin/ctest --test-dir build/livo-offline --output-on-failure
python3 tools/livo/compare_recorded.py
```

Detailed results: `comparison.json`, each trial's `trajectory.csv`,
`diagnostics.csv`, saved configuration and `.stderr` error log. The initial
0.5 m and 2 m trials are retained, including failed output; they are not
marked successful because an executable produced poses.

Sources: [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) and the
[Hesai JT16 coordinate drawing, section 1.2](https://www.hesaitech.com/wp-content/uploads/2025/01/JT16_User_Manual_J03-en-250110.pdf).
