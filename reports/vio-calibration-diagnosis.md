# Gemini VIO failure and calibration diagnosis — 2026-10-25

## Decision

Do not promote a new camera/IMU configuration or enable external navigation from
these results. The main moving tracking loss has a clear visual cause. A small
candidate timing correction improves rotational consistency, but does not
materially improve tracking against a repeated baseline. Calibration remains
provisional. No camera settings, original recordings, Cube parameters, navigation
source or boot service were changed.

All analysis and replay are offline. The new reusable tool is native C++:
`src/vio_diagnose.cpp`, executable `build/main/bin/gemini_vio_diagnose` and launcher
`bin/bhu vio-diagnose`. No Python was introduced.

## Evidence and failure timeline

Main dataset: `recordings/diagnostic/20261025T095519.847430960Z`.
Cross-check dataset: `recordings/diagnostic/20261025T091926.822317376Z`.
Main recording integrity was already checked: 3,597 stereo pairs, no frame-index
gaps, maximum stereo skew 1 microsecond, continuous accelerometer and gyro data,
15W throughout capture. Offline replay does not prove live processing latency.

Times below are elapsed from the first recorded image.

| Time | Observation | Interpretation |
| --- | --- | --- |
| 0.234 s | VIO loses its initial short track and restarts; stereo continues. | Initial inertial startup is not yet reliable. This short failure alone does not identify a calibration error. |
| 74–76 s | Image progressively faces a mostly blank stairwell wall. The diagnostic ORB detector falls from roughly 15 corners to zero. Stereo loses at 75.957 s and VIO at 76.024 s. Angular rate around loss is roughly 1.7–2.0 rad/s. | Strong evidence of visual feature starvation, with rapid rotation adding difficulty. A larger feature budget cannot create features on a blank surface. |
| 83–84 s | Camera turns close to a door/window frame. Useful stereo matches collapse; stereo is already lost, VIO loses again near 83.97 s. | Close surfaces, poor feature distribution and motion contribute. This is not evidence of missing IMU samples. |
| 88.405 s, original replay | Good-looking images and roughly 1,000 detected corners remain, but VIO resets after camera placement. Log reports insufficient motion during inertial initialization. | The camera becomes stationary before the new map completes inertial refinement. It cannot restart while the acceleration-excitation check is unmet. |
| 91–115 s | Stereo estimates only 1.85 mm endpoint displacement and 0.054 degrees endpoint rotation; raw gyro still reads about 0.034 rad/s. | Strong support for a stationary tail and a nonzero gyro bias. Stereo is not ground truth. |

The final approximately 32 seconds without VIO in the original replay were
therefore predominantly stationary. They should not be described as 32 seconds
of lost tracking during continued walking. The moving blank-wall loss remains a
serious GNSS-denied navigation limitation.

Images: `vio-diagnosis-final/tracking-loss-contact.png`.
Metrics: `vio-diagnosis-final/image-quality.csv`.
The independent image detector uses OpenCV ORB with its default FAST threshold
20, 1,000 features and a descriptor ratio test. It is a diagnostic proxy, not
ORB-SLAM3's actual inlier count or its per-cell fallback FAST threshold 7.
Zero reported matches is not a zero epipolar error; the match count must be read
alongside the vertical-error column.

## Calibration checks

### Units, transform direction and axes

The adapter's camera-to-gyro transform is constructed as factory left-to-gyro
rotation times inverse left rectification rotation; translations are converted
from SDK millimetres to metres. Acceleration is rotated into gyro axes. Both
recorded SDK streams use SI units, and replay adds the configured time offset
to both streams before synchronization. No extra degrees-to-radians or gravity
scale conversion is warranted by these data.

Independent stereo rotation increments over approximately 0.1 s agree with gyro:

- Fitted visual/gyro scale: 1.0018 on the first-half timing fit, 0.9999 on the
  second half. This strongly rejects a gross radians/degrees scale error.
- Fitted gyro-to-camera rotation is about 0.53 degrees from the factory rotation
  on the main run; about 0.77 degrees on the earlier recording. These are
  consistency estimates, not measured extrinsic calibration accuracy.
- The main run excites all three rotational axes. Covariance singular values
  are approximately 191.0, 59.3 and 31.4; earlier recording is less balanced.

The acceleration check fits quadratic stereo positions over approximately
0.47 s and compares their second derivative with accelerometer readings rotated
into the stereo world, using an approximate matching smoothing kernel. It uses
only the first uninterrupted stereo segment:

- Main run: gravity norm at unit scale 9.786 m/s²; fitted scale 0.988.
- Existing polarity: residual RMS 0.286 m/s²; reversed polarity: 3.068 m/s².
- Earlier run: gravity norm 9.771 m/s²; fitted scale 0.957; existing polarity
  agrees better there too, although the first uninterrupted segment is shorter.

These checks support the existing axis direction and SI scaling. They do not
measure the centimetre-scale camera/IMU translation, complete accelerometer
bias/scale calibration, or flight vibration response. Visual second derivatives
are noisy and lever-arm acceleration is not corrected in this diagnostic.

### Timestamp offset

Convention: camera-clock time = IMU timestamp + offset.

A -100 to +100 ms search at 1 ms increments, fitted to the first temporal half
of the main run, chooses **-7 ms**. Using the training bias on held-out samples:
rotation error falls from approximately 0.030 to 0.020 rad/s. First-half error
falls from 0.0506 to 0.0160 rad/s. Thus sensor timestamps need not be physically
aligned just because both streams are continuous and use the same device clock.

However, the earlier recording chooses about **-3 ms**, with only a small
held-out improvement (0.0596 to 0.0585 rad/s). Finite-interval angular
approximation, stereo errors, exposure timing and scene/motion differences can
influence this estimate. A joint target-based temporal/spatial calibration is
still required before declaring a physical time offset.

### Bias and noise

The main run's quiet tail gives approximate gyro means
`[-0.0313, -0.0116, +0.0086] rad/s`, norm approximately 0.0344 rad/s, or
1.97 degrees/s. This agrees closely with bias inferred independently from
stereo rotation. It is a real quantity to characterize, not a reason to insert
a fixed, universal bias subtraction: ORB-SLAM3 estimates bias and temperature
and startup state can change it.

Quiet two-second windows have roughly 0.0009–0.0010 rad/s gyro vector RMS and
0.011–0.013 m/s² acceleration vector RMS after demeaning. These are filtered,
short-window statistics, **not** continuous noise-density or random-walk values.
The SDK header also describes its `noiseDensity` field as in-run bias
instability, so copying its scalar blindly into ORB-SLAM3 is not justified.

`IMU.NoiseGyro`, `NoiseAcc`, `GyroWalk`, `AccWalk`, and time offset remain
provisional. `Calibration.imuValidated` remains 0.

## Controlled configuration experiment

Same original dataset, same executable, full 3,597 frames. The candidate changes
only `IMU.TimeOffsetSeconds` to -0.007 in a separate report-local YAML.

| Replay | Tracked | Percentage | Last tracked elapsed | Active reset requests | Final map |
| --- | --- | --- | --- | --- | --- |
| Original baseline | 2,416 | 67.17% | 88.372 s | 3 | Empty |
| Repeated unchanged baseline | 2,489 | 69.20% | 90.941 s | 4 | Empty |
| Candidate -7 ms | 2,497 | 69.42% | 90.808 s | 4 | Empty |

The candidate gains just eight frames over the repeated baseline. Both still
lose at the blank wall and end without a usable map. It is not a demonstrated
tracking improvement. Multi-threaded mapping/replay scheduling produces variation;
these runs are not a statistical performance benchmark. Some diagnostics/build
work ran concurrently during portions of replay, so wall-time latency must not
be compared as a controlled power/performance test.

Both new replays finish processing all frames but return exit 1 because there
is no surviving map to export, matching the original outcome. This is not a
recording-integrity error.

Artifacts: `vio-calibration-experiments/comparison.json`, baseline-repeat and
offset-minus-7ms tracking CSVs/logs, candidate YAML, static-visual-check JSON and
checksums. The original recording/YAML were not overwritten.

## Next measurements and changes, in order

1. **Characterize startup gyro bias:** use the existing two-minute boot recorder
   with the complete camera/Jetson assembly firmly stationary on a table. Keep
   hands off throughout and record the cold/warm condition. Repeat after warmup.
   This can characterize short-term bias; it is not a long-duration Allan test.
   **Update:** the latest boot already supplied this measurement; see
   [stationary run review](latest-stationary-run-review.md). It confirms a
   1.95 degrees/s gyro mean-vector norm. Another initial stationary capture is
   not required before proceeding to the controlled calibration sequence.
2. **Measure spatial/temporal calibration:** record a printed calibration target
   visible to both IR cameras, with smooth rotations about all three axes and
   translations at several depths. The ordinary walking sequence cannot fully
   validate the camera–IMU lever arm or separate all calibration errors. Retain
   original native images, device timestamps and SI IMU data.
3. **Address the visual failure envelope:** compare the same textured route with
   controlled turns and intentional brief views of the blank wall. Mount the
   camera so normal vehicle attitude keeps textured surroundings in view.
   Evaluate exposure and supported image profiles with measurements at 15W.
   Increasing feature count alone will not resolve an image with no usable
   corners. Keep this separate from calibration experiments.
4. **Then validate navigation loss and recovery:** map resets, relocalization and
   loop corrections require explicit frame/reset handling and Cube ground tests.
   Never substitute repeated stale poses for valid tracking. An independent
   navigation aid or an appropriate tested loss response is still needed for
   scenes that defeat vision; IMU propagation cannot bridge arbitrary outages.

The current three-beep recording cue means capture started/stopped. It does
not mean VIO is initialized or the Cube is ready to navigate.

## Reproduce

From `/home/atas/BHUslamV3`:

```sh
bin/bhu vio-diagnose \
  recordings/diagnostic/20261025T095519.847430960Z \
  reports/two-minute-run-stereo reports/two-minute-run-inertial \
  reports/NEW-diagnosis-output
```

Outputs: `diagnostics.json`, `timing-sweep.csv`, `rotation-residuals.csv`,
`image-quality.csv`, and `tracking-loss-contact.png`. Output directory must be
new. JSON fits compare against stereo, not ground truth. No calibration is
written back or sent to the Cube.

Validation: native build passed; CTest `vio_diagnostics_numeric` and `imu_sync`
passed. The synthetic test recovers a known +12 ms shift, its sign and gyro bias
on training and held-out samples, unit scale, and trapezoidal integration.
Launcher syntax passed `bash -n`. Checksums identify source, binary and YAMLs.
