# Latest boot stationary recording — 2026-10-25

The latest boot already contains the requested stationary two-minute capture.
Another stationary run is not needed to obtain this initial bias measurement.

## Identity and integrity

- Current boot and recording boot ID: `de34ab0c-895e-4fd2-943a-b49581c964d8`.
- Dataset: `recordings/diagnostic/20261025T104811.422872864Z`.
- Cube session: `runs/cube-record/20261025T104810.649952512Z`.
- Timed duration: 120.006205 seconds; first-to-last stereo timestamp span
  120.009336 seconds. These measure slightly different recorder boundaries.
- 3,597 stereo pairs; no frame-index gaps or stereo rejections; maximum skew 1 µs.
- 24,376 accelerometer and 24,378 gyro samples; approximately 202.583 Hz;
  maximum sample gap 4.937 ms; both bracket all recorded images.
- Capture completed and synced before the stop cue, at 15W at start and end.
- Cube bridge: navigation disabled, zero odometry forwarded, zero parameter
  writes, zero link drops, start/stop cues sent. One malformed incoming UART
  packet is separate from camera/IMU recording integrity.

## Stationarity assessment

The user's report, quiet IMU throughout, stable ten-second means and repeated
view of the same desk/monitor support treating this as a stationary capture.
There are small disturbances/vibration in some windows; it is not a laboratory
isolation-table measurement. The camera frame is slightly tilted; an individual
accelerometer axis cannot simply be forced to zero or ±g.

IR images are severely dark: median sampled image mean is only about 15.6/255.
The first frame also differs from later images during camera/display startup.
Contrast-enhanced contact images reveal the same scene after startup. These
enhancements are diagnostic only; original images were not altered.

Robust feature matching against a ten-second anchor does not produce enough
inliers on independent images for a reliable quantitative displacement test.
The tool requires at least 20 homography inliers and excludes the anchor's
self-match from displacement statistics. Missing displacement metrics mean
insufficient visual evidence, not measured zero movement. Consequently the
stationarity assessment combines the user's account, IMU stability and visual
inspection; it is not certified by visual odometry.

## Measured IMU statistics

Use the interval after the first ten seconds, before the last image, to avoid
startup effects. Each stream contributes 22,286 samples to these statistics.

| Quantity | X | Y | Z |
| --- | --- | --- | --- |
| Gyro mean, rad/s | -0.0309484 | -0.0113486 | +0.00846275 |
| Gyro sample standard deviation, rad/s | 0.0008239 | 0.0004563 | 0.0006250 |
| Accelerometer mean, m/s² | +0.0187308 | -9.7679311 | +0.2072062 |
| Accelerometer sample standard deviation, m/s² | 0.0076764 | 0.0183052 | 0.0089264 |

Gyro mean-vector norm: **0.0340325 rad/s = 1.9499 degrees/s**. This confirms the
roughly two-degrees/s stationary gyro bias seen in the earlier moving recording's
quiet tail. This bias is distinct from the much smaller sample noise.

Difference between the last twenty-second gyro mean and the early twenty-second
mean (10–30 s): `[-0.00013293, -0.00009487, +0.00006408] rad/s`;
vector norm approximately 0.0101 degrees/s. This is a small but measurable
change during the run. Temperature was not logged continuously, so it cannot be
assigned a temperature coefficient from these data.

Acceleration mean-vector norm: **9.77015 m/s²**. This is consistent with gravity
at this orientation with possible sensor bias/scale error. A single stationary
orientation cannot separate gravity direction, three-axis bias and scale.

## Noise analysis limits

The native tool computes overlapping Allan deviations after the settling
interval, at approximately 0.01–10 s averaging times, using the approximately
uniform 202.583 Hz device-time sampling. It exports the actual averaging times
and number of overlapping differences. Overlapping points are correlated;
they are not that many independent long-duration measurements.

At approximately one-second averaging time, gyro Allan deviations are roughly
`[3.45e-5, 3.05e-5, 3.24e-5] rad/s`. These are Allan deviations at one averaging
time, not directly ORB-SLAM3 parameter values. The two-minute record is useful
for bias and short-term noise characterization, but is too short to establish
long-term random walk or thermal behavior confidently. No noise/random-walk
settings, fixed bias subtraction or `Calibration.imuValidated` flag were changed.

## Outcome and next step

The initial stationary-bias measurement is now available; keep this capture.
Next is the controlled, textured camera–IMU motion/calibration capture, with
adequate IR image brightness. A stationary sequence cannot measure camera–IMU
time offset or fully validate the spatial transform. It also should not be
graded by whether VIO initializes: the current algorithm requires motion for
inertial initialization.

The observed dark indoor images also deserve an exposure/feature-quality check
before the next motion capture. Do not expect the stationary bias result alone
to solve the blank-wall tracking failure identified in the walking run.

## Artifacts and reproduction

Native C++ tool: `src/imu_stationary.cpp` / `build/main/bin/gemini_imu_stationary`.
No Python was introduced. No hardware access is needed for analysis.

```sh
bin/bhu imu-stationary \
  recordings/diagnostic/20261025T104811.422872864Z \
  reports/NEW-stationary-assessment
```

Final outputs: `reports/latest-boot-stationary-analysis/assessment.json`,
`windows.csv`, `allan.csv`, `visual-stability.csv`, `imu-window-means.png`, and
`stationary-contact.png`. The output directory must be new.

Build succeeded. Numeric tests for stationary statistics/Allan deviation,
camera–gyro timing diagnostics and IMU synchronization passed. Launcher shell
syntax passed. The Allan test checks a known linear ramp and constant channels.
Checksums are saved beside the final outputs. No camera, Cube or service
configuration was changed.
