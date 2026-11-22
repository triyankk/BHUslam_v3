# VINS-Mono versus ORB-SLAM3 — recorded Gemini data, 2026-10-26

## Outcome

VINS-Mono is a promising odometry candidate on these recordings: with the
provisional noise-convention bridge it initialized quickly and completed both
moving sequences without a native failure reset. It used much less process
memory than the current ORB build. This does **not** establish ground-truth
accuracy, reliable visual tracking throughout, or flight readiness.

Two important gaps remain visible: processing every image through this VINS
backend at 30 Hz exceeds the 15W compute budget, and trajectories disagree
materially between estimators despite the small stationary-tail variation.
Results are sensitive to noise settings. Calibration has not been performed.

Interactive report: [full/comparison.html](full/comparison.html).
Raw metrics: [full/metrics.md](full/metrics.md).
Inputs and methods: `docs/vins-mono-benchmark.md`.

## Scope and controls

- Two-minute moving capture:
  `recordings/diagnostic/20261025T095519.847430960Z`, 3,597 frames.
- Earlier 80-second moving capture:
  `recordings/diagnostic/20261025T091926.822317376Z`, 2,399 frames.
- All six full runs were sequential, with compilation stopped during replay.
- Jetson power mode was verified **15W / mode 0** at both start and end.
- OpenCV/OpenBLAS/OpenMP thread settings were one; native mapping/marginalization
  workers retain their own threading. Device timestamps and all images were used.
- VINS-Mono and monocular-inertial ORB use the same left IR images and IMU.
  Stereo-inertial ORB is a separate two-camera reference.
- Factory extrinsics and original zero time offset remain fixed. No online
  camera–IMU calibration or pre-subtraction of measured stationary bias.
- Loop closure was disabled for both odometry baselines. The VINS pose-graph
  backend was not built. This is not a full loop-closure/SLAM benchmark.
- Source recordings, active camera/Cube settings and boot services were not changed.

VINS-Mono's upstream numerical source was not modified. A C++ adapter replaces
ROS transport/log/header plumbing. Ceres 1.14 and glog/gflags are installed only
under the workspace. No ROS runtime, global package installation or Python was
introduced. Compiler/dependency choices are documented; the bounded -O1 build
is not each library's maximum-performance build.

## Two-minute run

| Configuration | Native initialized pose frames | First initialized pose | Native failure resets | Pipeline p95, all frames | Peak process RSS |
| --- | --- | --- | --- | --- | --- |
| VINS-Mono, noise bridge | 3,545 / 3,597 (98.55%) | 1.74 s | 0 | 78.7 ms | 102 MiB |
| VINS-Mono, upstream EuRoC noise | 3,017 / 3,597 (83.88%) | 1.74 s | 1 | 76.3 ms | 87 MiB |
| ORB monocular + IMU | 458 / 3,597 (12.73%) | 6.51 s | see reset log | 102.3 ms | 724 MiB |
| ORB stereo + IMU | 2,196 / 3,597 (61.05%) | 4.34 s | see reset log | 61.5 ms | 713 MiB |

**Flags differ:** VINS frames above mean its solver is NON_LINEAR; ORB frames
mean tracking OK **and** IMU initialized. VINS's upstream failure detector does
not return failure merely because too few visual features remain. Those
percentages must not be described as equivalent visual tracking success rates.

The noise-bridge VINS run had 3,465 frames passing the additional diagnostic
threshold of at least 20 persistent frontend and backend tracks. It had 80
initialized frames below that threshold, roughly 2.7 seconds of image times.
This is a useful visual-support indicator, not a correctness certificate: weak
or incorrect matches can still pass a feature-count threshold.

Both ORB runs ended without a surviving final map and returned exit 1 after
processing all frames. Online histories and timing summaries were saved.
The two VINS runs returned 0 because they initialized at some point; the native
noise run nevertheless ended uninitialized. Exit status alone is insufficient
to assess continuity.

### Stationary tail

During the known stationary interval 91–115 s, the noise-bridge VINS camera
estimate changed by **1.99 mm between endpoints**, with a maximum displacement
of **2.38 mm** from the first pose in that interval, over 23.96 s in one segment.
This is small reported-pose variation while stationary; it is **not** millimetre
absolute accuracy or proof that the final location is correct.

The other configurations do not have a single valid initialized segment covering
20 seconds of that interval. Their stationary drift is reported as unavailable,
not zero. VINS's estimated biases are saved in `bias.csv`; they are fitted
internal states, not an independently measured IMU calibration.

### Trajectory agreement

Using 1,648 matching initialized camera poses in 5.006–59.971 s, compare VINS
against the stereo-inertial ORB online reference. A mean camera-orientation
rotation and mean position translation align their coordinate gauges; **scale
stays fixed** for position RMS:

- Noise-bridge VINS: **2.116 m RMS agreement residual**.
- Upstream-noise VINS: **3.592 m RMS agreement residual**.
- A separate diagnostic reference/run scale fit gives 0.995 and 0.932
  respectively. Those scales were not used to reduce the RMS values.

Neither estimator is ground truth, and online histories can include frame/
refinement effects. These values establish material disagreement, not which
estimate is accurate. The monocular ORB comparison covers a shorter interval
and must not be ranked against the longer VINS interval as an accuracy score.
Actual ground truth or a controlled measured trajectory is needed next.

## Earlier 80-second moving run

| Configuration | Native initialized pose frames | First initialized pose | Native failure resets | Pipeline p95, all frames | Peak process RSS |
| --- | --- | --- | --- | --- | --- |
| VINS-Mono, noise bridge | 2,350 / 2,399 (97.96%) | 1.64 s | 0 | 75.9 ms | 81 MiB |
| ORB monocular + IMU | 728 / 2,399 (30.35%) | 6.81 s | see reset log | 71.5 ms | 605 MiB |

VINS ended initialized. Its diagnostic visual-support threshold passed on
2,199 frames; 151 initialized frames were below it. This earlier recording is
not assumed to have a stationary tail. The ORB run ended without a surviving
final map after processing all frames.

## Runtime interpretation

The two-minute noise-bridge VINS run took 216.2 s to process 120 s of data.
Its median pipeline time was 59.0 ms; active-pose p95 was 78.7 ms. On the earlier
run it took 136.2 s for 80 s of data. This particular all-images/backend-at-30-Hz
configuration cannot sustain the approximately 33.4 ms serial frame budget.

The table in `metrics.md` additionally gives p95 restricted to initialized
outputs. Uninitialized/reset periods have different workloads, so neither a
whole-run median nor an active-only statistic should be considered alone.
The current stereo ORB run also exceeded real-time on average (159.0 s for 120 s).

These are synchronous offline processing costs, not measured live pose age or
camera-to-Cube latency. The VINS frontend and backend are serialized by this
adapter, whereas ORB has asynchronous mapping. The upstream ROS example often
publishes frontend results to its backend at a lower rate (EuRoC config: 10 Hz);
that lower-rate/pipelined deployment was **not** benchmarked here. Lower backend
rate, solver budget and build optimization are appropriate subsequent tests.

VINS memory is for its local sliding-window core without loop-closure vocabulary
or global pose graph. ORB retains mapping/vocabulary structures. Enabling the
full VINS loop backend would change memory and runtime. These are practical
process measurements for these exact builds, not feature-equivalent memory
claims about all VINS versus all ORB deployments.

## Recommendation

Continue evaluating VINS-Mono alongside the existing pipeline. Its continuity
and stationary consistency justify the next controlled experiment, but do not
enable it for flight yet. Priorities are a measured trajectory/calibration test,
then a live 15W deployment with an appropriate backend rate and measured pose
age. Preserve the current record-only boot mode and Cube source selection until
fusion and loss handling are separately validated.

## Validation and artifacts

- Short native smoke run: 90 frames; first initialization 1.735 s; finite poses.
  Smoke timing is excluded because it overlapped compilation.
- Native integration test passed for SI acceleration, known gyro rotation,
  integration duration and bias-walk covariance convention.
- Report UI tests passed for finite trajectory/timeline coordinates, selections,
  reset gaps, empty histories and orbit/zoom, including the real six-run data.
- Main IMU synchronization, diagnostics and sparse-map UI regressions passed.
- Launch/build/benchmark shell syntax passed; upstream VINS working tree clean.
- Replay binaries match the checksums recorded by the full benchmark runner.

`full/` contains explicit exit codes, log files, generated configurations,
tracking and camera-pose histories, VINS bias histories, power-mode readbacks,
source commits, checksums, completion marker and generated report. Methods and
limitations accompany the data so initialized output is not mistaken for
verified navigation accuracy.
