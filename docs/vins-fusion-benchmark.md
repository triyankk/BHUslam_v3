> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> This benchmark evaluates the localization component on the path to that goal. See the [shared project goal](../PROJECT_GOAL.md).

# VINS-Fusion recorded-data benchmark

Native C++ stereo + IMU replay, isolated from boot recording and Cube navigation. This compares odometry with loop closure disabled, matching the existing VINS-Mono/ORB inertial baselines. No Python or ROS runtime is used.

## Reproduce

The official source is staged at `third_party/VINS-Fusion`, pinned to `be55a937a57436548ddfb1bd324bc1e9a9e828e0`. It shares the workspace-local Ceres 1.14/glog dependencies built for VINS-Mono; see `docs/vins-mono-benchmark.md` for dependency staging.

```bash
scripts/build-vins-fusion.sh
source scripts/env.sh
export LD_LIBRARY_PATH="$PWD/third_party/vins-deps/install/lib:$PWD/third_party/vins-deps/sysroot/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
ctest --test-dir build/vins-fusion --output-on-failure
bin/vins-fusion recordings/diagnostic/20261025T095519.847430960Z NEW_OUTPUT --opencv-threads 1
scripts/run-vins-fusion-benchmark.sh NEW_BENCHMARK_OUTPUT
```

The benchmark runner requires 15 W mode, runs sequentially and saves power-mode readbacks, boot ID, source commit, executable checksum, logs and exit codes. Replays do not access the camera or flight controller.

## Adapter and conventions

`tools/vins_fusion` compiles upstream camera models, optical-flow tracker, stereo projection factors, initialization, sliding-window optimization and IMU preintegration. The upstream checkout is unmodified. A build-directory copy of `estimator.cpp` substitutes its ROS visualization include with empty output hooks. Logging/Header compatibility shims and OpenCV legacy colour-name inclusion are the other compatibility layers. ROS publication, loop/global fusion and ROS queue scheduling are absent.

Input is the recorded 640×400 rectified IR pair at approximately 29.96 Hz, synchronized SI IMU at approximately 202.6 Hz, with endpoint interpolation from the existing native `bhu::synchronize` helper. All frames are processed synchronously, without drops, decimation or extrapolation. Initialization uses the upstream first-IMU gravity alignment. No stationary bias subtraction is applied. The native `processIMU` and `processImage` interfaces are used; no custom numerical estimator is introduced. After resets the upstream tracker remains alive, as in its native implementation.

For rectified cameras, both use recorded left intrinsics and zero distortion. The left `T_b_c` is copied from `orb.yaml`; the right centre is `t_b_c0 + R_b_c0 [baseline,0,0]`, with the same rotation. Baseline is 0.0947195968628 metres. Synthetic tests exercise the upstream stereo projection factor to check baseline sign and units. Exported poses are camera poses, `R_wc=R_wb R_bc` and `p_wc=p_wb+R_wb t_bc`, matching previous report conventions. Reset epochs are separate segments.

Settings: 150 corners, minimum distance 30 px, forward/backward optical-flow checking enabled, no CLAHE (upstream Fusion default), 10-state sliding window, 8 solver iterations and 0.04 s solver budget. Extrinsic and temporal calibration are fixed; recorded offset is converted by `td = -ORB_offset`. No loop closure is enabled. Contrast preprocessing differs from the previous VINS-Mono adapter, which used its original CLAHE frontend. Treat this as a comparison of these recorded configurations, not a controlled isolation of stereo depth alone.

Baseline noise uses the same provisional density-to-sample bridge as the previous Mono experiment: each recorded ORB noise/walk value multiplied by sqrt(200). A second two-minute run uses Fusion's own EuRoC defaults: acc_n=0.1, gyr_n=0.01, acc_w=0.001, gyr_w=0.0001. Neither policy is a Gemini calibration. The Fusion defaults differ from VINS-Mono's EuRoC defaults.

## Interpretation

The `NON_LINEAR` flag is recorded separately from a visual-support heuristic requiring at least 20 persistent frontend tracks (age at least two images) and 20 backend tracks. The threshold is diagnostic, not a proof of pose accuracy; this pinned Fusion revision disables failure detection entirely with an immediate `return false` in `Estimator::failureDetection`. Consequently zero native resets and a continuous initialized flag do not demonstrate robustness. Per-frame stereo match count is also saved. Runtime includes image decoding, stereo tracking and backend work. It is synchronous offline processing cost, not measured live camera-to-Cube latency.

Builds use -O1 and the same workspace Ceres build as Mono, on a 15 W Jetson. Run-to-run timing depends on scheduling and thermal state. Previous comparison runs are reused with their original provenance, rather than implying all algorithms ran simultaneously. VINS local-window memory excludes optional loop-closure vocabulary and pose graph; ORB includes map/vocabulary overhead.

No surveyed ground truth exists. The user's visual assessment favours the original ORB stereo route shape and VINS-Mono continuity. Estimator-to-estimator rigid agreement and stationary-tail displacement cannot establish absolute trajectory error or flight readiness.

The current comparison is in `reports/vins-fusion-benchmark-20261026/comparison/comparison.html`; its embedded data is standalone. Its input directories are relative symlinks to the new Fusion runs and original Mono/ORB benchmark. Run `build/vins-fusion/bin/vins_fusion_report reports/vins-fusion-benchmark-20261026/comparison` after all source summaries exist to regenerate the HTML, JSON, table and native PNG overview. The viewer keeps local coordinate systems and reset segments separate; the overview auto-fits each panel and is not an aligned accuracy overlay.
