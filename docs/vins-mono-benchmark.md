> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> This VINS-Mono benchmark is a historical localization comparison reference. See the [shared project goal](../PROJECT_GOAL.md).

# VINS-Mono recorded-data benchmark

This is an isolated native C++ replay of HKUST VINS-Mono's upstream feature
tracker and estimator. It reads the existing Gemini left rectified IR images
and SI accelerometer/gyro CSVs. It does not use the right camera, access live
hardware, start ROS or communicate with the Cube.

Sources are pinned to:

- VINS-Mono `90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d`
- Ceres 1.14.0 `facb199f3eda902360f9e1d5271372b7e54febe1`

Upstream estimator and feature-tracker source files are unmodified. The adapter
replaces ROS logging/header/transport with a small compatibility shim and direct
C++ calls; ROS publication/visualization and pose-graph nodes are not built.
This is an offline adapter, not a stock ROS runtime performance measurement.

## Build layout

- Source: `third_party/VINS-Mono`, `third_party/ceres-1.14`
- Workspace-only glog/gflags packages: `third_party/vins-deps/packages`
- Extracted packages: `third_party/vins-deps/sysroot`
- Ceres install: `third_party/vins-deps/install`
- Native adapter/configuration: `tools/vins_mono`
- Ceres build: `build/ceres-vins`
- Binaries: `build/vins-mono/bin/vins_mono_replay`, `vins_mono_report`
- Launcher: `bin/vins-mono` or `bin/bhu vins-replay`

No system packages were installed and no Python was added. Ceres 1.14 is used
because the upstream README warns of incompatibilities with Ceres 2.x. The
Ubuntu ARM64 glog/gflags `.deb` packages were downloaded and extracted locally:
`libgoogle-glog-dev`, `libgoogle-glog0v5`, `libgflags-dev`, `libgflags2.2`.
They must already be staged before using `scripts/build-vins-mono.sh`.

Source `scripts/env.sh` for the staged ARM64 CMake (the unrelated
`/usr/local/bin/cmake` is an x86 executable). Build serially:

```sh
scripts/build-vins-mono.sh
```

Ceres and the VINS adapter use bounded `-O1` compilation on the 8 GB Jetson;
Ceres uses generic Schur code, Eigen sparse support and no internal threading,
SuiteSparse or LAPACK backend. These choices affect performance. This measures
the present Jetson builds, not each library's maximally optimized performance.
VINS's own marginalization code can still use its native pthread workers.
The main ORB build also uses `-O1` on its expensive Eigen-heavy source files.

The tracker and estimator were separate ROS executables upstream. Their global
`ROW`, `COL`, `FOCAL_LENGTH`, `WINDOW_SIZE` and `NUM_OF_CAM` names conflict in one
process, so only the tracker translation units receive name-prefix macros.
No numerical constants or estimator algorithms are changed by those renames.
The estimator uses static storage, matching the upstream node: its constructor
expects its pointer arrays to have been zero-initialized.

## Input and configurations

```sh
bin/vins-mono RECORDING NEW_OUTPUT --opencv-threads 1
# Sensitivity test using upstream EuRoC noise values:
bin/vins-mono RECORDING ANOTHER_NEW_OUTPUT --native-noise --opencv-threads 1
```

`--max-frames N` supports short integration tests. A new output directory is
required. Input images and IMU are never modified. `vins-config.yaml` records the
actual configuration used by each run.

The camera model uses recorded rectified left intrinsics with zero remaining
distortion. Factory camera-to-gyro rotation and metre translation are read from
`IMU.T_b_c1`. Acceleration was already rotated into gyro axes by the recorder.
Acceleration is in m/s² including gravity, gyro in rad/s, and device timestamps
in seconds. The adapter uses the existing bounded accelerometer-to-gyro
interpolation and integrates to each image timestamp using an interpolated IMU
endpoint. It does not extrapolate or submit an endpoint twice.

ORB's adapter convention is camera time = IMU time + offset; VINS uses image
time + td = IMU time, so `td = -IMU.TimeOffsetSeconds`. Both data sets currently
specify zero. `estimate_td=0` and `estimate_extrinsic=0`: no online calibration
is enabled and no stationary-derived bias is subtracted in advance. Calibration
remains provisional throughout this comparison.

Two noise policies are explicit:

1. The baseline provisionally bridges ORB's continuous noise/walk settings to
   VINS's sample-noise/midpoint-Jacobian convention using `sqrt(IMU.Frequency)`.
   This is a configuration convention bridge, not a measured calibration or
   proof of identical information weighting between estimators.
2. `--native-noise` uses upstream EuRoC values: acceleration noise 0.08, gyro
   noise 0.004, acceleration walk 0.00004, gyro walk 0.000002. This is a
   sensitivity test; those values are not a Gemini calibration either.

Frontend parameters use the upstream 150-feature budget, 30-pixel minimum
separation, CLAHE enabled and 1-pixel fundamental-matrix threshold. The upstream
460-pixel normalized-coordinate convention is retained. All recorded images
are offered at their approximately 30 Hz device cadence; no feature publication
downsampling is used. Upstream startup skips are retained: first image callback,
first feature-population frame and first feature message before estimator use.
The solver retains its eight-iteration/0.04-second limits.

The monocular ORB baseline uses the same left images and IMU and original
provisional calibration. Its 1,000-feature frontend differs from VINS's frontend.
Stereo-inertial ORB is an additional two-camera reference, not an input-equivalent
algorithm comparison. Loop closure is disabled in the benchmark copy of ORB
settings and is absent from the VINS core-only adapter. Existing live/recording
and visualization settings are unchanged.

## Sequential benchmark

After selecting and verifying 15W, with no compiler or other benchmark running:

```sh
scripts/run-vins-benchmark.sh reports/NEW-vins-benchmark --both-recordings
```

This runs the two-minute walking dataset and the earlier 80-second moving
dataset. The former includes a noise sensitivity run and a stereo ORB reference.
All runs are sequential, with one OpenCV thread and OpenBLAS/OpenMP thread
limits of one. This does not serialize the libraries' independent native worker
threads. Power mode, boot ID, commits, logs, explicit exit codes and binary
checksums are recorded. The runner requires 15W but does not change power mode.

```sh
source scripts/env.sh
build/vins-mono/bin/vins_mono_report reports/NEW-vins-benchmark
```

The report produces `metrics.md`, `comparison.json` and a standalone interactive
`comparison.html`. It compares initialization, native pose availability, gaps,
pipeline time, peak RSS and stationary-tail consistency. No Internet server is
required. Plots preserve independent reset segments and do not overlay local
coordinate systems without alignment.

## Interpreting results

There is no ground truth in the recordings. No ATE, absolute accuracy or official
benchmark score should be claimed. Agreement with stereo is not ground truth.

VINS `NON_LINEAR` and ORB `tracking OK + IMU initialized` have different health
semantics. In particular, VINS's native failure detector logs low visual track
count but its corresponding `return true` is commented out upstream. VINS can
therefore continue publishing propagated/weakly constrained poses through a
visual outage. Its additional `visual_support_20` field requires at least 20
persistent frontend tracks and 20 backend tracked features. This is a diagnostic
heuristic, not an accuracy or flight-readiness guarantee.

ORB pose history is filtered to tracking OK and initialized for comparison.
VINS camera poses are converted from estimated IMU/body pose through camera–IMU
extrinsics, so both histories refer to the camera. VINS reset segments use its
native initialization/failure transitions; ORB history also breaks on map/frame
changes. The full final pose history is distinct from an optimized trajectory.

The two-minute run's known stationary tail is examined over 91–115 seconds.
Endpoint/max displacement is reported only if a single initialized segment
covers at least 20 seconds there. Unavailable tracking is not zero drift.
The earlier 80-second run is not assumed to have a stationary tail.

Pipeline timing includes image decode, frontend and backend work in synchronous
offline replay, with vocabulary/configuration loading excluded. ORB mapping is
asynchronous; VINS frontend/backend calls are serial in this adapter. Logging and
process startup are not fully included in per-frame timing. This is not measured
live capture-to-Cube latency. Active-pose p95 avoids diluting time statistics with
long periods spent uninitialized. A 30 Hz frame budget is approximately 33.4 ms.
Peak process RSS includes required libraries/model assets. 15W is the selected
power profile, not a measurement of energy consumption.

Single passes, fixed order, scheduler variation, thermal state, different
frontends and provisional noise remain limitations. These results guide the
next experiment; they cannot select a flight navigation system on their own.
