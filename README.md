# BHUslamV3: autonomous GPS-denied drone missions

**End goal: Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.**

The [shared project goal](PROJECT_GOAL.md) connects localization, measured-depth
obstacle detection, Cube flight control and autonomous mission progress.
[Agent instructions](AGENTS.md) carry this objective into future project work.
VINS-Fusion is the existing primary localization implementation; the selected
[joint LIVO experiment](docs/livo-offline-replay.md) evaluates a possible
improvement, and the [Gemini depth avoidance plan](docs/gemini-depth-avoidance-plan.md)
describes the selected obstacle-detection/replanning direction. These are
workstreams toward the complete system, with their own validation status.

The detailed bring-up sections below retain historical test conditions. Use
the dated workflow documents and latest configuration evidence for operation.

The primary VINS-Fusion workflow, current configuration, operator test methods,
and architecture/pipeline charts are consolidated in the
[operator testing guide](docs/operator-testing-guide.md).

For isolated performance tests, use **[`bin/vio-test`](bin/vio-test)**:
QR start/stop, 15 W profile checks, loop-closure evidence and 3D reports.
See [the VIO performance workflow](docs/vio-performance.md).

Cube Orange+ external-navigation bench integration is documented in
[`docs/cube-external-nav.md`](docs/cube-external-nav.md). The live producer,
MAVLink interface, UART relay and tests are C++17. Fusion is not yet enabled.

Direct C++ Orbbec SDK capture and ORB-SLAM3 stereo / stereo-inertial replay for
Ubuntu 22.04 ARM64. No ROS dependency. All tools and build dependencies are
staged inside this workspace; system packages and camera firmware are untouched.

The connected camera's configuration is
[`configs/gemini336l-CPC2963000JJ-fast.yaml`](configs/gemini336l-CPC2963000JJ-fast.yaml).
Measured capture results are in [`reports/bringup.md`](reports/bringup.md).
Its firmware is already 1.8.10; no upgrade was needed.
The default is 1,000 ORB features. On the 30-second camera recording it tracked
895/895 frames with 29.36 ms median tracking-call time. The original 1,500-feature
profile remains available without the `-fast` filename suffix for comparison.

## Build

```bash
bash scripts/bootstrap.sh   # downloads dependencies; no sudo or package installation
bash scripts/build.sh      # defaults to one compiler job on the 8 GB host
source scripts/env.sh
```

The host's `/usr/local/bin/cmake` is x86-64. `env.sh` selects the staged ARM64
CMake. The build links the host's Ubuntu OpenCV 4.5.4 shared libraries and stages
matching missing headers, Eigen and Boost serialization. The largest Eigen-heavy upstream translation units compile at `-O1` to bound
compiler memory use; other files use the Release configuration. Pangolin is linked for
upstream compatibility; replay is headless unless `--viewer` is supplied.

Pinned sources: Orbbec SDK **2.10.6**, ORB-SLAM3
`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`, Pangolin
`3667bb76edcc3e4111d2bee7935ce204f04026d9`. The bootstrap checksum is the locally
verified downloaded SDK archive, not a separately signed publisher checksum.
ORB-SLAM3 and its bundled third-party sources retain their upstream licenses.
`patches/orb-slam3-cxx17-ba-counter.patch` corrects the upstream incremented
bundle-adjustment generation field from `bool` to `int` for C++17 compilation.
`patches/orb-slam3-rectified-settings.patch` initializes the shared right-camera
calibration for rectified stereo, fixing an upstream settings-printing crash.
`patches/orb-slam3-imu-status.patch` guards the upstream IMU-status timer before
a mapping keyframe exists, making startup status reporting safe.
The benchmark adds loop-correction event logging and waits for actual mapping/
bundle-adjustment workers before exporting the final trajectory; these changes
are preserved in `orb-slam3-loop-events.patch` and `orb-slam3-safe-shutdown.patch`.

## Camera access

```bash
build/main/bin/gemini_inspect
```

If Linux reports access denied, an administrator can grant temporary access to
the device node shown by `lsusb`. For the device initially detected here:

```bash
sudo setfacl -m u:atas:rw /dev/bus/usb/002/003
```

The bus/device number can change after reconnecting. To remove that grant:
`sudo setfacl -x u:atas /dev/bus/usb/002/003`. Orbbec's persistent udev installer
is also available at `third_party/orbbec/shared/install_udev_rules.sh`; inspect
its rules before choosing a system-wide installation.

## Record and check

```bash
build/main/bin/gemini_record recordings/stationary-01 30
python3 scripts/analyze_recording.py recordings/stationary-01
```

Keep the rig stationary for a gravity and jitter check. Then record a separate
sequence with continuous, smooth translation and gentle rotation through a
textured scene for the full capture duration. Include translations of roughly
30–50 cm in several directions where space permits; small rotations in place
alone may not satisfy ORB-SLAM3's initialization motion checks:

```bash
build/main/bin/gemini_record recordings/motion-01 45
python3 scripts/analyze_recording.py recordings/motion-01
```

Optional image dimensions are positional, e.g. `... 45 1280 800`. The device
must advertise that Y8 stereo profile at 30 FPS. Capture writes rectified PNG
pairs, both image timestamps and frame indices, SI accelerometer/gyro CSVs,
factory matrices and an ORB settings file. Existing recordings are never
overwritten. `quality.json` reports observed rates, timing gaps, stereo skew,
gravity magnitude and sampled ORB feature/alignment diagnostics. A passing
timing check does not establish accurate VIO or a calibrated camera–IMU offset.

## Replay

```bash
build/main/bin/gemini_replay recordings/motion-01 third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt
build/main/bin/gemini_replay recordings/motion-01 third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  --inertial --allow-provisional
```

The second command is an experiment with provisional IMU noise parameters.
After calibration, use `--settings PATH` with validated settings and omit
`--allow-provisional`. `--realtime` paces the recording and `--viewer` enables
Pangolin. Output goes to `slam_stereo/` or `slam_inertial/` inside the recording:
TUM trajectory, keyframes and per-frame tracking state / processing time.
Existing SLAM output directories cause an error rather than overwriting results.
Use `--output DIRECTORY` to compare another configuration on the same recording.
The tracking CSV also records whether the IMU initialized. Inertial replay returns
a nonzero status if it never initializes, even when visual tracking succeeds.
The full TUM trajectory contains left-camera-to-world poses recentered at the
first keyframe, following upstream behavior. The keyframe export retains the
map's own world frame. Empty maps produce a clear failure without calling the
upstream trajectory exporter that requires an existing keyframe.

Record/replay remains available. The C++ `gemini_live` bench producer adds direct
camera/IMU processing and optional gated odometry publication; see
[the Cube guide](docs/cube-external-nav.md) for its current validation status and
run commands. RGB-D comparison and measured IMU noise/time-offset calibration
remain pending. Configuration conventions and restoration behavior are detailed
in [configs/README.md](configs/README.md).

For QR-controlled tests without a display, use the
[Cube buzzer and VIO test workflow](docs/cube-vio-test.md). Actual configuration
and the first QR-started/stopped moving test are recorded in
[Cube validation results](reports/cube-vio-validation.md).

To diagnose initialization resets, the [diagnostic recording mode](docs/vio-diagnostic-recording.md)
automatically saves two minutes of native/rectified stereo and native-axis SDK
IMU/replay files after boot. The Cube gives three beeps at recording start and
three after saving; no QR or switch is needed. `bin/vio-test boot-record` selects
it for battery startup; `bin/vio-test boot-vio` restores the VIO test.

Offline camera/IMU consistency checks are available through
`bin/bhu vio-diagnose RECORDING STEREO_REPLAY VIO_REPLAY NEW_OUTPUT`.
The native tool exports failure images, image-quality measurements and a
gyro-versus-stereo timing sweep without changing calibration or Cube settings.
See the [measured failure and calibration diagnosis](reports/vio-calibration-diagnosis.md)
for the two-minute run, repeated baseline and timing experiment. These checks
do not constitute a complete spatial/temporal or IMU noise calibration.

Use `bin/bhu imu-stationary RECORDING NEW_OUTPUT` for native C++ stationary
statistics, short-record Allan deviations and visual consistency checks.
The [latest stationary run review](reports/latest-stationary-run-review.md)
documents the confirmed boot capture and the limits of these measurements.

To visualize landmarks and camera/keyframe geometry from recordings, add
`--sparse-map` to replay and open its `sparse-map.html` output. See the
[recorded sparse 3D SLAM viewer](docs/recorded-sparse-slam.md) for controls,
PLY export, reset handling and the distinction from a dense reconstruction.

An isolated C++ VINS-Mono adapter is available as
`bin/vins-mono RECORDING NEW_OUTPUT`. The
[recorded-data benchmark](docs/vins-mono-benchmark.md) documents its workspace-only
dependencies, matched monocular ORB baseline, noise policies and limitations.
It does not enable external navigation or change the recording boot workflow.

The C++ stereo-plus-IMU adapter uses
`bin/vins-fusion RECORDING NEW_OUTPUT`. See the
[VINS-Fusion benchmark](docs/vins-fusion-benchmark.md) for its isolated build,
stereo conventions and comparison with the existing Mono/ORB results.
For synchronized stereo feature tracking above sparse 3D landmarks, use
`bin/vins-fusion-viewer RECORDING NEW_OUTPUT`; the
[recording viewer guide](docs/vins-fusion-viewer.md) describes playback and exports.

Sources: [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3),
[Orbbec SDK release](https://github.com/orbbec/OrbbecSDK_v2/releases/tag/v2.10.6),
[SDK API guide](https://orbbec.github.io/docs/OrbbecSDKv2_API_User_Guide/source/3_Application_Guide/ApplicationGuide.html).

The current GPS-aided [VINS observation transport](docs/vins-observation-forwarding.md) forwards only gate-qualified poses through the existing Cube UART owner. Source set 1 and `EK3_SRC_OPTIONS=0` are preserved; VIO fusion remains a separate validation.

An isolated [Gemini/JT16 LIVO replay experiment](docs/livo-offline-replay.md)
is available for offline evaluation. The initial flight-213 trials diverged;
no localization improvement or live estimator replacement is validated.

Fresh-GPS origin/Home initialization for the selected VIO/optical-flow/internal-compass setup: [bootstrap procedure](docs/gps-bootstrap.md). Clear-sky fix/disconnection and RTL validation remain separate from indoor setup checks.

Current front ToF / GPIO proximity and JT16 recording-only split: [setup and limitations](docs/tof-front-proximity.md).

H-Flow integrated downward DroneCAN range + relocated TF03 rear avoidance: [assignment and verification](docs/hflow-down-rear-rangefinder.md).
