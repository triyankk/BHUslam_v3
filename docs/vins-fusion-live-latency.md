> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> This bench method measures the live localization and transport contribution to that goal. See the [shared project goal](../PROJECT_GOAL.md).

# VINS-Fusion live latency bench test

The native C++ adapter runs the existing ROS-free VINS-Fusion core on live
Gemini stereo/IMU data. It uses the same fixed factory extrinsics and provisional
noise convention as the recording benchmark. Calibration is not validated.
Loop closure is off.

## Run

```bash
# Select 15 W first if necessary; the launcher verifies it, never changes it.
bin/vio-test power-15w

# No Cube navigation transmission; creates a new output directory.
scripts/run-vins-fusion-latency.sh reports/NEW-dry-test 20

# Disarmed USB-connected Cube: eligible live poses can be forwarded.
scripts/run-vins-fusion-latency.sh reports/NEW-cube-test 60 --cube
```

Build with `bash scripts/build-vins-fusion.sh` and rebuild the main project's
`cube_vio_bridge` target for its USB timing diagnostics. Use a new output path
for each test. This workflow does not install a service or change the boot
recorder, Cube parameters, EKF source selection or arming state. The bridge
requires a disarmed Cube; the launcher stops the camera process if the bridge
exits. The bridge is the only owner of the Cube USB serial port.

Direct adapter command (dry run unless `--udp-port` is supplied):

```bash
bin/vins-fusion-live configs/gemini336l-CPC2963000JJ-fast.yaml \
  configs/vins-fusion-extnav.yaml reports/NEW-live \
  --seconds 30 --allow-provisional
```

## Timing and health behavior

- Camera callbacks replace a single pending stereo pair. The estimator chooses
  the newest pair instead of draining a queue of old images.
- Separate 800-sample accelerometer/gyro histories preserve intervening IMU
  samples. Acceleration is rotated into the gyro frame. IMU integration uses
  interpolated acceleration at gyro timestamps and at each image endpoint.
  Nonmonotonic timestamps, missing integration history and excessive IMU gaps
  stop the test.
- Rectification runs before feature tracking. All video/profile calibration is
  checked against the connected camera, reusing the existing SDK conventions.
  Camera properties are restored on normal shutdown and stack unwinding.
- Image device timestamps map to host monotonic time using a lower envelope of
  delivery offsets. Estimated age includes processing and waiting for the
  chosen image, but exposure/USB delay is not independently measured.
- The publication gate requires initialized, finite, aligned VIO, plausible
  finite-difference velocity, at least 40 persistent features, 20 backend tracks
  and 20 stereo matches, followed by 0.5 seconds of continuously good capture timestamps.
  These are conservative **bench heuristics**, not calibrated confidence.
  Timestamp freshness is limited to 150 ms. Upstream VINS failure detection is
  disabled, so initialization alone is not treated as tracking health.
- VINS camera poses are converted to local FRD and vehicle FRD attitude. Initial
  yaw is arbitrary; the VOXL-compatible Cube backend handles heading alignment.
  Position remains the camera origin; the Cube's `VISO_POS_X=0.15` applies its
  lever arm once. This frame/offset behavior still needs live axis validation.
- The Fusion launcher sets relay quality minimum 40; the Cube setup preset
  sets `VISO_QUAL_MIN=40`. Raw quality values are forwarded unchanged.
- The relay forwards newest eligible packets at up to 10 Hz. It does not repeat
  old poses. Rate limiting can add waiting time or cause packets to expire.

`tracking.csv` records processing stages, estimated capture age, support counts,
gate status and overwritten pending images. `pose_history.csv` contains native
VINS camera poses, including initialized samples withheld from publication;
this is not a log of poses accepted by the Cube. `summary.json` gives
percentiles. `bridge/events.jsonl` records age at the serial write and serial
write duration. Those are host transport measurements, not Cube receipt times
or EKF fusion delays. Reports contain no raw image recording.

The 28 October 2026 measurements and limitations are in
[`fusion-live-15w-20261028/review.md`](../reports/fusion-live-15w-20261028/review.md).
