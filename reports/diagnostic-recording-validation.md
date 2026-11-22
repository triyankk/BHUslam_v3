# Diagnostic recording mode validation, 2026-10-25

The C++ `gemini_record` now supports diagnostic capture and the existing native
QR state machine and Cube feedback sender. The launcher uses a feedback-only
Cube bridge and does not run SLAM. Native Y8 and rectified image pairs are stored
as lossless uncompressed PGM; native SDK IMU and replay-axis SI CSVs are both
saved, with device/system/host timing, factory calibration and completion state.

## Verified

- Main recording/checker/bridge targets compiled on ARM64. Build records:
  `reports/diagnostic-record-check-build.log` and related diagnostic-record logs.
- All eight native/UI checks passed: `reports/diagnostic-record-tests.txt`.
  The Cube fixture now checks feedback-only mode sends a cue but does not
  forward otherwise valid odometry. The recording integrity fixture verifies
  missing native images and nonmonotonic IMU are rejected, and shortened
  protocol / IMU-gap warnings are retained. The final checker also removes a
  stale prior success report before checking a dataset again.
- A four-second real-camera stationary smoke recording is saved in
  `recordings/diagnostic/prepare-smoke-01/`: 116 stereo pairs, 842 accelerometer
  samples and 842 gyro samples. Both native and replay CSV sample counts and
  timestamps match. IMU brackets every image. No left/right frame-index gaps;
  maximum stereo skew 1 us. Observed image rate 29.96 Hz and IMU rate 202.61 Hz.
  These are smoke-capture timing measurements, not a 15W performance benchmark;
  the smoke ran in MAXN_SUPER and some compilation overlapped the capture.
- Integrity check: `reports/diagnostic-record-check-smoke.json`.
  The expected warning notes that four seconds does not complete the full
  10/60/10-second protocol. Total dataset size about 115 MiB.
- Stereo replay accepted the PGM images and tracked 116/116 frames:
  `reports/diagnostic-record-replay-stereo.log`.
- Inertial replay read the same dataset with `--allow-provisional` and exited 1
  after insufficient acceleration, without a usable map. This is expected for
  a short stationary smoke and does not validate VIO. Its log and exit status
  are in `reports/diagnostic-record-replay-inertial.*`.
- Actual Cube + QR-idle startup reached ARMED. No QR was presented; it exited 1
  after a three-second wait, preserved an INCOMPLETE dataset, and sent ready
  and error cues. Cube summary confirms navigation_enabled=false, zero odometry,
  zero parameter writes, zero link drops and zero serial CRC errors:
  `runs/cube-record/20261025T074527.758142972Z/bridge/summary.json`.
  Dataset: `recordings/diagnostic/20261025T074527.901010042Z/`.
- Systemd verified `deploy/systemd/bhu-vio-record.service`.
  `bin/vio-test boot-record` installed/enabled the recording unit and disabled
  the previous `bhu-cube-vio-test.service` boot unit. User lingering remains yes.
  The recording unit is inactive until started manually or on the next boot.

## Recording protocol

After the ready cue, the test_vio QR starts recording. The start cue requests
10 seconds stationary; the ascending move cue requests 60 seconds of smooth
translations and rotations; the low settle cue requests 10 seconds stationary.
Capture stops automatically at approximately 80 seconds. Re-presenting the QR
can stop early; metadata explicitly marks shortened protocols. Cube tones and
QR debounce logic are reused from the previously exercised workflow.

The full QR-triggered diagnostic recording, its IMU preroll, phase cues and
80-second automatic stop have not yet been exercised as a physical moving run.
Battery boot of this new recording unit is also pending; the shared device-wait
and Cube startup path were validated in the preceding VIO battery workflow.

The current Jetson mode at setup is MAXN_SUPER. Set 15W before using the default
recording service. A different profile requires an explicit named comparison
argument; it is always recorded in metadata. No Cube navigation parameters or
source selection were changed by this recording-mode work.

User instructions and output layout: `docs/vio-diagnostic-recording.md`.
