> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> Dataset collection supports repeatable localization and sensor diagnosis for that goal. See the [shared project goal](../PROJECT_GOAL.md).

# Automatic two-minute camera recording

This mode records a dataset for offline diagnosis of VIO resets. It starts
automatically after boot and records for 120 seconds from the first valid stereo
frame. It runs native C++ capture without QR detection, SLAM or navigation poses.
The Cube bridge runs with `--feedback-only`: it does not bind the odometry input
socket. Cube parameter settings and navigation source selection are unchanged.

## Battery or display-free use

`bin/vio-test boot-record` selects diagnostic recording for the next boot and
disables the previous VIO-test boot unit. The installed user unit is
`bhu-vio-record.service`. User lingering and the persistent USB permissions from
the earlier battery test are reused. No recording starts just by selecting it.

Before shutting down for a battery run:

```bash
bin/vio-test power-15w
sudo poweroff
```

Keep Cube USB and Gemini USB connected. On battery startup, no login, display
or network is required. Startup waits for the devices and the disarmed Cube
link. **Three beeps** mean recording has started. No QR or switch is required.

| Cue | Action |
| --- | --- |
| Three distinct beeps | Recording started; two-minute timer is running |
| Three distinct beeps after two minutes | Recording stopped and capture data synced to storage |
| Six rapid low notes | Error; inspect logs |

There are no intermediate movement prompts. Each boot/start performs one
120-second recording, saves it, then exits without restarting. The final three
beeps follow capture-file flushing and filesystem synchronization; they do not
power Linux off. Shut down normally before disconnecting the battery. An early
service stop preserves data but marks the requested timed duration incomplete.

To start manually without reboot, use
`systemctl --user start bhu-vio-record.service`. For a terminal invocation use
`bin/vio-test cube-record`. The default requires 15W; an explicitly labelled
comparison can use `--power-profile MAXN_SUPER`.

To restore the previous VIO test at boot: `bin/vio-test boot-vio`.
To disable automatic recording:
`systemctl --user disable --now bhu-vio-record.service`.

## Saved data

Datasets are in `recordings/diagnostic/TIMESTAMP/`:

- `native_left/`, `native_right/`: native Y8 images before software rectification.
- `left/`, `right/`: rectified images, directly readable by `gemini_replay`.
- Images are lossless, uncompressed PGM grayscale. This avoids PNG compression
  work during capture. A 120-second run at 640x400/30 FPS uses about 3.7 GB.
- `accel_native.csv`, `gyro_native.csv`: SDK values in native sensor axes and
  SI units, with device microseconds, SDK system microseconds, host monotonic
  arrival microseconds and SDK index. These are SDK measurements, not ADC data.
- `accel.csv`, `gyro.csv`: timestamped SI measurements for replay, with
  accelerations rotated into gyro axes. No synchronization/interpolation is
  applied to the saved samples. Capture includes up to 0.5 s of IMU preroll and
  0.2 s of trailing IMU capture to bracket images.
- `frames.csv`: stereo device timestamps, image paths and per-stream indices.
- `capture.csv`: image system timestamps, host receive time and file-write time.
- `factory.yaml`: stereo calibration, rectification, camera/IMU transforms,
  SDK IMU intrinsic matrices/biases and factory noise fields. Factory noise
  fields are preserved as evidence, not substituted for measured calibration.
- `orb.yaml`: replay configuration; IMU noise/time offset remain provisional.
- `recording.json`: mode, power profile, duration, sample counts, frame gaps,
  stop reason, completion status and camera properties.
- `events.jsonl`: start/stop/saved host timestamps.
- `recording-check.json`: automatic offline integrity/timing check; it does not
  claim VIO success. Inspect its warnings, including interrupted timed capture.
- `console.log`, binary hash and boot ID identify the run and executable.

Cube logs are in `runs/cube-record/TIMESTAMP/`, with boot ID, service invocation,
`bridge/summary.json`, cue events and incoming telemetry. Bridge summaries must
show `navigation_enabled=false` and `odometry_forwarded=0`. The companion
`live.console.log` identifies the dataset path. Boot messages append to
`runs/record-boot.log`.

Capture checks estimated required storage plus a 1 GiB reserve before starting,
and stops on a low-storage condition. Write failures, missing images, malformed
timestamps and native/replay sample-count mismatches are not silent successes.
Interrupted/error recordings retain their data and completion status.

## Replay and checks

```bash
bin/vio-test record-check recordings/diagnostic/RUN
bin/bhu replay recordings/diagnostic/RUN third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt
bin/bhu replay recordings/diagnostic/RUN third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  --inertial --allow-provisional
```

Replay never sends navigation data to the Cube. Stereo and inertial replay use
separate default output directories. This dataset allows comparisons on the
same images and measurements. The diagnostic capture itself is not a VIO
performance benchmark and does not produce a live trajectory; trajectories
come from subsequent replay.
