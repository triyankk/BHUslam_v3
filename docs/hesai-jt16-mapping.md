> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> Mapping supplies scene evidence and supports localization evaluation toward that goal. See the [shared project goal](../PROJECT_GOAL.md).

# Hesai JT16 configuration assessment

Inspected 2026-11-17. EKF fusion work is deferred in [the saved checklist](ekf-verification-deferred.md). This assessment did not change sensor firmware/settings, serial ownership, existing services, Cube parameters or power mode.

## Current connection and working software

The JT16 is already attached through a Prolific USB serial adapter, serial `DCCEb114J19`, using `/dev/jt16_usb` (currently `/dev/ttyUSB0`). Ethernet has no carrier. The running C++ owner is:

```
/home/atas/vscode/optFlow_slam/build/jt16_bridge/optflow-jt16-bridge
  --device /dev/jt16_usb --baud 3000000
  --correction /home/atas/vscode/optFlow_slam/data/calibrations/jt16/JT16_sample_angle-2.csv
  --startup-timeout 5 --viewer-port 18776
```

It is a child of the active `optflow-flight-logger.service` runtime in that checkout. It uses the official C++ Hesai SDK v2.0.12, pinned revision `534c707846a810e8211b93446f878dbf415f7000`; the checked-out revision matches. Reuse its acquisition rather than opening a second serial reader. Existing Python orchestration in that other project was not introduced or modified here.

The SDK sample default of 3,125,000 baud is not the correct general assumption: Hesai's manual assigns that speed to software versions 00.B0.1/00.B0.5, and 3,000,000 to other versions. Keep the currently working 3,000,000 rate; firmware identity still needs a readback before making device configuration changes.

Only the LiDAR USB adapter was present among the project sensors in this snapshot; Gemini and Cube USB devices were not visible. No simultaneous camera/LiDAR mapping test was attempted.

## Read-only reception check

A new standalone C++ observer received the existing nonblocking loopback viewer feed for 20 seconds, without touching serial hardware or the owning process:

| Measurement | Result |
|---|---:|
| Complete clouds | 99 |
| Cloud rate | 4.989 Hz |
| Median original points per cloud | 9,584 |
| Cloud interarrival p95 / maximum | 203.38 / 204.19 ms |
| Delivered internal-IMU callbacks | 4,042, about 202.11 Hz |
| Malformed side-feed datagrams / incomplete clouds | 0 / 0 |
| Side-feed frame/callback index gaps | 0 / 0 |

This is a local decoded-feed check, not a raw LiDAR packet-loss or clock-synchronization audit. The SDK can suppress identical IMU values. The viewer feed retains only XYZ/intensity and drops per-point timestamps/rings. Its small delivery latency is measured after SDK decoding and does not establish sensor acquisition latency.

Evidence: [raw measurements](../reports/jt16-context-20261117/shared-feed.json). Native observer source: `tools/jt16/inspect_feed.cpp`; standalone CMake project: `tools/jt16/`. It binds exclusively to loopback port 18776 and exits if another listener owns that port. Never run it alongside a viewer that needs the same feed port.

The Jetson currently reports **MAXN_SUPER**, not 15 W. Combined VINS/driver/mapping performance needs a later 15 W test; the rates above do not establish battery-mode performance.

## JT16-only implementation now available

The user subsequently selected LiDAR-only mapping instead of the VINS-driven geometry mapper below. See [the implemented C++ workflow](jt16-only-mapping.md) and [startup result](../reports/jt16-mapping-20261117/review.md). The VINS-driven steps below remain an earlier proposal, not the active mapping implementation.

## Earlier proposed VINS-driven mapping configuration

Use VINS-Fusion for pose estimation and JT16 for geometric map measurements. This produces a denser environment map; it does not itself correct VINS drift or tracking loss.

1. Retain the working RS485 transport, current scan cadence, one C++ SDK owner and raw point fields. Record the complete clouds and internal IMU with both sensor and host-monotonic timestamps.
2. Use the existing bridge's full mapping stream (`--mapping-port 18777`) for point timestamps, ring and confidence. This option exists in source but is **not enabled** in the inspected running process. Enabling it later needs an orderly change to that owner, not a second serial reader. The viewer stream is not an adequate mapping/deskew input.
3. Verify the angle table against this device. The current file is Hesai's published sample table, with historical wall/plane checks in the other checkout; it is not a serial-number calibration certificate. Do not promote old verification flags into fresh metric-mapping evidence. The existing driver does not have a working command-port connection (`rs232_com=/dev/null`), so device queries may need the separate supported control connection.
4. Measure the rigid LiDAR-to-Gemini transform and the current vehicle mount. The other project's saved 10 cm-above-CG / 180-degree mount is historical and must not be silently applied to this assembly. Preserve the SDK sensor axes until applying an explicit, checked transform.
5. Establish the LiDAR/camera/IMU clock relationship and verify per-point firing-time precision before deskew. Both streams having host receipt timestamps does not establish capture synchronization. Preserve raw clocks, estimate offset/drift and quantify residual timing error.
6. Convert the SDK v2.0.12 IMU scale explicitly: g to m/s², degrees/s to radians/s. Verify axes, stationary gravity, gyro bias and noise before treating the internal IMU as a calibrated navigation sensor.
7. Start with 0.20 m map voxels, a 0.30–30 m trial range filter, a measured airframe mask and bounded local-map memory. These are trial engineering choices, not factory specifications. Record unfiltered measurements so the filters can be changed on replay.
8. Interpolate/predict trustworthy VINS poses to scan/point times, deskew each scan and transform points into the per-flight local map. A 5 Hz scan spans about 0.2 seconds: a single pose for the entire moving scan can visibly bend walls. A gated or 10 Hz Cube forwarding stream must not be reused as the only deskew pose source.
9. Pause map integration during invalid/stale VIO and split epochs on pose resets. Do not place new clouds using the last good pose after motion continues. Save raw input and the validity history alongside each per-flight PCD/PLY map and trajectory.

The advisory machine-readable profile is [jt16-mapping-proposed.yaml](../configs/jt16-mapping-proposed.yaml). No runtime consumes that file yet; it has not been applied.

## If LiDAR should also stabilize the trajectory

A geometry mapper driven by VINS inherits VINS drift and gaps. To test LiDAR as an independent motion source, compare the C++ Hesai FAST-LIO2 implementation using JT16's built-in IMU against VINS on the same recording. Hesai provides a JT16 configuration and requires ring/per-point timestamps and correct IMU units. Verify its extrinsics, time alignment and unit-specific calibration first; existing identity-extrinsic/noise starting profiles are not calibration results. Full loop closure is a separate capability and has not been established by this driver or FAST-LIO setup.

Keep the first LIO comparison independent of Cube output. Evaluate stationary drift, known distances, loop return, scan alignment, motion distortion, tracking gaps, CPU/RAM and power at 15 W before considering estimator fusion or changing flight navigation. Combining correlated pose sources is a separate estimator-design task.

## Sources

- [Hesai JT16 manual J03-en-260120](https://www.hesaitech.com/wp-content/uploads/2026/01/JT16_User_Manual_J03-en-260120.pdf)
- [Official C++ SDK and model support](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0)
- [Official SDK serial input configuration](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0/blob/master/docs/parsing_lidar_data_online.md)
- [Built-in IMU documentation](https://hesaitechnology.github.io/dev/docs/how_to_guides/Get_IMU_data/)
- [Hesai FAST-LIO2 configuration and timing requirements](https://hesaitechnology.github.io/dev/docs/how_to_guides/fast-lio2-hesai-jt-quickstart/)

Locally inspected existing source and metadata: `optFlow_slam/native/jt16_bridge/{jt16_bridge.cc,viewer_stream.h}`, `scripts/fetch_hesai_sdk.sh`, `config/system.yaml`, `data/calibrations/jt16/source.json`. No files in that checkout were edited.
