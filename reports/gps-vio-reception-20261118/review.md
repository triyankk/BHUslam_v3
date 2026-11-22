# GPS-aided VIO recording and Cube reception — 18 November 2026

Configured and verified at the operator's explicit request. The earlier forwarding-disabled override is superseded. Propeller removal was confirmed, and native filtered autopilot heartbeats confirmed disarm before writes and reboot. Cube remains on GPS navigation; this is not a VIO fusion/source-switch test.

## Applied and persistent state

Only persistent Cube change: **VISO_TYPE 0 -> 3**, saved/read back and followed by a disarmed Cube reboot. Primary source set 1 remains GPS horizontal position and XYZ velocity, barometric height and compass yaw: POSXY=3, VELXY=3, POSZ=1, VELZ=3, YAW=1, with EK3_SRC_OPTIONS=0. GPS1_TYPE=9, GPS2_TYPE=0 and COMPASS_ENABLE=1 remain unchanged. No automatic source switch, arming, mode, mission or motor command was sent.

The existing native parameter tool's observation-only mode now audits the exact GPS observation contract used by the dual-profile bridge instead of the superseded GPS/flow hybrid. It changes only VISO_TYPE, with before/requested/after snapshots and rollback. Its new scoped `--disarmed-log 0/1` mode changes only LOG_DISARMED. Updated simulated-Cube tests passed in 16.41 s, including both log toggles and unrelated-parameter preservation.

For receipt evidence, LOG_DISARMED was temporarily 0 -> 1 before reboot, then restored to **0** after capture. Independent after-audit found only VISO_TYPE changed among common scoped parameters; log backend/mask, source sets 2/3, mounting, rangefinders, proximity, RC and notification settings were preserved. The resulting profile is `gps_observation` in both relay and final owner status. The complete current parameter file is `gps-aided-vio.param`; it does not switch EKF sources.

## Actual Cube reception proved

The Cube's own DataFlash log 207 was downloaded (4,092,711 bytes) using the existing disarmed log-transfer helper; no new Python runtime or hardware-owner process was introduced. Native C++ offline review decoded the log without missing/truncated bytes:

- **31 VISP position records and 31 VISV velocity records**.
- **All 62 records accepted** (`Ign=0`), with pose quality 67–100; no below-threshold poses.
- Every position and velocity `RTimeUS` matches one of the **31 actual serial-write capture timestamps** in the existing owner's log.
- **4,446 XKFS records**, all `SS=0`: source set 1 remained selected in the recorded cores/samples. This is recorded source-selection evidence, not merely parameter inference.
- The recorded source parameters match GPS navigation with VISO_TYPE=3 and OPTIONS=0.

This proves Cube frontend position/velocity reception for real live VINS output. It does not prove EKF fusion of VIO; source1 still selects GPS, barometer and compass. No synthetic/replayed pose, fake health or stale timestamp was sent.

## Health result in the requested stationary view

The operator explicitly chose continuing the stationary view. During the 60-s sampled window the Cube was always disarmed; the bridge's parameter audit remained fresh (maximum age 3580.02 ms). Twenty-two new serial writes occurred between the first and last sampled counters; including output before/after that window there were 31 writes in the downloaded capture. Cube vision health was true in 6/60 one-second samples.

Publication is **intermittent and not healthy continuously**: stationary stereo support ranged 13–32 matches, while the gate requires 20 continuously alongside other checks for 500 ms. Of the 60 sampled gate states, 20 reported insufficient visual support, 36 were qualifying, 3 hard-guard and 1 eligible. One-second sampling is a diagnostic snapshot, not a frame-level rejection distribution. Real accepted messages spanned 99.35 s, with maximum gap **36.325 s** and 16 gaps greater than 300 ms. The observed reset-counter value in this log was consistently 1. Image support/freshness therefore remains a flight-reliability gap; transport delivery is now proven.

The gates and thresholds were not relaxed. `VisOdom: not healthy` is a genuine expected consequence of these gaps. Final owner pre-arm history also included `PRX1: No Data` and GCS failsafe; `GCS Failsafe Cleared` was the latest text after the log-transfer tool's GCS heartbeats. LiDAR decoded near 4.98 scans/s, but this does not establish Cube proximity health or obstacle-avoidance readiness. GPS had no fix during the audit. No arming check/failsafe was disabled to conceal any fault.

## Recording, ownership and boot

All five services are active and boot-enabled: existing sole Cube/JT16 owner, native relay, native VINS, independent JT16 mapper and performance logger. USB remains the configured Cube link; GPIO transmit is unresolved. VINS stayed warm. The mapper was paused during exclusive log transfer to prevent its restart/Wants dependency from reopening the Cube owner, then restored. No flight recording was active during these disarmed operations.

VIO/JT16 recording still starts on arm and closes after fresh disarm plus the existing 3-s tail. Ten rapid Cube recording-ready beeps still occur after first stereo/IMU files are flushed; the code was not changed and no arm/beep test was triggered here. Reception logging is independent from recording readiness. Current recording thread is healthy and auto-record-on-arm is true. Leave power on through finalization.

## Evidence

- `before-audit.json`, `after-audit.json`, `preservation.json`: independent source/readback preservation.
- `observation-enable/`: VISO-only before/requested/after/changes, summary and rollback.
- `log-enable/`, `log-restore/`: temporary logging changes and restoration evidence.
- `stationary-check.jsonl`, `stationary-summary.json`: 60-s live diagnostic observation.
- `owner-reception-log.jsonl`, `timestamp-correlation.json`: actual serial writes and per-packet Cube matches.
- `cube-bin/cube-log-207.BIN`, `cube-review/summary.json`, `cube-review/cube-extnav.jsonl`: original Cube evidence and C++ decoding.
- `reception-timing.json`: accepted-packet timing, gaps, quality and reset values.
- `status-before.json`, `status-after-check.json`, `status-final.json`, `native-after-check.json`: live state and final profile.
- `integration.log`, `build.log`, `config-release.sha256`: narrow-tool test/build evidence.

Firmware references: [Copter 4.7.1 visual-odometry backend selection and reboot requirement](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_VisualOdom/AP_VisualOdom.cpp), [quality-gated position/velocity logging](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp). Validation of camera/IMU calibration, motor-on tracking, north/origin alignment, VIO fusion, tracking-loss response and GPS-free flight remain separate.
