> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> Shadow flights are an intermediate evidence-collection stage toward the GPS-free mission target. See the [shared project goal](../PROJECT_GOAL.md).

## Current pose timing policy — 18 November 2026

At the operator’s explicit request, live VINS forwarding delegates pose freshness to ArduCopter. The producer has no pose-age maximum or startup/recovery wait; velocity differences have no150 ms frame-gap cutoff. Both relay stages have no pose-age cutoff or10 Hz throttle. Capture timestamps and logged ages remain unchanged; latest-only queues and one-shot monotonic samples prevent replay. Feature support, IMU health, initialization, valid transforms/velocity, duplicate/future rejection, source-profile qualification, heartbeat and parameter-audit checks remain. ArduCopter keeps its300 ms VisualOdom reception watchdog and EKF timestamp/delay processing. No Cube parameters, source switching, recording state machine or buzzer behavior changed. Historical150 ms/500 ms/10 Hz descriptions below are superseded for this live path. This policy change is not EKF accuracy or flight validation.

# GPS-aided VIO and mapping observation flight

Current operator-selected hybrid profile: VIO horizontal position (POSXY6), optical-flow horizontal velocity (VELXY5), barometer height (POSZ1), VIO vertical velocity (VELZ6), internal compass yaw (YAW1, COMPASS_ENABLE1). Both bridge stages recognize vio_optflow_compass; unused sets2/3 remain cleared. GPS is disabled; no automatic source switching. [Current configuration and health evidence](../reports/vio-optflow-compass-20261118/review.md). The recovery-gate comparison found no benefit in the sampled window, so live qualification stays500ms. Older pure-VIO/GPS settings below are historical.

GPS-free source correction: unused sets 2/3 now use POSXY/VELXY/VELZ/YAW=0, POSZ=1, clearing their GPS pre-arm dependency. Primary source1 remains VIO. The native apply mode and Desktop parameter export include this cleanup. [Changes and verification](../reports/ek3-unused-source-fix-20261118/review.md). Refresh and re-save the transmitter profile.

Current bench configuration: at the operator's request, the GPS-free profile is now applied to the Cube for saving on the ground-control transmitter. GPS1_TYPE=0, COMPASS_ENABLE=0, source1 POSXY/VELXY/VELZ/YAW=6, POSZ=1, OPTIONS=0, VISO_TYPE=3. PRX1_TYPE=2 and recording/beeps are preserved. This supersedes the older GPS-aided configuration below. [Applied values, readback and GPS rollback](../reports/gps-free-apply-20261118/review.md).

Verified backend correction, 18 November 2026: JT16 messages are MAVLink DISTANCE_SENSOR, requiring PRX1_TYPE=2 (MAVLink). The earlier PRX1_TYPE=4 (RangeFinder) selection was wrong for this transport. PRX1_TYPE=2 is now read back after reboot and Cube proximity is healthy/stable. See [readback and health evidence](../reports/jt16-mavlink-backend-fix-20261118/review.md).

Latest activation, 18 November 2026: the operator explicitly requested replacing the forwarding-disabled override. `VISO_TYPE=3` is now saved, read back and active after a props-off disarmed reboot; source1 remains GPS POSXY/VELXY/VELZ=3, barometer POSZ=1, compass YAW=1, OPTIONS=0. The Cube accepted 31 real live position and 31 velocity records; all timestamps matched the serial log, and all 4,446 recorded XKFS source entries were primary. LOG_DISARMED was restored to 0. Current stationary output is intermittent (maximum recorded gap 36.325 s), so VisOdom health is not continuously good. [Current activation and reception evidence](../reports/gps-vio-reception-20261118/review.md). This supersedes older VISO_TYPE=0 statements below.

Current GPS recording profile, 18 November 2026: source set 1 is GPS position and horizontal/vertical velocity (`POSXY=3`, `VELXY=3`, `POSZ=1`, `VELZ=3`, `YAW=1`), with `EK3_SRC_OPTIONS=0` and `VISO_TYPE=0`. VINS/JT16 remain independent boot estimators with arm/disarm recording and the existing ten-beep recording-ready cue. No source-selection command or VIO fusion is enabled. Current Cube link uses the USB by-id endpoint; keep USB attached because TELEM2/GPIO transmit remains unresolved. All five units are enabled and active. See the [current configuration, rollback and unapplied compass-free VIO plan](../reports/gps-recording-20261118/review.md). Reboot before flying with the saved profile and resolve actual GPS/GCS/proximity faults.

Bridge update: the relay/serial owner now recognize the explicit GPS and compass-free VIO profiles, qualifying changes only while disarmed. Current VISO_TYPE=0 still blocks all forwarding; see [bridge verification](../reports/dual-profile-bridge-20261118/review.md).

The older configuration notes below describe earlier stages and do not override the current profile above.

Current proximity update, 2026-11-18: the operator requested restoring JT16/BendyRuler configuration. Only `PRX1_TYPE` was restored to 4 and read back; existing horizontal BendyRuler (4 m lookahead, 1.5 m margin) and all EKF/VIO settings were preserved. LiDAR power was disconnected and the operator waived scan testing. No Cube reboot was performed: power JT16 and reboot Cube to load the backend, then verify reception before relying on avoidance. See [configuration evidence](../reports/jt16-bendyruler-enable-20261118/review.md). Older proximity-disabled statements below describe the earlier state.

The Cube uses the operator's existing GPS-aided setup. Two independent background estimators are evaluated. The [later reception-only VINS profile](vins-observation-forwarding.md) forwards gate-qualified VIO poses for Cube reception/logging, while the active GPS/flow source set 1 and `EK3_SRC_OPTIONS=0` are preserved. No GPS denial, source switching, mission commands or movement commands are introduced. On 2026-11-18 the operator requested removal of the absent proximity sensor: only `PRX1_TYPE` was changed from 4 to 0 and the disarmed Cube rebooted. Cube proximity sensing is disabled for this profile. Rangefinder and avoidance tuning parameters were preserved. See `reports/cube-proximity-disabled-20261118/` for backup and verification.

## Automatic separation between flights

Connection update, 2026-11-18: the boot profile now uses the physically verified **TELEM2 -> Jetson pins 6/8/10 -> `/dev/ttyTHS1`, 115200 baud**. Cube USB is not required by boot startup. The unit environment overrides the existing checkout's saved USB endpoint without editing its base YAML. Missing/reconnecting LiDAR is recovered inside its own worker; Cube status and VIO remain active and LiDAR stays explicitly unhealthy until valid scans return; the Cube proximity backend remains disabled in the current profile. Mapping waits for the actual JT16 device. Boot units no longer order themselves after an owner that starts after `default.target`, avoiding the startup cycle found during the connected check.

For this GPS-aided observation phase, the Cube's `VISO_TYPE` was changed from 3 to 0 with a rollback and readback. Jetson VIO is still active; the Cube does not require or receive visual poses. All EKF source sets and other audited parameters were preserved. Before a later real external-navigation test, deliberately restore/configure the backend and verify fusion; this observation profile does not do so automatically. See `reports/cube-connection-audit-20261118/gps-observation-profile/`.

Updated on 2026-11-17: no Jetson access is required between normal flights. Boot starts and warms the estimators after Cube/camera readiness. A fresh Cube **armed** state opens a new set of recording files. A fresh **disarmed** state finalizes them after a three-second tail. Re-arming after finalization opens another flight folder; re-arming within the three-second tail continues the same recording. Keep Jetson powered during finalization. Telemetry loss is not treated as disarm, so recordings continue across link interruptions rather than closing falsely.

VIO and LiDAR estimation continue between flights to preserve their initialized state. Bulk camera/IMU input files, per-flight poses/maps and raw LiDAR packet files are arm-gated; lightweight boot diagnostics/performance can continue. VINS files are under `live/flights/flight-<MONOTONIC_US>/`; maps are under mapping `data/flights/flight-<MONOTONIC_US>/`. The independent readers observe arm within their polling intervals, so their IDs/times can differ slightly. Boot ID and host monotonic times associate the records; do not assume identical folder names.

For each new VIO recording, **ten rapid beeps** are requested only after a stereo pair and both accelerometer/gyro samples have been written and flushed successfully. The fixed cue goes through the existing Cube serial owner using a bounded localhost request/ACK protocol; no second Cube reader is started. Duplicate requests do not repeat the tune while that owner remains running. About 1.25 s of ten notes/rests are encoded using MAVLink `PLAY_TUNE`. Acknowledgement means the existing owner sent the tune, not that sound was heard. Retries expire after 30 s. The cue confirms recording, not tracking accuracy, mapping health or flight readiness. It is not a generic arm/mode-change tune.

Normal sequence: power up -> allow Cube/camera startup -> arm -> wait for the recording cue -> perform the GPS-aided test -> land/disarm -> allow finalization -> repeat. No SSH, display, QR or manual recording stop is needed between flights. Sound has not been verified on the absent Cube.

## Boot configuration

- `optflow-flight-logger.service`: retains sole ownership of Cube USB and JT16 serial. The native C++ JT16 reader has opt-in mapping/raw defaults plus an arm-state file from its existing owner; raw files open on arm and close after disarm/tail. A user-unit drop-in enables the full timestamped feed on loopback 18777 and the recording-cue listener on 14602. The existing Python serial owner has a small fixed-cue/arm-status hook; recording and estimator code remain C++. No new Python process or `config/system.yaml` changes were introduced.
- `bhu-vins-fusion.service`: waits for the exact Gemini USB device and configured Cube UART and newly updated, connected, disarmed Cube heartbeat status. Runs continuous stereo/IMU VINS-Fusion, mild CLAHE clip 2, four OpenCV threads, unchanged 0.5 s / 40 feature gate. Gated UDP output feeds the native reception-only relay; the existing UART owner performs the final write. Saves native poses, timing/support diagnostics and replayable selected stereo pairs/full delivered IMU samples.
- `bhu-vio-observe.service`: native ODOMETRY-only observation filter and read-only source guard, 10 Hz maximum handoff, logging actual owner writes separately from reception proof.
- `bhu-jt16-map.service`: independently waits for a live disarmed Cube heartbeat, then runs the JT16 built-in-IMU FAST-LIO2 mapper using only the shared feed. Gemini/VINS is not an input. It does not open Cube or LiDAR serial. Shared-mode failure never stops the acquisition/proximity service.
- `bhu-shadow-performance.service`: after the same Cube startup condition, records tegrastats once per second.

All units start without login using existing user lingering. VIO/mapping initialization does not assert healthy Cube EKF fusion. After startup, arming does not stop or reset either estimator. If an estimator crashes, its separate service can restart into a new run/map; a fresh disarmed Cube is required before the restarted estimator begins. Consequently an armed crash may leave it waiting until landing. Other observation and flight-control paths continue independently.

## Evidence locations

| Evidence | Location |
| --- | --- |
| VINS per-flight poses, timing and input recording | `runs/vins-fusion-boot/<UTC>/live/flights/flight-*/` |
| VINS boot association | `runs/vins-fusion-boot/<UTC>/boot-id.txt` |
| JT16 per-flight map, trajectory, SI IMU and scan timing | `runs/flight-shadow/<BOOT_ID>/mapping-<UTC>/data/flights/flight-*/` |
| Complete map checkpoints | Mapping `data/checkpoints/<NUMBER>/` |
| JT16 original serial packet bytes | `/home/atas/vscode/optFlow_slam/data/recordings/jt16-shadow/<BOOT_ID>/capture-*.raw` |
| Jetson performance | `runs/flight-shadow/<BOOT_ID>/tegrastats.log` |
| Existing Cube GPS/local-position recording | `/home/atas/Desktop/Drone_Sensor_Recordings/<SESSION>/cube_mavlink.csv` |
| Independent autopilot flight evidence | Matching onboard Cube DataFlash `.BIN`, download separately |

The existing Desktop recorder starts on an observed disarmed-to-armed transition and keeps a short tail after disarm. Verify its recorder thread/health when hardware is available; saving Cube telemetry is not guaranteed merely by enabling a unit. Keep the matching Cube BIN for GPS quality, EKF source, innovations, vibration and other aircraft evidence.

Camera `input/` contains rectified **selected processed pairs before CLAHE**, uncompressed PGM, `frames.csv`, `capture.csv`, `accel.csv`, `gyro.csv` and `orb.yaml`. It does not preserve camera pairs overwritten before processing or native unrectified pixels. Acceleration is SI rotated into the gyro frame; gyro is SI. A native asynchronous writer has bounded queues; overflow/write errors are reported in `status.json`, final recording metadata and VINS summary. Below 2 GiB free it stops input recording and reports an error while VIO can continue. At 20 processed pairs/s, stereo images alone consume about 0.61 GB/min; check disk capacity before a long run. Replaying these selected pairs can inspect tracking/geometry but cannot recreate the original live CPU/callback scheduling.

JT16 produces about 5 scans/s in prior captures. The exported map is capped at 250,000 voxels, with overflow counts, and viewer trajectory at 50,000 poses; full trajectory continues to CSV. Continuous mode disables the backend's additional growing PCD save buffer and exports the adapter's capped PLY map. The first checkpoint occurs after poses are available and five seconds have elapsed, then approximately every 30 seconds. Completed generations are separately saved; an interrupted `.tmp` generation is ignored. No loop closure or cross-sensor VINS/LIO fusion is enabled. Sensor timestamps and frames remain independent and provisionally calibrated.

## Test and post-flight operation

```bash
cd /home/atas/BHUslamV3
bin/flight-shadow status

# After landing/disarming, while Jetson power is still available:
bin/flight-shadow stop
bin/flight-shadow review
```

Review writes `reports/flight-shadow/<BOOT_ID>/index.html`, with VINS 3D trajectory, JT16 3D map links and a machine-readable summary. After reboot, supply the prior boot ID: `bin/flight-shadow review BOOT_ID`. An optional workspace argument to the Node review tool supports offline review of a copied workspace. Retain input recordings and raw packets for later replay.

Disarm now automatically finalizes flight recordings, so the manual stop above is optional when finishing the entire test day. Prefer graceful shutdown after finalization when practical. VINS CSVs are flushed periodically and mapping keeps checkpoints, but abrupt power loss can lose recent buffered samples, final summaries or the last map generation. Checkpoint file contents are synced; storage/controller power-loss behaviour and directory durability are not guaranteed. A recovered checkpoint may precede touchdown by tens of seconds. Raw JT16 packet files have no added per-packet host timestamps; mapping scan/IMU logs supply timing evidence. Frame/clock alignment must be established before comparing VINS, LIO and Cube GPS trajectories as accuracy measurements.

`stop` leaves the existing proximity/acquisition service running, with raw capture still controlled by Cube arm/disarm. It does not disable future boot startup. With Cube/Gemini absent, units waiting for them is expected; a green systemd active state is not estimator health. The current software setup was made without camera/Cube attached, so a connected ground check, combined 15 W test and actual flight results remain outstanding. Power mode is logged, never changed automatically.

## Later simulation phase

After reviewing this GPS-aided flight, test pose serialization, coordinate/lever-arm transforms, quality/freshness gates and reset epochs offline. Then test forwarding and EKF acceptance in an isolated ArduPilot SITL setup with compatible simulated vehicle motion and estimator observations. Feeding a moving recorded trajectory into an otherwise stationary vehicle is not a valid accuracy/fusion test. Check selected EKF sources, innovations/rejections and accepted pose continuity; do not infer fusion from packet delivery. Real Cube acceptance will still need a separate disarmed hardware check afterward. No simulation transport is enabled by this boot profile.

## Install/rollback

`bin/flight-shadow install` refuses an attached Cube, saves prior service definitions/enabled states under `runs/flight-shadow/install-*`, installs observation units and the native-reader environment drop-in, then restarts acquisition. It must be run with host systemd access; this is not an in-flight operation.

To remove the added mapping/raw side feed during a later hardware-absent setup: stop/disable `bhu-jt16-map.service` and `bhu-shadow-performance.service`, remove only `~/.config/systemd/user/optflow-flight-logger.service.d/50-bhu-jt16-shadow.conf`, reload systemd and restart the original owner. The optional native environment support has no effect when those environment variables are absent. The previous native source/binary and its diff are retained in `reports/flight-shadow-20261117/`. Restore other desired unit states individually from the saved list. Do not blanket-restore aircraft parameters or modify Cube safety/proximity settings.
