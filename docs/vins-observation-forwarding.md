> 20 November update: the authorized fresh-GPS bootstrap adds an exact hybrid profile with GPS1_TYPE=9; see [current bootstrap procedure](gps-bootstrap.md). This supersedes older fixed-profile descriptions below.

> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> Observation forwarding is an integration step toward verified GPS-denied flight control. See the [shared project goal](../PROJECT_GOAL.md).

## Current pose timing policy — 18 November 2026

At the operator’s explicit request, live VINS forwarding delegates pose freshness to ArduCopter. The producer has no pose-age maximum or startup/recovery wait; velocity differences have no150 ms frame-gap cutoff. Both relay stages have no pose-age cutoff or10 Hz throttle. Capture timestamps and logged ages remain unchanged; latest-only queues and one-shot monotonic samples prevent replay. Feature support, IMU health, initialization, valid transforms/velocity, duplicate/future rejection, source-profile qualification, heartbeat and parameter-audit checks remain. ArduCopter keeps its300 ms VisualOdom reception watchdog and EKF timestamp/delay processing. No Cube parameters, source switching, recording state machine or buzzer behavior changed. Historical150 ms/500 ms/10 Hz descriptions below are superseded for this live path. This policy change is not EKF accuracy or flight validation.

# VINS-Fusion profile-guarded forwarding

Current activation: VISO_TYPE=3 is enabled/rebooted with GPS sources intact. Actual Cube receipt is proven by 31 accepted VISP and 31 accepted VISV records with matching capture timestamps; source1 remained primary. Current stationary VIO publication is intermittent; health is not continuously good. See [current reception report](../reports/gps-vio-reception-20261118/review.md). Earlier forwarding-disabled state below is superseded.

Current bridge update, 18 November 2026: both the native relay and sole-owner adapter support exact GPS-observation and GPS-free VIO profiles. The current GPS recording profile remains blocked with VISO_TYPE=0. See [dual-profile contract and verification](../reports/dual-profile-bridge-20261118/review.md). The earlier fixed GPS/flow source guard below is superseded. No Cube parameters were changed by this bridge update.

Enabled at the operator's request on 2026-11-18. This profile transmits only poses passing the existing VINS publication gate. It does not validate calibration, accuracy, EKF fusion, or source switching.

## Cube configuration

The disarmed live audit confirmed source set 1 is **GPS horizontal position plus optical-flow horizontal velocity**, barometer height, no vertical velocity source, compass yaw: `EK3_SRC1_POSXY=3`, `VELXY=5`, `POSZ=1`, `VELZ=0`, `YAW=1`. These values and `EK3_SRC_OPTIONS=0` were preserved. No change to GPS velocity is required for reception-only observation.

The sole applied Cube parameter change was `VISO_TYPE: 0 -> 3`, reported before applying, with a disarmed reboot and independent readback. The rollback is `reports/vins-forwarding-20261118/observation-profile/restore.param`. Source set 2 already has ExternalNav position configured; it remains inactive at boot and was not changed. All 16 RC option parameters were read; none was 90 (source selector). `SCR_ENABLE=0`. Firmware initializes the primary source set at boot. Neither transport sends a source-switch command. Do not select another source through the GCS during this observation experiment. Confirm actual per-core source selection after flight using `XKFS.SS=0`; source parameters alone are not active-source telemetry.

Enabling the visual backend can cause `VisOdom: not healthy` or attitude mismatch pre-arm faults even during GPS-aided operation. No check is disabled or quality threshold reduced to remove them. The initial `LOG_BACKEND_TYPE=1`, `LOG_BITMASK=180222`, `LOG_DISARMED=0` were preserved: retain the matching armed-flight Cube BIN. No disarmed BIN reception evidence is promised.

## Measured mount update, 2026-11-18

The operator reports a 22.8-degree downward camera pitch, optical centre 0.13 m forward and 0.05 m below the Cube. Lateral offset and additional roll/yaw are assumed zero. `configs/vins-fusion-extnav.yaml` now contains `R_body_optical = R_y(-22.8 degrees) * R_upright`, so optical forward points forward/down in body FRD. The existing C++ conversion removes this mount from the outgoing vehicle attitude and rotates angular/body velocity consistently. The SDK camera/IMU intrinsics and internal extrinsics are unchanged.

Only Cube `VISO_POS_X` changed 0.15 -> 0.13 and `VISO_POS_Z` changed 0 -> 0.05; Y remains 0. Position remains the camera origin in ODOMETRY and the Cube applies its lever arm once. `VISO_ORIENT=0` remains unchanged because attitude is already body-aligned by C++. Source configuration, gates, recording and beeps are unchanged. Mount-only backup/readback and transform tests are in `reports/vins-mount-20261118/`. Physical dimensions supplied by the operator are not complete camera/IMU timing or noise calibration. New boot runs save `navigation-mount.yaml` and `camera.yaml` beside the run metadata.

## Runtime

`bhu-vins-fusion.service` still waits for Gemini and a fresh disarmed Cube at startup, then stays warm across arm/disarm. Its existing C++ gate requires 500 ms sustained initialized support, at least 40 persistent features, 20 backend tracks and 20 stereo matches, finite motion and valid velocity, maximum capture age 150 ms. Mild CLAHE remains 2. The pose goes to localhost UDP 14600.

### Health corrections after the operator's GPS-flight override, 2026-11-18

The operator disabled forwarding (`VISO_TYPE=0`) and changed EKF selection for GPS flight. The health-fix work preserves that override. Do not automatically restore the earlier source configuration or enable forwarding. The source guard intentionally remains blocked until a separately authorized, verified configuration is applied.

The native live wrapper now selects 200 features, 25 px minimum spacing, and the rectified stereo policy. Reverse LK starts at the known left observation; the existing 0.5 px round-trip check remains, with positive disparity and <=1 px vertical error required. Explicit experiment flags override these defaults. Upstream VINS sources are unchanged; CMake builds a checked generated tracker copy. Initial/recovery qualification remains 500 ms and all support thresholds remain unchanged.

`DurationGate` separates a temporary publication recovery from a coordinate-frame reset. It checks all usable estimator poses, including poses withheld for weak visual support. An unchanged epoch and continuous observed poses preserve `reset_counter`. Epoch changes, position/attitude jumps, or an unobserved usable-pose interval >1 s mark a reset when qualification completes. Provisional continuity bounds are position prediction residual >0.20 m + 5*dt^2 and attitude prediction residual >15 degrees + 10*dt^2 radians. These are engineering guards, not calibrated uncertainty or a tracking guarantee.

Near-rail camera IMU measurements now withhold output for 200 ms and are counted in status. The attached camera advertises only +/-4 g acceleration and +/-1000 deg/s gyro ranges, although it supports additional sample rates. No unsupported range, IMU rate, calibration or noise change was made. Clipped motion cannot be repaired by filtering recorded values. The original flight reached the digital acceleration rails on 1,967/42,690 samples (4.6%); 2,043 samples (4.8%) were within 1% of a rail; its live pose velocity was already invalid before arm and all recorded flight frames were withheld. Stereo tuning alone cannot fix this.

If initialized VINS produces invalid pose velocity continuously for two seconds with adequate support and no recent clipping, a watchdog can clear and reinitialize the estimator only when the existing Cube status is freshly confirmed disarmed and recording is inactive (including its three-second tail). Reset attempts have a ten-second cooldown. It never resets the estimator while armed, on stale/disconnected Cube status, or during recording. Recording and cue state are independent. Events are saved to `estimator_events.jsonl`, and a new estimator epoch is announced only after qualification.

`status.json` now exposes the active frontend profile, support/hard rejection masks, pose speed, explicit gate reason, IMU limit counts, publication recoveries, reset causes and disarmed reinitialization count. Tracking CSV includes matching diagnostics. Hard rejection bits are initialization=1, alignment=2, invalid pose=4, velocity=8, stale=16, future=32, unhealthy IMU=64. Support bits are persistent features=1, backend tracks=2, stereo matches=4. Reset cause bits are epoch=1, position jump=2, attitude jump=4, unverified interval=8.

Genuine support/clipping gaps still produce VisOdom unhealthy if forwarding is later enabled: [Copter 4.7.1 requires a visual update within 300 ms](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_VisualOdom/AP_VisualOdom_Backend.cpp). No stale pose, synthetic heartbeat pose, reduced quality threshold or disabled arming check is used to hide this condition. See `reports/vins-health-fix-20261118/review.md` for measured results and limits.

`bhu-vio-observe.service` runs native `cube_vio_observe`, which reuses the native packet filter, accepts only one complete unsigned MAVLink2 ODOMETRY per datagram from component 197, local-FRD/body-FRD, VIO estimator, quality 40–100, valid quaternion/finite state, monotonic capture time and age at most 150 ms. It keeps latest data and caps handoff at 10 Hz. It sends localhost 14603, never opens UART. Incoming Cube telemetry arrives on localhost 14604.

The existing `optflow-flight-logger.service` remains the sole `/dev/ttyTHS1` owner at 115200 baud. Its explicit opt-in `OPTFLOW_VIO_OBSERVATION=1` enables the narrow adapter in its own repository. No new Python process is introduced; estimator and native filtering remain C++. This adapter rechecks packet integrity, identity, timestamps, finite pose, quality, fresh heartbeat, and source configuration before the shared UART write. It retains the newest pending pose between 10 Hz writes and never replays old poses. The fixed read-only parameter audit refreshes the oldest reply first, retrying lost replies without relaxing its five-second freshness limit. Unexpected source-set-1 values or `EK3_SRC_OPTIONS!=0` block output. A reported non-primary source selection latches output off until restart. Startup/reconnection requires observing a disarmed heartbeat; subsequent arming does not stop this observation path.

No parameter-write, source-switch, mode, arm, mission or movement interface is exposed by the forwarding adapter. The legacy disarmed-only `cube_vio_bridge` is unchanged and is not used here. Factory/provisional camera calibration and operator-reported mount geometry require physical validation before control use.

## Recording and beeps

The recording state machine and cue protocol are unchanged. Arming opens a new flight recording; disarm finalizes after the three-second tail. Re-arm inside that tail continues the same session. Ten rapid Cube beeps occur only after the first stereo pair and both IMU streams have been flushed. Pose forwarding does not trigger a recording cue, and a cue does not assert healthy VIO. No new flight beep or source-switch automation was added.

## Evidence and review

- `runs/vins-forwarding/<boot-id>/<UTC>/events.jsonl` and atomic `status.json`: C++ input, rejection, guard and handoff health; Cube status and vision-health proxy. Check freshness on saved status files.
- Existing owner's `data/recordings/vio-observation/<UTC>.jsonl`: boot ID, parameter changes, actual serial-write count, capture timestamps, age at write, write latency, quality, reset counter, armed state and Cube vision-health samples. The file path and current counters are in `cube.vio_observation` of the existing owner status.
- Existing per-flight `tracking.csv`, input images/IMU, pose history and Cube Desktop recording remain separate.
- `bin/flight-shadow review [BOOT_ID]` includes forwarding summaries and evidence paths alongside VIO and mapping views.
- After downloading the matching Cube BIN through the GCS, run `bin/flight-shadow cube-review /absolute/path/FLIGHT.BIN /absolute/path/NEW_REVIEW_DIRECTORY`.

The last command runs a C++ offline DataFlash reader, producing `cube-extnav.jsonl` and `summary.json` for `VISP`, `VISV` and `XKFS`. `VISP`/`VISV` establish Cube frontend reception; compare their `RTimeUS` with the serial log's `capture_host_us`. `Ign=0` means frontend acceptance, not that the active EKF fused VIO. `XKFS.SS` is zero-based: all observed cores must remain 0 (primary). Preserve full BIN for later EKF analysis. Logs can end incompletely after power loss; absence of samples or source records is unproven, not success.

Stop forwarding alone with `systemctl --user stop bhu-vio-observe.service`; remove its owner opt-in drop-in and restart the owner only while disarmed to disable future transport. `bin/flight-shadow stop` stops estimator/mapper/performance and native forwarding, leaving the UART owner and recording hooks running. Changing `VISO_TYPE` back to 0 is a separate disarmed parameter action and reboot. Do not automatically change it on tracking loss.

This is an observation experiment. VIO fusion, source transitions, GNSS denial, combined 15 W performance, and flight readiness require separate evidence.

## Temporary USB transport, 2026-11-18

The operator requested USB testing to isolate the TELEM2/GPIO failure. Identical C++ named-parameter probes got 0/14 matching replies on UART versus 201/202 matching replies over USB in 30-second tests. UART received heartbeats with no malformed frames; this proves receive direction only. The failing path is TELEM2/UART transmission or handling, not a verified specific GPIO pin defect. UART2 readback remains MAVLink2 / 115200, options=0, RTSCTS=0. Physical wiring, Jetson TX pinmux/electrical behaviour or per-port reception still need separate diagnosis.

The existing Cube owner now uses the exact Cube USB if00 by-id device through its installed `70-bhu-cube-usb-test.conf` drop-in. VINS startup uses the same device through its `70-bhu-vins-usb-test.conf` drop-in. The underlying UART profile is preserved. USB must remain connected for this test profile, including boot. Current VINS was kept warm; only its future startup wait endpoint changed. No Cube parameter, estimator gate, five-second audit freshness limit, source set, frame transform, recording or beep logic was changed. The proposed timeout/scheduling change was not applied because the A/B test isolated a transport-path failure.

Rollback while disarmed: remove only the two USB-test drop-ins, reload user systemd, and restart the owner after the UART path is verified bidirectionally. Source files are under deploy/systemd/. Do not treat an existing /dev/ttyTHS1 or incoming heartbeats alone as proof that Jetson-to-Cube communication works.
