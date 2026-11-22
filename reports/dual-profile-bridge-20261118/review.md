# One VINS bridge for GPS observation and GPS-free VIO

Implemented and loaded on 18 November 2026 at the operator's request. Both the native C++ relay and the small existing sole-owner serial adapter recognize the same exact profiles. No new Python process, estimator, Cube serial owner or parameter writer was introduced. No Cube parameter, source, mode, arming, recording state machine or tune definition changed.

## Supported configurations

All three profiles require `AHRS_EKF_TYPE=3`, `EK3_ENABLE=1`, `EK3_SRC_OPTIONS=0`, `VISO_QUAL_MIN=40`, `VISO_ORIENT=0`, `VISO_POS_X/Y/Z=0.13/0/0.05`, `GPS2_TYPE=0`, and barometric height `EK3_SRC1_POSZ=1`.

| Field | gps_recording (current) | gps_observation | gps_free_vio |
|---|---:|---:|---:|
| VISO_TYPE | 0 | 3 | 3 |
| EK3_SRC1_POSXY | 3 | 3 | 6 |
| EK3_SRC1_VELXY | 3 | 3 | 6 |
| EK3_SRC1_VELZ | 3 | 3 | 6 |
| EK3_SRC1_YAW | 1 | 1 | 6 |
| GPS1_TYPE | 9 | 9 | 0 |
| COMPASS_ENABLE | 1 | 1 | 0 |
| Forwarding | Blocked deliberately | Gate-qualified only | Gate-qualified only |

These are exact profiles for this assembly, not permission to forward under arbitrary source combinations. The former GPS-position/optical-flow-velocity hybrid, visual height and XY-only/no-vertical-velocity variants are not accepted by this contract. Add and test any different profile explicitly rather than weakening matching. The source1 VELZ=6 choice requires validated VIO vertical velocity; barometer remains height. Calibration, motor-vibration/clipping, geographical alignment and EKF validation are unresolved and the future profile remains unapplied.

Both guards audit 17 identical parameters. Values must be finite, within `1e-5` of the profile and refreshed within five seconds. Both retain a fresh <=1.5-second autopilot heartbeat and require disarmed qualification before armed forwarding. A different recognized profile cannot take over while armed. After a telemetry gap/reconnection while armed, disarmed qualification is required again. Reported non-primary source selection remains latched off until restart, as before. Recognizing a profile never selects an EKF source or writes parameters.

`gps_recording` is recognized but always returns `VIO backend disabled (GPS recording)`. A source mismatch is `Unsupported Cube navigation profile`. Both native status and owner status show `profile` and `pinned_profile`; gate/profile transitions and actual serial writes are logged with the profile. The legacy status field `source_configuration_ok` still indicates whether all forwarding conditions pass, so it is false for intentionally disabled recording; consult `profile` and `block_reason` together.

Packet handling remains ODOMETRY only, component 197, LOCAL_FRD/BODY_FRD, VIO estimator, quality >=40, valid quaternion/finite state, monotonic capture timestamp, <=150 ms age and maximum 10 Hz, with latest-only pending data. The owner rechecks age at the actual serial write. Invalid/stale inputs never cause stale-pose replay, parameter rewrites or source changes. Existing upstream 500 ms support qualification and IMU clipping guards are unchanged.

## Changes and deployment

- `include/bhu/observation_guard.hpp`: native profile recognition, float-aware mount checks, disarmed pinning and heartbeat-gap handling.
- `src/cube_vio_observe.cpp`: heartbeat state, profile/gate events and status; `--print-profiles` emits the configuration contract without opening sockets or hardware.
- Existing owner's `src/optflow_slam/vio_observation.py`: matching profile audit and pinning at the final serial boundary, profile-labelled write and gate logs. It still shares the existing owner and creates no additional process.
- Native and owner tests cover both enabled profiles and disabled recording. Documentation was updated in both projects.

Before restart, the existing owner status was fresh, connected, disarmed and recording inactive; VISO_TYPE remained 0. Only `bhu-vio-observe.service` and `optflow-flight-logger.service` were restarted. VINS remained warm. All five boot services were active afterward and the two reloaded services remained enabled. The current Cube USB endpoint, recording/cue configuration and JT16 sharing were preserved.

## Verification

- Native protocol, bridge/config integration and external-navigation transport tests: **3/3 passed** (14.68 seconds). Transport test uses isolated localhost ports and a pseudo-terminal, never the real Cube input ports.
- Existing adapter packet tests: **11 passed**, including actual fake-connection writes for both enabled profiles, no writes for recording, per-parameter stale/NaN failures, wrong mount, unsupported hybrid, in-flight profile change, armed startup/reconnection, CRC/identity/quality/duplicate rejection and expired queued poses after tracking loss.
- Native/adapter table parity: all three profiles and all 17 keys match within `1e-6`; saved JSON exports and `profile-parity.json` are evidence. This catches drift between the two independent project-local implementations.
- Existing owner's full suite: **381 passed, 2 failed**. Failures are in untouched `.gitignore` generated-data rules and an external workspace reference in untouched `docs/JT16_VIEWER.md`; see `full-adapter-suite.log`. They are unrelated to the forwarding adapter and were left unchanged.
- **65-second actual Cube observation**, including 55 samples after startup: Cube remained connected/disarmed, `gps_recording` continuously identified, largest parameter age 3549.63 ms (<5000), **zero serial pose writes and zero native handoffs**. The native relay received and blocked a real VINS input packet. Recording thread stayed alive with no recorder error, JT16 maximum observed scan age 210 ms and final rate 4.975 Hz. No arm/cue test was triggered. GPS bad-fix/GCS-failsafe pre-arm conditions remain.
- Scoped whitespace check passed after documentation cleanup.

`live-summary.json`, `live-check.jsonl`, status snapshots, test logs, profile exports and `release.sha256` preserve the evidence. The current 17 audited Cube values were cross-checked against the saved GPS recording configuration and remained unchanged.

## What remains separate

No live GPS-observation forwarding or GPS-free EKF fusion was enabled to test this change. Successful simulated transport is not actual Cube receipt/fusion evidence. Once an appropriate profile is deliberately configured while disarmed, the same installed bridge can forward; it needs no alternate binary/service. Continue logging Cube VISP/VISV, XKFS active-source evidence and EKF innovations.

Entire Here4 removal also removes compass yaw. GPS-free VIO yaw is relative to startup and needs a verified north alignment for latitude/longitude missions. Correct origin/Home, calibrated camera/IMU timing/extrinsics, motor-on tracking, clipping mitigation and tracking-loss/restart response require separate ground/simulation validation before flight reliance. Current saved origin is empty. No arming check or quality/freshness threshold was weakened.

Arm still opens flight recordings; fresh disarm finalizes after the existing three-second tail, while telemetry loss does not close them falsely. Ten rapid recording-ready beeps still mean the first stereo pair and both IMU streams were flushed, not healthy navigation. Nothing in this bridge change makes recording depend on successful forwarding.

## Rollback

This directory preserves the previous native executable, header/source and owner adapter/tests. To roll back, first confirm a fresh disarmed Cube and no active recording, stop the forwarding service and existing owner, restore the previous native executable and corresponding source/header plus the adapter/tests, then restart the owner and forwarding service. Keep estimator/recorder inputs and Cube parameters unchanged. Do not restore whole unrelated documentation/configuration files over later user edits. Disabling forwarding with VISO_TYPE=0 remains a separate explicit Cube setting; it is already the live setting and was not rewritten here.
