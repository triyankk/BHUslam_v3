# VINS-Fusion observation forwarding — 2026-11-18

The user authorized VIO pose reception/logging during a GPS-aided observation flight, including while armed, while preserving primary EKF source configuration, EK3_SRC_OPTIONS=0, arm/disarm recording and recording-ready beeps. VIO fusion and source-transition validation remain separate.

## Cube configuration and source boundary

Fresh disarmed TELEM2 audit confirmed `/dev/ttyTHS1`, 115200 baud, ArduCopter 4.7.1 dbe79216. Source set 1 remains POSXY=3 (GPS), VELXY=5 (optical flow), POSZ=1 (barometer), VELZ=0, YAW=1 (compass). It is not a GPS-velocity profile, and no GPS-velocity change is required for this reception experiment. EK3_SRC_OPTIONS=0 was preserved. Source set 2 retains the previously configured ExternalNav position; source set 3 retains its GPS profile.

Reported the only required Cube change before applying it: VISO_TYPE 0 -> 3, then a freshly disarmed reboot. `observation-profile/` contains before/after snapshots, the one write/readback, and rollback `VISO_TYPE 0`. `parameter-preservation.json` confirms all other readable audited parameters were preserved. Independent `after-reboot.json` reconfirmed source settings, VISO_TYPE=3, proximity disabled, all 16 RC option assignments with none equal to 90, scripting disabled and zero malformed UART packets. No arming, mode, mission, movement or source-selection command was sent. The mode changed through operator/RC activity, not this implementation.

Firmware defaults to primary source on reboot; this implementation never selects a source. Parameter values alone do not prove active source selection. Source set 1 was not explicitly selected with a command because automatic source switching was prohibited. Actual per-core flight source evidence must show XKFS.SS=0 in the matching Cube BIN. Do not select another source externally during observation. The code stops forwarding upon a reported non-primary selection or source-configuration mismatch; it cannot prevent independent GCS commands or claim continuous source-state telemetry that the protocol does not supply.

## Implementation

Native C++ VINS gate and transforms retained, with unchanged 500 ms / 40 persistent / 20 backend / 20 stereo / 150 ms age policy and mild CLAHE=2. Native `cube_vio_observe` reuses the C++ packet filter, validates narrow local ODOMETRY input, caps handoff at 10 Hz, and logs packet/guard/telemetry health. It does not own UART. The sole existing UART owner has an explicit opt-in narrow adapter to receive these packets, recheck freshness/source settings and serialize actual writes. No new Python process was introduced. The owner's own adapter lives in its own repository; no sibling code/config is imported.

The fixed read-only source/backend audit refreshes the oldest reply first, retrying lost replies before they expire. Both native and UART guards require a fresh heartbeat, exact primary source values and EK3_SRC_OPTIONS=0, with a five-second parameter-reply freshness limit. They require a disarmed startup heartbeat, then permit observation transmission while armed. No arbitrary command proxy exists. Latest-only buffering and actual-write age checks prevent replaying stale poses. A detected observation fault stops its output while the existing recording/Cube service continues.

`bhu-vio-observe.service` is installed/enabled and VINS boot output targets localhost:14600. New boot graph verification succeeded without a cycle. A fresh physical cold boot of this new forwarding profile has not been performed. Existing arm/disarm recording and ten-beep recording-ready cue code were preserved; no actual arm command or physical cue test was performed. Native parameter/transport mocks include armed observation tests.

## Real ground evidence

The user performed a 45-second disarmed camera-motion check. Snapshot deltas: 779 processed VINS frames, 751 gate-qualified UDP poses, and 265 serial writes. It initially exposed audit expiry pauses; the oldest-reply retry policy was then deployed and tested without weakening the age guard. The Cube was freshly disarmed before the owner restart. Subsequent native logs show no source-audit expiry transitions after initial startup completion.

Frozen `forwarding-metrics.json` summarizes the real post-fix window: 2,345 serial writes over 283.31 s, 8.27 Hz mean, 106.11 ms median gap, 198.88 ms p95 gap and 903.29 ms maximum gap. Estimated capture age at write was 146.76 ms p95, maximum 149.95 ms. Serial write duration measures the host write call, not wire completion or calibrated end-to-end camera latency. Of 572 Cube vision-health samples, 484 were healthy. All writes were disarmed; armed behaviour was tested only on simulated connections. This run used MAXN_SUPER mode 2; it does not establish 15 W performance.

Health and forwarding are intermittent; this is not a tracking-continuity or flight-readiness pass. Actual Cube frontend health became true during real VINS output. Serial writes establish transmission; SYS_STATUS vision health is a reception proxy, not per-pose acknowledgment. No actual Cube BIN was collected in this disarmed run (`LOG_DISARMED=0` preserved), so VISP/VISV receipt and XKFS active-source proof remain pending the matching flight BIN.

The current observation brought back genuine VisOdom arming checks. Latest fresh messages included GPS bad fix and **VisOdom roll/pitch difference 22.8 degrees (>10 degrees)**. Camera/Cube relative mounting/attitude must agree before flight. The configured mount is provisional and was not changed to hide this disagreement. Checks remain enabled. Tracking gaps can also produce VisOdom not healthy. GPS, mount agreement, actual recording/cue operation and 15 W behaviour still need their appropriate checks.

## Post-flight evidence

`bin/flight-shadow review [BOOT_ID]` now includes native forwarding runs and actual UART-write logs alongside VIO/map review. Boot ID and monotonic time allow joining writes to separate flight recordings. Keep the matching Cube BIN from the GCS. Run:

```
bin/flight-shadow cube-review /absolute/path/FLIGHT.BIN /absolute/path/NEW_REVIEW_DIRECTORY
```

The C++ offline tool exports VISP/VISV/XKFS, counts ignored/low-quality samples, checks zero-based source selections and tolerates a truncated final packet. VISP/VISV RTimeUS matches the Jetson capture_host_us. Ign=0 means frontend acceptance, not proof of VIO fusion by the primary EKF. Primary-only source records must be SS=0 for all observed cores. Missing records are unproven, not a pass. Preserve complete BIN for later EKF analysis.

Runtime evidence: `owner-final.json`, `owner-final-events.jsonl`, `owner-motion-events.jsonl`, `relay-final.json`, `relay-events.jsonl`; original paths are in snapshots. Source/mount configuration, clock alignment and VINS failure detection remain provisional for control.

## Checks

- Three native CTest checks passed: protocol, transport/guard and Cube parameter integration; observation mode wrote only VISO_TYPE in the fixture, with no source command.
- Eighteen focused owner tests passed, including packet CRC/identity/type/freshness/quality, armed observation, source/heartbeat guards, retry behaviour and existing recorder/cue hooks.
- Native Cube BIN parser synthetic test passed for reception/quality/ignored/source fields and truncated tail. This fixture is not live Cube receipt proof.
- Flight review test passed, including distinguishing native handoff from actual UART writes/reception.
- Owner full suite: 374 passed, 2 unrelated failures remain in existing generated-file ignore policy and a JT16 viewer documentation workspace-reference test. Full output: `owner-full-suite.log`. These failures were also present in the first broad run before the retry/docs changes.
- Boot transaction verified; service status/readback verified after deployment. No physical cold boot or real armed flight was performed.

Primary references: [ArduPilot ExternalNav transport](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html), [firmware source options](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_NavEKF/AP_NavEKF_Source.cpp), [Cube vision logging schema](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_VisualOdom/LogStructure.h), [XKFS source logging](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_NavEKF3/AP_NavEKF3_Logging.cpp).
