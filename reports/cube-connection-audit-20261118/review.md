# Cube / Jetson connected configuration audit — 2026-11-18

**TELEM2 wiring and bidirectional communication are verified. Host boot and recovery faults were corrected. GPS-aided recording-only mode is configured, but the connected bench system is not flight-ready.**

## Wiring and serial link

| Jetson physical pin | Connection |
| --- | --- |
| 6, GND | Cube TELEM2 GND, pin 6 |
| 8, UART TX | Cube TELEM2 RX, pin 3 |
| 10, UART RX | Cube TELEM2 TX, pin 2 |

Live port `/dev/ttyTHS1`, 115200 baud. Read-only UART inspection received 8 heartbeats in the initial check and 16 in the expanded check, plus firmware/parameter responses, with zero malformed packets. This establishes both TX and RX operation, not just byte reception. Reference pinouts: [Cube Orange](https://docs.px4.io/main/en/flight_controller/cubepilot_cube_orange), [NVIDIA header documentation](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html). The actual Linux device was verified on this host rather than inferred from another JetPack version's pinout.

Cube firmware: ArduCopter 4.7.1, `dbe79216`, system 1/component 1, disarmed throughout inspection/configuration.

| Readback | Value |
| --- | --- |
| `SERIAL2_PROTOCOL` | 2, MAVLink2 |
| `SERIAL2_BAUD` | 115, 115200 baud |
| `SERIAL2_OPTIONS` | 0 |
| `BRD_SER2_RTSCTS` | 0, flow control disabled for the three-wire link |
| `NTF_BUZZ_TYPES` / `NTF_BUZZ_VOLUME` | 5 / 100 |
| `EK3_SRC1_POSXY`, `VELXY`, `POSZ`, `YAW` | 3 / 5 / 1 / 1 |
| `GPS1_TYPE` / `FLOW_TYPE` | 9 / 6 |
| `LOG_BACKEND_TYPE` / `LOG_DISARMED` | 1 / 0 |
| `ARMING_SKIPCHK` | 0, unchanged |

Source set 1 is configured for GPS horizontal position, optical-flow horizontal velocity, barometer height and compass yaw. This is a configured GPS-aided profile, not proof that the active EKF lane/source currently fuses all of those inputs. No active-source command was sent.

## Corrections applied

1. **Boot ordering cycle:** cold-start logs showed the three new BHU services ordered after `optflow-flight-logger.service`, which itself orders after `default.target`. Their start jobs had been deleted to break the cycle. Removed that unnecessary ordering; readiness guards already provide runtime synchronization. Actual installed `default.target` transaction verification reported the cycles before the change and succeeded without cycles afterward. No reboot was performed, so a new physical cold boot remains to be observed.
2. **Wrong primary transport for the cable in use:** the runtime still targeted Cube USB. The owner now uses `/dev/ttyTHS1` at 115200 through explicit unit environment overrides. VIO's device wait uses that UART too. Cube USB is not required by this boot profile. The base `config/system.yaml` USB value was preserved; runtime status now exposes the effective endpoint/baud to avoid ambiguity.
3. **LiDAR taking down Cube/VIO startup:** missing JT16 previously terminated the entire Cube/proximity/recording owner. LiDAR acquisition now retries/restarts within its own worker, with the existing stale-scan protections intact. Cube heartbeat/status and VIO remain active; proximity remains visibly unhealthy. Mapping waits for actual JT16 access instead of launching a backend with no feed. No clear/healthy proximity observations are fabricated.
4. **Unused visual-nav backend blocking GPS-only observation:** changed **only `VISO_TYPE: 3 -> 0`**. The Jetson estimator and arm-triggered recordings remain enabled; pose forwarding stays off. The native tool saved before/after, requested values, one write, and rollback. **37 other audited parameters were unchanged**, including all three EKF source sets, GPS, UART, rangefinders, proximity and arming-check configuration. A fresh independent USB inspection confirmed `VISO_TYPE=0`, disarmed state and zero malformed packets. Subsequent fresh pre-arm reports no longer contained the visual-odometry fault. No controller reboot or source/mode/arming command was issued.

Rollback for that sole Cube change: [`gps-observation-profile/restore.param`](gps-observation-profile/restore.param), containing `VISO_TYPE 3`. Restore/configure deliberately during a later external-navigation test, with readback and any required reinitialization; do not automatically enable physical pose forwarding for a SITL test.

## Live VIO and recording status

VINS-Fusion started on the actual Gemini and initialized. In a 600-processed-frame window it ran about **23.74 fps**, with **77.78 ms p95 estimated capture age** in current MAXN_SUPER mode. All 600 samples were initialized, but **zero passed the publication gate**. Median persistent features were 67; median stereo matches were 16 (p95 22), below the required sustained >=20 stereo matches. The current stationary scene does not establish tracking-qualified poses. Thresholds/calibration were not weakened or changed.

Recording is correctly idle while disarmed. VIO UDP output is zero/disabled, no recording-start request has been sent, and no ten-beep confirmation was deliberately faked. On a real arm, per-flight files begin and the fixed cue follows successful stereo/IMU writing; disarm finalizes after the three-second tail. The actual audible cue and complete connected arm/disarm recording cycle were not tested because the vehicle was not armed.

All four boot units read back enabled/active, with zero restarts in the final snapshot. VIO and the Cube owner have remained stable despite missing LiDAR. Mapping's active unit is **waiting**, not producing a map. One process owns `/dev/ttyTHS1`; temporary USB inspection used the separate USB endpoint and exited afterward.

## Outstanding observed issues

- **JT16 absent:** no `/dev/jt16_usb` or matching USB adapter. Mapping and LiDAR proximity cannot be validated until connected/powered.
- **GPS:** receiver report fix type 1, zero satellites, eph 10000 cm. EKF flags 39 do not include horizontal position validity. No Home/Origin messages were received during bounded requests; that absence alone is not proof of unset values, but GPS navigation readiness is not established.
- **Current pre-arm reports:** RC not found; excessive magnetic field (~1100 against reported max 875); battery below minimum arming voltage; Rangefinder 1 no data; PRX1 no data. These checks were not disabled. Full aircraft power/sensors and a suitable environment need separate live verification.
- **Proximity backend mismatch to review:** `PRX1_TYPE=4` is rangefinder-based proximity ([ArduPilot reference](https://ardupilot.org/copter/docs/common-rangefinder-setup.html)); the existing JT16 software profile intends a direct MAVLink backend (`proximity_type: 2`). Left unchanged for this observation-only phase. Do not assume the JT16's full directional coverage is accepted by the current backend.
- **Power mode:** still MAXN_SUPER. Combined operation at the intended 15 W profile remains untested.
- Physical camera/IMU timing/noise calibration, map accuracy, tracking in flight, obstacle avoidance and external-nav EKF fusion are not established by this wiring/configuration audit.

## Validation and evidence

- Fourteen existing-owner tests passed, including new UART override preservation and LiDAR retry isolation cases.
- Native Cube integration test passed, including exactly one `VISO_TYPE=0` write and preservation of unrelated values on simulated hardware.
- Live UART and independent USB parameter inspections passed with zero malformed packets; GPS observation change read back on a new connection.
- Actual installed boot transaction verification succeeded after the ordering fix. Source/destination unit state and runtime snapshots are retained here.
- Audit-only inspector was extended to request TELEM2, recording, GPS/avoidance parameters and passive telemetry snapshots. Missing/unsupported names (`SYSID_THISMAV`, `NTF_BUZZ_ENABLE`, `ARMING_CHECK`) were not interpreted as zero. Initial expanded UART pass had some transient missing replies; the separate USB audit returned the remaining relevant values.

See `telem2-115200-initial.json`, `telem2-full-audit.json`, `usb-reference-audit.json`, `post-profile-independent-readback.json`, `gps-observation-profile/`, `live-owner-status.json`, `live-vio-status.json`, `vio-live-metrics.json` and build/test logs beside this report.
