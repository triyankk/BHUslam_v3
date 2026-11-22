# JT16 / BendyRuler configuration — 18 November 2026

The operator requested enabling JT16-assisted BendyRuler. The connected USB-RS485 adapter is visible as `/dev/jt16_usb -> ttyUSB0`, but the operator confirmed that LiDAR power was disconnected, then explicitly requested skipping scan verification and configuring the path only. No sensor-health, directional-coverage, braking, routing or flight-readiness claim is made.

## Applied configuration

A fresh filtered Cube autopilot heartbeat confirmed disarm before exclusive inspection and again before application. The existing Cube owner was paused only for exclusive parameter access and restored automatically after both sessions. The native C++ configuration tool saved before/requested/after snapshots and a rollback file.

**Only `PRX1_TYPE` changed: 0 -> 4 (MAVLink proximity).** Readback matched 4. No malformed packets were observed by the writer. All other parameters audited by the mode matched their pre-write values, including all three stored EKF source sets, `EK3_SRC_OPTIONS=0`, `VISO_TYPE=0`, camera mounting, rangefinders, avoidance tuning and RC7 mapping. In particular, the operator's current `EK3_SRC2_POSXY=3` was preserved rather than restoring an older ExternalNav preset. No source-selection, arming, mode, motor, reboot or buzzer command was sent.

Existing values were verified and preserved:

| Parameter | Readback |
|---|---:|
| OA_TYPE | 1: BendyRuler |
| OA_BR_TYPE | 1: horizontal |
| OA_BR_LOOKAHEAD | 4 m |
| OA_MARGIN_MAX | 1.5 m |
| OA_DB_SIZE | 100 |
| OA_DB_EXPIRE | 5 s |
| OA_DB_ALT_MIN | 1.5 m |
| AVOID_ENABLE | 7 |
| AVOID_MARGIN | 2 m |
| AVOID_ALT_MIN | 1.5 m |
| PRX_IGN_GND | 1 |
| PRX_FILT | 0.25 Hz |
| RC7_OPTION | 40 |
| VISO_TYPE | 0: VIO forwarding remains blocked |

The data path remains the existing JT16 RS485/USB -> native Hesai decoder at 3,000,000 baud -> Jetson body-frame proximity sectors -> paced MAVLink distance messages -> Cube USB link. No second serial owner, companion movement output, new Python process, EKF source switch or new startup cue was introduced. Arm/disarm recording and the ten-beep recording-ready cue are unchanged.

## Activation and limits

**No Cube reboot was performed.** Power the JT16 and reboot/power-cycle the Cube to instantiate the newly selected proximity backend. The firmware initializes proximity drivers at boot; a parameter readback alone does not prove that the runtime driver has loaded. Without valid powered scans the feed remains unknown, and after startup the Cube may report a proximity pre-arm fault. No health flag was forced or arming check disabled.

BendyRuler operates in **AUTO, GUIDED and RTL**, not Loiter/FlowHold/AltHold. The current Cube mode during configuration was Loiter. RC7 remained low and unchanged. In this firmware, RC7 option 40 gates simple avoidance and is not a universal BendyRuler cancellation switch. The configured 1.5 m altitude/database gates and existing downward-range dependency also remain unchanged. This is horizontal obstacle avoidance, not a verified all-direction collision-avoidance system.

The required current-assembly physical and scan checks were not completed. Propeller removal was requested but not confirmed; accordingly no reboot or motor action was performed. The operator explicitly waived scan testing. Verification of decoded scans, Cube PRX health and target direction/distances remains outstanding before relying on avoidance.

## Tool and verification

Added `--enable-proximity-only` to the existing native `cube_vio_config`. This mode writes only PRX1_TYPE=4, requires BendyRuler already selected, backs up state, checks disarm, reads back the write and checks preservation of audited unrelated settings. It is mutually exclusive with other configuration modes. Existing default VIO presets are not applied by this mode.

The simulated Cube integration fixture exercises the enable path and proves a single PRX write with unrelated-parameter preservation. It passed together with existing configuration/bridge scenarios in 12.74 s. Real scan verification was skipped at the operator's request. No full external-project suite was required: that runtime's code and config were not changed.

The Cube owner and VINS services were confirmed active afterward; the existing owner remains boot enabled. Saved evidence:

- `before-audit.json`: independent read-only Cube inspection.
- `config/before.json`, `config/requested.json`, `config/after.json`, `config/changes.json`, `config/summary.json`.
- `config/restore.param`: rollback for the single changed parameter (`PRX1_TYPE 0`); not applied.
- `preservation.json`, `status-before.json`, `status-after.json`, `integration.log`.

Firmware references: [BendyRuler modes/configuration](https://ardupilot.org/copter/docs/common-oa-bendyruler.html), [Copter 4.7.1 proximity initialization](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_Proximity/AP_Proximity.cpp).
