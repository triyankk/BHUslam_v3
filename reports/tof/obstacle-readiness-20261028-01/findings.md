# Cube ToF obstacle-avoidance readiness — 28 October 2026

The Cube supports the intended rangefinder-based proximity pipeline, but its
current configuration and vehicle state are not ready for obstacle avoidance.
This audit made no parameter writes, mode changes, arm commands, or reboots.
The bounded 25-second bridge test finished and released the UART and mux.

## Verified on the connected Cube

Firmware is ArduCopter 4.7.1, commit `dbe79216`, with a quadcopter heartbeat.
The Cube was disarmed, in AltHold (custom mode 2). UART runs at 57600 baud,
SERIAL1_PROTOCOL=2 and SERIAL1_BAUD=57. Seventy-two parameters were observed;
the complete readbacks and messages are in audit.json. Initial pipelined reads
missed some responses; sequential reads resolved every requested setting except
AVOID_ANG_MAX. An absent reply is not treated as a zero value.

| Setting | Readback | Implication |
|---|---:|---|
| PRX1_TYPE through PRX4_TYPE | 0 | All proximity backends disabled |
| AVOID_ENABLE | 7 | Proximity-source bit is enabled, but has no proximity backend |
| AVOID_BEHAVE | 1 | Stop behavior on approach in the applicable position-control mode |
| AVOID_MARGIN | 2 m | Configured simple-avoidance margin |
| AVOID_DIST_MAX | 2 m | Stored non-GPS distance setting; does not prove AltHold support |
| AVOID_ACCEL_MAX | 0.5 m/s² | Response limit requires validation with speed and stopping margin |
| AVOID_BACKUP_SPD | 1 m/s | Horizontal backup remains configured, despite approach behavior being Stop |
| AVOID_BACKZ_SPD | 0.75 m/s | Vertical backup configured |
| AVOID_ALT_MIN | 1.5 m | Downward range data needed to validate this altitude gate |
| PRX_IGN_GND | 1 | Ground rejection configured; depends on downward range data |
| RC7_OPTION | 40 | Avoidance switch assigned, but RC input currently missing |
| OA_TYPE / OA_BR_TYPE | 1 / 1 | Horizontal BendyRuler selected for applicable autonomous navigation |
| OA_BR_LOOKAHEAD / OA_MARGIN_MAX | 4 m / 1.5 m | Existing path-planner settings, separate from simple-avoidance margin |
| GUID_OPTIONS | 64 | Existing Guided waypoint-navigation option |

RNGFND1 remains downward MAVLink (TYPE=10, ORIENT=25, MIN=0.08 m, MAX=30 m).
RNGFND2 remains rear MAVLink (TYPE=10, ORIENT=4, MIN=0.01 m, MAX=5 m).
RNGFND3 through RNGFND6 are disabled. The current bridge reads one selected
mux channel; four-sensor acquisition is not implemented yet.

## Live transport check

Mux 0x70/channel 0 and TMF8820 serial 23937420 produced 372 calibrated samples
at 15 Hz. Independent binary decoding matched all 372 outgoing distance packets
to the sensor log. All 102 Cube rear reports matched a recently transmitted
distance, with sensor ID 1, orientation 4, source system 1/component 1. UART bad
packet count was zero. This confirms rear rangefinder acceptance, not an active
proximity map or avoidance response.

The nearest return remained 39–40 mm during this capture. Controlled target
tests over the intended distance range are still needed; this stationary bench
return does not validate obstacle distance accuracy or field-of-view coverage.

## Remaining requirements

1. For the current rangefinder architecture, the concrete next configuration
   is PRX1_TYPE=4 (RangeFinder), followed by the required reboot and proximity
   validation. One such backend scans all configured horizontal and upward
   rangefinders; a separate proximity backend per physical ToF is unnecessary.
   This is a proposed next step, not a change performed by this audit.
2. Verify rear proximity-map updates, direction, target distances, dropout
   behavior, and the RC7 enable/disable switch before expanding coverage.
3. Add left, right, and upward sensors to the bridge with separate calibration
   files and appropriate RNGFND slots/orientations. Front-camera SLAM does not
   itself establish a front obstacle-detection stream.
4. Resolve missing downward Rangefinder 1 data and validate the configured
   altitude gate and ground rejection. Do not treat these as validated simply
   because their parameters are enabled.
5. Resolve current prearm messages: RC not found, Need Alt Estimate, EKF attitude
   bad, AHRS not using configured type, low battery arming voltage, Rangefinder 1
   No Data, and VisOdom not healthy. GPS configuration waiting messages persist.
   Current EK3_SRC1_POSXY=3 selects GPS for source set 1; VISO_TYPE=3 does not
   establish a working or selected external-navigation solution by itself.
6. Validate behavior for the intended flight mode. Do not assume horizontal
   avoidance in AltHold for this binary: AVOID_ANG_MAX was unavailable and the
   matching source defaults AP_AVOIDANCE_ALTHOLD_ENABLED to 0. This is evidence
   consistent with that feature being omitted, not direct inspection of the
   binary's build flags. Loiter additionally requires healthy position estimation.

## Primary references

- [Matching firmware RangeFinder proximity backend](https://github.com/ArduPilot/ardupilot/blob/dbe79216/libraries/AP_Proximity/AP_Proximity_RangeFinder.cpp)
- [Matching firmware avoidance parameters](https://github.com/ArduPilot/ardupilot/blob/dbe79216/libraries/AC_Avoidance/AC_Avoid.cpp)
- [Matching firmware AltHold build default](https://github.com/ArduPilot/ardupilot/blob/dbe79216/libraries/AC_Avoidance/AC_Avoidance_config.h)
- [ArduPilot simple avoidance guide](https://ardupilot.org/copter/docs/common-simple-object-avoidance.html)
- [Rangefinder setup and reboot requirement](https://ardupilot.org/copter/docs/common-rangefinder-setup.html)
- [Proximity ground rejection](https://ardupilot.org/copter/docs/common-proximity-landingpage.html)
