# Fresh-GPS bootstrap for VIO + optical flow + internal compass

This initializes a geographic reference for the GPS-denied mission workstream in
[PROJECT_GOAL.md](../PROJECT_GOAL.md). It does not validate VIO fusion, RTL control
or obstacle avoidance.

## Selected configuration

Exact bridge profile: `vio_optflow_compass_gps_bootstrap`.

| Parameter | Value | Purpose |
| --- | --- | --- |
| GPS1_TYPE | 9 | Here4 DroneCAN driver for startup reference |
| GPS2_TYPE | 0 | No second GPS |
| AHRS_OPTIONS | 8 on the current Cube | Record allowed; bit16 previous-origin reuse disabled |
| EK3_SRC1_POSXY | 6 | VIO position |
| EK3_SRC1_VELXY | 5 | Optical-flow horizontal velocity |
| EK3_SRC1_POSZ | 1 | Barometric height |
| EK3_SRC1_VELZ | 6 | VIO vertical velocity |
| EK3_SRC1_YAW | 1 | Selected internal compass |
| EK3_SRC_OPTIONS | 0 | Existing source policy |

Source sets2/3, compass device priorities/use flags, mount/quality, proximity,
arming checks and all other audited settings are preserved. GPS driver enable
requires a Cube reboot. No automatic source-selection command is added.

## Operator sequence

1. With the aircraft disarmed, connect Here4 **before powering the Cube**. Keep
   the camera unobstructed and place the aircraft stationary with a clear sky view.
2. Wait for **three slow beeps** from the Cube. They mean the Jetson has checked a
   fresh GPS fix and confirmed Cube EKF origin and Home readbacks for this boot.
   They do not mean that all flight checks passed.
3. For the ground validation stage, disconnect Here4 while the Cube remains
   powered and disarmed. Check that origin/Home remain and VIO forwarding and
   the internal compass continue. Any remaining pre-arm errors must be reviewed.
4. Repeat from step1 after every Cube reboot. Do not disconnect/reconnect GPS in
   flight as part of this workflow. Keep the Cube powered between initialization
   and the intended GPS-disconnected operation.

The existing **ten rapid recording-ready beeps** and arm/disarm recording logic
are unchanged. Three slow beeps are a separate geographic-reference cue.

## Implementation and guards

`cube_vio_observe` runs the native C++ state machine inside the existing relay.
It consumes read-only telemetry from the sole Cube serial owner. It waits for:

- A recent Cube boot clock and AHRS_OPTIONS readback with reuse bit16 clear.
- At least five seconds of advancing GPS timestamps, at least three unique
  samples, 3D fix or better, at least eight satellites, reported horizontal
  accuracy <=2m, vertical accuracy <=5m and speed <=0.5m/s.
- A recently reported Cube origin within100m horizontally of the fresh GPS fix.

The Cube establishes its EKF origin from GPS; the Jetson never fabricates an
origin, uses0,0 or sends SET_GPS_GLOBAL_ORIGIN. If Home is absent, C++ may request
`COMMAND_INT / MAV_CMD_DO_SET_HOME` with explicit fresh GPS latitude/longitude and
AMSL altitude. It sends at most three attempts, separated by two seconds. A
matching HOME_POSITION readback is required; sending a command is not success.
An existing Home farther than5m horizontally or vertically is a conflict and is
never overwritten automatically. Readiness checks also reject launch movement.

The existing Python serial-owner adapter independently validates the narrow
Home whitelist, fresh disarmed profile, boot/options, GPS quality, nearby origin,
command targets/units and retry bound. It forwards no arbitrary command packets.
A local NAVREF_OK marker can produce one three-slow-beep cue only after the owner
also verifies the matching Cube Home. Origin proximity also has a5m vertical
check at this boundary. All checks and writes are logged separately from poses.

Four read-only MAV_CMD_REQUEST_MESSAGE requests (SYSTEM_TIME, GPS_RAW_INT,
GPS_GLOBAL_ORIGIN, HOME_POSITION) and AHRS_OPTIONS are cycled every250ms in this
profile: each item is requested every1.25s. The original17-key profile audit and
all VIO pose-health gates are retained. Other profiles do not get these requests.

GPS loss after successful initialization does not change sources, block pose
forwarding or erase the same-boot reference. A backwards Cube boot clock clears
reference qualification and requires fresh disarmed startup. The reference is
not persisted as a reusable Jetson launch location. Forwarding does not depend
on a GPS fix, so initialization cannot deadlock VIO startup or recording.

## Status and evidence

- Native relay: `runs/vins-forwarding/latest/status.json`, `navigation_bootstrap`.
- Sole owner: `/home/atas/vscode/optFlow_slam/runtime/obstacle_avoidance_status.json`,
  `cube.vio_observation.navigation_reference`.
- Owner JSONL includes GPS quality, origin/Home readback, Home command writes,
  command ACK, verified reference cue and reboot events.
- Native events log state changes and bootstrap handoffs; ordinary pose counters
  and logs remain independent.
- Configuration/evidence: [20 November setup](../reports/gps-bootstrap-20261120/).

Indoor no-fix checks validate waiting behavior only. A clear-sky fix, confirmed
Home/origin, physical GPS disconnection, fusion/heading continuity and a separate
controlled RTL test are required before claiming this workflow validated for
flight. Reference initialization alone does not fix the previously recorded
pose gaps, vibration clipping or EKF divergence.

## Configuration tool and recovery

Native `cube_vio_config --gps-bootstrap-only --output NEW_DIRECTORY` first audits
current sources, DroneCAN bus and the selected internal compass. Add `--apply
--reboot` only during an authorized disarmed, props-off setup. It writes only
GPS1_TYPE and the origin-reuse bit if required, saves before/requested/after
snapshots and restore.param, and refuses unexpected source/compass/CAN settings.
It never changes source sets or selects an active source.

Restore files are specific to their saved setup. Review them before use; do not
restore an old flight profile or old geographic reference automatically.

Protocol references: [ArduPilot Home and origin](https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html),
[installed-firmware GPS driver metadata](https://github.com/ArduPilot/ardupilot/blob/dbe79216/libraries/AP_GPS/AP_GPS_Params.cpp).

## QGC messages and arming readback —20 November2026

The existing serial owner now sends `NavRef:` STATUSTEXT to the Cube for normal
MAVLink routing to QGC Messages. It reports waiting/reference success/conflicts;
reference success accompanies the three slow beeps. Messages are rate limited
(minimum2s, unchanged state every30s), so a later GCS connection can receive the
current state. The existing ten rapid recording beeps are unchanged. Sending was
verified; QGC display remains unconfirmed because QGC was unavailable.

A bounded read-only startup audit exposes ARMING_SKIPCHK, ARMING_NEED_LOC and
fence settings in navigation_reference. These are a connection-time snapshot,
not a continuous arming-configuration guard. The20November readback shows
ARMING_NEED_LOC0 and existing ARMING_SKIPCHK304374, which skips several checks
including INS, compass and VIO. No skip mask was changed. Current ALT_HOLD
pre-arm health passes with no GPS fix, but this does not prove skipped systems
healthy or physically disconnected-GPS arming. Operator kept GPS connected.
Position-mode navigation, the fresh-reference procedure and RTL still require
separate estimator/origin/Home validation. See
[QGC/arming audit](../reports/gps-reference-qgc-20261120/README.md).
