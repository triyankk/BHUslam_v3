# Front ToF proximity through the shared Cube USB owner

The operator selected shared USB forwarding later on 21 November 2026 after
GPIO/TELEM2 dropouts. This supersedes GPIO as the active transport. Acquisition,
calibration, confidence >50 and primary-zone averaging stay native C++; only
the Cube transport changes. No Cube parameter or flight command is part of this
handoff. JT16 remains recording/mapping only.

The operator selected this source split on21November2026 for the autonomous
GPS-denied mission project. It supersedes JT16 obstacle publishing and the
previous Gemini-depth selection for the current test setup. This is a front-only
sensor integration, not demonstrated all-around autonomous obstacle avoidance.

## Ownership and startup

| Function | Owner / link |
| --- | --- |
| TMF8820 acquisition and front DISTANCE_SENSOR | Native C++ tof_viewer, I2C7; local handoff to existing USB owner |
| ToF USB transmission, Cube telemetry, VINS handoff, arm/disarm recording and cues | Existing optflow-flight-logger.service, sole Cube USB owner |
| JT16 raw point acquisition and shared mapping feed | Existing native JT16 reader managed by the USB recording owner |
| Flight mapping | Independent bhu-jt16-map.service / FAST-LIO shared feed |
| Camera/VINS estimator | Existing bhu-vins-fusion.service |

Only one process owns each physical serial/sensor path. ToF opens neither GPIO
UART nor Cube USB. ToF never receives/forwards VIO UDP; its vio-udp-port is0.
The native relay still hands validated VINS to the USB owner. Existing EKF/source
profiles, fresh-GPS initialization, QGC text and recording beeps are preserved.

`bhu-tof-front.service` starts `bin/tof-front-boot` at user boot after/wanting the
existing USB owner. It retains
front orientation0, direct I2C (no mux), confidence strictly above50 and mean of
accepted primary zones, minimum one active zone. Every boot/session has a unique
capture path under runs/tof-front/, binary identity, raw TX/RX, measurements and
summary. Viewer: http://127.0.0.1:8765 . No sensor aggregation/calibration change
is implied by selecting this as the proximity source.

Historical GPIO wiring: Jetson pin6 GND to TELEM2 pin6 GND;
pin8 TX to TELEM2 pin3 RX; pin10 RX from TELEM2 pin2 TX. Do not connect the Cube
5V pin to a Jetson UART signal. This wiring is unused by the shared USB path.
Sensor power/I2C stays on pins 1 (3.3 V), 3 (SDA), 5 (SCL) and 9 (GND).

## Shared USB transport

Native ToF publishes one MAVLink2 distance datagram per eligible frame to
127.0.0.1:14605. The installed optFlow `tof_transport.py` adapter, running in
the existing sole USB writer thread, accepts only CRC-valid unsigned front
DISTANCE_SENSOR packets: source 1/191, ID 0, orientation 0, laser type, and
1–500 cm bounds. Other messages (including commands and parameter writes),
repeats, future/out-of-order timestamps and data older than 150 ms are rejected.
The latest fresh packet is written once. Missing input is never replayed or
replaced by a clear-space value. The existing Cube proximity timeout is unchanged.

The owner requests distance telemetry and PRX1_TYPE readback internally, and
returns genuine Cube heartbeat, front-sector ID 10, status/health and parameter
feedback on loopback port 14606. It rejects unavailable front sentinels outside
the ToF range; another rear sensor can keep global PRX health active while front
data is missing. Per-front feedback age and value are the acceptance criteria.

Native `mavlink.bin` records local handoff packets; USB-owner
`cube.tof_transport.forwarded` counts actual USB writes. The native API reports
`transport=shared-usb` and `uart=null`. The USB owner's existing VIO, GPS-reference
whitelist, recording state machine and cue/beep logic remain separate and intact.

Service environment: OPTFLOW_TOF_UDP_PORT=14605 and
OPTFLOW_TOF_FEEDBACK_PORT=14606. Other obstacle publishers must be disabled when
this adapter is enabled. The ToF input cannot relay VIO or flight commands.

## Preventing mixed obstacle inputs

The persisted USB-owner configuration sets obstacle_avoidance.mavlink_output_enabled
and lidar_enabled to false. Its JT16 reader still streams native points for raw
recording/mapping but does not run LiDAR obstacle extraction. Recording metadata
has source jt16_recording_only and unknown sectors; unknown is never replaced
by clear space. Queueing, serial proximity sending and JT16 obstacle-alert beeps
are separately gated off, including an already queued scan. The Cube USB owner
continues heartbeat, telemetry, VIO and arm/disarm recording/cues.

The old generic JT16 Cube preset refuses this external-ToF configuration instead
of silently disabling PRX1 or restoring an old LiDAR profile. Changes to Cube
avoidance settings require a scoped explicit procedure and fresh readback.

## Cube profile

The selected backend is PRX1_TYPE2 (MAVLink). Front DISTANCE_SENSOR uses orientation0;
the Cube reports its processed front proximity sector as ID10. The front input
does not need an RNGFND slot. Preserve the independently configured rear TF03
and downward H-Flow rangefinders; see docs/hflow-down-rear-rangefinder.md.

The existing bench profile has AVOID_ENABLE7 (proximity bit2 enabled),
AVOID_BEHAVE1, AVOID_MARGIN2m, OA_TYPE1, horizontal OA_BR_TYPE1,
OA_BR_LOOKAHEAD3m, OA_MARGIN_MAX2m, database size100/queue80/expiry5s.
AVOID_ALT_MIN2m, PRX_IGN_GND1 and OA_DB_ALT_MIN1.5m are retained: do not expect
an on-desk obstacle to cause avoidance or a near-home database insertion while
these altitude/ground gates apply. A2m margin is not an exact sensor-referenced
turn trigger or a verified stopping distance. The sensor/CG/propeller envelope
and stopping-distance test remain necessary.

Current firmware evidence identifies4.7.1 dbe79216 with horizontal AltHold and
FlowHold avoidance disabled in the build. Parameters cannot add compiled-out
behavior. Loiter is the position-controlled reactive stopping/sliding test
candidate; BendyRuler applies to AUTO/GUIDED/RTL with valid position and geographic
reference. No mode, mission, arm, motor or source-selection command is part of
this setup.

## Current limitations and next validation

Historical GPIO reception was intermittent, despite clean/fresh local sensor output.
The prior independent USB observation saw a16.04s gap. ArduPilot's matching MAVLink
proximity backend times out after500ms. A single received front report is not
proof of sustained freshness. Never relax this timeout, replay old ranges, or
substitute JT16 distances to hide missing ToF data.

One forward sensor does not observe side/rear obstacles or prove a BendyRuler
side path clear. Its confidence-filtered mean can also conceal smaller near
objects behind farther zone returns. The existing aggregation is retained at
the operator's selection; validate target materials, thin obstacles, sun,
field-of-view and the physical airframe before relying on it.

The shared USB route bypasses that UART fault; it does not repair the GPIO link.
Verify sustained per-front Cube reception, then perform disarmed target-distance
and direction checks, confirm RC avoidance
toggle and altitude gates, and separately test braking/response in the supported
mode. Localization fusion, valid Home/origin, loss handling and full mission
avoidance remain separate acceptance stages.

Evidence: [integration report](../reports/tof-front-integration-20261121/README.md).

The later shared-USB activation and 60-second reception/dropout validation are
recorded in [shared USB validation](../reports/tof/shared-usb-20261121T094259360406Z/README.md):
293 valid front reports, maximum feedback gap 0.269 s, no CRC errors, continued
VIO forwarding and no JT16 obstacle output. This verifies transport; physical
obstacle-clearance and autonomous path-changing tests remain separate stages.

References:
- https://ardupilot.org/copter/docs/common-simple-object-avoidance.html
- https://ardupilot.org/copter/docs/common-oa-bendyruler.html
- https://github.com/ArduPilot/ardupilot/blob/dbe79216/libraries/AP_Proximity/AP_Proximity_MAV.cpp

## Rear rangefinder added later21November

The operator subsequently moved the180m TF03 to rear. It remains RNGFND1_TYPE27
with orientation4 and is now consumed by separate PRX2_TYPE4. Front PRX1_TYPE2
and GPIO ToF are preserved; H-Flow's integrated downward range is RNGFND2_TYPE24,
orientation25. See [down/rear assignment](hflow-down-rear-rangefinder.md). Front
and rear beams still leave lateral coverage unknown. Global proximity health
and GCS max-range values can now reflect the rear backend; the ToF viewer must
not treat a180m unavailable-front sentinel as real front reception.

### Boot ordering repair and response tuning —21November2026

A subsequent battery boot exposed an ordering cycle: the USB owner has
`After=default.target`, while front ToF was ordered after that owner and installed
under the same target. Systemd deleted the USB-owner start job to break the cycle.
The sensor still measured at15Hz, but no owner received/forwarded its UDP packets;
Cube consequently reported `PRX1: No Data`. Remove the ToF unit's `After=` edge
while retaining `Wants=optflow-flight-logger.service`. UDP publication does not
need receiver-first startup. Installed and repository units now match; dependency
verification passes. A real future reboot remains the end-to-end boot test.

The operator also approved PRX_FILT2Hz and AVOID_ACCEL_MAX1m/s². Actual fresh
readback already had PRX_FILT2 and acceleration0.75, so native scoped configuration
wrote only acceleration0.75->1. Stop behaviour1, margin2m, altitude gate2m and
RNGFND_FILT0.5 remained unchanged, along with audited VIO/EKF/range settings.
`cube_vio_config --responsive-avoidance-only` provides backup, rollback, fresh
ArduPilot disarmed heartbeat checks and named write/readback. It refuses an
unexpected front/rear backend or Stop/margin configuration. It does not reboot,
arm, change mode or select sources. Preserve the single USB owner by stopping
it for exclusive parameter access and restoring it on every exit.

See `reports/proximity-response-20261121/` for before/after/restore, recovered
front-specific Cube reports and post-change health. Native PTY tests cover exact
two-setting scope, unrelated preservation and armed/backend-mismatch refusal;
existing Cube integration passes. Configuration and disarmed reception do not
establish stopping distance or flight avoidance performance.

Actual reboot follow-up: boot39f51815-f44a-4eb5-9875-9936c1daee1f
started front ToF and sole USB owner automatically and recovered real front
reports/healthy proximity before intervention. Read-only post-boot audit confirmed
PRX_FILT2/AVOID_ACCEL_MAX1/Stop1/margin2 persisted. See the response report's
reboot-check directory. EKF position variance and absent origin/Home remain
separate navigation prerequisites; no flight validation follows from this check.
