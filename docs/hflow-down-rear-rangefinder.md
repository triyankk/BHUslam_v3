# H-Flow downward range and TF03 rear avoidance

On21November2026 the operator moved the180m TF03 from downward-facing to rear,
and requested that optical flow use the H-Flow module's integrated DroneCAN
rangefinder. This supports the GPS-denied navigation/avoidance project; it is
not a flight-validation result.

## Sensor assignment

| Slot / backend | Assignment |
| --- | --- |
| RNGFND1_TYPE27, ORIENT4 | Existing serial TF03, now rear; existing0.1–180m limits retained |
| RNGFND2_TYPE24, ORIENT25 | H-Flow integrated downward DroneCAN rangefinder;0.08–30m configured limits |
| PRX1_TYPE2 | Existing front ToF MAVLink backend, preserved |
| PRX2_TYPE4 | Cube RangeFinder proximity backend, consumes the rear-facing rangefinder |

PRX2_TYPE4 scans horizontal/upward rangefinders, so it consumes rear RNGFND1 and
ignores downward RNGFND2 for horizontal obstacles. Front ToF remains a separate
MAVLink backend. JT16 remains recording/mapping only.

Optical flow remains FLOW_TYPE6 (DroneCAN), on the existing CAN2 port/driver1,
with existing calibration/orientation/offsets preserved. RNGFND2_ADDR0 matches
standard H-Flow sensor address; the actual backend/node detection must be
verified after reboot. RNGFND2_POS_X/Y/Z copy the existing FLOW_POS_X/Y/Z (currently
0/0/0.1m), because the new range sensor is integrated in the same module.

No EKF source switch is needed. Matching firmware's EKF range reader accepts
only downward orientation25 from its first two rangefinder slots. Moving RNGFND1
to rear excludes it from flow's downward-height input; enabling downward RNGFND2
supplies that input. Existing VIO/optical-flow/barometer/compass sources and
EK3_SRC_OPTIONS0 remain unchanged. This does not assert validated fusion.

## Scoped changes and retained values

The native tool mode `cube_vio_config --hflow-down-rear-only` audits the current
TF03, vacant/DroneCAN slot2, front MAVLink backend, vacant/RangeFinder proximity
slot2 and existing working DroneCAN flow setup. It refuses unexpected occupied
slots, backend types and transforms. It saves before/requested/after/restore
files, requires recent disarmed heartbeats before writes/reboot, and checks all
other audited values for preservation. Operator confirmed props removed.

The actual six writes are RNGFND1_ORIENT25->4, RNGFND2_TYPE0->24,
RNGFND2_MIN0.01->0.08m, RNGFND2_MAX5->30m, RNGFND2_POS_Z0->0.1m and
PRX2_TYPE0->4. Other requested values already matched. Backend/type activation
requires a Cube reboot.

TF03 driver, serial port/baud, limits and offsets are preserved. Its existing
position offset has not been remeasured after moving to the rear. RNGFND2_GNDCLR
retains its existing0.2m value; this is not a new measurement of the installed
H-Flow's on-ground clearance. Measure both mounting geometry and ground clearance
before interpreting calibration or flight accuracy.

Existing BendyRuler/simple-avoidance settings, altitude/ground gates, RC toggle,
front ToF link, GPS reference policy, arming checks and JT16 recording roles are
preserved. Range30m is the configured manufacturer rating, not proof of useful
optical-flow navigation to30m in every scene. The rear180m rating is likewise
not a validated avoidance distance; the existing planner lookahead is3m.

## Verification

After reboot verify actual downward orientation25 readings from slot2 and rear
orientation4 proximity readings, including target response. The firmware filters
horizontal rangefinder telemetry when the RangeFinder proximity backend is
active, so rear readings appear as a proximity-sector ID (typically14), rather
than the former slot0/downward telemetry. The separate downward range remains
visible as sensorID1.

A global proximity-health flag may reflect the healthy rear source while the
front ToF GPIO link is stale. It cannot certify front coverage. Keep checking
per-direction data; do not treat adding rear proximity as a fix for the existing
front GPIO transmit problem. No stale ranges or extended timeouts are added.

A disarmed readback proves configuration and range reception only. Validate
floor-distance response and optical-flow scaling at controlled heights, rear
range/direction and airframe clearance, supported-mode braking, and range/flow
loss behavior before flight reliance. Single front/rear beams do not cover the
sides. Existing firmware evidence excludes horizontal AltHold/FlowHold avoidance;
Loiter reactive or AUTO/GUIDED/RTL BendyRuler need their separate prerequisites.
The Cube reboot also requires fresh GPS-origin/Home qualification again under
the existing bootstrap procedure.

Evidence: [setup report](../reports/hflow-down-rear-rangefinder-20261121/README.md).

Sources:
- https://docs.holybro.com/peripherals/h-flow-dronecan/setup-guide
- https://github.com/ArduPilot/ardupilot/blob/dbe79216/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp
- https://github.com/ArduPilot/ardupilot/blob/dbe79216/libraries/AP_Proximity/AP_Proximity_RangeFinder.cpp

The Cube's GCS proximity messages use a combined maximum-distance limit. During
verification an unavailable front sector was sent as18000cm, because the rear
sensor extends that limit to180m. The native front-ToF viewer now excludes values
outside its actual0.01–5m source range from accepted reception/count/age/plots and
counts them separately as unavailable_cube_reports. Raw RX logs retain them.
This prevents the rear sensor's far-limit sentinel from appearing as a new clear
front observation; it does not repair GPIO reception or change outgoing ranges.

The OPTICAL_FLOW ground_distance field is EKF-estimated height above ground,
not a direct copy of the H-Flow laser reading. Check DISTANCE_SENSOR slot1/down
for raw H-Flow range; do not expect that field to equal it exactly.

Parallel transport update: the operator-authorized ToF chat subsequently moved
front readings onto the existing USB owner after GPIO dropouts. This rangefinder
change preserves that shared-owner work. PRX1 remains the front MAVLink backend
regardless of serial transport; PRX2 is the rear RangeFinder backend.
