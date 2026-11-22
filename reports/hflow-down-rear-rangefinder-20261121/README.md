# H-Flow down / TF03 rear rangefinder —21November2026

The operator moved the180m TF03 from down to rear, requested H-Flow integrated
DroneCAN range for optical flow and rear obstacle use, and confirmed props off
with Cube kept disarmed.

Applied exactly six writes with native scoped backup/write/readback/reboot:
- RNGFND1_ORIENT25->4 (rear).
- RNGFND2_TYPE0->24 (DroneCAN).
- RNGFND2_MIN0.01->0.08m and MAX5->30m.
- RNGFND2_POS_Z0->0.1m (copies existing FLOW_POS_Z; X/Y remain0).
- PRX2_TYPE0->4 (RangeFinder proximity).

RNGFND1_TYPE27,0.1–180m limits/serial settings and existing offsets remain.
RNGFND2_ORIENT25/ADDR0 already matched. FLOW_TYPE6, its CAN/calibration settings,
front PRX1_TYPE2, primary/backup EKF/GPS/compass sources/options, VISO settings,
avoidance tuning, arming checks and JT16 recording-only role were preserved.
Post-reboot native full scoped readback verified exactly those same six
changes and no other audited differences (see preservation.json). No arm,
mode, source-selection or mission command was sent. Rollback is saved in
applied/restore.param; restoring the former downward assignment also requires
reviewing the actual physical mount, not blindly importing old settings.

Ground reception:35sread-only USB audit returned173down readings, sensorID1,
orientation25,15–16cm (median15cm), largest report gap0.21045s. It returned173rear
proximity readings, ID14/orientation4,17–18cm (median18cm), same maximum gap.
Optical flow returned173samples with median quality86. The legacy OPTICAL_FLOW
ground_distance field is EKF HAGL (median~0.308m), not a raw LiDAR range; no
calibration/scale or fusion success is inferred from it. The existing
stationary audit now includes the correct slot2 range parameters plus rear
slot1 type/orientation.

Current rear detection is only17–18cm away. Check whether that is a real bench
object or the aircraft itself before relying on rear coverage. H-Flow ground
clearance RNGFND2_GNDCLR0.2m and TF03 offsets are preserved/unmeasured after this
mount change. Configured maximum ranges are ratings, not demonstrated flight
performance. Planner lookahead remains3m and clearance2m with existing altitude
and ground gates; this does not command avoidance on the bench or in every mode.

Front telemetry issue found during the audit: combined proximity GCS messages
can use the180m rear limit as an unavailable-front value. Native ToF viewer now
rejects distances outside its actual1–500cm range from accepted plots/count/age;
raw RX remains and unavailable_cube_reports counts them. Self-test checks95/500
accepted and0/18000 rejected. Outgoing ToF acquisition/aggregation/thresholds
were not changed by this work. A parallel operator-authorized ToF chat then
moved the front transport from GPIO to the existing USB owner; those unrelated
changes are preserved. Our configuration preservation claim concerns the Cube
backend/parameters, not freezing that separately managed transport.

JT16 recovery investigation: after service restarts its reader stopped producing
scans and mapper hit its restart-rate limit. The operator confirmed JT16 power.
Adapter identity and3Mbaud matched saved configuration. A four-second exclusive
native byte probe captured740171bytes with9603EEFFheaders; after clean port reopen
and owner restart, point acquisition recovered near5Hz. No LiDAR firmware,
calibration, baud change, USB reset or Cube parameter change was needed. Exact
root cause of the transient port/decoder state was not conclusively isolated.
The mapper was reset/restarted only after actual point data returned. Recovery
samples and final health are retained separately. No fake arm state was used.

Verification:
- Native rangefinder PTY test: exact scoped writes, durable rollback/readback,
  source preservation, zero writes/reboot when armed or slot occupied.
- Existing native Cube configuration integration regression passes.
- Native ToF full self-test passes, including the far-limit sentinel guard.
-51focused owner H-Flow/VIO/recording tests pass.
- Full owner suite retains only two pre-existing failures: generated-data
  .gitignore expectations and docs/JT16_VIEWER.md external-workspace references.
- Recording/VIO/GPS reference/beep semantics were not changed.

This verifies configuration and ground data reception. It does not prove flow
scale/noise/timing calibration, rear target-direction response, EKF flight
fusion, stopping/avoidance response or autonomous mission safety. Front/rear
beams still leave side coverage unknown. See ../../docs/hflow-down-rear-rangefinder.md.

Final recovery observation:151JT16frames in30s, all31sampled states connected, maximum point-frame age195ms;502VIO serial writes. A separate subscriber confirmed live FAST-LIO poses after restart. Five required services are active. Front ToF transport now follows the parallel shared-USB integration; it was not reverted to GPIO.
