# Front-ToF BendyRuler refinement — 20 November 2026

The operator requested a 1.5–2 m obstacle-clearance target measured from the
front sensor. Applied a disarmed bench profile, supporting the project goal of
GPS-denied autonomous missions without claiming demonstrated path avoidance.

| Parameter | Before | After | Purpose |
|---|---:|---:|---|
| OA_BR_LOOKAHEAD | 5 m | 3 m | Reduce distant planning horizon while retaining advance visibility |
| OA_MARGIN_MAX | 1.5 m | 2 m | Increase the planner clearance target |

These were the only parameter writes. Both were read back immediately and
again during the complete post-change audit. All other observed parameters
were unchanged. No reboot, arming, flight-mode, mission, or EKF-source command
was issued. The saved configuration records the fresh disarmed heartbeat.

Horizontal BendyRuler (OA_TYPE=1, OA_BR_TYPE=1) was already enabled. The
simple-avoidance settings remain AVOID_MARGIN=2 m, AVOID_BEHAVE=1, AVOID_ENABLE=7.
Planner continuity remains ratio 1.5 and angle 75 degrees. The obstacle database
remains size 100, queue 80, expiry 5 s; it was not disabled or shortened to
pretend that unobserved space is free. PRX1_TYPE remains 2 (MAVLink).

Current gates also remain AVOID_ALT_MIN=2 m and PRX_IGN_GND=1. OA_DB_ALT_MIN=1.5 m
can reject observations below that altitude in the 3 m region around home.
These are altitude gates, not obstacle-distance trigger settings. The earlier
WPNAV_SPEED, WPNAV_ACCEL and WPNAV_JERK names did not return values on this
firmware; they were not treated as zero or changed.

## Interpretation and limits

The 3 m lookahead and 2 m margin are not an exact 1.5–2 m reaction window.
The matching firmware adapts its current lookahead and tests candidate path
segments against obstacle margins. Planning can begin before the vehicle
reaches the requested clearance. Sensor/vehicle reference offsets, uncertainty,
velocity, braking distance and delay need physical validation; setting a
parameter does not establish a guaranteed 2 m clearance from the sensor or
propellers during a turn.

The current front bridge still reports the operator-selected arithmetic mean
of valid primary zones with confidence >50. This step did not silently replace
that rule with nearest-zone selection. A small close obstacle can be diluted
by farther zones; nearest reliable distance is the proposed collision-protection
input to evaluate separately.

One forward ToF does not verify free side/rear corridors or the aircraft's
swept volume. The selected project plan requires simulation/shadow validation
and observed free space before autonomous detours; unknown directions must
not become assumed-clear paths. The shared VIO-to-Cube relay remains paused
after the prior authorized JT16 handoff. Working localization/goal delivery
and the installed firmware's applicable navigation mode remain prerequisites
for a mission/path-changing test. No such flight test was performed here.

The sole ToF USB owner and viewer were resumed with their previous front,
direct-I²C, confidence-51 mean settings. JT16 remains runtime-masked and was
not restored. Effective bridge settings and executable hash are saved locally.

## Evidence

- audit-before.json: current parameter backup and unavailable names.
- configuration.json: exact requested writes and complete after-readbacks.
- identity.json: executable hash and scope.
- live-viewer/: resumed capture and API snapshot.
- firmware-AP_OABendyRuler.cpp: matching source used to check behavior.

[ArduPilot BendyRuler guide](https://ardupilot.org/copter/docs/common-oa-bendyruler.html)

[Installed-commit BendyRuler source](https://github.com/ArduPilot/ardupilot/blob/dbe79216/libraries/AC_Avoidance/AP_OABendyRuler.cpp)
