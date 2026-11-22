# BendyRuler refinement resumed — 21 November 2026

Completed the verification interrupted in the previous session. Readback on
the connected, disarmed Cube confirmed the requested 3 m OA_BR_LOOKAHEAD and
2 m OA_MARGIN_MAX persisted. All 23 audited settings matched the prior
post-change record, including AVOID_MARGIN=2 m, horizontal BendyRuler, and
PRX1_TYPE=2. The firmware still reports commit dbe79216. No parameter, mode,
arming, EKF-source, mission or reboot command was sent during this verification.

The prior runtime JT16 mask had cleared after restart, and the optFlow/JT16
service again owned Cube USB. A fresh, disarmed status was checked before
reinstating the previously authorized current-boot mask at
`/run/user/1000/systemd/user.control/optflow-flight-logger.service` and stopping
that service. JT16 was not restored. One native ToF viewer now owns primary
Cube USB and I²C7. Camera/VINS and the VIO-observer services remain active,
but their former shared VIO-to-Cube relay is paused with the JT16 link owner.
The mask remains current-boot only; startup policy is not a persistent ToF service.

Live API verification showed fresh sensor readings and front proximity-sector
reports, with zero malformed packets. Independent pymavlink decoding checked
outgoing front packets and received source 1/component 1, sensor ID 10,
orientation 0. Raw measurement logs were independently checked against the
mean of primary zones with confidence strictly above 50. Counts at verification
are in verification.json; the live capture continues to grow.

The current profile remains a supporting bench milestone toward GPS-denied
mission flight. No physical 1.5–2 m target sweep, stopping-distance test, or
autonomous path change was performed. The original report describes the
sensor/vehicle reference and forward-only coverage limitations. A 2 m planner
margin is not a guaranteed sensor-referenced reaction boundary.

Evidence: readback.json, runtime-identity.json, api-snapshot.json,
verification.json, and live-viewer/bridge-001/.
