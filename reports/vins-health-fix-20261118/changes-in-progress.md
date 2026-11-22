# VINS health fixes, 2026-11-18

Operator explicitly disabled forwarding and changed EKF configuration for GPS flight. Preserve that state. No Cube parameters or source-selection commands may be issued by this work.

Changes under validation:

- DurationGate: retain publication hard checks and 500 ms recovery, separate publication recovery from estimator/frame reset. Monitor finite initialized poses even when visual support is withheld. Reset only for changed estimator epoch, position/attitude discontinuity, or an unobserved interval >1 s. Bounds are provisional engineering checks, not calibrated uncertainty.
- Native stereo tracker: optional rectified policy seeds reverse LK with the original left observation; retains the 0.5 px forward/backward test and requires positive disparity and <=1 px vertical error. Upstream files remain unchanged; CMake patches a generated build copy and fails if the expected reverse call is absent.
- Explicit feature-count/distance knobs for controlled native A/B replay; defaults stay upstream until evidence supports selection.
- Replay accepts both seven-column capture and five-column arm/disarm recorder frame CSVs. Offline executable has no navigation transport.
- Live status/tracking add support-specific reasons, recovery count, reset counter and reset causes.

Cube 4.7.1 reports unhealthy when no visual update arrives within 300 ms. Retaining 500 ms qualification means genuine support outages still produce unhealthy intervals. Never retransmit old or unqualified poses to keep the watchdog green.

Primary source: https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_VisualOdom/AP_VisualOdom_Backend.cpp
