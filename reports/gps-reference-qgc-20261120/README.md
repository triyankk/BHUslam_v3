# QGC navigation-reference status and arming audit —20 November2026

Added fixed MAVLink STATUSTEXT messages through the existing sole Cube writer.
Native C++ initialization/pose processing stays unchanged. No new process, Cube
parameter write, source switch, mode, arm or motor command was introduced.

QGC Messages text includes waiting for boot/options, qualified GPS, stable GPS,
EKF origin and Home; origin/Home mismatch and retry-limit notices; and
`NavRef: GPS reference verified (origin + Home)` after validated readback/cue.
Messages have at least2s separation and unchanged state repeats every30s for
late GCS connections. INFO waiting messages and NOTICE reference success do not
claim flight readiness. Conflicts use WARNING. Text is <=50 ASCII bytes. A stale
boot/options readback suppresses a verified notice; changed origin/Home clears
its qualification. Existing ten rapid recording-ready beeps and three slow
reference beeps are retained.

The current owner is sysid1/component196. Standard Cube MAVLink routing forwards
broadcast STATUSTEXT to other learned active links unless forwarding is disabled.
Actual serial transmission is logged; QGC was unavailable, so visible display
was not confirmed. No forwarding option was changed.

Verification:32 focused transport tests pass, including message packet encoding,
rate limiting, other-profile isolation, stale boot and changed Home handling.
The full suite retains its two pre-existing failures (.gitignore generated-data
expectations and docs/JT16_VIEWER.md external-workspace text). Changed-file
whitespace check passed.

A45s settled observation remained disarmed, forwarded419 poses, reported the
Cube pre-arm health flag true45/45samples and visual health true44/45samples.
GPSfix_type1, satellites0, origin/Home absent; zero Home commands. Waiting text
was actually sent/logged. See summary.json, live-samples.json and sent-texts.json.

Arming audit (bounded read-only requests on owner connection) confirms:
ARMING_NEED_LOC0, ARMING_SKIPCHK304374, FENCE_ENABLE1, FENCE_TYPE1 (altitude fence).
ARMING_CHECK is unavailable on this4.7firmware; SKIPCHK replaces its old role.
Skip mask already existed in the prior readback and was preserved. It skips
barometer, compass, INS, parameters, RC channels, board voltage, logging,
system, rangefinder and VisualOdometry checks. GPS lock is not skipped. Therefore
pre-arm healthy under this mask is not evidence that the skipped systems passed.
No skip bit or arming parameter was changed during this work.

Installed firmware's Copter gps_checks requires GPS lock only when AHRS uses GPS
and the mode/fence requires position; mandatory position/AHRS checks still run.
The selected hybrid uses VIO position, optical-flow velocity and compass yaw,
so lack of GPS fix does not by itself prevent arming. The current ALT_HOLD
pre-arm flag passes without fix, but this is not an actual arm test. Position
modes still need a usable estimator; Home/RTL and the fresh-reference startup
procedure have separate requirements. Booting without any GPS never initialized
is not a validated GPS-free mission startup.

Operator chose to keep Here4 connected and QGC was unavailable. Physically
absent GPS, QGC display, outdoor origin/Home qualification and subsequent
disconnection remain untested. Do not claim unconditional armability or flight
readiness. No forced arming or bypass of checks was attempted.

Firmware references:
- https://github.com/ArduPilot/ardupilot/blob/dbe79216/ArduCopter/AP_Arming_Copter.cpp#L324-L369
- https://github.com/ArduPilot/ardupilot/blob/dbe79216/libraries/AP_Arming/AP_Arming.cpp#L176-L182
- https://github.com/ArduPilot/ardupilot/blob/dbe79216/libraries/GCS_MAVLink/MAVLink_routing.cpp#L59-L73
