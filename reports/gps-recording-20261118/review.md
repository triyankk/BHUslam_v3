# GPS-aided recording configuration and later Here4 removal — 18 November 2026

## Current scope

Use GPS/Here4 compass for the observation flight while VINS-Fusion and independent JT16/FAST-LIO2 warm up at boot and record on arm. No VIO navigation/fusion is enabled. The operator confirmed the entire Here4, including its compass, will be removed for the later GPS-free phase; no independent compass will remain.

Native read-only inspection found Copter 4.7.1 (`dbe79216`), Cube disarmed, primary position GPS but horizontal velocity optical flow. The requested GPS profile changes `EK3_SRC1_VELXY` 5 -> 3 and `EK3_SRC1_VELZ` 0 -> 3. `VISO_TYPE=0` is explicitly requested and already present. Primary source configuration is therefore POSXY=3, VELXY=3, POSZ=1, VELZ=3, YAW=1, OPTIONS=0. This uses GPS position/velocity, barometric height and compass yaw. No source-selection command is sent; configuration readback does not itself prove which source is actively selected. Power-cycle before flight and review Cube BIN `XKFS.SS=0` after flight.

Source sets 2/3, GPS backend, compass, optical-flow backend, rangefinders, RC mappings, avoidance tuning, source options, arming/failsafe and notification settings are preserved. `PRX1_TYPE=4` and BendyRuler remain as previously requested; this task does not validate avoidance. Current transport is the verified USB endpoint, not the unresolved GPIO/TELEM2 transmit path. USB must remain attached in flight with this installed profile.

All five services were enabled and active, with user lingering enabled: existing sole Cube/JT16 owner, VINS-Fusion, JT16 mapping, native observation relay, performance recording. VINS runs native C++ stereo/IMU with mild CLAHE 2, 200 features/25 px spacing, 500 ms qualification and 150 ms freshness. JT16 scans were arriving at about 5 Hz and FAST-LIO2 logged IMU initialization/map-tree startup. Proximity health was false; decoded scans alone do not prove Cube proximity reception. The VIO gate was not continuously qualified in this stationary scene and calibration/fusion remain unvalidated.

Arm/disarm recording and ten rapid recording-ready beep behavior are unchanged. Arm creates each sensor's flight files; fresh disarm closes after the existing 3-second tail. Leave power on through finalization. Loss of telemetry is not treated as disarm. No arming or audible cue was triggered for verification. Disk free was approximately 290 GiB. Current startup power report is 15 W. Estimation is independent; JT16 is not a VINS tracking fallback.

At audit time GPS fix_type=1 (no fix), zero satellites, and the owner reported GPS bad-fix and GCS failsafe pre-arm errors. Cube mode was RTL while disarmed and was not changed. Configuration completion is not permission to arm with these faults.

## Later GPS-free VIO setup — NOT APPLIED

These deltas are relative to the GPS recording profile, for barometer height with VIO position, velocity and yaw:

| Parameter | GPS recording | Future VIO | Purpose |
|---|---:|---:|---|
| VISO_TYPE | 0 | 3 | Enable MAVLink visual odometry backend |
| EK3_SRC1_POSXY | 3 | 6 | ExternalNav horizontal position |
| EK3_SRC1_VELXY | 3 | 6 | ExternalNav horizontal velocity |
| EK3_SRC1_VELZ | 3 | 6 | ExternalNav vertical velocity, only after validated |
| EK3_SRC1_YAW | 1 | 6 | ExternalNav yaw, necessary without compass/GPS |
| GPS1_TYPE | 9 | 0 | Disable physically removed Here4 GPS |
| COMPASS_ENABLE | 1 | 0 | Disable absent compasses; do not bypass arming checks |

Keep `AHRS_EKF_TYPE=3`, `EK3_ENABLE=1`, `EK3_SRC1_POSZ=1` (barometer), `EK3_SRC_OPTIONS=0`, `GPS2_TYPE=0`. Keep measured mount `VISO_POS_X/Y/Z=0.13/0/0.05`, `VISO_ORIENT=0` and `VISO_QUAL_MIN=40`. The native transform already compensates 22.8 degrees camera downward pitch; do not apply that rotation twice in the Cube. `COMPASS_USE/USE2/USE3` need no additional edits when the entire compass subsystem is disabled. Do not clear compass calibration/device IDs for this experiment.

For an initial XY-only ground/SITL stage, `EK3_SRC1_VELZ=0` is an alternative that explicitly omits external vertical velocity. Using visual height instead of barometer would additionally require `EK3_SRC1_POSZ=6`; neither is selected automatically. Source sets 2/3 are deliberately not converted: they currently contain GPS/compass sources and are not viable fallbacks after Here4 removal. No automatic switching, RC option 90 or scripting is introduced.

Current readback `VISO_DELAY_MS=10`, position-noise floor 0.2 m, velocity-noise 0.1 m/s, yaw-noise 0.2 rad are existing provisional settings. Determine delay/covariance/noise from actual latency and calibrated flight data; no defensible replacement numbers are known. Preserve source/failsafe/arming checks until a separate validation determines justified changes. Do not relax gates to conceal VisOdom failures.

### Origin, home and heading

The live `AHRS_OPTIONS=0` and saved `AHRS_ORIGIN_LAT/LON/ALT=0/0/0`. A real EKF origin must be set before GPS-free positioning; there is no usable saved location currently. Set the actual site coordinates and MSL altitude using the ground controller/authorized origin setup, and establish a consistent Home. A fake 0,0 location does not preserve a geographic mission.

Copter 4.7.1 supports optional persistent origin: `AHRS_OPTIONS |= 8` before a later GPS-origin acquisition records it to `AHRS_ORIGIN_*`; `AHRS_OPTIONS |= 16` permits restoring a verified saved origin in non-GPS operation. From current zero these are 8 and 24 for record+restore, respectively. They are NOT enabled by this task. Firmware records on first origin acquisition, so setting the record bit after an origin already exists may require another boot/acquisition. Verify nonzero saved values before enabling restore; update them when moving site. Manual per-flight origin setup is also valid.

VINS-Fusion currently defines local yaw from its startup camera direction. Removing compass/GPS leaves this heading relative, not geographically north-referenced. A mission planned as latitude/longitude needs a surveyed/otherwise independently known launch heading and a consistent translation/rotation between VIO local coordinates and geographic navigation. Saved origin alone does not supply north heading. Validate yaw, all translation/velocity axes and the camera lever arm before fusion. Do not seed north from an unreferenced Cube yaw and claim alignment.

### Software and validation still required

Subsequent bridge update: the native relay and existing sole-owner adapter now recognize exact GPS-observation and GPS-free ExternalNav profiles, plus the disabled GPS-recording profile. See ../dual-profile-bridge-20261118/review.md. The GPS-recording profile remains blocked. The VIO parameter candidate is still unapplied and unvalidated for flight. Profile changes must qualify while disarmed; no second serial owner or automatic source change is introduced.

Validate steady >=4 Hz ODOMETRY reception (current maximum is 10 Hz), actual Cube VISP/VISV reception, external yaw alignment, continuous EKF fusion/innovation and active-source evidence, correct origin/home, and loss/restart behavior in simulation and props-off ground tests first. Existing 300 ms Cube visual-health timeout and 500 ms requalification can still yield unhealthy periods; stale poses are not a remedy. Previous flight input had accelerometer clipping and VINS divergence, and calibration is still marked unvalidated. Mount/vibration, camera/IMU calibration and motor-on tracking must be addressed before reliance on VIO control. No universal parameter set can make a lost or inaccurate pose safe.

## Applied result

The native tool applied exactly two writes: `EK3_SRC1_VELXY=3` and `EK3_SRC1_VELZ=3`, each verified by named readback. All scoped unrelated parameters were checked equal before/after. `VISO_TYPE=0` remained unchanged. No reboot or source-selection command was issued. The native integration fixture passed against the final binary in 14.57 seconds. Independent after-audit compared 81 returned parameters and found only the two intended changes (see preservation.json for the exact count). The owner was restored, all five services were active, Cube remained disarmed, recording thread was healthy/arm-triggered, and serial VIO write count was zero. JT16 resumed at approximately 4.9 scans/s; Cube proximity remained unhealthy. No live arming test was performed.

## Evidence and rollback

`before-audit.json` is the independent read-only firmware/parameter audit; `installed-units.txt` captures actual installed boot services. `config/` contains durable before/requested/after snapshots, verified changes, summary and `restore.param`. `gps-recording.param` is the narrow current profile. `gps-free-vio-NOT_FOR_FLIGHT.param` is an unapplied candidate, never installed or automatically loaded. `integration.log` records native simulated-Cube preservation/reboot/command-policy tests. Post-application state and cross-audit preservation are saved separately.

References: [ArduPilot ExternalNav configuration and ODOMETRY](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html), [Copter 4.7.1 AHRS persistent-origin implementation](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_AHRS/AP_AHRS.cpp), [compass subsystem parameters](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_Compass/AP_Compass.cpp). The compass-less GSF GPS fallback is not applicable after removing GPS.
