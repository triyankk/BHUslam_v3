# Current stereo scene and VisOdom pre-arm diagnosis — 2026-11-18

The current scene does have stereo matches. Before capture, the most recent 300 live VINS frames had 19–41 matches, median 30; 292/300 passed the complete publication gate. Median estimated capture age was 70.593 ms. This differs from the earlier low-match/old-frame period and must not be described as a permanent camera/stereo failure.

## Capture and C++ matching analysis

Cube freshly confirmed connected/disarmed before briefly pausing VINS. Saved 8.007 s / 241 actual 640x400 stereo pairs plus native frames and built-in IMU at 15 W. Left/right timestamp differences: median 0 us, maximum 1 us; zero timestamp rejections and zero combined frame-index gaps. SDK timestamps are not independently verified exposure time. Native camera tool sent no Cube cue, parameter, mode, source or arm commands. VINS was restored automatically.

Current view: dark, glossy floor and furniture legs. Raw mean intensity is approximately 34.5/255 left and 32.1/255 right; most pixels are not clipped black, but contrast is compressed. Illumination is weak and floor reflections differ between views. Scene diagnosis is an inference from the images, reject stages and successful matches, not a controlled lighting experiment.

Analyzed 16 evenly spaced pairs using C++ goodFeaturesToTrack plus forward/backward LK and optional CLAHE:

| Diagnostic | Original IR median | Mild CLAHE 2 median |
|---|---:|---:|
| Corners | 38 | 150 |
| Forward/backward checked stereo matches | 10 | 25 |
| Round-trip error >0.5 pixel rejects | 20 | 102 |
| Forward LK failures | 7 | 12 |
| Reverse LK failures | 3 | 7 |
| Right-image border rejects | 0 | 1 |
| Positive disparity and vertical error <1 px | 8 | 15 |

Per-frame rejection counts were checked to reconcile with all input corners. Most enhanced corners fail the 0.5-pixel left-right-left consistency test: extra visible corners are not necessarily stable stereo correspondences. Some correct raw matches have approximately 0.13-pixel median vertical disagreement, so these samples do not demonstrate a gross left/right ordering or rectification error. Outliers remain, especially after enhancement; calibration correctness is not established by this diagnostic. Mild enhancement helps but cannot create real texture or remove specular reflections. No live enhancement, exposure, calibration or gate parameter was changed.

Snapshot matcher counts differ from live VINS counts because VINS retains temporal tracks and uses its own point selection. To check the actual library, replayed 240 saved frames through the native VINS-Fusion C++ adapter with existing mild CLAHE 2: 230 initialized frames, 214 gate-eligible frames after initialization/recovery, zero failure resets. Offline replay age is artificial zero and is not evidence of live timing or accuracy. This stationary sample confirms stereo can work before arming.

## Pre-arm error reproduction

The actual Cube still reports VisOdom not healthy while valid VINS poses are being published locally. Both forwarding guards report a stale parameter audit (AHRS_EKF_TYPE and other source/backend reply ages exceed the five-second limit). Latest actual serial write was more than 20 minutes old at final observation. Cached values match the expected GPS-position/optical-flow-velocity source set 1 and EK3_SRC_OPTIONS=0, but stale values do not prove unchanged current configuration. This transport/audit failure alone explains the persistent timeout; do not attribute it solely to stereo matching. The underlying cause of delayed parameter replies has not been isolated by this scene test.

Offline simulation replayed the official firmware's health condition, update age <300 ms, against actual serial-write and Cube health events. At age 299 ms the model is healthy; at 300 ms and later it returns VisOdom not healthy. Latest long no-write interval predicts the same unhealthy state reported by Cube. Overall proxy agreement in the captured log was about 89.7%; host write time is not Cube arrival time, and serial/telemetry delays are omitted. This is a reception-watchdog replay, not full ArduPilot SITL, a complete pre-arm simulation, or proof of fusion. No synthetic pose or health message was sent to the real Cube. Details: prearm-timeout-simulation.json.

The next engineering fix should target reliable parameter audit/forwarding, separately from image support. For image testing, retain mild enhancement and compare with a matte textured scene under better illumination. Do not lower the 20-match gate or relax the Cube 300 ms timeout to mask the delivery fault. Arm/disarm recording and buzzer behaviour, Cube source sets, arming checks and power mode were preserved. No flight-readiness claim.

## Artifacts

- capture/: native/rectified stereo, IMU, factory metadata and timing.
- quality-stages/: measured brightness, rejection-stage counts, correspondence overlays. Green points indicate forward/backward consistency, not guaranteed correct geometry.
- vins-offline/: native C++ replay metrics and viewer.
- tracking-before.csv and owner/relay snapshots: actual live state before the diagnostic pause.
- findings.json and prearm-timeout-simulation.json: summaries and limited simulation.

Firmware references: [VisOdom reception health](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_VisualOdom/AP_VisualOdom_Backend.cpp), [300 ms timeout constant](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_VisualOdom/AP_VisualOdom.h), [VOXL pre-arm attitude checks](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_VisualOdom/AP_VisualOdom_IntelT265.cpp).
