# VINS-Fusion health corrections — 18 November 2026

Software corrections are installed in the native C++ pipeline. The operator's disabled forwarding and Cube EKF configuration are preserved. This does not establish flight readiness or validate VIO fusion.

## What the flight actually showed

The original warm live estimator became persistently invalid about **199 seconds before arming**. During the recorded flight, **all 5,567 processed frames were withheld by the velocity guard**. Estimated pose speed was physically implausible (median about 868 m/s); the matching Cube GPS-aided local-velocity estimate peaked at about 1.62 m/s. Cube velocity is corroborating estimator telemetry, not ground truth. Stereo counts alone did not explain this flight failure: no initialized frame in the first 1,800-frame live segment had fewer than 20 stereo matches, and capture-age rejection was absent in that segment.

The camera acceleration recording contains **1,967 samples at its digital rails (4.6%)**, with **2,043/42,690 samples within 1% of a rail (4.8%)**. The SDK advertises only +/-4 g acceleration and +/-1000 deg/s gyro ranges on this connected camera. Additional sample rates are advertised, but neither range, rate, calibration nor noise settings were changed. The acceleration rails are approximately -39.2/+39.1988 m/s². Acceleration norm median was 18.1 m/s²; raw gyro norm median was 2.06 rad/s. These observations warrant investigating camera mounting/vibration. They do not prove vibration is the sole cause of the earlier pre-arm divergence. Camera/IMU calibration and initialization remain unvalidated.

Raw recordings remain available, but this original VINS pose trace is not a valid flight trajectory. Filtering the already clipped samples cannot recover their missing motion.

Evidence: `original-flight-health.json`, `flight-imu-limits.json`, `cube-flight-motion.json`, `camera-profiles.log`. Original data: `runs/vins-fusion-boot/20261118T091238.639064632Z/live/flights/flight-331497527/input/`, its parent live logs, and the corresponding Desktop Cube recording `18-11-2026_14-47-05_071`.

## Implemented corrections

- **Separate recovery from reset.** The duration gate still stops immediately on rejection and requires 500 ms of valid recovery. Well-observed, continuous poses preserve the MAVLink reset counter after a temporary publication interruption. Estimator epoch changes, measured pose jumps and long unverified intervals still announce discontinuity. Pose prediction spans short processing stalls up to one second; the prediction bounds remain provisional (0.20 m + 5*dt², 15 degrees + 10*dt² radians). Longer intervals are explicitly unverified. Capture freshness remains **150 ms**; this continuity horizon does not permit sending a stale pose.
- **Recover a stuck estimator only on the ground.** Two seconds of initialized but invalid pose velocity, with adequate visual support and no current clipping holdoff, can trigger reinitialization only with freshly observed disarm and inactive recording. A ten-second cooldown prevents rapid repeated attempts. Armed, stale/disconnected state, recording and disarm tail suppress recovery. New epochs and reasons are logged. This recovers from a bad retained state; it does not calibrate the sensor or cure bad measurements.
- **Detect suspect IMU rails.** Native capture counts near-rail samples and withholds navigation for 200 ms after them. Raw input recording continues. The gate records unhealthy IMU as rejection bit 64. This is a clipping guard, not a general vibration/noise-quality certification.
- **Improve stereo validation and feature selection.** Reverse LK is seeded at the known left observation. The existing 0.5 px round-trip threshold remains, and the rectified policy requires positive disparity and <=1 px vertical error. The selected live profile uses 200 features / 25 px spacing / CLAHE 2. No support threshold was lowered. The upstream source is untouched; CMake generates a checked build copy.
- **Expose the cause.** Live status/tracking now report support and hard rejection masks, pose speed, gate reason, clipping counts, publication recoveries, reset causes and disarmed reinitialization count. Estimator recovery events are in `estimator_events.jsonl`.
- **Keep replay usable.** The C++ replay reads both seven-column diagnostic capture and five-column arm/disarm recording manifests. Offline replay never transmits navigation. Its IMU rail check now mirrors the live holdoff for these camera recordings.

## Selected frontend evidence

These are actual VINS feature-tracker tests, not snapshot corner counts. A supported frame in this table means >=40 persistent features and >=20 geometry-checked stereo matches. It does **not** prove backend motion validity, calibrated accuracy or full publication eligibility.

| Recording sample | Pairs | Valid stereo matches, median | Frontend supported frames |
|---|---:|---:|---:|
| Stationary desk | 241 | 26 | 239 (99.17%) |
| Earlier outdoor transitions, first 1,800 pairs | 1,800 | 188 | 1,798 (99.89%) |
| Today's flight, first 1,800 pairs | 1,800 | 43 | 1,798 (99.89%) |

The old tracker had only 16/140/34 geometry-consistent matches at the median in those desk/outdoor/flight samples, respectively, although it accepted more matches without the geometry check. A 20 px spacing candidate improved the desk but caused 2.84–5.54 s unsupported intervals in the flight frontend test, so it was rejected. The selected 25 px spacing avoided that regression. The flight prefix spans approximately 76 seconds because recorded images were the processed selections, not every nominal camera frame.

Full stationary estimator replay with the selected profile produced 231 initialized frames and 188 eligible frames, zero reset-counter changes and one recovery. The older profile produced 199 eligible frames. Consequently this work claims improved correspondence validation and ground recovery, **not universal improvement in publication availability**. Offline backend timings/iterations were affected by concurrent host work; use the final live check for timing, and do not treat the offline trajectories as ground-truth accuracy tests.

Evidence: `selected-frontend.json`, the `*-frontend/` directories, `desk-selected200-spacing25/`, and the rejected candidate directories. Earlier full flight-prefix replay diagnostics predate the final clipping holdoff and final short-stall counter correction; they establish the invalid-motion problem, not final-policy availability.

## Validation and operating limits

Nine native VINS tests passed, including actual stereo tracker checks against known-disparity/reversed/vertical/blank pairs, clipping/holdoff, disarmed-only recovery/cooldown, continuous-pose recovery versus real position/attitude/epoch resets, recording and cue behavior. The recording/cue test initially failed in the restricted sandbox and passed with host loopback access; it uses an ephemeral test port and sends no real Cube commands. The native MAVLink conversion/protocol test also passed. The additional coherent-short-stall regression passed after the first live check exposed that case. See `ctest-final.log`, `gate-final-test.log` and `protocol-test.log`.

The first 120-second live stationary check at **15 W** processed 1,775 monitored frames, with 1,756 eligible (98.93%), median stereo count 34 and capture age p95 105.67 ms. Four frames breached the 150 ms age guard; three had invalid derivative timing. No clipping or automatic reinitialization occurred. One conservative reset followed a subsecond processing stall; that specific behavior was corrected afterward and retained as a regression test. See `live-first-summary.json` and `live-monitor.jsonl`. Final-release verification is recorded below.

The Cube's visual reception watchdog is 300 ms, while safe requalification is 500 ms. Genuine measurement or processing outages can therefore still cause VisOdom unhealthy when forwarding is later enabled. No stale packet, invented pose, fake health stream, disabled arming check or relaxed quality threshold was added to hide a fault. Firmware reference: [Copter 4.7.1 VisualOdom backend](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_VisualOdom/AP_VisualOdom_Backend.cpp).

The next flight-related increment must address mounting/vibration and then validate camera/IMU calibration and motion quality. The new clipping guard intentionally rejects suspect data. A green stationary check cannot validate motor-on vibration, airborne performance, alignment, EKF fusion or GNSS-denied operation.

## Services, preservation and rollback

Only `bhu-vins-fusion.service` was paused/restarted for the software checks. The existing Cube serial owner was not restarted or modified. It remains connected over the existing USB transport. No Cube parameter writes, source-selection commands, arming commands or real buzzer commands were issued by this work. `VISO_TYPE=0` and the operator's forwarding/source override remain in force. Recording still opens on arm, closes after the three-second disarm tail, and requests ten rapid Cube beeps only after input recording is ready. No calibration or mounting transform changed.

`bin/vins-fusion-live` carries the selected default frontend profile; explicit CLI flags override it. Boot still waits for Gemini plus a fresh disarmed Cube and uses that wrapper. New boot runs hash both wrapper scripts as well as the native executable. Runtime code is C++; no Python process was introduced.

Rollback files are `live-before`, `live-wrapper-before.sh` and `boot-wrapper-before.sh` in this report directory. Restoring a binary/wrapper requires stopping VINS while freshly disarmed with no active recording, then restoring the matching pair and restarting; it does not restore or alter Cube parameters. Source-level rebuilds should use `scripts/env.sh` and the existing serial build workflow. `release.sha256` records the new live executable and relevant sources/wrappers; `upstream-tracker.sha256` verifies the vendor tracker stayed unchanged.

## Final-release stationary verification

The final native build ran for 120.12 seconds at 15 W in the user's unchanged stationary view. Of 1837 monitored frames, **1826 (99.40%) were eligible**, with **zero reset-counter changes, zero clipping detections and zero automatic estimator reinitializations**. Median geometry-checked stereo count was 32; capture age p95 was 102.96 ms. Maximum displacement within the monitored stationary interval was 4.51 mm, ending 3.04 mm from its initial pose. This is a stationary consistency result, not independently measured accuracy.

There were still 3 age rejections (maximum capture age 191.93 ms), 2 invalid derivative-timing samples and one publication recovery. Those samples were withheld. The brief stall did **not** change the reset counter after the correction. Thus occasional genuine delivery outages remain possible; no claim is made that Cube's 300 ms health watchdog will stay continuously healthy in flight.

The Cube remained disarmed and recording inactive throughout. Actual forwarding writes stayed 0, with VISO_TYPE=0. The existing owner start time and audited source/backend parameters matched the pre-work snapshot. No source-switch or parameter-write commands were issued. The VINS service is active and remains enabled at boot; startup still waits for the camera and fresh disarmed Cube. Arm/disarm recording and the ten-beep recording cue are preserved.

The actual native backend/frontend reset sequence was additionally exercised through three repeated reset/reload cycles in the stereo integration test, with initialization, temporal tracks and retained IMU integration checked after reset. See reset-integration-tests.log.

Final evidence: final-live-summary.json, final-live-monitor.jsonl, release-final.json, final-run.txt, ctest-release.log and reset-integration-tests.log. Live run: /home/atas/BHUslamV3/runs/vins-fusion-boot/20261118T105518.703485886Z.

To reproduce the selected offline profile without camera or Cube access:

```bash
bin/vins-fusion RECORDING NEW_OUTPUT --clahe-clip 2 --opencv-threads 4 --max-features 200 --feature-distance 25 --stereo-policy rectified
```

The recorded flight is **not a successful VIO run**. Resolve the measured acceleration clipping/mounting vibration and validate initialization/calibration before another flight intended to validate VIO. Keep EKF fusion and source-switch tests separate.
