# ArduCopter-owned pose timing — 18 November 2026

Implemented the operator's explicit request to remove Jetson pose timing limits and retain ArduCopter's freshness constraints. Activated in the existing boot-enabled services after fresh disarmed/idle verification; no Cube parameter writes or source/mode/arm commands. All17 audited source/backend/mount settings match before/after. Current hybrid remains VIO horizontal position, optical-flow horizontal velocity, barometer height, VIO vertical velocity and internal-compass yaw, EK3_SRC_OPTIONS=0.

## Removed from the active path

- Producer150 ms capture-age cutoff.
- Producer500 ms initial/recovery qualification wait.
- Producer150 ms maximum interval for pose-difference velocity. It now uses the actual positive interval; finite data and the existing speed plausibility check remain. This is still an interval-average velocity, not a newly validated instantaneous velocity estimator.
- Relay ingest/take150 ms age checks.
- Serial-owner decode/final-write150 ms age checks.
- Both artificial10 Hz forwarding throttles.
- Time-gap-only synthetic coordinate resets in the immediate publication policy. Actual estimator epochs and detected position/attitude discontinuities still propagate reset counters.

ArduCopter keeps its300 ms VisualOdom update-reception watchdog, timestamp/delay handling and EKF quality/innovation/reset logic. There is no newly introduced300 ms pose-age cutoff on Jetson. Original capture timestamps and age logging are preserved, not rewritten to make a pose look fresh. Latest-only queues, monotonic unique captures, future/invalid-data rejection, initialization, visual/IMU checks, quality>=40, disarmed profile qualification, heartbeat freshness, parameter-audit freshness and armed-profile lock remain. Link/config freshness checks are independent from pose age. Missing tracking is never filled by resending the last pose. Historical age-limited gates remain available as inactive diagnostics/legacy defaults; the live producer/relay explicitly select the new policy.

The native producer remains C++; the existing sole serial owner's narrow adapter was updated without adding Python processes. Input recording, arm/disarm session splitting and ten-beep recording-ready logic were not edited. Mapper was stopped before its serial owner during reload, then both restored. No active flight recording was interrupted. Native VINS was restarted to load the new binary, so this starts a new live estimator run; that deployment restart is separate from subsequent in-run reset counts.

## Verification

C++ gate replay of the stored five-minute stationary sample:3026/3026 accepted versus2059/3026 under old baseline, including252 poses older than150 ms. Replay reconstructs pose-difference velocity, uses zero angular velocity because those logs lack it, and therefore is only gate regression evidence, not a new estimator run or EKF/accuracy proof.

Native protocol and real loopback-to-pseudo-UART tests:2/2 passed. The end-to-end fixture forwards a400 ms-old capture with its original timestamp. Nine VINS tests passed, including immediate/delayed publication, duplicate/future/IMU rejection, epoch handling, no time-gap-only origin reset, recording sessions and buzzer packet tests. Existing adapter's12 focused cases passed, including600 ms-old decode,200 ms-old final write, new captures without a rate wait, no retransmission, invalid packets and all source profiles. Initial sandbox socket tests failed because socket creation is blocked; the unchanged tests passed on the host with loopback access. Adapter diff whitespace check clean.

## Disarmed live60-second result

-657 actual serial pose writes.
-44 writes older than150 ms; median write age117.478 ms, maximum239.920 ms.
-Maximum interior serial gap202.310 ms.
-Cube VisOdom healthy in120/120 received health reports.
-No reset-counter changes within the check:0->0.
-No profile blocks; vio_optflow_compass throughout.
-Recorder thread alive in all60 snapshots; no recording errors.
-All five existing VINS/owner/forwarding/mapping/performance services active and enabled at completion.
-Power mode25 W; this is not15 W performance evidence.

The earlier stationary check had59/90 healthy reports; this separate later observation is encouraging but is not a controlled accuracy or flight comparison. Health proves the observed reception watchdog stayed satisfied; it does not prove EKF fusion, correct calibration or safe motor-on performance. Post-reset consumption holds and other Cube fusion checks remain firmware-owned.

Artifacts:status-before.json,parameter-preservation.json,live-summary.json,live-window.json,live.jsonl,cube-events.jsonl,copied live producer logs,build/test logs,power-mode.txt,release.sha256,source/binary backups,and reproducible activation/verification scripts. Deployment run path is in live-summary.json. No memory files were changed.
