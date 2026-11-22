# Stationary VINS frame health investigation — 18 November 2026

Read-only investigation. No parameter, gate, service or estimator changes. The existing hybrid profile remained active: VIO horizontal position, optical-flow horizontal velocity, barometer height, VIO vertical velocity, internal-compass yaw. All 45 observed snapshots were disarmed. Power query returned 25 W, so this is not a 15 W benchmark.

## Direct comparison

Fresh 45.08-second observation, 443 completed VINS samples (~44.55 s capture span), matched to the existing sole serial owner's timestamped Cube health and actual write events:

| Median metric | Published by VINS (267) | Hard-rejected (52) | Valid, waiting for recovery (124) |
|---|---:|---:|---:|
| Persistent features | 138 | 132 | 139 |
| Backend tracks | 136 | 130 | 137 |
| Geometry-checked stereo matches | 50 | 49 | 49 |
| Capture age at pipeline completion | 109.440 ms | 158.859 ms | 114.190 ms |
| Processing time | 93.603 ms | 137.172 ms | 94.473 ms |

No sample failed visual support (40 persistent / 20 backend / 20 stereo), IMU-health or initialization checks. Even hard-rejected frames had >=80 persistent features and >=41 stereo matches. The frame-age logged at CSV completion is slightly later than the gate's decision timestamp; rejection-mask bits are authoritative at boundary values.

41 samples exceeded the 150 ms age limit. 17 had invalid velocity; all 17 followed processed device-frame intervals >150 ms (166.865–266.984 ms). Six failed both checks, hence 52 unique hard rejections. Velocity is currently derived from successive camera positions, and live.cpp explicitly invalidates it when their interval exceeds .15 seconds. Stationary motion does not bypass timing requirements.

Another 124 samples passed current hard/support checks but were withheld during the live 500 ms continuous-good recovery qualification. There were 19 recovery completions and zero reset-counter changes (5->5). Recovery events here are not VINS reinitializations or origin resets.

## Actual Cube correlation

90 Cube health reports: 59 healthy, 31 unhealthy. Age since the most recent actual Jetson serial pose write:

- Healthy reports: median62.892 ms, p95234.671 ms, maximum302.224 ms.
- Unhealthy reports: minimum321.191 ms, median705.170 ms, maximum2023.869 ms.

The owner wrote174 poses. The largest interior write interval was2030.649 ms. Every observed unhealthy report followed an extended delivery gap. Healthy timing near302 ms is consistent with serial/message sampling latency and should not be treated as an exact independent measurement of the Cube watchdog threshold. Host write events are not per-packet receipt proof; Cube health is separately measured from SYS_STATUS. No guard profile block appeared; profile was vio_optflow_compass throughout.

## Five-minute context

3026 frames:2059 published,315 hard-rejected,652 qualifying. No visual-support, IMU-health or initialization failures.252 age failures,117 velocity failures; every invalid velocity followed >150 ms processed device-frame gap. Published and hard-rejected frames both had median49 stereo matches. Reset counter changed4->5 once outside the fresh45-second interval; this report does not claim all five minutes were reset-free.

Example adjacent captures: published frame135 persistent/52 stereo/134.724 ms; next rejected frame144 persistent/53 stereo/163.824 ms; next frame111 persistent/54 stereo/135.531 ms passes instantaneous checks but waits for recovery. More visual matches did not rescue the late frame.

## Interpretation and next work

The measured difference is processing/delivery timing, rather than feature scarcity in this stationary scene. Both frontend and backend processing were slower in the rejected group (medians56.715/66.426 ms versus41.074/40.808 ms published). Concurrent VINS, mapping, LiDAR and desktop processes were active. This is evidence of timing variability, not proof of which process causes it; a controlled scheduling/load comparison is still needed before assigning causality.

Fix latency variability first, investigate robust estimator-provided velocity rather than rejecting a position difference solely because of a processed-frame gap, and validate recovery policy against real interruptions. Do not increase feature acceptance or replay stale poses to conceal the delivery gaps. The earlier brief-recovery candidate remains diagnostic; no active gate was changed.

Raw stereo images were not being saved during this unarmed interval (recording is arm-triggered). Therefore this report compares per-frame visual tracking metrics and timing, not pixel brightness/contrast/sharpness. No image-level similarity claim is made.

Artifacts: analysis.json contains complete numerical results; live.jsonl fresh snapshots; window.json exact monotonic range; tracking.csv and recovery_comparison.csv copied producer logs; cube-events.jsonl existing serial-owner log copied at observation end. collect.js/analyze.js reproduce collection and analysis without adding runtime services or Python code.
