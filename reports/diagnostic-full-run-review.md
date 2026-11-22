# Full diagnostic recording review — 2026-10-25

Dataset: `recordings/diagnostic/20261025T091926.822317376Z/`.
Cube/boot run: `runs/cube-record/20261025T091925.991617920Z/`.
Boot ID `581589fb-f11d-4f54-b93a-ba9a19275219` matches the dataset and current
boot. The diagnostic service completed with Result=success / ExecMainStatus=0.
The similarly timed `.502699056Z` directory belongs to an interrupted earlier
attempt and contains ARMED metadata, not this completed dataset.

## Capture passed

| Item | Measurement |
| --- | --- |
| Recording interval | 80.022746 s |
| Mode | 15W at start and end |
| Stereo pairs | 2,399 |
| Stereo rate from device timestamps | 29.964 Hz |
| Accelerometer / gyro samples | 16,364 each |
| IMU rate from device timestamps | 202.556 Hz |
| Maximum IMU timestamp gap | 4.937 ms |
| Maximum stereo timestamp skew | 1 us |
| Combined left/right frame-index gaps | 0 |
| Image interval bracketed by both IMU streams | Yes |
| Native/replay IMU count and endpoint timestamps | Match |
| Dataset disk size | About 2.4 GiB |

The native checker was rerun successfully. Its report is
`reports/diagnostic-full-run-check.json` and the dataset's recording-check.json.
QR start, MOVE, SETTLE, automatic stop and SAVED events are present. Stop reason
is duration_limit: the complete timed protocol ran, rather than a second-QR
stop. Protocol duration completion does not verify that each physical action
was followed.

Cube feedback-only mode stayed enabled: six cue transmissions, zero odometry
and zero parameter writes, zero link drops. One malformed incoming serial
packet was counted; this does not imply corruption of camera/IMU files.

## Stationary sections were moving

The gyro norm in the intended first ten seconds averaged 0.434 rad/s (RMS
0.514 rad/s). In the last ten seconds it averaged 0.379 rad/s (RMS 0.586 rad/s).
Sampled frames independently show large viewpoint changes at 0/5/10 seconds
and movement near 80 seconds. See `reports/diagnostic-full-run-contact.png`.

Thus the recording is usable for stereo/VIO diagnosis, but these intervals
cannot be treated as clean stationary bias/noise calibration data. This does
not establish why VIO fails, and stationary sections are not a replacement for
the translational excitation needed by inertial initialization.

## Replay on the same saved measurements

| Replay | Tracked frames | Fraction | Additional observations |
| --- | --- | --- | --- |
| Stereo only | 2,098 / 2,399 | 87.45% | Three tracking-loss transitions; map merges logged |
| Stereo + IMU | 1,802 / 2,399 | 75.11% | Seven active-map resets; 1,345 initialized frames; initialized at end |

Stereo replay: `reports/diagnostic-full-run-stereo/` and corresponding `.log`.
Inertial replay: `reports/diagnostic-full-run-inertial/` and corresponding `.log`.
Inertial first initialization was 2.536 seconds after the first image. Its exit
code 0 indicates it tracked frames and initialized at least once, not that VIO
was reliable. Neither replay has ground truth or independently validated IMU
noise/time offset/extrinsics. These are individual runs of an asynchronous
SLAM implementation, not repeatability or accuracy measurements.

Stereo-only tracking losses occurred around 9.34, 30.50 and 45.22 seconds.
Sampled images include low-texture walls and bright windows alongside darker
indoor regions. These are plausible visual challenges; sampled images do not
prove a single root cause. The successful full-rate recording and continuous
IMU timestamps show that VIO resets can be reproduced without a live processing
backlog. Timing/calibration and visual tracking remain diagnostic priorities.

No camera settings, Cube parameters, boot mode or arming state were changed
during this inspection. Offline replays sent no navigation messages.

## Pose graphs

`reports/diagnostic-pose-graphs/pose-graph.png` compares the optimized exports.
`pose-graph.html` provides rotation, zoom, a time cutoff and keyframe/orientation
toggles; `poses.json` contains the filtered data. Only tracking-state-2 poses
are drawn, with breaks across losses. Keyframe coordinates are transformed into
the same recentered frame as their corresponding TUM trajectory export.

Stereo has 2,098 plotted poses and 176 keyframes over 0–80.03 seconds. VIO has
176 plotted poses and 29 keyframes over 74.19–80.03 seconds: earlier VIO pose
history was discarded by resets before export. The graphs use separate local
coordinate frames and scales. Edges show temporal adjacency; no loop-closure or
covisibility constraint graph was saved. These plots do not represent position
accuracy or the full 80-second VIO trajectory.

Native C++ regeneration into a new directory:
`build/main/bin/gemini_pose_plot recordings/diagnostic/20261025T091926.822317376Z reports/diagnostic-full-run-stereo reports/diagnostic-full-run-inertial NEW_OUTPUT`.
