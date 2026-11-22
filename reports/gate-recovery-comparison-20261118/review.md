# Recovery-gate comparison — 18 November 2026

The operator approved comparing the current 500 ms gate with 100 ms recovery after brief, continuous interruptions. This is a same-sample diagnostic comparison on the current stationary live scene. Only the original gate forwards actual VIO packets. No Cube parameter, source-selection, MAVLink transport guard, camera exposure, feature threshold, velocity calculation, recording state machine or buzzer behavior changed.

## Policies

Baseline: unchanged 500 ms initial and recovery qualification, 150 ms freshness, all existing support/finite/velocity/IMU/frame checks and reset/continuity handling.

Candidate: 500 ms initial qualification; 100 ms only when the first valid recovery pose arrives within 200 ms of the last published capture, with observed pose continuity, unchanged epoch and no pending reset or serious fault. A longer interruption, epoch change, measured pose/attitude jump, lost initialization/alignment, unhealthy IMU, nonfinite pose/motion or invalid quality keeps 500 ms. Stale, future, duplicate, invalid velocity and weak-support samples remain rejected immediately. Subsequent interruption during qualification restarts it and re-evaluates eligibility for short recovery. The 200 ms horizon does not permit stale transmission. This does not guarantee a <=300 ms packet gap: sample timing/quantization and transport add delay.

The optional comparison arguments were added to native DurationGate; its one-argument/default construction retains the previous policy. The live producer instantiates the original gate with 500000 and a separate shadow gate with 500000/100000/200000. The candidate is used only for diagnostic CSV/status; outgoing ODOMETRY remains conditioned solely on the original gate. One estimator processes each actual stereo/IMU sample, then both gates evaluate exactly the same Sample and host time. Comparison rows retain timestamps, gate decisions, hard/support masks, reset/recovery counters and pose/epoch.

## Conditions and verification

Power was read from the Jetson as **25 W**, not 15 W. The power profile was not changed. Results are not battery-representative 15 W validation. JT16 mapping and proximity remain running alongside VINS.

Before restart, the Cube was freshly connected/disarmed with recording inactive. The existing VINS binary was backed up from its running process and only bhu-vins-fusion.service was restarted to enable diagnostic logging. The Cube owner, bridge, LiDAR mapper, power mode and their configuration were not restarted/changed for this comparison. New estimator startup is excluded from the measured window by waiting for initialization, existing-gate publication and at least 100 processed frames.

Native gate safety tests exercise unchanged 500 ms baseline, 500 ms candidate startup, 100 ms coherent brief recovery with unchanged reset counter, full 500 ms after IMU fault/epoch/pose jump/long interruption, and unchanged 150 ms freshness. All nine native VINS tests passed with the project's required dependency paths. Both native external-navigation protocol/transport tests passed. The first full suite invocation lacked libglog in its runtime search path; rerunning with the existing project dependency paths passed without changing application behavior. A numerical analysis fixture verified known modeled receipt coverage (60% baseline, 80% candidate).

## How results are interpreted

Raw eligible-frame counts are exact decisions from the two gates on the same live samples. The receipt timeline is a counterfactual sender model: localhost polling every 5 ms, latest-only pending pose, 10 Hz cap and 150 ms age at modeled write. Receipt coverage is the fraction of the observation window within 300 ms of a modeled packet. It is not measured candidate Cube health, EKF fusion, accuracy, motor-on performance or a health flag forced green. The candidate never sends packets to the Cube. Actual baseline owner writes/health are summarized separately.

Additional candidate poses must have zero hard/support rejection mask. Pose-step metrics are estimator continuity diagnostics on the same stationary scene, not ground-truth accuracy. Reset-counter transitions and initialization/epoch changes are reported separately. This short ground comparison cannot validate flight safety or calibration.

Evidence: recovery_comparison.csv, tracking.csv, pose_history.csv, estimator_events.jsonl, live-observation.jsonl, window.json, status.json, analysis/comparison.json and receipt-timeline.svg; native tests/build logs; power-mode.txt; backed-up executable/header/source and release hashes. tools/compare_recovery_gates.js produces an offline report without opening hardware.

## Results

120.235 s at 25 W, 1,569 identical live samples: both policies accepted 1,552 (98.9165%). Both modeled 1,144 packets, maximum inter-packet gap 900 ms, two interior gaps >300 ms, 98.7122% receipt-window coverage and zero reset transitions. No extra candidate poses were accepted. The two interruptions returned a first valid pose after 367.104 and 333.730 ms from the last published capture, both exceeding the candidate's 200 ms brief-interruption limit. Full 500 ms recovery was therefore correctly retained. This run provides no reason to promote the candidate; the active policy remains 500 ms. There were no IMU clipping events or automatic estimator reinitializations during the window.

Actual Cube reception could not be compared: all 120 sampled vision-health states were false and no serial poses were written because the current source configuration had changed to VELXY=5/YAW=1 with COMPASS_ENABLE=0, outside the previously accepted profiles. Parameter changes were not made by this comparison. Treat the receipt-coverage figures strictly as modeled, not actual Cube health. The operator subsequently confirmed the intended hybrid optical-flow velocity/internal-compass yaw setup; that is a separate profile/compass configuration increment.
