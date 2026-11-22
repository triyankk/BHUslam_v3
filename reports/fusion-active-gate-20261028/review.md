# Active VINS-Fusion gate update

Later update: the downstream thresholds were subsequently aligned to 40; see [downstream alignment](../fusion-downstream-align-20261028/review.md). The validation below records the earlier producer-only update.

Selected policy: 500 ms of continuously valid capture timestamps, with at least 40 persistent features, 20 backend tracks and 20 stereo matches. Retains 150 ms freshness/capture-gap bounds and initialized, aligned, finite pose/velocity checks. Failed support or hard guards close output immediately; recovery requires the full duration. Output discontinuities increment the MAVLink reset counter on resumed output.

The native `DurationGate` returns the reset counter needed by the live MAVLink producer. The retained timed shadow uses the same implementation. Live `eligible` now controls optional publication through this policy. Replay `eligible` mirrors it for diagnostics only and has no network output. Previous 30-frame / 60-feature, 15-frame / 40-feature and timed comparison decisions remain logged.

Configuration: `configs/vins-fusion-extnav.yaml`, with `Publisher.persistent_min: 40`. The Fusion latency launcher uses this configuration. The old shared ORB configuration, relay, Cube parameters, boot recording service and EKF source selection were not changed. Mild CLAHE and four OpenCV launcher threads are retained.

Validation:

- Native IMU integration, stereo geometry and gate safety tests: 3/3 passed.
- Added gate tests verify first publication after 500 ms, initial reset counter zero, immediate failed-support closure, full recovery duration, stable reset counter during continuous output, and discontinuity reporting after gap/epoch recovery.
- Short paced recording smoke: 45 processed frames, 28 eligible; every active decision matched the retained 0.5 s / 40-feature shadow.
- Camera-only 12-second check at verified 15 W: 202 processed frames, 192 initialized, 182 eligible; active and shadow decisions matched. Mild CLAHE 2 and four OpenCV threads verified. No UDP packets were sent.
- CSV checks verify support thresholds, hard guards and count/summary consistency. Logs, hashes and check runner are retained alongside this report.

The reported MAVLink quality remains the persistent-feature count capped at 100. Relay/Cube quality minimum remains 60, so output rated 40–59 is still rejected downstream. This update selects the producer gate; it does not establish Cube fusion or flight readiness. The short live check is startup validation, not a new movement/accuracy benchmark.
