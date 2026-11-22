> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> This experiment evaluates a possible localization improvement for that goal; it is not a validated live replacement. See the [shared project goal](../PROJECT_GOAL.md).

# Offline Gemini/JT16 LIVO experiment

The selected architecture evaluates JT16 + Gemini images + Gemini IMU in a
joint localization/mapping estimator. Gemini depth obstacle avoidance remains
independent. No live estimator replacement or flight-output path is enabled.

The [flight-213 experiment report](../reports/livo-replay-readiness-20261120/experiment-review.md)
contains the actual results: two initial FAST-LIVO2 replays diverged, and the
recorded VINS baseline also ran away. No localization improvement is validated.
The recorded accelerometer has substantial near-range-limit samples and the
usable LiDAR segment starts mid-flight. Stationary-start data and measured
timing/extrinsics are needed for a defensible next comparison.

## Tools

- `scripts/build-jt16-raw-decoder.sh`: builds a C++ offline decoder using the
  existing pinned Hesai SDK and local libraries from the dependency checkout.
- `build/jt16-offline/jt16_decode_raw RAW CORRECTION NEW_OUTPUT`: saves complete
  scans in SDK axes/time; preserves rings/timestamps and refuses old outputs.
- `scripts/build-livo-offline.sh`: builds the native offline numerical adapter.
- `tools/livo/prepare_flight213.py`: initial mounting and single-anchor timing
  preparation, explicitly uncalibrated. Uses saved camera calibration/mount.
- `bin/livo-replay CONFIG NEW_OUTPUT [MAX_IMAGES]`: offline replay only;
  outputs camera-origin trajectory and per-frame measurement diagnostics.
- `tools/livo/compare_recorded.py`: compares to recorded VINS, with no claim
  that VINS is ground truth. Plot distances are estimator outputs, not measured
  travel. Output pose coverage alone is not healthy tracking.
- `tests/livo_stationary_replay_test.py`: stationary regression and missing
  LiDAR-return coverage check. Native CTest verifies offline time/rotation and
  rejects publication attempts.

Dependencies remain workspace-local: `third_party/FAST-LIVO2`,
`third_party/fast-livo-vikit`, old `third_party/fast-livo-sophus`, existing PCL
runtime in `/home/atas/vscode/optFlow_slam/runtime/lio`, and staged OpenCV/Eigen.
Exact upstream commits are saved in the experiment's
`initial-transform-and-clock.json`. Upstream licenses remain in their source
directories. Build copies contain the documented old-Eigen and IMU timestamp
initialization compatibility changes; original numerical source trees are not
edited. The C++ adapter handles replay sequencing; Python tools only prepare,
audit and summarize offline files.

A failed experiment preserves partial output and returns nonzero. Its missing
completion summary must not be interpreted as a passing localization result.
