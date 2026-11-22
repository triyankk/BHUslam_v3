> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> Actual EKF fusion verification is a prerequisite for dependable GPS-denied mission control. See the [shared project goal](../PROJECT_GOAL.md).

# EKF fusion verification — deferred

Deferred at the user's request on 2026-11-17 while evaluating Hesai JT16 mapping. No EKF, origin, Home, source-selection or Cube parameter changes are authorized by this deferral.

Last BHUslamV3 evidence (2026-10-28, historical): VINS-Fusion producer recovery 0.5 s / 40 persistent features; Fusion relay minimum 40; Cube `VISO_QUAL_MIN=40`, independently read back. The disarmed link forwarded 129 poses, including qualities 49, 53 and 59. EKF flags stayed 1024 and startup warnings included attitude/yaw, GPS configuration and visual-odometry health. Transport was verified; EKF fusion was not.

Resume with fresh hardware readback; another project now also has sensor/flight services, and old saved settings must not be assumed current.

1. Identify the sole Cube serial owner, firmware, disarmed state and current configuration. Preserve unrelated parameters and services.
2. Check EKF attitude, yaw and altitude initialization and determine the actual blockers from telemetry. Do not infer every old warning is still present.
3. Read the existing EKF geographic origin. For GPS-free operation, use a manually supplied test-site reference and verify the returned origin. A fresh VIO map can start at local (0,0,0) each flight; this is distinct from geographic latitude/longitude (0,0). Origin alone does not align yaw or prove fusion.
4. Verify ExternalNav source selection explicitly. Historical source set 2 used horizontal ExternalNav position, no external velocity, barometer height and compass yaw. Do not assume it remains configured or active. Onboard `XKFS.SS=1` denotes the secondary set; inspect the active EKF lane.
5. Log stationary data, then known horizontal movements and a return to the starting point with Cube and camera rigidly attached. Compare raw VINS poses and Cube-estimated local position in aligned frames, with timestamps and sensor lever arms accounted for.
6. Inspect onboard EKF position/innovation/rejection/status records, external-nav health and active-source evidence. A forwarded MAVLink packet, command ACK or moving local-position output alone is insufficient proof of VIO fusion.
7. Perform a bounded disarmed aiding-interruption/recovery check. Verify freshness enforcement, uncertainty/status changes and reset/discontinuity handling. Do not repeat stale measurements as fresh.
8. Report separate conclusions for transport, source selection, EKF measurement acceptance, trajectory accuracy and eventual flight readiness.

Physical calibration and timestamp validation remain outstanding unless newer measured evidence establishes otherwise. An arbitrary new local map or in-flight estimator reset requires explicit discontinuity/frame handling.

References:
- https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html
- https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html
- https://ardupilot.org/copter/docs/common-non-gps-to-gps.html
- Historical local evidence: `reports/fusion-downstream-align-20261028/`.

2026-11-17 saved-data audit: [report](../reports/ekf-saved-run-audit-20261117/review.md). Native decoding of 20 raw tlogs found 2,090 EKF status reports, all 1024, no healthy horizontal status and no Cube local positions. Existing data does not establish fusion. One adjacent JSON event stream contains an additional tail uninitialized report missing from its raw log; the coverage difference is documented.
