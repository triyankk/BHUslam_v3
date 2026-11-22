> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> Isolated mapping is a development tool supporting that goal, with no mission-control claim. See the [shared project goal](../PROJECT_GOAL.md).

# JT16-only mapping workflow

Selected on 2026-11-17. Uses the Hesai FAST-LIO2 C++ backend with JT16 point clouds and its own built-in IMU. No VINS, FAST-LIVO2, camera or Cube navigation input/output is involved. The C++ adapter and Bash launcher reuse the already installed SDK/ROS2/C++ backend from `/home/atas/vscode/optFlow_slam`; no new Python runtime implementation was added.

## Build

```bash
cd /home/atas/BHUslamV3
scripts/build-jt16-mapping.sh
```

Existing runtime dependency: `/home/atas/vscode/optFlow_slam/runtime/lio/`, SDK bridge `build/jt16_bridge/optflow-jt16-bridge`, and its sample angle table. No packages or flight services are installed by this builder. The dependency checkout must remain available.

## Bench capture

With only JT16 attached and the Cube USB disconnected:

```bash
bin/jt16-map runs/jt16-map-01 120 --isolated
```

Keep the rigidly mounted sensor still for the first five seconds, then walk smoothly around a small textured/geometrically structured area and return to the start. Duration is bounded to 5–600 seconds. Use a new output directory each run.

`--isolated` refuses a connected Cube, pauses the previous JT16 service only if it was active, takes its existing advisory lock, checks exclusive serial ownership, starts a single C++ SDK reader, and restores the prior service on exit/failure. It never disables boot units or opens Cube serial/UART. Service restoration was verified after the startup run; mocked failures verify connected-Cube refusal and service restoration after lock/serial conflicts.

The default mode without `--isolated` listens to the **full mapping feed** on loopback port 18777 and never takes the device or changes services. The current restored acquisition owner only enables viewer port 18776, so shared mapping will wait/fail until that owner is deliberately configured to publish `--mapping-port 18777`. The reduced viewer feed cannot replace it. For eventual in-flight recording alongside the current proximity service, enable sharing in the single acquisition owner; do not use isolated mode or a second serial reader.

## Outputs

- `data/map.html`: offline interactive 3D map and trajectory viewer, with orbit/pan/zoom and layer toggles.
- `data/map.ply`, `full-map.pcd`: accumulated geometric map.
- `data/trajectory.csv`: estimated poses and backend covariance fields.
- `data/imu.csv`, `data/clouds.csv`: timestamps and reception/filtered-input diagnostics.
- `jt16-packets.raw`: raw serial packet bytes in isolated mode, for later parser replay. Shared mode does not create this raw capture automatically.
- `data/summary.json`, logs, copied estimator configuration and power-mode snapshot.

The map viewer displays a saved accumulated map; it is not an animated chronology of point insertion. It uses native estimator coordinates and makes no North/body-axis calibration claim. Rendering is local without external assets. Controls/export were tested under mocked WebGL, not an actual browser GPU in this session.

## Estimator/data contract

- Working RS485 speed: 3,000,000 baud; about 5 Hz full clouds and 200 Hz delivered IMU callbacks.
- Preserve SDK sensor XYZ, per-point timestamps and ring values; reject malformed/incomplete/nonmonotonic scans and invalid ring/range values. Cloud start stamp follows the first accepted point so backend relative times remain consistent after near-range filtering.
- ROS PointCloud2 carries aligned `x,y,z,intensity,ring,timestamp` fields; scan span is checked at 0.10–0.30 seconds.
- Explicit IMU conversions: SDK g × 9.80665 to m/s², SDK degrees/s × pi/180 to radians/s. Unmodified common device time is used internally; it is not verified UTC and differs substantially from the current host clock. No host-time relabeling is performed.
- Local ROS domain 91 and localhost-only transport isolate this mapping session from the existing project's estimator topics. No MAVLink interface exists in the new mapper.
- Trial map/scan voxels 0.20 m, 0.30–30 m input range, 16 rings, fixed provisional identity internal LiDAR/IMU transform. No existing aircraft mount transform is silently applied.
- Online extrinsic estimation is disabled; manufacturer-starting noise and identity extrinsics remain provisional. The sample angle table is not a serial-number calibration certificate. This setup is a mapping bench prototype, not flight-qualified localization.
- The exported viewer/PLY map is capped at 250,000 voxels; overflow is counted. The backend uses a sliding local map, and its save buffer is periodically voxel-compacted. Runs are duration-bounded; broader memory/power performance still needs measurement.
- No loop closure is enabled. Returning to the start can leave accumulated drift.

## Initial validation

The 20-second startup check saved 93 clouds, 3,800 delivered IMU samples, 90 odometry/registered-cloud samples and 1,058 map voxels. No malformed/incomplete/invalid input scans were counted. PCD and PLY each contain 1,058 points. Pose timestamps and quaternions, scan span, finite map/IMU data and field/count consistency passed checks. Endpoint displacement was about 0.026 m, but operator motion was not independently confirmed, so this is not a measured stationary drift/accuracy result. Initial backend zero covariance is not a calibrated certainty claim.

Power mode was MAXN_SUPER. A loop/motion capture and combined 15 W benchmark are still needed. Startup artifact: `reports/jt16-mapping-20261117/startup-01/data/map.html`.

## Deferred Cube verification

The saved Cube tests were audited separately with a native C++ MAVLink decoder: 20 tlogs, 9,282 records and 2,090 EKF reports; all EKF reports show 1024, no healthy horizontal-status reports and no Cube local-position samples. Existing runs cannot verify EKF fusion. See [saved-run audit](../reports/ekf-saved-run-audit-20261117/review.md) and [deferred procedure](ekf-verification-deferred.md). Replaying a saved VINS recording alone cannot establish what the real Cube fused.

## Moving capture follow-up

The operator repeated the stationary attempt in a 60-second motion run. [Latest 3D map](../reports/jt16-mapping-20261117/loop-02/data/map.html) and [results](../reports/jt16-mapping-20261117/loop-02/review.md): 294 clouds, 11,918 IMU samples, 291 poses, 5,111 map voxels; zero malformed/incomplete/invalid input scans. Estimated path 7.38 m, maximum excursion 0.554 m and endpoint separation 0.0445 m. No surveyed ground truth or loop-closure correction was available, and this does not establish metric accuracy. Output maximum gap was 204.74 ms. Existing service restored after the run.
