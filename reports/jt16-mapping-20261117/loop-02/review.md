# JT16-only moving capture

[Open the 3D map and trajectory](data/map.html)

The user repeated the capture after confirming `loop-01` had been stationary. This run lasted 60 seconds, with initial still time, a requested handheld loop and a return/hold instruction. IMU and estimated poses show substantial motion. No independent path or endpoint ground truth was measured.

| Measurement | Result |
|---|---:|
| Accepted input scans | 294 |
| Built-in IMU samples | 11,918 |
| Estimated poses / registered clouds | 291 / 291 |
| PLY / PCD map points (20 cm voxels) | 5,111 / 5,111 |
| Estimated accumulated path | 7.38 m |
| Maximum displacement from first pose | 0.554 m |
| First-to-last pose separation | 0.0445 m |
| Odometry output gap, p95 / maximum | 200.84 / 204.74 ms |
| Malformed / incomplete / invalid input scans | 0 / 0 / 0 |

The estimated 4.5 cm endpoint separation is descriptive, not a surveyed loop-closure error or proof of accuracy. No loop-closure optimizer is active. The motion included angular speeds up to about 238 degrees/s, making timing, deskew and the provisional internal IMU/LiDAR transform particularly relevant. Continuous 5 Hz output is not by itself proof that every scan registration was correct.

Finite inputs/geometry, pose quaternion normalization, monotonic pose/IMU timestamps, scan duration bounds, pose/scan timestamp association and PLY/PCD/summary counts passed checks. The offline viewer's camera/control behavior passed mocked-WebGL checks; actual browser GPU rendering was not automated.

The current sample angle table, identity LiDAR-to-internal-IMU transform and noise are provisional. Device timestamps were retained as a shared relative time basis and were not presented as current UTC. No camera or Cube inputs/outputs were used. SDK serial packets, IMU/scan diagnostics and the estimated trajectory were saved for subsequent analysis.

The original LiDAR service was restored active with one serial reader after capture. Power mode stayed MAXN_SUPER; 15 W performance is not yet established.

Files: `data/map.html`, `data/map.ply`, `full-map.pcd`, `data/trajectory.csv`, `data/imu.csv`, `jt16-packets.raw`, logs, `motion-evidence.json`, `artifact-check.json`, `operator-context.json`.
