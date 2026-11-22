# JT16-only mapping results

Latest: [60-second moving capture, 5,111-point map](loop-02/review.md) · [Open latest 3D map](loop-02/data/map.html). The first requested loop (`loop-01`) was confirmed stationary by the operator and retained as stationary data.

The C++ JT16-to-FAST-LIO2 mapping path ran successfully for a bounded startup check. Inputs were JT16 clouds and its own built-in IMU; no camera, Cube serial connection, MAVLink publication or flight-controller parameter changes were involved.

[Open the saved 3D map and trajectory](startup-01/data/map.html)

- 93 full input scans and 3,800 delivered IMU samples published.
- 90 estimated poses and registered clouds saved.
- 1,058 map voxels; matching 1,058-point PLY and PCD exports.
- Zero malformed, incomplete or invalid mapping-feed scans counted.
- Field/count, finite geometry, quaternion and timestamp checks passed. Ownership failure-path mocks passed; saved-data viewer controls passed mocked-WebGL checks. No actual browser GPU rendering check was performed.
- SDK packets, input timestamp diagnostics, estimated trajectory, logs and copied estimator configuration retained under `startup-01/`.

The existing JT16 service was temporarily paused while the Cube USB was absent. Its existing ownership lock and a serial-owner check prevented a second reader. After the bounded test, mapping processes exited and `optflow-flight-logger.service` was restored active/enabled, with one JT16 serial owner. No source code or configuration in that other checkout was edited.

This is a startup result, not a confirmed walked-loop accuracy measurement. Identity internal LiDAR/IMU extrinsics and noise remain provisional. The sample angle table is not a unit calibration certificate, and no loop closure is enabled. The run used MAXN_SUPER, so battery-mode 15 W performance is still untested.

Next capture command, with Cube USB disconnected:

```bash
cd /home/atas/BHUslamV3
bin/jt16-map runs/jt16-map-01 120 --isolated
```

Keep still for the first five seconds, then move smoothly and return to the start. The launcher restores the previous service afterwards. In-flight recording alongside the existing proximity service needs the full shared mapping feed enabled in its single owner; isolated mode deliberately refuses a connected Cube.

[Workflow and limitations](../../docs/jt16-only-mapping.md)

## Saved Cube evidence

The separate [native saved-run audit](../ekf-saved-run-audit-20261117/review.md) examined 20 tlogs: 9,282 MAVLink records, 2,090 EKF reports, all flag 1024, with no healthy horizontal status or Cube local-position samples. Existing recordings do not verify EKF fusion. A new disarmed test with initialization/origin/source selection and motion/innovation evidence is needed when the Cube is available. No old camera trajectory was transmitted as live Cube input.
