# GPS-aided shadow-flight preparation — 2026-11-17

User intent: fly with the existing GPS-aided Cube setup and independently evaluate VINS-Fusion and JT16 mapping afterward. Pose forwarding and EKF fusion are deferred to a later simulation stage.

## Installed behaviour

- Existing Cube/JT16 acquisition and proximity owner remains `optflow-flight-logger.service`. No changes to `config/system.yaml` (SHA256 verified unchanged), Cube parameters, source selection, flight modes, missions or power mode.
- Added opt-in environment defaults to its native C++ JT16 reader: full timestamped loopback mapping feed and raw packet recording under that checkout's own data directory. Existing proximity stdout and reduced viewer feed remain intact. The source diff and original native binary/source are retained here. No Python source was changed or introduced.
- Added/enabled independent `bhu-jt16-map.service` and `bhu-shadow-performance.service`. Existing `bhu-vins-fusion.service` remains enabled and now records replay inputs through a bounded native asynchronous writer.
- VINS and mapping each wait for fresh, disarmed Cube startup status; VINS additionally waits for Gemini USB access. Neither estimator stops merely because the vehicle subsequently arms. No VINS UDP output, Cube VIO bridge, pose forwarding or movement output enabled. Existing proximity can still act through its already configured Cube settings.
- Run identifiers, boot IDs, original inputs, pose/timing CSVs, map checkpoints, thermal/load logging and an offline 3D review exporter support post-flight analysis. Sudden power loss is not a substitute for graceful finalization.

Current service readback: all four units enabled/active; VIO/mapping/performance are waiting for absent camera/Cube, not producing flight results. LiDAR acquisition remains the single serial reader and writes raw packets. Original Cube/aircraft settings were not inspected live because the Cube is absent. Jetson remains MAXN_SUPER, not 15 W. Prior service state: `runs/flight-shadow/install-20261117T105435.122656900Z/`.

## Validation

- Serial native builds succeeded for Fusion, JT16 adapter and the native SDK reader.
- Six Fusion tests passed: fixed-memory diagnostics, fresh/disarmed Cube startup, asynchronous input recording/frame ownership/timestamp preservation, native IMU integration, stereo geometry and publication gate safety.
- Existing isolated-mode ownership refusal/restoration fixture passed.
- Review fixture passed boot grouping, truncated CSV handling, checkpoint recovery, VINS trajectory export and saved-map WebGL controls under a mocked context. Actual browser GPU rendering was not tested.
- Invalid native mapping-port environment rejected before hardware acquisition.
- Real shared-feed continuous-mode smoke capture: 17.48 s, 87 scans, 3,530 delivered IMU callbacks, 83 poses/registered clouds and 1,216 map voxels; zero malformed/incomplete/invalid scans or map overflow. Its first completed checkpoint at about 5 s held 25 scans, 21 poses and 1,121 voxels. Graceful interrupt produced final map/trajectory and HTML. This was an indoor startup check without requested motion, not flight/performance/accuracy proof.
- Backend log confirms continuous-mode extra PCD accumulation disabled. One real JT16 serial owner retained; the acquisition service was not paused by mapping.

[Shared-feed smoke map](shared-continuous-smoke/data/map.html). No independent ground truth, calibration approval or loop closure.

## Next connected test

Verify both estimators start before takeoff, inputs/logs are advancing without writer errors, the existing Desktop Cube recorder starts on arm and GPS reference logging works. Measure the combined workload at the intended 15 W profile. Complete these checks before treating the observation setup as tested on the aircraft. After a GPS-aided flight, gracefully stop and run the review command; keep the corresponding Cube BIN and Desktop recorder session. GPS trajectories are comparison signals, not surveyed ground truth.

Then validate recorded-pose transforms/gates and forwarding in isolated simulation with compatible simulated motion, followed by a separate disarmed Cube fusion check. No simulation has been run and no fusion acceptance is claimed here.

See [operation, evidence paths and limits](../../docs/flight-shadow-test.md).
