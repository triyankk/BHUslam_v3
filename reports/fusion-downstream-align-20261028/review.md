# VINS-Fusion downstream quality alignment

The Fusion producer, relay profile and Cube quality threshold are aligned to 40. Producer recovery remains 500 ms of continuously valid capture timestamps, with 20 backend tracks, 20 stereo matches and the existing pose/velocity/alignment/freshness guards. Quality remains the persistent-feature count capped at 100; no score inflation is applied.

Changes:

- Native relay socket accepts an explicit quality minimum. `cube_vio_bridge --quality-min 40` selects the Fusion policy; generic/legacy callers retain default 60. The Fusion latency launcher passes 40 explicitly and logs the threshold in the bridge summary.
- Cube setup and proposed parameter presets now use `VISO_QUAL_MIN=40`.
- Added `cube_vio_config --apply --quality-only` to write only this parameter while saving before/after snapshots, rollback value, independent named readback and preservation checks for other observed settings.
- Fusion configuration and current operator documentation reflect the aligned thresholds.

## Hardware result

Verified Cube disarmed; exclusive USB serial ownership. Exactly one parameter write: `VISO_QUAL_MIN` 60 -> 40. A separate connection read it back as 40. Other observed parameters, including source-set configuration, sensor offsets, serial settings and rangefinder configuration, were unchanged. No reboot or source selection command was sent.

Artifacts: [before](cube-apply/before.json), [after](cube-apply/after.json), [change](cube-apply/changes.json), [rollback value](cube-apply/restore.param), [independent readback](cube-readback/after.json).

## Validation

- Native protocol, loopback UDP-to-PTY transport and simulated Cube integration tests: 3/3 passed. Boundary tests accept 40 and 59, reject 39, retain stale/future rejection and preserve raw quality. Simulated Cube tests verify a quality-only update writes exactly one parameter and preserves other settings; the generic 60-quality relay behavior remains tested.
- At verified 15 W, a bounded 15-second disarmed live VINS-Fusion check published 217 eligible UDP poses and forwarded 129 over Cube USB. Three actual forwarded camera poses had qualities **49, 53 and 59**; all would have been rejected by the old relay threshold. The 10 Hz newest-only serial relay intentionally does not forward every producer output.
- Host-measured age at serial-write completion: median 109.17 ms, p95 139.71 ms, maximum 149.44 ms. These are host timings, not Cube consumption latency.
- Bridge: zero link drops, zero malformed UART packets, no parameter writes during the live check. Four OpenCV threads and mild CLAHE 2 retained. All processes stopped and USB serial ownership released afterwards.

The bridge also counted 14 odometry rejections; the existing aggregate counter does not classify them individually. They were not bypassed or disabled by this update. Automated checks associate every forwarded capture timestamp with a valid, published source sample.

Threshold mismatch is resolved. EKF fusion is **not established**: Cube EKF flags remained 1024, with waiting-for-GPS/attitude/origin-related startup messages and `VisOdom: not healthy` / `VisOdom: reset`. This validation proves the configured thresholds and host transport path, not successful vehicle navigation. No origin, heading alignment, source selection or flight setup was changed.
