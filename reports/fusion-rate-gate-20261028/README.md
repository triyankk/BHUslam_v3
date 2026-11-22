# VINS-Fusion rate/gate experiment artifacts

Final comparison uses only `paced-t1/`, `paced-t2/`, `paced-t4/` with
`imu_endpoint_owned=true` and `images_preloaded=true`. Source images are loaded
before the playback clock to isolate tracking work from filesystem stalls.
The cache is a benchmark artifact, not part of the live camera pipeline.

`pre-endpoint-fix/` contains exploratory runs and binaries from before the
[IMU endpoint lifetime correction](../../docs/vins-replay-endpoint-issue.md).
They are excluded. `disk-input/` contains corrected but storage-stalled replay
runs and is also excluded from CPU-rate ranking. `live-t*/` are real camera
checks; their generation/flags are recorded in each summary.

Estimator core: official VINS-Fusion stereo + IMU only, commit in
`vins-fusion-commit.txt`; `upstream-status.txt` is empty. No ORB-SLAM3 or
VINS-Mono estimator is used. `orb.yaml` is a legacy calibration filename.

All candidates use mild CLAHE, clip limit 2. The actual publication policy stays
at 30 good samples and ≥60 persistent features. Timed gates are independent
shadows that never publish. No Cube configuration or source selection changed.

See [workflow and interpretation](../../docs/vins-fusion-rate-gates.md).
