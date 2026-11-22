# Saved Cube EKF audit

Audited on 2026-11-17 using the native C++ MAVLink decoder. Inputs are the 20 BHUslamV3 timestamp-prefixed Cube telemetry logs retained from the earlier October tests. No Cube connection, parameter write or replay transmission was used.

Results: 9282 MAVLink records; 18 logs contain 2090 EKF reports. Every recorded EKF flag is 1024 (EKF_UNINITIALIZED); healthy horizontal-status reports = 0; Cube LOCAL_POSITION_NED samples = 0. CRC failures = 0; truncated records = 0. Adjacent event logs contain 213 host serial-forward events.

| Saved log | EKF reports | Healthy horizontal | Local positions | Host pose writes |
|---|---:|---:|---:|---:|
| reports/cube-buzzer-ready-smoke/cube.tlog | 0 | 0 | 0 | 0 |
| reports/cube-extnav-link-20261028/cube.tlog | 0 | 0 | 0 | 0 |
| reports/fusion-downstream-align-20261028/live-link/bridge/cube.tlog | 18 | 0 | 0 | 129 |
| reports/fusion-live-15w-20261028/motion-bridge/cube.tlog | 109 | 0 | 0 | 53 |
| runs/cube-record/20261025T074527.758142972Z/bridge/cube.tlog | 46 | 0 | 0 | 0 |
| runs/cube-record/20261025T074951.755625488Z/bridge/cube.tlog | 603 | 0 | 0 | 0 |
| runs/cube-record/20261025T090159.290173800Z/bridge/cube.tlog | 249 | 0 | 0 | 0 |
| runs/cube-record/20261025T091925.991617920Z/bridge/cube.tlog | 110 | 0 | 0 | 0 |
| runs/cube-record/20261025T091926.363488866Z/bridge/cube.tlog | 62 | 0 | 0 | 0 |
| runs/cube-record/20261025T095519.002131376Z/bridge/cube.tlog | 137 | 0 | 0 | 0 |
| runs/cube-record/20261025T100658.056026398Z/bridge/cube.tlog | 166 | 0 | 0 | 0 |
| runs/cube-record/20261025T104810.649952512Z/bridge/cube.tlog | 136 | 0 | 0 | 0 |
| runs/cube-record/20261025T131913.875494992Z/bridge/cube.tlog | 24 | 0 | 0 | 0 |
| runs/cube-record/20261026T090303.190378872Z/bridge/cube.tlog | 130 | 0 | 0 | 0 |
| runs/cube-record/20261026T093843.707946608Z/bridge/cube.tlog | 10 | 0 | 0 | 0 |
| runs/cube-vio/20261024T112456.177462229Z/bridge/cube.tlog | 17 | 0 | 0 | 0 |
| runs/cube-vio/20261024T112633.212059172Z/bridge/cube.tlog | 92 | 0 | 0 | 12 |
| runs/cube-vio/20261024T114213.161561677Z/bridge/cube.tlog | 101 | 0 | 0 | 0 |
| runs/cube-vio/20261024T114522.544520968Z/bridge/cube.tlog | 79 | 0 | 0 | 19 |
| runs/cube-vio/20261024T115722.917440384Z/bridge/cube.tlog | 1 | 0 | 0 | 0 |

These recordings establish that the observed Cube EKF never became initialized/healthy during these tests. They cannot demonstrate successful VINS measurement fusion. Some are deliberately record-only or buzzer checks, so absent pose forwarding in those files is expected. VINS-only replays cannot retroactively establish what the Cube EKF fused.

No GPS_GLOBAL_ORIGIN messages were recorded. Their absence is not proof that no origin existed: these receive logs did not independently query and verify it. Active EKF source-set selection and on-controller innovations are also not proven by these logs.

The latest aligned-threshold test forwarded 129 poses (including quality 49/53/59), but EKF flags stayed 1024. Other older runs used earlier algorithms and thresholds; their transport counts must not be described as VINS-Fusion accuracy/fusion results. Do not mix the other optFlow_slam project’s historical IM10A/D415 navigation experiments with this Gemini/VINS validation.

Next verification needs a new bounded disarmed Cube test: fresh health/origin/source readback, estimator initialization, explicit ExternalNav selection evidence, motion comparison and onboard EKF innovation/rejection logs. See [deferred verification checklist](../../docs/ekf-verification-deferred.md). No flight validation is inferred.

Cross-check: 19 adjacent JSON event logs match the raw decoder's EKF counts and flag distributions exactly. In `runs/cube-record/20261025T091926.363488866Z/bridge/`, JSON contains 63 uninitialized reports while the raw tlog contains 62. The final JSON EKF timestamp is later than the final saved raw record, consistent with an incomplete binary-log tail; the exact historical cause is not established. Both sources show only flag 1024. This one-report coverage difference is retained in `cross-check.json` and does not alter the fusion conclusion.
