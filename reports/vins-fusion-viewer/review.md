# Synchronized stereo / sparse 3D viewer

Open [the recording selector](index.html), choose a recording and press Play.

| Export | Source pairs processed | Synchronized saved views | Duration | Distinct solved landmarks shown |
|---|---:|---:|---:|---:|
| Two-minute moving | 3597 | 1200 | 120.009 s | 11535 |
| Eighty-second moving | 2399 | 801 | 80.028 s | 5208 |

The top panel contains both rectified cameras, native tracked feature observations, feature trails, optional stereo links and IDs. The lower panel contains the current camera, trajectory, active sliding-window points and optional accumulated landmark history. Feature selection links observations to solved 3D geometry when available. Controls include speed, frame stepping, scrubbing, orbit, zoom, pan, camera follow and PNG export. The recording selector resets state when switching inputs. Another exported folder can be opened with the folder picker.

[Usage and another recording](../../docs/vins-fusion-viewer.md). Estimation/export is C++; JavaScript only handles browser rendering. The default saves about 10 display views per second while processing all approximately 30 Hz stereo pairs. Native point reconstruction uses Fusion's camera/body/anchor transforms. The display excludes unsolved/invalid depths and depths above 80 m. Geometry history is frozen at each landmark's last saved observation; there is no global optimization or loop closure. Reset epochs and time prefixes are kept separate. Native initialization is not an accuracy guarantee, particularly because this Fusion revision disables its failure detector.

Both full replays completed. Artifact checks confirmed source timestamps, camera poses against replay CSVs, synchronized image counts, finite geometry, PLY counts and landmark reprojection consistency. Reprojection p95 was 2.449 px across 94517 matched observations for the two-minute export and 2.803 px across 36340 for the eighty-second export. This validates export/frame consistency, not ground-truth accuracy.

The native IMU and stereo factor tests passed. UI tests exercised both complete exports, backward seeking without future geometry, independent reset epochs, feature selection, recording changes, playback stepping, orbit/zoom/fit and no network requests. These use software and mocked WebGL contexts; actual browser GPU rendering was not tested by automation. Embedded scripts compile. The native OpenCV preview was visually inspected for correct feature/image alignment and sparse geometry.

Viewer runs are fresh diagnostic replays, with encoding/serialization overhead and a wall-time solver budget, and are not additional runtime benchmarks. The prior measured executable was preserved under the original benchmark directory. Source and executable checksums for this viewer are saved beside this review.
