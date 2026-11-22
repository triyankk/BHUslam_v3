# Two-minute boot recording review — 2026-10-25

Dataset: `recordings/diagnostic/20261025T095519.847430960Z/`.
Cube run: `runs/cube-record/20261025T095519.002131376Z/`.
Boot ID `dc1c6e0f-37a8-48e2-8375-55efa94a2c26` matches the current machine and
dataset. This distinguishes the user's new boot recording from the earlier
setup-validation dataset, whose filename has a later UTC clock value.

## Recording passed

- Automatic start without QR, timed stop at 120.002182 seconds.
- Power profile 15W at both start and end.
- 3,597 native/rectified stereo pairs, 640x400; 29.964 Hz by device timestamps.
- 24,376 samples from each IMU stream; 202.579 Hz, max gap 4.937 ms.
- No frame-index gaps or rejected stereo pairs; maximum stereo skew 1 us.
- Both IMU streams bracket all images; native/replay counts and endpoint
  timestamps match. Native integrity check passed without warnings:
  `reports/two-minute-run-check.json`.
- Timed duration complete and filesystem_synced_before_stop_cue=true.
- Start/stop/saved events logged, two Cube cue transmissions, zero navigation
  messages and parameter writes, zero link drops. One malformed incoming serial
  packet was counted; this is separate from camera/IMU file integrity.
- Service completed successfully with ExecMainStatus=0 and is now inactive.

## Replay comparison

| Replay | Tracked frames | Tracking fraction | Observations |
| --- | --- | --- | --- |
| Stereo | 3,410 / 3,597 | 94.80% | Two tracking-loss transitions |
| Stereo + IMU | 2,416 / 3,597 | 67.17% | Three active-map resets, four tracking-loss transitions |

VIO first initialized at 4.338 s and reported initialization on 2,347 frames.
It completed initial VIBA refinement but later lost tracking. Its last tracked
pose was at 88.372 s, and it was uninitialized at the end. Inertial replay
returned 1 because no usable final map remained for optimized export; this is
not a recording-format failure. Logs: `reports/two-minute-run-stereo.log` and
`reports/two-minute-run-inertial.log`. Per-frame states are in the matching replay
directories. IMU calibration remains provisional; no tuning was applied.

## Pose plots

`reports/two-minute-pose-graphs/pose-graph.png` compares stereo optimized poses
with VIO pose history. `pose-graph.html` offers rotation, zoom, a time cutoff,
and graph selection; `poses.json` contains the data.

The C++ replay now writes `pose_history.csv` as poses are tracked, preserving
earlier estimates before resets discard them from final exports. This is an
additional diagnostic log and does not alter the tracking algorithm. All 2,416
tracked VIO frames have history rows. The graph has six pose-history segments
(including initialization/frame-change boundaries), ending at 88.372 s. It
does not fabricate poses for the untracked final approximately 32 seconds.

Stereo optimized export spans 0–120.009 s, with 3,410 plotted tracked poses and
57 exported keyframes. VIO's final optimized export is empty and is shown as
unavailable in the graph selector. Online VIO has no optimized-keyframe overlay.
Graphs/reset segments use separate local coordinate frames and scales; do not
interpret the combined VIO segments as a single continuous world path. Edges
show temporal adjacency, not saved loop/covisibility constraints. Green means
IMU initialized at that frame, not measured accuracy or navigation readiness.

The graph generator and page now use the recording's actual duration instead
of an 80-second constant, and handle empty final exports. A Node-based rendering
fixture checked finite geometry, the 120-second timeline and no edges across
reset boundaries. The static PNG was visually inspected. Neither graph has
ground truth, and the VIO plot remains evidence of unstable tracking.

No camera/Cube settings, navigation source or boot configuration were changed
during this check. All replays are offline and send no navigation data.

## Stereo display aligned to VIO reference

The original panels used separate world frames: stereo begins in camera axes,
whereas VIO rotates its world during gravity initialization. A stereo camera-Z
axis is therefore not directly comparable to the VIO vertical axis.

`reports/two-minute-pose-graphs-aligned/` contains an additional display-aligned
plot. A proper rigid rotation/translation (determinant +1, scale unchanged)
was fitted from 2,146 timestamp-matched camera poses in the largest shared
initialized segment, 4.338–75.924 s. Camera orientations determine rotation;
mean position offsets determine translation. Orientation-fit RMS is 0.506 deg
and relative position-fit RMS is 0.316 m. These residuals describe agreement
between the two estimators on that segment, not measured accuracy.

The fitted vertical mapping is approximately
`Z_vio = -0.04991 X_stereo - 0.99801 Y_stereo - 0.03844 Z_stereo + 0.61577`.
It is a coordinate rotation, rather than a single-axis sign reversal.

Stereo pose and keyframe display coordinates are transformed consistently.
The VIO graph was checked byte-for-byte at the JSON graph level and is
unchanged. Original exports and the original graphs are retained; neither
the algorithms, camera mounting calibration nor Cube navigation frames were
modified. Other reset segments remain independent local frames.
