> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> This legacy sparse-SLAM viewer is a comparison and diagnostic reference. See the [shared project goal](../PROJECT_GOAL.md).

# Sparse 3D SLAM visualization from recorded Gemini data

The native replay can export incremental ORB-SLAM3 landmark maps and keyframes
from existing rectified stereo recordings. It does not require the camera or
Cube, alter their settings, or send external-navigation messages.

From the project root:

```sh
bin/bhu replay recordings/diagnostic/RECORDING \
  third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  --sparse-map --output reports/NEW-stereo-map
```

For experimental stereo-inertial replay, add `--inertial --allow-provisional`.
The provisional override permits diagnosis, not a claim of calibrated VIO.
Choose a new output directory for every replay; original recordings are read only.

Open `sparse-map.html` from the output directory. Everything runs locally;
JavaScript and geometry are embedded and image previews are relative files.
Keep the output folder and its `images/` subfolder together. No CDN, Python,
ROS, network server or Internet connection is required by the viewer.

## Viewer

- Sparse landmarks colored by height in the displayed local frame.
- Camera/keyframe frusta, a keyframe route, and recorded image previews with
  green tracked-map-point association boxes.
- Optional spanning-tree/strong-covisibility graph and actual loop edges.
- Timeline, play/pause, speed, map selector, orbit, pan, zoom, top view and PNG save.
- WebGL rendering with a software fallback.

The initial view chooses a snapshot with the most valid landmarks. Play begins
from the selected time; drag the timeline to zero to inspect initialization.
Map selection jumps to that map's latest saved snapshot. During tracking loss,
the image continues and a **STALE MAP** label identifies the last valid geometry.
No new camera pose is fabricated. Each map/reset epoch is separate; maps are
never overlaid or joined without a measured alignment.

Stereo display uses camera-world X/Z/−Y so nominal initial camera-up is vertical.
It is not a gravity calibration. VIO uses its current world XYZ; initialization,
reset and refinement may change the frame. Neither view is ground truth.

## Data and limits

`sparse-map.json` stores snapshots about every two seconds and at the final frame.
For every valid snapshot it includes the current map epoch (map ID and initial
keyframe ID), finite landmark XYZ/observation counts, keyframe poses, current
camera pose and graph edges. The map-update mutex and object getters protect
geometry reads; mapping and optimization may refine estimates between snapshots.
The current camera pose comes from the latest tracking result, so it can differ
slightly from a concurrently refined keyframe estimate.

Rendering/export samples deterministically to at most 12,000 landmarks per
snapshot. `valid_landmarks` retains the full count. XYZ display/export values
are rounded to 1 mm to bound file size; that precision is not measured accuracy.
No data are synthesized where tracking fails. Earlier snapshots survive later
map resets, even if ORB-SLAM3's final trajectory export becomes empty.

Graph edge types: 0 spanning tree, 1 covisibility weight at least 100, 2 explicit
loop edge. These are not all optimizer residuals. The cyan route follows current
keyframe centers chronologically and breaks when a known tracking loss lies
between the keyframes. This is a keyframe route rather than a full-rate pose
trace. Green image boxes indicate point associations, not certified inliers or
accuracy. Green cameras/initialization likewise do not establish navigation health.

Each map's last saved point set is also written as `map-ID-EPOCH.ply`; it can be
opened in point-cloud software. It is a sparse online snapshot, not a dense
surface, textured mesh or guaranteed final optimized map. `sparse-map-preview.png`
shows a representative richly populated snapshot. Previously saved snapshots
are not retrospectively corrected by a later loop closure.

Recording/thumbnail export and snapshot serialization add CPU and storage work.
Do not use these replay timings as a controlled real-time/15W benchmark.
The two-minute moving recording is useful for mapping; the stationary capture
is useful for IMU statistics and cannot reconstruct a substantial route.

Implementation: `include/bhu/sparse_map.hpp`, `src/replay.cpp`,
`web/sparse-map.html`. The backend and export remain C++; browser interaction
uses JavaScript/WebGL. `tests/sparse_map_test.js` checks finite software-rendered
geometry, stale-map labeling, independent map selection and empty exports.
