# Recorded Gemini sparse 3D SLAM visualization — 2026-10-25

Source moving recording: `recordings/diagnostic/20261025T095519.847430960Z`.
Both replays use the original `orb.yaml`; no experimental timing/bias adjustment
was applied. Original recordings, older plots, camera settings, Cube parameters
and boot services were not modified.

Open `index.html` for stereo/VIO viewer links, or either replay's
`sparse-map.html` directly. The HTML embeds geometry; keep its `images/` folder
beside it for the synchronized feature-overlay previews. No HTTP server is needed.

| Replay | Tracked frames | Snapshots | Independent map epochs | Largest valid landmark count | Last saved geometry |
| --- | --- | --- | --- | --- | --- |
| Stereo | 3,410 / 3,597 | 61 | 3 | 19,028 | 120.009 s |
| Stereo + IMU | 2,488 / 3,597 | 61 | 4 | 18,106 | 90.107 s |

Each viewer contains rotatable landmarks, camera/keyframe frusta, a keyframe
route, optional actual map-graph edges, a timeline and play/pause. The initial
view selects the snapshot with the greatest valid landmark count. Use the map
selector to inspect other epochs and the timeline to review growth and loss.

Both replays process all 3,597 frames. Stereo returns 0 and exports its final
trajectory. VIO returns 1 because its final map is empty; map snapshots and
previews were already saved successfully and preserve the earlier reconstruction.
The final approximately 30 seconds in that viewer show explicitly stale geometry
alongside the later images. No pose is fabricated during that interval.

There are **zero exported loop edges** in both replays. Loop processing remained
enabled, but these runs do not demonstrate a successful loop closure. The viewer
can show actual loop edges if a future replay produces them. Covisibility edges
must not be mistaken for confirmed loop closures.

Rendering is bounded to 12,000 sampled landmarks per snapshot; the true valid
count is reported separately. XYZ is rounded to 1 mm for display/export size,
not because mapping accuracy was measured at that precision. Maps and reset
epochs remain separate coordinate frames. The stereo view uses X/Z/−Y display
axes; VIO uses its current world XYZ. These are sparse landmarks, not dense walls,
textured surfaces or ground truth. This visualization does not establish
navigation reliability.

The static `sparse-map-preview.png` in each replay shows a richly populated
representative snapshot. It need not be the viewer's default time: the PNG uses
the snapshot with most displayed points, while the viewer defaults to most
valid landmarks before sampling.

## Validation

- Native `gemini_replay` rebuilt successfully, without modifying ORB-SLAM3 itself.
- Offline UI fixtures passed for finite rendering, rotation/zoom, empty maps,
  independent map selection and stale-map labeling.
- Real stereo and VIO exports passed the same software-renderer/map-selector
  tests with their actual geometry.
- Artifact checks verified finite XYZ/quaternions, map ownership, graph node
  references, routes not crossing known tracking losses, valid JPEG previews,
  PLY vertex counts and embedded HTML JavaScript syntax.
- Both native PNG previews were visually inspected.
- Targeted CTest suite (`sparse_map_ui`, `imu_sync`, `vio_diagnostics_numeric`)
  passed. No flight-controller communication occurred in these replays.

Automated browser inspection could not be performed: the browser tool blocks
the `file://` protocol. No alternate server/browser workaround was used.
WebGL rendering has therefore not been exercised by that browser automation;
the software fallback and actual-data controls were verified offline.

These exports add image/geometry processing and influence replay scheduling.
Tracking counts may vary between runs; do not compare their wall times as a
controlled 15W performance benchmark. The recording itself was captured at 15W.

## Reproduce

```sh
bin/bhu replay recordings/diagnostic/20261025T095519.847430960Z \
  third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  --sparse-map --output reports/NEW-stereo-map

bin/bhu replay recordings/diagnostic/20261025T095519.847430960Z \
  third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  --inertial --allow-provisional --sparse-map --output reports/NEW-inertial-map
```

Backend and exporter remain C++; interaction uses local JavaScript/WebGL. No
Python was added. Details: `docs/recorded-sparse-slam.md`.

Each replay directory includes `artifact-check.json`, the original tracking and
pose-history CSVs, `sparse-map.json`, `sparse-map.html`, PNG preview and PLY point
clouds. Parent directory contains logs, explicit replay exit codes, test output
and checksums.
