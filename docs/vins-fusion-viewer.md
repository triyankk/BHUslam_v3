> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> This viewer supports localization and obstacle-scene diagnosis from recorded evidence. See the [shared project goal](../PROJECT_GOAL.md).

# VINS-Fusion stereo / sparse 3D recording viewer

Open `reports/vins-fusion-viewer/index.html` and choose the two-minute or eighty-second moving recording. The upper panel shows the recorded rectified left/right IR images with feature observations, track trails and optional stereo match lines/IDs. The lower panel shows synchronized camera motion, active sliding-window landmarks and optional accumulated landmark history. It works offline, including a software renderer when WebGL is unavailable.

Play/pause, speed control, frame stepping, scrubbing, orbit, zoom, shift-drag pan, fit/current-map, whole-route and camera-follow controls are included. Click a feature in either eye to highlight the corresponding solved 3D landmark, when available. Space toggles playback; arrow keys step. Selecting a different recording in the outer page resets playback and geometry. The folder picker can open another **exported viewer folder**, including its JSON and images.

## Run another chosen recording

All estimation and export runs in C++; browser JavaScript only renders the saved results. No ROS or Python is required.

```bash
# Build if necessary, using the workspace ARM64 toolchain.
scripts/build-vins-fusion.sh

# Choose an existing recording directory and a new export directory.
bin/vins-fusion-viewer recordings/diagnostic/RECORDING reports/my-fusion-view
# Open reports/my-fusion-view/viewer.html after processing completes.
```

The launcher runs `bin/vins-fusion RECORDING NEW_OUTPUT --viewer --opencv-threads 1`. The default noise policy matches the provisional-noise Fusion benchmark. Use `--native-noise` for the separately labelled upstream EuRoC policy. Existing output directories are rejected to preserve prior results. Raw recordings need this native preprocessing before they can be played with a 3D map; the browser cannot run the estimator on an arbitrary raw folder.

Default `--viewer-stride 3` processes **all** approximately 30 Hz image pairs through Fusion but exports synchronized displays at approximately 10 Hz. `--viewer-stride 1` exports every processed frame for smoother playback at the cost of more disk/memory. An optional `--max-frames N` bounds a smoke run. Export work adds image encoding, geometry copying and serialization overhead, so viewer-run wall times/RSS are not benchmark measurements. `summary.json` marks `viewer_export: true`.

## Meaning of the display

Each view has a single source image index and hardware timestamp. Stereo feature coordinates come from the native tracker before backend rejection; green left points have track age at least two frames, yellow are new left points and blue are right-eye matches. Trails show the same feature ID in previous exported frames of the same epoch. A feature observation is not itself a solved 3D point, and the tracker can retain points that the backend rejects.

Landmark reconstruction follows upstream `pubPointCloud`: the solved feature's anchor ray and depth are transformed through camera-to-body and anchor-body-to-world transforms. Only finite, positive solved depths with mature anchors are shown; a display-only 80 m anchor-depth limit excludes very distant geometry. Coordinates are rounded to 1 mm to bound export size, without claiming millimetre accuracy.

Active landmarks are the estimator's current sliding-window cloud. Accumulated history updates a landmark's position while it remains observed and freezes its last saved position afterward. This is **not a globally optimized persistent SLAM map**, and drift can leave duplicated/misaligned surfaces when a place is revisited. Loop closure remains disabled. PLY files contain the final accumulated history separately for each reset epoch. Seeking backward rebuilds from past observations only; future map points do not appear early. Separate epochs are never joined.

Native Fusion failure detection is disabled in the pinned upstream revision. The status panel therefore separates initialization from the same diagnostic visual-support heuristic used in the benchmark (20 persistent frontend and 20 backend tracks). Weak-support trajectory portions are amber; this is not an independent accuracy certificate or flight health gate.

The default whole-route view uses the full camera-route extent to keep the viewpoint stable while geometry appears progressively; it does not display future trajectory or landmarks. Fit current map uses geometry available at the selected time. Follow camera uses an 8 m local view. All axes use the native local inertial coordinate system in metres.

Exports contain `viewer.html` with embedded geometry/observations, `viewer-data.json`, `viewer-images/*.jpg`, separate-epoch PLYs, and the original replay CSV/config/summary outputs. Keep the image folder alongside the HTML when copying an export. No local server, network fetch, camera access or flight-controller connection is required to play the exported viewer.

A native static preview can be generated with `build/vins-fusion/bin/fusion_viewer_preview EXPORT 40`. Add `--refresh-viewer` after the time to re-embed the latest UI template in an existing export without rerunning estimation. This is an OpenCV-rendered export preview, not a browser screenshot.

Viewer exports are fresh replays with Fusion's original wall-time solver budget; they can differ slightly from prior benchmark trajectories because optimization scheduling is not bitwise deterministic. The original measured Fusion executable was preserved as `reports/vins-fusion-benchmark-20261026/full/vins_fusion_replay.benchmark`, matching its original SHA-256, before adding the optional exporter.

Enable **First-person / behind camera** to view the sparse scene from 0.8 m behind the current camera, facing along its optical axis. The view follows translation and full orientation, including camera roll, with image-right and image-down directions preserved. Wheel zoom adjusts this perspective separately from the orbit view. Orbit/pan and the older position-only Follow camera control are inactive in this mode. Uncheck the box to return to orbit, or use Fit current map, Whole route view or Top view. Before pose initialization, the label explicitly says that camera perspective is waiting. Near/far clipping prevents geometry behind the viewer from being mirrored into view.
