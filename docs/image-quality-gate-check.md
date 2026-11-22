> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> These diagnostics evaluate image observability and localization health for that goal. See the [shared project goal](../PROJECT_GOAL.md).

# Image quality and gate diagnostics

The 28 October stationary investigation is documented in
[`reports/fusion-image-gate-20261028/review.md`](../reports/fusion-image-gate-20261028/review.md).
It includes native image diagnostics, 30/15 Hz recording replays and an actual
live dry gate check. No calibration or publication thresholds were changed.

The subsequent [moving A/B test](../reports/fusion-moving-image-gate-20261028/review.md)
compares original and contrast-enhanced images from the same 45-second capture.
Its [viewer](../reports/fusion-moving-image-gate-20261028/index.html) contains
stereo feature overlays and sparse geometry for each replay.

Build and run the offline C++ image tool:

```bash
source scripts/env.sh
cmake -S tools/vins_fusion -B build/vins-fusion -DCMAKE_BUILD_TYPE=Release
cmake --build build/vins-fusion --target fusion_image_quality --parallel 1
build/vins-fusion/bin/fusion_image_quality RECORDING NEW_OUTPUT
# Balanced corpus sampling, including first/last frames:
build/vins-fusion/bin/fusion_image_quality RECORDING NEW_OUTPUT --samples 60
```

Input is the existing stereo recording format with `frames.csv` and grayscale
left/right images. Every fifteenth pair is sampled for pixel statistics,
fresh corner coverage and stereo optical-flow consistency. An offline CLAHE
comparison at clip limits 2 and 3 is measured on the same pairs. `--samples N`
instead selects up to N evenly spaced frame indices including the endpoints;
it does not average pixels or modify the original recording. It does not modify the source images
or VINS configuration. JPEG overlays/contact sheet are for inspection; JSON
statistics are computed from lossless source images. The tool sends no network
or flight-controller messages. Image statistics have no universal pass/fail
threshold, and snapshot corners differ from continuous VINS tracks.

The offline replay supports an explicit contrast-enhancement experiment:

```bash
bin/vins-fusion RECORDING NEW_OUTPUT --opencv-threads 1 --viewer --clahe
# Explicit mild alternative; supplying the limit also enables enhancement:
bin/vins-fusion RECORDING NEW_OUTPUT --opencv-threads 1 --viewer --clahe-clip 2
```

`--clahe` applies OpenCV CLAHE independently to both images, clip limit 3 and
8×8 tiles, before feature tracking. `--clahe-clip VALUE` selects an explicit
positive finite clip limit up to 10 and enables the same preprocessing.
Enhancement defaults off. The exported viewer shows
the processed images. `summary.json` records `frontend_clahe` and preprocessing
parameters; `tracking.csv` appends `preprocess_ms`, whose cost is included in
`pipeline_ms`. It does not enable enhancement in the live adapter, modify the
recording, or change camera exposure/calibration and publication policy. Upstream
configuration's `equalize=0` is retained because this is adapter preprocessing.

The [recording corpus comparison](../reports/contrast-corpus-20261028/review.md)
includes 15 nonempty recordings and full replays of the two-minute outdoor,
stairwell and indoor transition. Four empty failed captures are listed in its
manifest, and the old invalid-IMU recording is used for image analysis only.

For live gate tests, use the [latency workflow](vins-fusion-live-latency.md).
Its default is a dry run. Offline visual-policy simulations exclude some live
checks and must never be used to retimestamp old poses as live navigation.
