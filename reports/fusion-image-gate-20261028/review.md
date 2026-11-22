> **Numerical replay caveat (28 October 2026):** the recorded-data IMU endpoint adapter had a confirmed lifetime bug. Pre-fix VINS numerical replay results below are provisional; image-only and live-camera observations are unaffected by this specific expression. See [issue and corrected runs](../../docs/vins-replay-endpoint-issue.md).

# Image quality and publication gate — 28 October 2026

## Finding

The unchanged live publication gate works continuously in the latest stationary
scene after its initial warmup. Earlier output interruptions correlate with
lower visual support and are amplified by the 30-good-frame recovery rule.
This does not establish that all withheld estimates were wrong, or prove that
relaxing the gate would make navigation reliable.

The user confirmed the entire new recording was **stationary**. It must not be
interpreted as a motion-tracking test. Calibration, camera exposure settings,
VINS preprocessing and the actual live gate thresholds were left unchanged.
No navigation data was sent to the Cube during this investigation.

## Image observations

The full 640×400 stereo views show a room with bright windows and much darker
interior surfaces. The lens is not covered, and useful furniture/window edges
are visible in both eyes. Image quality is uneven across the scene; a single
brightness average does not describe the usable feature distribution.

Across 60 sampled stereo pairs:

| Image diagnostic | Left | Right |
| --- | ---: | ---: |
| Median mean intensity, 0–255 | 48.83 | 45.45 |
| Median fraction at or above 250 | 4.92% | 4.21% |
| Median fraction at or below 5 | 0.93% | 1.14% |
| Median Laplacian variance | 437.3 | 375.2 |

Laplacian variance is scene/noise dependent and is not a calibrated blur score.
The first sampled frame was brighter (left mean 119) than subsequent samples
(about 49 by 0.5 s). Exposure settling is a possible explanation, but actual
exposure settings were not logged over time, so its cause is not established.

An independent single-pair feature check used the VINS-style 150-corner cap,
30 px separation and forward/backward stereo optical flow. Offline CLAHE
(clip limit 3, 8×8 tiles) gave:

| Median snapshot diagnostic | Original | CLAHE candidate |
| --- | ---: | ---: |
| Freshly detected corners | 52 | 102 |
| Grid cells with corners, out of 12 | 9 | 12 |
| Forward/backward stereo matches | 41 | 70 |
| Positive disparity and vertical mismatch below 1 px | 41 | 67 |
| Absolute vertical mismatch, median | 0.059 px | 0.092 px |

These are snapshot diagnostics, not VINS's accumulated temporal tracks. They
are not an A/B test of trajectory accuracy. CLAHE improves candidate coverage
in this scene but also increases the stereo residual tail; it is not enabled
in the estimator. The original matched points are consistent with good
rectification in this stationary view, not a complete camera/IMU calibration.

Inspect [initial stereo](images/stereo-0.jpg),
[stereo after 25 seconds](images/stereo-750.jpg), and the
[contact sheet](images/contact-sheet.jpg). Green/yellow markers denote
stereo-matched/left-only corners; right matches are cyan. Images are diagnostic
overlays on the original IR intensity, not enhanced display images.

## Continuous tracking versus publication policy

The same recording was replayed through the unchanged C++ VINS-Fusion core:
all 896 stereo pairs at approximately 30 Hz, then every second pair at
approximately 15 Hz with **all original IMU samples retained**. The latter
approximates image spacing; it does not reproduce the live latest-frame
schedule, USB timing or freshness checks.

| Replay diagnostic | 30 Hz input | 15 Hz input |
| --- | ---: | ---: |
| Initialized frames | 886 / 896 | 438 / 448 |
| Median persistent tracks | 141 | 141 |
| Median stereo matches | 92 | 91 |
| Frames meeting current visual thresholds | 886 | 438 |
| Frames passing visual-only 30-frame warmup | 857 | 409 |
| Visual-only first pass after first input | 1.30 s | 2.60 s |
| Maximum position change from pose at 5 s | 3.59 mm | 0.71 mm |

Both replays stayed visually supported throughout after initialization. The
stationary position changes describe this short recording only; different
window timing and solver wall-time budgets mean the lower value at 15 Hz is
not proof that decimation improves accuracy. Loop closure was off.

## Actual live check

A subsequent 20-second stationary dry run at 15 W processed 297 images at
14.77 Hz. It initialized on 287, maintained at least 128 persistent tracks and
81 stereo matches, and produced **257 gate-eligible samples**. The gate opened
at 2.603 s and stayed open to the last processed image. UDP was disabled.

Estimated pose age was 82.43 ms median / 110.04 ms p95 / 124.58 ms maximum.
There were no IMU-bracketing skips. Age uses the existing provisional
device-to-host delivery clock mapping and excludes an independently measured
exposure/USB delay. `live-steady/` contains per-frame logs and summary.

## What the previous motion log says about the gate

The previous live motion run had only 58 persistent tracks at the median.
Replaying its logged support counts, capture gaps, epochs and age through a
support-only counterfactual gives:

| Offline policy candidate | Would-pass samples / 822 initialized | Longest closed interval |
| --- | ---: | ---: |
| Current: persistent ≥60, stereo ≥20, recovery 30 | 67 (8.2%) | 39.58 s |
| Same thresholds, recovery 10 | 116 (14.1%) | 16.52 s |
| Persistent ≥40, stereo ≥20, recovery 10 | 343 (41.7%) | 5.34 s |
| Persistent ≥20, stereo ≥20, recovery 10 | 523 (63.6%) | 3.74 s |

The current-policy count reproduces the observed 67 eligible samples, which
supports the diagnosis that visual support and recovery dominate that run.
Counterfactuals omit velocity/alignment validity because those flags were not
logged; their counts are upper bounds, not safe output guarantees. No poses
from these offline calculations were transmitted. Dropping the stereo
threshold alone leaves the current-policy result at 67, so it does not solve
the motion-run interruption.

The earlier stationary live test was different: it had only 8 stereo matches at
the median (maximum 13), so its stereo threshold prevented publication. There
are no saved images from that earlier test, so its exact scene/image cause
cannot be reconstructed. The fresh recording does not retroactively describe
that view.

## Next action

Capture a genuinely moving sequence with saved images, then compare original
and CLAHE preprocessing on exactly the same input. Check persistent feature
coverage, stereo consistency, pose continuity and movement consistency before
choosing preprocessing. Preserve age/reset checks. Evaluate recovery in elapsed
time rather than an unqualified frame count, since 30 samples mean about one
second at 30 Hz but two seconds at 15 Hz. Any reduction of support thresholds
needs motion/dropout evidence; these stationary results do not justify it.

Calibration can follow this image/gate investigation. This stationary test does
not explain motion blur, occlusion, vibration, or real tracking failure.

## Reproducibility

- Original dataset: `recordings/diagnostic/image-gate-20261028`.
- Operator correction: `operator-context.json`. Ignore the recorder's generic
  10/60/10 protocol fields for this custom 30-second stationary capture.
- Recording check passed with 896 pairs, no frame-index gaps and maximum stereo
  skew 1 µs; the expected warning says the generic 80-second protocol was not
  completed. Raw and rectified IR, IMU and factory settings are retained.
- `steady-15hz-input/provenance.json` documents the derived image selection.
- `images/samples.json`, `images/summary.json`, `gate-analysis.json`, binary
  hashes and replay/live logs hold the measurements. Rerun the offline count
  analysis with `node reports/fusion-image-gate-20261028/analyze-gates.js`.
- Image processing and VINS replay/live code are C++. The count-report script
  uses Node; no Python was introduced. Native VINS core was not modified.
