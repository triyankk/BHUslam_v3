> **Numerical replay caveat (28 October 2026):** the recorded-data IMU endpoint adapter had a confirmed lifetime bug. Pre-fix VINS numerical replay results below are provisional; image-only and live-camera observations are unaffected by this specific expression. See [issue and corrected runs](../../docs/vins-replay-endpoint-issue.md).

# Moving image-quality and gate A/B test — 28 October 2026

## Result

Both image contrast and reduced image rate contribute to weak feature support.
The current publication policy amplifies those weak periods into long output
gaps. Offline contrast enhancement improves support substantially, but does not
give continuous gate availability and increases processing cost at 15 W.

The 45-second moving capture saved 1,350 stereo pairs, with no frame-index gaps
or timestamp mismatches and maximum stereo skew 1 µs. The native recording
check passed without warnings. Camera and IMU data are retained in
`recordings/diagnostic/moving-image-gate-20261028`.

## Controlled replay comparison

Original and CLAHE replays use the exact same every-second-image selection
(675 pairs, approximately 15 Hz) and retain **all original IMU samples**.
Only image preprocessing changes: CLAHE clip limit 3, 8×8 tiles, applied to
each eye before tracking. Noise, extrinsics, timing offset, solver settings,
feature cap/minimum distance and support-policy thresholds are identical.
No VINS core code was changed. An original-image 30 Hz replay is a reference
for the effect of image spacing, not a demonstrated live operating rate.

| Measurement | Original, 15 Hz | CLAHE, 15 Hz | Original, 30 Hz |
| --- | ---: | ---: | ---: |
| Initialized frames | 665 / 675 | 665 / 675 | 1,340 / 1,350 |
| Basic VINS support ≥20 tracks | 642 | 665 | 1,339 |
| Median persistent tracks | 56 | 81 | 81 |
| Median stereo matches | 53 | 75 | 69 |
| Frames meeting current visual thresholds | 291 (43.8%) | 527 (79.2%) | 1,059 (79.0%) |
| Frames passing visual-only gate with recovery 30 | 10 (1.5%) | 228 (34.3%) | 482 (36.0%) |
| Longest visual gate closed interval after initialization | 41.78 s | 11.95 s | 10.81 s |
| Median processing time | 59.05 ms | 87.18 ms | 67.17 ms |
| Processing p95 | 84.32 ms | 102.19 ms | 87.97 ms |

Percentages use initialized frames as their denominator. The visual-only gate
requires persistent tracks ≥60, backend tracks ≥20, stereo matches ≥20 and
30 consecutive qualifying samples; it also resets on capture gaps above 150 ms
or estimator epochs. The simulation **excludes live age, velocity/alignment and
other transport checks**. These percentages are upper bounds on actual live
publication availability, not measured Cube output rates.

The 30 Hz reference retained more persistent features than the original 15 Hz
run. That supports the diagnosis that wider image spacing makes tracking harder
in this recording. Recovery also takes about twice as long in seconds at 15 Hz
because the policy counts frames. The runs are not ground-truth accuracy tests.
The upstream failure detector is disabled, so remaining initialized is not
proof that the path is correct.

## Image quality

The camera moved through a room with bright windows and dark interior surfaces,
including views dominated by walls/ceiling. The median left-image mean was
44.77 on the 0–255 scale (sampled range 33.36–57.54). Inspect the
[contact sheet](images/contact-sheet.jpg) and, for example,
[the view at approximately 15 s](images/stereo-450.jpg).

Across 90 snapshot pairs, original versus CLAHE gave:

| Median snapshot diagnostic | Original | CLAHE |
| --- | ---: | ---: |
| Fresh corners | 35 | 99 |
| Occupied feature grid cells, out of 12 | 9 | 12 |
| Forward/backward stereo optical-flow matches | 29 | 63 |
| Matches with positive disparity and vertical mismatch <1 px | 28 | 53 |
| Absolute vertical mismatch, median | 0.079 px | 0.139 px |
| Within-snapshot vertical mismatch p95, median across snapshots | 0.504 px | 1.935 px |

Enhancement reveals useful dark-region detail, but also produces more candidate
matches with larger stereo residuals. More candidates do not automatically mean
more accurate poses. Snapshot statistics differ from accumulated VINS tracks.
Calibration, exposure and camera properties were not changed for the A/B test.

## Why not simply enable CLAHE or loosen the gate now

The enhancement itself cost 4.29 ms median / 4.33 ms p95. Total processing rose
by about 28 ms at the median, so its cost is not limited to the pixel operation;
the estimator also processes different feature sets. The enhanced replay took
60.19 s wall time for 45 s of recorded input, versus 42.32 s for the original
15 Hz replay. Viewer export contributes to wall time; reported per-frame
pipeline duration excludes viewer export but includes image loading and CLAHE.

These are offline measurements under 15 W, not camera-to-Cube latency. On the
live newest-frame adapter, more processing time can increase image spacing
further, change tracking support and alter gate behavior. No claim is made that
the enhanced pipeline sustains 15 Hz live or meets the 150 ms age limit.
Solver termination has a wall-time budget, so differing workload/scheduling can
also affect resulting poses; trajectory differences cannot all be attributed
solely to contrast.

Offline gate alternatives illustrate the recovery penalty:

| Visual-only policy at 15 Hz | Original would-pass | CLAHE would-pass |
| --- | ---: | ---: |
| Persistent ≥60, stereo ≥20, recovery 30 | 1.5% | 34.3% |
| Same thresholds, recovery 15 (~1 s) | 11.7% | 48.1% |
| Same thresholds, recovery 10 (~0.67 s) | 17.6% | 54.3% |
| Persistent ≥40, stereo ≥20, recovery 10 | 55.6% | 83.9% |

Even the relaxed candidates still have gaps. They were not applied to the live
gate and are not validated confidence models. A healthy initialized sample
count does not establish that the rejected samples were safe to send.

## Next step

Keep the actual publication gate and calibration unchanged. Use enhancement
as an experimental mode, measure its real live pose-age/support tradeoff at
15 W, and investigate cheaper exposure/contrast or frontend improvements that
preserve tracking rate. Compare a time-based recovery rule in shadow mode;
retain immediate rejection of stale, invalid and reset poses. Before replacing
the gate, check trajectory consistency and deliberate tracking-loss recovery,
not merely the number of packets it would allow.

## Viewer and verification

Open the [moving A/B viewer](index.html) and select Original 15 Hz, Contrast
15 Hz or Original 30 Hz. Each shows synchronized stereo feature overlays and
sparse geometry, including the existing behind-camera perspective checkbox.
Switching variant resets playback. **The viewer's supported/weak status uses
the estimator's ≥20-track diagnostic, not the stricter publication policy.**
Use the table above for publication-gate interpretation. Contrast-viewer
intensities are the enhanced estimator inputs, not the original raw recording.

All three export checks passed: source timestamps and camera CSV agree, image
pairs and PLY epochs are consistent. Solved-landmark reprojection checks had
p95 2.600 px (original 15 Hz), 2.418 px (CLAHE 15 Hz) and 2.562 px (original
30 Hz). Those are export/internal-consistency checks, not ground-truth accuracy.
Software/mocked-WebGL viewer checks and the two native VINS IMU/stereo tests
passed. Browser GPU rendering was not automatically verified.

`gate-analysis.json`, `images/summary.json`, replay summaries, per-frame CSV,
logs, checks and binary/source hashes are retained here. Run
`node reports/fusion-moving-image-gate-20261028/analyze-gates.js` to reproduce
the support-policy calculations. `15hz-input/provenance.json` documents the
derived input. [Offline tool instructions](../../docs/image-quality-gate-check.md)
document the new explicit `--clahe` replay option, which defaults off.

Capture and analysis processes have finished. The Jetson remains at 15 W.
No Cube poses, source-switch commands or parameter changes were sent, and
boot services were not modified. Loop closure was off and no ground truth was
available. Runtime image processing and VINS remain C++; no Python was added.
