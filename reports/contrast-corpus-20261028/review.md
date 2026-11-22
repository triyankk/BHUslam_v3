> **Numerical replay caveat (28 October 2026):** the recorded-data IMU endpoint adapter had a confirmed lifetime bug. Pre-fix VINS numerical replay results below are provisional; image-only and live-camera observations are unaffected by this specific expression. See [issue and corrected runs](../../docs/vins-replay-endpoint-issue.md).

# Contrast across recordings and two-minute transitions — 28 October 2026

The two-minute recording is a useful field-transition test: sampled images show outdoor sunlight/shade, a stairwell and the indoor room. This investigation uses it directly, rather than treating the desk test as representative of all environments.

**Conclusion:** enhancement improves snapshot support across much of the corpus, but whole-run visual-gate availability only changes from 49.5% to 52.7% / 52.4%. All three remain at 0% in the approximate stairwell/entry phase. This does not support making contrast enhancement the primary fix for transition outages; gate/recovery and tracking-rate validation remain priorities.

## What was tested

- All **15 nonempty recordings**: 862 evenly spaced original stereo pairs, up to 60 per recording including its first/last frame. Four empty failed captures are listed in manifest.json. No pixels were averaged together.
- Original images versus CLAHE clip limit 2 (mild) and 3 (moderate), both with 8×8 tiles. Each method sees the same image pairs.
- Three full sequential C++ VINS-Fusion replays of the two-minute transition recording at approximately 15 Hz image spacing, retaining all original IMU samples. The derived frame list and source path are recorded in two-minute-15hz-input/provenance.json.
- Calibration, noise, solver settings, feature cap/separation and actual publication policy were unchanged. Enhancement is an explicit offline option and remains off in the live adapter. No Cube navigation, parameter or source-switch commands were sent.

## Main result: temporal tracking and the visual gate

The following percentages are **offline visual-policy simulations**, not actual Cube output or pose accuracy. They require initialized VINS, ≥60 persistent features, ≥20 backend tracks, ≥20 stereo matches and 30 consecutive qualifying samples. Capture gaps/epochs reset the streak. Live age, velocity, alignment and relay checks are excluded, so these are optimistic upper bounds for live publication.

| Setting | Whole run | Outdoor 0–65 s | Stairs/entry 65–85 s | Indoor 85–120 s | Longest gate-closed interval |
| --- | ---: | ---: | ---: | ---: | ---: |
| original | 49.5% | 44.4% | 0.0% | 87.0% | 27.43 s |
| mild | 52.7% | 49.8% | 0.0% | 88.2% | 26.97 s |
| moderate | 52.4% | 48.9% | 0.0% | 88.8% | 26.77 s |

Phase boundaries are approximate visual annotations, not surveyed environment labels. The indoor phase includes a mostly stationary tail; its strong availability must not be mistaken for recovery during all movement. All three replays retained the upstream initialized state; upstream failure detection is disabled, so that is not proof of tracking health.

## Cost and stationary tail

| Setting | Enhancement median, ms | Pipeline median, ms | Pipeline p95, ms | Quiet-tail endpoint change, mm | Quiet-tail maximum change, mm |
| --- | ---: | ---: | ---: | ---: | ---: |
| original | 0.00 | 67.40 | 91.64 | 3.40 | 3.49 |
| mild | 4.26 | 78.93 | 99.32 | 2.82 | 3.97 |
| moderate | 4.26 | 79.82 | 100.63 | 0.59 | 1.38 |

These are **offline processing times under 15 W**, not camera-to-Cube age. Pipeline includes image loading and preprocessing but excludes viewer export. Full wall time includes export. Improved feature support also changes tracking/optimization workload, so total cost can exceed the CLAHE pixel-operation cost. Each estimator run was sequential and used one OpenCV thread, fixed solver settings and the same baseline provisional noise convention. Solver termination has a wall-time budget; scheduling can affect estimates.

Quiet-tail displacement uses the existing 91–115 s window, relative to its first available pose. This measures short stationary consistency, not ground-truth accuracy or complete trajectory drift. No ground truth, loop closure or field flight was used.

## Corpus image statistics

The diagnostic match count below means a fresh corner that passes forward/backward stereo optical flow, positive disparity >0.5 px and vertical mismatch <1 px. It is a useful screening proxy, not a guarantee of a true correspondence or a navigation confidence metric. Far-depth attitude features can also be useful even when excluded by this disparity screen.

Recording-equal-weight mean screened matches: **21.4 original, 32.8 mild, 35.7 moderate**. For recordings at least five seconds long, the corresponding means are 22.7, 34.7, 37.5. These weighting schemes prevent longer recordings from dominating; individual sampled frames remain paired across methods.

Mild wins/ties/loses against original on 758/35/69 sampled pairs. Moderate gives 767/32/63. The lower fifth percentile screened-match count remains zero under all three modes. More detected corners alone would conceal these weak cases.

| Recording (contact sheet) | Sampled pairs | Original mean | Mild mean | Moderate mean |
| --- | ---: | ---: | ---: | ---: |
| [baseline-01](corpus/baseline-01/contact-sheet.jpg) | 60 | 19.5 | 31.6 | 37.5 |
| [baseline-02](corpus/baseline-02/contact-sheet.jpg) | 60 | 13.8 | 20.1 | 19.9 |
| [diagnostic__20261025T091926.822317376Z](corpus/diagnostic__20261025T091926.822317376Z/contact-sheet.jpg) | 60 | 26.6 | 46.4 | 50.4 |
| [diagnostic__20261025T095519.847430960Z](corpus/diagnostic__20261025T095519.847430960Z/contact-sheet.jpg) | 60 | 72.5 | 88.5 | 90.5 |
| [diagnostic__20261025T100658.190042160Z](corpus/diagnostic__20261025T100658.190042160Z/contact-sheet.jpg) | 60 | 2.4 | 3.0 | 3.8 |
| [diagnostic__20261025T104811.422872864Z](corpus/diagnostic__20261025T104811.422872864Z/contact-sheet.jpg) | 60 | 1.7 | 0.4 | 0.3 |
| [diagnostic__20261026T090303.978496952Z](corpus/diagnostic__20261026T090303.978496952Z/contact-sheet.jpg) | 60 | 0.2 | 2.6 | 2.6 |
| [diagnostic__image-gate-20261028](corpus/diagnostic__image-gate-20261028/contact-sheet.jpg) | 60 | 40.9 | 62.5 | 67.4 |
| [diagnostic__moving-image-gate-20261028](corpus/diagnostic__moving-image-gate-20261028/contact-sheet.jpg) | 60 | 27.9 | 45.9 | 51.6 |
| [diagnostic__prepare-smoke-01](corpus/diagnostic__prepare-smoke-01/contact-sheet.jpg) | 60 | 23.4 | 35.3 | 41.2 |
| [export-smoke-01](corpus/export-smoke-01/contact-sheet.jpg) | 26 | 13.3 | 23.7 | 25.0 |
| [extnav-helper-smoke-01](corpus/extnav-helper-smoke-01/contact-sheet.jpg) | 56 | 19.1 | 30.7 | 35.3 |
| [interrupted-01](corpus/interrupted-01/contact-sheet.jpg) | 60 | 15.0 | 20.7 | 21.4 |
| [motion-01](corpus/motion-01/contact-sheet.jpg) | 60 | 19.0 | 33.7 | 34.2 |
| [motion-02](corpus/motion-02/contact-sheet.jpg) | 60 | 25.5 | 47.5 | 54.6 |

The old baseline-01 has invalid IMU units and is included **only for image statistics**, never VIO replay. Short smoke captures are included to honor the full-corpus scope but are separately excluded from the ≥5 s aggregate. These are 15 recordings, not 15 independent environments. No repeated sparse image fingerprints were found, which is not proof that all scene content is independent.

The near-black stationary recording diagnostic__20261025T104811.422872864Z is a counterexample to unconditional enhancement: screened matches fall from about 1.7 to 0.4/0.3 on average. Enhancement cannot manufacture reliable scene texture or recover clipped pixels. Some additional candidates have larger stereo residuals. All modes still have weak sampled frames.

## Transition recording: paired snapshot sections

| Time | Original mean screened matches | Mild | Moderate |
| --- | ---: | ---: | ---: |
| 0-20s | 110.2 | 129.2 | 130.2 |
| 20-40s | 126.7 | 138.2 | 138.0 |
| 40-60s | 125.5 | 139.3 | 139.0 |
| 60-80s | 44.7 | 61.9 | 64.9 |
| 80-100s | 12.9 | 28.1 | 32.0 |
| 100-120s | 14.7 | 34.2 | 38.9 |

These sparse snapshots explain image-level trends; they do not substitute for the continuous replay above. Outdoor frames often already have strong support; improvements are larger in the darker entry/interior. Average image brightness alone does not identify a safe preprocessing mode. The diagnostic brightness bins in corpus-analysis.json are descriptive, not learned or validated control thresholds.

## Policy implication

The evidence supports keeping enhancement as a bounded candidate, not enabling aggressive enhancement universally or selecting it from raw corner count. Any automatic controller should use stereo consistency, persistent/mature tracks, spatial coverage and measured pose age, with slow/hysteretic transitions. A fixed mild baseline remains worth testing live, but must satisfy the 15 W latency budget; the snapshot gain does not establish that it solves transition outages. Recovery policy and processing rate need separate validation. Do not loosen the gate merely to increase packet counts.

No automatic contrast controller, live preprocessing change or recovery-threshold change was deployed. The next decision should use the temporal phase table and live latency, not the corpus average alone.

## Inspect and reproduce

Open the [two-minute stereo/3D comparison viewer](index.html), choosing original, mild or moderate. The viewer's supported/weak label uses the estimator's basic ≥20-track diagnostic, **not** the stricter publication gate. Enhanced variants display processed estimator inputs. Native coordinates/scale are retained; no scale fitting or ground-truth alignment is applied.

The original [transition contact sheet](corpus/diagnostic__20261025T095519.847430960Z/contact-sheet.jpg) documents the environmental sequence. manifest.json, per-record samples/summary, corpus-analysis.json, gate-analysis.json, temporal-analysis.json, replay CSV, settings and completion markers retain the evidence. The input recordings were not modified. All three native export checks passed (601 sampled views each), the software/mocked-WebGL viewer check passed, and both native VINS IMU/stereo tests passed. Browser GPU rendering was not automatically verified. Preprocessing flags/clip limits and completion markers were cross-checked against all three 1,799-frame outputs. Native image/replay code is C++; offline orchestration/aggregation uses Bash and Node, with no Python added.

The first combined sampling/replay command terminated before finishing its original replay. Its partial output is preserved under incomplete/ and excluded. A detached retry did not persist; the final three complete replays ran separately as foreground host jobs. No permanent service or automation was installed.
