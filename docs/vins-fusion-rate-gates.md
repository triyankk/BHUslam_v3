> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> These rate and health experiments evaluate localization continuity needed for that goal. See the [shared project goal](../PROJECT_GOAL.md).

# VINS-Fusion tracking-rate and shadow-gate tests

These estimator tests use **VINS-Fusion stereo + IMU only**, upstream commit
`be55a937a57436548ddfb1bd324bc1e9a9e828e0`. ORB-SLAM3 and VINS-Mono are not run.
The historical `orb.yaml` filename is a shared camera/IMU calibration schema;
it does not select an estimator. ROS-header stubs shared from another tools
directory are compatibility helpers, not another SLAM implementation.

## Defaults and test controls

The live adapter now defaults to mild CLAHE (clip limit 2, 8×8 tiles), as
requested. `--clahe-clip 0` disables it for a controlled comparison.
The `bin/vins-fusion-live` launcher selects four OpenCV threads when no explicit
thread flag is present, based on this Jetson's 15 W tests. Explicit overrides
are preserved. The native binary's internal baseline remains one thread.
`--opencv-threads N` selects OpenCV's worker count; Ceres solver settings,
feature cap/separation, IMU noise and physical calibration are unchanged.
The active VINS-Fusion publication gate now requires 0.5 seconds of continuously
good capture timestamps, ≥40 persistent features, ≥20 backend tracks and ≥20
stereo matches. Finite/alignment/velocity/reset/freshness guards remain unchanged.
`configs/vins-fusion-extnav.yaml` selects `Publisher.persistent_min: 40`; this
Fusion-specific key defaults to 40 if absent. The legacy `Publisher.quality_min`
key remains for ORB callers and is not used by Fusion. The old shared ORB
configuration and boot recording service are unchanged.

MAVLink reset counters increment on output recovery after a loss/reset/gap.
Raw reported quality remains persistent-feature count capped at 100. The Fusion
Cube test launcher passes `--quality-min 40` to the C++ relay; the Cube setup
preset sets `VISO_QUAL_MIN=40`. Generic/legacy relay callers retain their default
of 60. Apply only the Cube quality setting with
`bin/vio-test cube-config --apply --quality-only`; it snapshots/readbacks the
change and preserves all other observed settings. Alignment of these thresholds
does not demonstrate Cube EKF fusion.

```bash
# Camera check; no UDP or Cube data unless explicitly requested.
bin/vins-fusion-live configs/gemini336l-CPC2963000JJ-fast.yaml \
  configs/vins-fusion-extnav.yaml reports/NEW-live-rate-test \
  --seconds 20 --opencv-threads 4 --clahe-clip 2 --allow-provisional
```

Changing thread count can change image spacing and optimization termination;
more workers do not guarantee better accuracy. Run performance comparisons
sequentially at verified 15 W, with no concurrent build or replay.

## Paced recorded input

```bash
bin/vins-fusion recordings/diagnostic/20261025T095519.847430960Z \
  reports/NEW-paced-rate-test --opencv-threads 4 \
  --clahe-clip 2 --realtime-latest --preload-images
```

The source recording becomes available at its original timestamps relative to
a host playback start. Each iteration chooses the latest available stereo pair
and skips earlier pending images. All intervening IMU samples are integrated,
with an owning interpolated image endpoint. `source_index` records each selected
frame, and summary counts distinguish processed and skipped frames.

`--preload-images` loads the lossless source images before the timing clock, with a 512 MiB available-memory reserve check. This avoids storage stalls contaminating rate comparisons; the benchmark cache is not part of the live adapter. Cache bytes/preload time and timed image-access cost are logged separately.

This is a rate-sensitive **recorded-input simulation**, not live capture.
`playback_age_ms` is time since a frame's scheduled playback availability;
it includes file loading and processing but no camera USB/exposure delay or
live rectification cost. It cannot establish camera-to-Cube age. The offline
executable has no navigation transmission path. Synthetic timing is used only
for diagnostics; recorded poses are not sent as live data.

Paced mode requires an explicit thread count and rejects `--viewer` because
encoding/export would contaminate timing. Ordinary unpaced viewer replay still
works without `--realtime-latest`.

## Shadow gates

Both replay and live logs include the active decision and independent comparison gates.
Replay `eligible` is simulation only; live `eligible` controls optional UDP output.
Summaries record the active `publication_gate` policy:

| Decision | Persistent minimum | Recovery |
| --- | ---: | --- |
| `eligible` (active policy) | 40 by default | 0.5 s of good capture timestamps |
| `gate_frame30` / live `shadow_frame30_feature60` | 60 | 30 consecutive good samples |
| `shadow_time60_500ms` | 60 | 0.5 s of good capture timestamps |
| `shadow_time40_500ms` | 40 | 0.5 s of good capture timestamps |
| `shadow_frame15_feature40` | 40 | 15 consecutive good samples |

All shadows retain the same shared hard guards, including 150 ms age/gap
limits and reset handling. They close immediately on an invalid processed
sample. Only their recovery/count criteria differ. They **never publish** and
are independent of the active gate. The 0.5 s / 40-feature policy was explicitly
promoted after the desk and outdoor tests; its shadow remains a cross-check.
The downstream quality filters are explicitly aligned to 40 for the Fusion
profile, as described above.

The hard-rejection mask is 1=uninitialized, 2=unaligned, 4=invalid pose,
8=invalid velocity/finite motion, 16=stale or missing time, 32=future time.
Support failures are visible separately in the feature-count columns. Gaps
and epochs reset gate state but are not separate mask bits. The velocity
sanity ceiling is the existing provisional 10 m/s bench limit, not a calibrated
flight envelope. Replay assumes alignment after initialization and does not
validate the real mount or Cube heading alignment.

Native tests check timed recovery at different sample rates, immediate closure,
epoch/gap behavior, stale/future/duplicate times, nonfinite/nonrigid poses and
invalid velocity. They verify the mechanism, not the correctness of arbitrary
quality thresholds or a flight-safe recovery duration.

The 15-frame candidate is appended to CSV logs and summary counts in both
adapters. It remains logging-only; the 0.5 s / 40-feature policy is active.
At 20 processed fps its warmup takes approximately 0.75 seconds, and it takes
longer at lower processing rates. The controlled 45-second recorded-motion
comparison and its reproducible runner are under
`reports/fusion-frame15-feature40-20261028/`.

## IMU endpoint correction and results

An [endpoint lifetime bug](vins-replay-endpoint-issue.md) was found during these
tests and fixed in the VINS-Fusion recorded-data adapter. Corrected summaries
carry `imu_endpoint_owned: true`. Pre-fix exploratory runs are excluded from
the final rate comparison; image-only contrast diagnostics remain usable.

Results, provenance, binaries/source hashes and regression checks are retained
under `reports/fusion-rate-gate-20261028/`. No Cube parameters, EKF source
selection, arming commands or boot services are changed by these tests.
