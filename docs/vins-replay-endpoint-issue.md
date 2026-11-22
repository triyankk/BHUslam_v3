> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> This recorded-input correctness investigation supports defensible localization tests. See the [shared project goal](../PROJECT_GOAL.md).

# Recorded-data IMU endpoint lifetime issue

Discovered during the VINS-Fusion rate/gate investigation on 28 October 2026.

The replay adapter stored a lazy Eigen expression over temporary vector
conversions before passing it to the estimator:

```cpp
auto aa = vec(a.a) + (vec(b.a) - vec(a.a)) * alpha;
auto ww = vec(a.w) + (vec(b.w) - vec(a.w)) * alpha;
```

Those expressions can reference expired stack temporaries. A minimal reproduction
of the same pattern with AddressSanitizer reports `stack-use-after-scope`;
evidence is in `reports/fusion-rate-gate-20261028/endpoint-lifetime-audit.txt`.
This is an adapter bug, not a change or identified defect in upstream
VINS-Fusion. Discovery does not by itself prove it caused every observed drift.

The VINS-Fusion replay now calls `bhu::interpolate_endpoint`, which returns
owning acceleration/gyro arrays after checking timestamp brackets, gap and
finite inputs. The live adapter already materialized its gyro interpolation
and evaluated its acceleration within the call; it did not use the unsafe
saved expression. It now shares the owning helper for consistent endpoint
handling. The numerical noise/extrinsic configuration was not changed.

Native tests verify known interpolation values, output independence from mutated
inputs, bracket rejection and consumption by VINS-Fusion's native preintegrator.
An owning-value reproduction is also checked with AddressSanitizer.

## Historical reports

**Pre-fix offline VINS-Fusion numerical results are provisional.** They may
be affected by undefined behavior and should not be used as validated trajectory,
contrast, gate or algorithm-ranking references without a corrected rerun.
Their output is retained as historical evidence, not silently replaced.
Image-only contrast measurements do not consume IMU and are unaffected.
Live-camera results did not use this specific saved-expression pattern, but
remain bench observations with provisional physical calibration.

The same expression appears in the legacy VINS-Mono adapter. That estimator
was not run or modified in this investigation; its old replay comparisons also
need review before being treated as validated. Current rate/gate estimator
tests run **VINS-Fusion stereo + IMU only**.

Corrected rate/gate runs are identified by `imu_endpoint_owned: true` in their
summary. Pre-fix exploratory paced runs and tested binaries are preserved under
`reports/fusion-rate-gate-20261028/pre-endpoint-fix/` and excluded from the final
post-fix comparison.
