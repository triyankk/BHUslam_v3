# Live VINS-Fusion latency at 15 W — 28 October 2026

## Result

Live processing with a single newest-image slot avoids accumulating a video
backlog at 15 W. Latency is measurable but not the main failure in this run:
the conservative publication gate could not sustain continuous output. This
is not a flight-readiness or trajectory-accuracy result.

| Measurement | Stationary, 20 s | Motion, 60 s |
| --- | ---: | ---: |
| Processed images | 285 | 832 |
| Processing throughput | 14.19 Hz | 13.83 Hz |
| Initialized images | 275 | 822 |
| Processing duration median / p95 | 69.68 / 78.88 ms | 71.40 / 94.24 ms |
| Estimated pose age median / p95 | 84.19 / 103.40 ms | 89.29 / 115.66 ms |
| Maximum estimated age | 117.63 ms | 176.15 ms |
| Images older than 150 ms | 0 | 1 |
| Pending stereo pairs overwritten | 310 | 963 |
| Frames skipped waiting for bracketing IMU | 0 | 0 |
| Gate-eligible poses / UDP publications | 0 / 0 | 67 / 67 |

The motion run's first/last 10-second median ages were 94.23/90.78 ms. Processed
image intervals had a 66.75 ms median and 100.12 ms p95; the newest-frame policy
kept age bounded while skipping camera images. This does not establish the
accuracy effect of those skipped images. No reset of the upstream initialized
state occurred, which is not proof of tracking continuity.

## Transport

The existing disarmed-only USB bridge forwarded 53 poses. Estimated age at the
serial write was **117.60 ms median, 141.82 ms p95, 145.73 ms maximum**. Serial
write-call duration was 0.029 ms median / 0.037 ms p95; it measures host write
completion, not USB transmission completion or Cube consumption.

Matching producer and bridge capture timestamps gives an additional
post-publication-to-write delay of **14.98 ms median / 41.86 ms p95**, maximum
66.01 ms. This includes scheduling and waiting in the 10 Hz relay; it is not
an isolated measurement of localhost UDP cost. Forwarded inter-packet spacing
was 102.21 ms median / 153.87 ms p95 during the burst.

The relay reported 14 rejections. This version does not classify rejection
reasons, so they cannot all be labelled network loss or stale packets. No USB
link drops, malformed inbound MAVLink packets or parameter writes occurred.
The bridge received 108 heartbeats. Tests and processes have stopped and the
Cube USB port was released; 15 W remained selected.

## Why output was intermittent

The stationary scene had 8 stereo matches at the median (maximum 13), below the
20-match publication requirement. It therefore produced no eligible poses.

During motion, 430 of 822 initialized images had fewer than 60 persistent
features; 110 had fewer than 20 stereo matches, and 12 had fewer than 20 backend
tracks. These reason counts overlap. Only 363 initialized images met all three
visual-support thresholds, and the 30-consecutive-good-sample warmup reduced
eligible output further. Publications ran from 14.45 s to 20.26 s after the
first processed frame; the longest gate-closed interval was 39.58 s.

This is withheld output under provisional thresholds, not proof that all the
underlying VINS poses were incorrect. Conversely, remaining initialized is not
proof that those poses were correct. The thresholds were not relaxed to make
this benchmark pass.

## Cube result

Cube telemetry had EKF status flags 1024 (uninitialized), repeated
`EKF3 waiting for GPS config data`, and `VisOdom: not healthy`. Other existing
pre-arm messages included missing RC/rangefinder data, battery voltage, compass
and attitude issues. No EKF origin, source-selection, arming or parameter command
was sent. This proves fresh live VINS packets reached the host's Cube USB write
path, **not that the Cube fused them**. A known EKF origin and an actual
source-switch/fusion validation remain necessary.

## Power and scope

`nvpmodel` verified mode 0 / 15W before and after. Across 62 tegrastats samples,
the reported VDD_IN rail averaged 6.97 W (5.89–7.95 W); maximum reported CPU
temperature was 53.72 C. This rail is not a battery-level measurement including
all peripherals, and 15 W is a configured operating mode, not measured constant
consumption. No concurrent build was intentionally run during captures.

The calibration, IMU noise, mounting offset and delivery-based clock mapping
remain provisional. No loop closure or ground truth was used. The live gate
uses support heuristics because upstream failure detection is disabled.

## Recommendation

Keep the separate bridge, but treat its rate-limit waiting as part of the
latency budget. Investigate short pose availability first: inspect feature
support/scene quality and measure calibration, then validate the gate against
tracking errors. Any proposal to loosen it needs evidence. Revisit the 10 Hz
relay cap for the dedicated USB link after freshness behavior is tested; it
was originally chosen for the shared low-baud UART. Repeat longer motion tests
and deliberate occlusions before enabling EKF source set 2.

## Evidence and implementation

- `stationary/` and `motion/`: settings, per-frame CSV, pose history and summary.
- `motion-bridge/`: inbound Cube tlog, events, link/readiness and summary.
- `motion-tegrastats.log`, power snapshots and `analysis.json`.
- `tested-binaries/` and `tested-binaries.sha256`: exact executables used for
  capture, preserved before subsequent input-validation cleanup/rebuild.
- Native VINS IMU/geometry tests and native navigation protocol, simulated Cube
  integration and transport tests passed before the hardware motion run.
- After the final build, both VINS tests and all three navigation/transport
  tests passed again. A three-second dry run verified the reusable launcher;
  a twelve-frame recording replay initialized successfully after the shared
  defaults refactor. These smoke runs are separate from the timing results.
- Runtime and hardware transport are C++; the temporary offline analysis used
  Node to join CSV/JSON timestamps. No Python was introduced.

See [repeatable commands](../../docs/vins-fusion-live-latency.md). No boot service
was changed. The official VINS-Fusion checkout remains unmodified.
