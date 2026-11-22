# Previous-flight LIVO replay readiness

20 November 2026. Initial offline input audit below. **Follow-up: two exploratory
LIVO replays have now been run and both diverged; no improvement is established.**
See the [experiment results](experiment-review.md) for current status, decoded
scans, initial mounting/timing, software checks and input limitations. Live
services and Cube settings were not touched. Depth avoidance remains separate.

## Available recordings

Matched the latest four camera recordings to JT16 raw captures by boot ID.
Raw packet timestamps, sequence continuity and SDK CRC were checked. The raw
stream contains JT16 point packets, not IMU packets; use the saved Gemini IMU
for the first joint estimator experiment. The camera IMU includes saturation
in these flights, so compare results with that limitation explicitly reported.

| Cube log | Saved camera images | Selected image rate | Long raw LiDAR segment | Assessment |
| --- | ---: | ---: | ---: | --- |
| 211 | 237 | 11.0 Hz | 19.0 s overlapping candidate | Short supplementary test; other same-boot captures predate it |
| 212 | 827 | 9.1 Hz | 32.3 s and 38.3 s | Up to 11 s LiDAR gap; fault/recovery case |
| 213 | 2,941 | 20.2 Hz | 96.6 s | Preferred initial candidate, approximately 2,065 overlapping images |
| 214 | 1,352 | 9.7 Hz | 87.8 s | Secondary candidate; camera rate lower |

The long segment for 213 contains 287,758 SDK-CRC-valid point packets,
zero point sequence discontinuities and a maximum packet timestamp gap of
0.804 ms. Packet rate is not full-cloud rate: these are small packets within
approximately 5 Hz scans. Its file is:

`/home/atas/vscode/optFlow_slam/data/recordings/jt16-shadow/85f03d02-ec4c-4f9d-b518-57d9820692ec/flight-567516634-8234.raw`

The matching camera dataset is:

`/home/atas/BHUslamV3/runs/vins-fusion-boot/20261118T141316.161559760Z/live/flights/flight-518567821/input`

**Overlap is provisional:** file-open monotonic time and sensor span provide
coarse selection, not capture synchronization. The second segment starts
roughly 49 s into flight 213's camera recording. Do not concatenate across the
missing early interval or use camera data from another boot.

Flight 214's long raw file contains 1,009 skipped bytes and a partial tail;
accepted packet CRCs pass. Flight 211's later file has a partial tail. Do not
feed skipped/partial bytes to an estimator as valid packets.

The mapper's decoded per-flight cloud logs have only one cloud in flights
213/214, and no useful full-flight coverage. Logs report `No mapping feed for
10 seconds`. Replaying saved trajectories or accumulated PLY files cannot
substitute for replaying raw measurement data.

## Gaps found during the initial audit

Mounting clarification received after the audit: the JT16 physical centre is
0.035 m behind, 0.040 m right and 0.130 m above the Gemini left lens centre,
in aircraft body axes. JT16 leans 20 degrees forward relative to the aircraft
platform's upright reference, not relative to the camera. These initial
measurements are saved in `configs/livo-mount-initial.yaml`. They replace the
absence of a physical starting estimate, but do not establish calibrated SDK
axis rotation, optical/LiDAR origins or capture-time alignment. Confirming that
the mount was unchanged was subsequently supplied for flight 213.

1. No validated JT16-to-Gemini transform was located. The advisory mapping
   config leaves it null, and camera-to-body calibration in the other checkout
   is explicitly unverified. An assumed identity transform would not be a
   meaningful accuracy test.
2. No verified camera–LiDAR clock offset/drift mapping. Raw recording stores
   concatenated packet bytes without per-packet host arrival timestamps.
   Sparse saved decoded logs provide limited host anchors. Offline geometric
   and motion alignment may recover timing, but it must be measured and its
   uncertainty reported rather than inferred from filenames alone.
3. No FAST-LIVO2 runtime was found in the inspected dependency locations.
   Existing FAST-LIO is LiDAR/IMU only. Upstream FAST-LIVO2 uses a ROS1/catkin
   application; the installed mapper uses ROS2 and current VINS is native C++.
   A replay adapter/build is required, including JT16 point timing/ring support;
   it is not a config switch on the existing VINS executable.
4. These recordings have no established independent ground-truth trajectory.
   Cube EKF using the same VIO is correlated and cannot serve as ground truth.

## Proposed experiment

1. Decode flight 213's valid raw packets with the pinned SDK and angle table,
   preserving XYZ, rings and per-point timestamps. Build complete scans;
   reject partial scans and retain discontinuity markers.
2. Recover/check clock mapping and LiDAR–camera extrinsics against image/cloud
   overlays and independent motion/geometry constraints. Use rectified-left
   camera intrinsics and the corresponding camera-to-Gemini-gyro transform.
   Initial timing/extrinsic values must be labeled estimates, not calibration.
3. Pin/build FAST-LIVO2 in an isolated workspace-local replay environment.
   Replay one Gemini image stream, Gemini IMU and JT16 point measurements with
   network/serial flight output absent. Do not reuse gated Cube odometry as
   the measurement source.
4. Run VINS and LIVO on the same usable timestamps, initialization interval
   and saved image subset. Distinguish an offline estimator initialization
   failure from live heartbeat/publication gating. No run may fill missing
   measurements by duplicating samples.
5. Compare tracking coverage, resets, drift on known return segments, trajectory
   discontinuities, geometric map consistency and compute/RAM/latency. Use
   held-out geometry where possible; a lower residual on the estimator's own
   fitted map is not independent accuracy proof. Preserve metric scale, using
   rigid alignment only, not scale fitting that conceals scale error.
6. Repeat on 214 and the gapped 212 recordings, including reasonable timing and
   extrinsic perturbations. Report exploratory consistency separately from
   absolute localization accuracy. Ground-truth ATE/RPE needs an independent
   reference or a future measured route.

Proceed with 213 first once timing/extrinsics can be established. Keep the
current estimator as baseline until repeated evidence supports replacement.

## Reproduce the input audit

```bash
cd /home/atas/BHUslamV3
g++ -std=c++17 -O2 tools/jt16/audit_raw_crc.cpp \
  -I /home/atas/vscode/optFlow_slam/third_party/HesaiLidar_SDK_2.0/libhesai/Common/include \
  -o /tmp/bhu-jt16-audit-crc
python3 tools/audit_livo_recordings.py --crc-auditor /tmp/bhu-jt16-audit-crc
```

The CRC auditor accepted a real saved packet and rejected its bit-flipped
counterpart. Detailed per-file results are in `inventory.json`. This audits
recorded transport, not XYZ decoding or estimator performance.

Upstream reference: https://github.com/hku-mars/FAST-LIVO2
