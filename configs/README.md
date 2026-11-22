> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> These configuration conventions support the localization and sensor integration needed for that goal. See the [shared project goal](../PROJECT_GOAL.md).

# Gemini 336L configurations

`gemini_record` generates `orb.yaml` and `factory.yaml` in each recording.
There is deliberately no runnable file with invented intrinsics or identity
camera–IMU extrinsics. Profiles are requested explicitly; unavailable modes fail
instead of silently changing resolution, frame rate, format or IMU range.

Baseline request: 640×400 Y8 left/right IR at 30 FPS, accelerometer ±4 g and
gyro ±1000 degrees/s at 200 Hz. Projector off through
`OB_PROP_LASER_CONTROL_INT`; native IR calibration is read from the selected
profiles. Software rectification uses those matched
parameters and a common principal point. If a rectification toggle is supported,
it is disabled and restored; the tested firmware does not expose that toggle.
Changed settings are restored on normal exit, SIGINT, SIGTERM and exception
unwinding. A process kill or power loss cannot execute restoration. Exposure is
left at its existing device setting until image quality
can be inspected; no unverified exposure units are written.

Factory stereo translation is converted from millimetres to metres. The gyro
frame defines the body frame. Acceleration is rotated into that frame. The
camera-to-body transform includes the inverse left rectification rotation.
The public SDK frame-value contract supplies m/s² and rad/s; the CSV writer
preserves these units. This was checked against the bundled SDK IMU README and
a stationary gravity measurement. Internal decoder units must not be used to
infer an extra conversion at the public API boundary. No gravity subtraction
is applied. All timestamps use `getTimeStampUs()` in the device clock domain,
converted to seconds; host arrival times are not used for integration.

IMU noise density and random walk fields start with explicitly provisional
upstream example values. `Calibration.imuValidated: 0` prevents inertial replay
unless `--allow-provisional` is deliberately specified. Replace them after
stationary noise/Allan analysis and verify camera–IMU time offset, axis signs,
and extrinsics with a moving calibration sequence before setting the flag to 1.
Do not directly copy SDK noise fields without checking definitions and units.

Stereo skew above 500 µs is rejected during capture. IMU replay rejects duplicate
or decreasing timestamps, nonfinite data and sample gaps above 20 ms. It uses
linear acceleration interpolation at gyro sample times and requires image times
to lie within IMU coverage. These thresholds are diagnostics to tune from actual
measurements, not statements about manufacturer synchronization accuracy.

Replay accepts the adapter-specific `IMU.TimeOffsetSeconds` settings field:
`camera-clock time = IMU timestamp + offset`. It defaults to zero and shifts both
accelerometer and gyro timestamps before interpolation. ORB-SLAM3 itself does
not interpret this field. Zero is a starting assumption, not a measured offset.
