# Cube external-navigation configuration — 28 October 2026

Live USB inspection confirmed the Cube Orange+ is disarmed, running ArduCopter
4.7.1 (`dbe79216`), with EKF3 enabled. The native configuration utility checked
its fixed receiver/source-set-2 parameter list using `--apply`: all values
already matched, so it made **zero writes**, requested no reboot and did not
select a source set. Before/after snapshots and a restore file are in
`cube-extnav-config-20261028/`. The broader inspection is
`cube-extnav-inspection-20261028.json`.

| Setting | Verified value |
| --- | --- |
| VISO_TYPE | 3 — VOXL-compatible MAVLink odometry receiver |
| VISO_ORIENT | 0 — publisher performs optical-to-body rotation |
| VISO_POS_X/Y/Z | 0.15 / 0 / 0 m — provisional camera location |
| VISO_QUAL_MIN | 60 |
| EK3_SRC2_POSXY | 6 — ExternalNav |
| EK3_SRC2_VELXY / VELZ | 0 / 0 — velocity fusion disabled |
| EK3_SRC2_POSZ | 1 — barometer |
| EK3_SRC2_YAW | 1 — compass |

Primary source parameters, rangefinders, proximity and TELEM1 configuration
were preserved. Source-set selection was not queried by these tools; no source
switch command was sent. Configured source set 2 does not itself prove it is
active or that the EKF is fusing VIO.

A six-second feedback-only USB check received 18 heartbeats with zero link
drops or malformed packets and forwarded zero poses. The Cube reported
`AHRS: DCM active` and `EKF3 waiting for GPS config data`; EKF navigation was
therefore not demonstrated. Evidence: `cube-extnav-link-20261028/`.

## Remaining integration work

The existing native live producer uses ORB-SLAM3. VINS-Fusion currently has an
offline recording adapter and viewer, **not a live camera-to-Cube publisher**.
Recorded poses must not be retimestamped and sent as live navigation data.

For VINS-Fusion, implement live stereo/IMU ingestion and publish eligible poses
through the existing bounded C++ ODOMETRY relay. Convert the gravity-aligned
VINS world and optical attitude into the documented LOCAL_FRD/body-FRD frames;
retain camera-origin position so the Cube applies the 0.15 m offset once.
Measure live pose age and output rate at 15 W, and gate weak tracking, resets
and stale data independently of upstream's disabled failure detector.

Then verify direction signs, receiver health, EKF origin and innovations in a
disarmed moving bench test before selecting source set 2. A GNSS-denied setup
needs a known EKF origin if GPS has not established one; none was invented.
This configuration retains barometer height and compass yaw and does not claim
full VIO-only navigation or flight validation.

Reference: [ArduPilot external-navigation interface](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html)
and [EKF source selection](https://ardupilot.org/copter/docs/common-ekf-sources.html).
