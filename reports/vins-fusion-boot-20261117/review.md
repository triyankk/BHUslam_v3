# VINS-Fusion boot setup — 2026-11-17

Implemented the user's narrowed request: configure startup now without attached hardware, but wait for both Gemini and a ready Cube before opening the camera.

- Installed/enabled `bhu-vins-fusion.service`; readback active/running while waiting for missing devices. No estimator process was running during this check.
- User lingering already enabled (`Linger=yes`), so login is not required.
- Disabled competing BHU recorder/legacy ORB user services. Prior unit files and enabled states saved in `runs/vins-fusion-boot/install-20261117T103844.506532494Z/`.
- Existing `optflow-flight-logger.service` remained active/enabled, same main PID 19418. No source/configuration changes in that checkout.
- Exact Gemini/Cube access is required, followed by a newly updated status file from the existing Cube owner, connected link, fresh <=1500 ms heartbeat and disarmed startup state. Link readiness does not imply EKF readiness.
- Continuous native C++ VINS-Fusion has no fixed timeout or QR trigger. Fixed-memory recent timing statistics, periodic CSV flush and atomic status snapshots support extended operation. Existing mild CLAHE, four-thread and 0.5 s / 40 feature gate preserved.
- No UDP navigation output or Cube bridge enabled in this service. No Cube parameters, source selection, origin, mission or Jetson power mode changes. Host remains MAXN_SUPER from inspection.

Validation: serial ARM64 build succeeded; five Fusion CTest checks passed (bounded diagnostic storage, Cube readiness, native IMU integration, stereo geometry and gate safety). Existing USB wait fixture passed delayed enumeration, changed device number, wrong camera and missing Cube checks. CLI rejects conflicting `--continuous --seconds` settings. systemd unit verification succeeded on the host, and enabled/active states were read back after installation.

No Gemini or Cube was attached, so camera startup/continuous tracking and full reboot-through-initialization were not tested. Saved EKF fusion remains unverified. Single-owner combined proximity/navigation transport and flight validation remain outstanding. This setup is not evidence of flight readiness.

See [operation and limits](../../docs/vins-fusion-boot.md). Service status/process snapshots and test/build logs are saved beside this report. `journalctl --user` returned no journal files in the final evidence collection environment; initial `systemctl status` did show waiting-for-devices startup messages.
