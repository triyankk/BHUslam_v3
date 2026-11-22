# VIO workflow validation

The workflow is implemented in native C++17, with shell entry points and a
browser-rendered 3D view. Full instructions: [vio-performance.md](../docs/vio-performance.md).

## Verified

- All five CTest checks passed: IMU synchronization, external-nav protocol,
  simulated transport, native QR/report workflow and offline 3D plot behavior.
  See [vio-benchmark-tests.txt](vio-benchmark-tests.txt).
- Native QR fixtures decoded `test_vio` and its rotated image, rejected wrong
  case/extra spaces, and verified start/stop confirmation, removal and minimum
  run-time rules. The report fixtures distinguish rejected candidates from
  confirmed loop events and preserve IMU validity in optimized-map plots.
- The native resource worker recorded real process memory during its isolated
  fixture. Offline metric fixtures checked rail-energy integration and timing
  percentiles. This is not a live camera power-performance measurement.
- The default launcher rejected MAXN_SUPER when 15W was required. No power-mode
  write was performed.
- An explicitly labelled MAXN startup check reached ARMED and served the
  dashboard state API. With no QR presented, it timed out and exported an empty
  test report with `stop_reason=idle_timeout`. Its nonzero process status is
  expected because no VIO test started. Logs:
  [vio-benchmark-camera-wait.log](vio-benchmark-camera-wait.log),
  [dashboard snapshot](vio-benchmark-dashboard-smoke.json).
- A service-style SIGINT check preserved the console/report and recorded
  `stop_reason=signal`. Its timeout wrapper returned the expected status 124.
  See [vio-benchmark-interruption.log](vio-benchmark-interruption.log).
- The post-check SDK inspection confirmed serial CPC2963000JJ, firmware 1.8.10,
  and laser-control value 1 restored. The power profile remained MAXN_SUPER.
  See [camera inspection](vio-benchmark-camera-after-checks.txt).
- The shutdown/export regression replay tracked 26/26 stereo frames and saved
  its trajectory. It does not demonstrate VIO or a real loop closure.
  See [shutdown smoke log](vio-benchmark-shutdown-smoke.log).
- The optional user-service unit passed host systemd verification. It was not
  installed/enabled. The udev rule is also provided for review/manual installation.
- All upstream compatibility patches passed reverse-apply checks. The new
  shutdown wait tracks actual global-BA worker lifetimes, including detached
  workers, instead of depending only on upstream's running-state flag.

## Still requires a physical test

Present the printed QR to the actual camera, run a continuous motion loop in
15W mode, return to the starting scene and present the QR again. Confirm actual
recognition, VIO initialization/retention, any applied loop correction, sustained
processing rate and measured module power under that profile. No 15W performance
or loop-closure success is claimed by these startup tests. No Cube configuration
was changed and no navigation packets were transmitted. GPIO/buzzer remains off.

For a concrete plot preview, [the historical startup example](vio-workflow-example/report.html)
and [its native 3D PNG](vio-workflow-example/trajectory.png) reformat the earlier
extnav-live-dry-02 capture. They are labelled historical, with unlogged power;
they are not a new QR-triggered or 15W benchmark result.
