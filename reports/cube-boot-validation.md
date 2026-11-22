# Headless battery startup validation

## Verified before the physical power cycle

- Jetson power mode readback: 15W, mode 0 (changed by the user).
- `loginctl show-user atas -p Linger`: yes.
- `bhu-cube-vio-test.service`: enabled for user default.target.
- Older system `vio-flight.service`: disabled and inactive.
- Persistent Gemini USB rule installed; camera is accessible to plugdev.
- Seven checks passed, including delayed enumeration, changed USB device
  number, wrong camera serial, missing Cube and bounded wait timeout. Full
  results: `reports/cube-boot-tests.txt`.
- Unit syntax verified by systemd, including clean interrupt exit statuses.
- Actual service startup at 15W reached camera/QR ARMED with a verified disarmed
  Cube link. Run: `runs/cube-vio/20261024T114213.161561677Z/`; associated camera
  report: `runs/vio/20261024T114213.293892706Z/`.
- No QR was presented: zero processed frames, zero navigation packets, two
  cue transmissions, zero Cube link drops or serial CRC errors.
- `systemctl --user stop` finished cleanly: Result=success, ExecMainStatus=130,
  ActiveState=inactive. Evidence: `reports/cube-boot-service-stop.txt`.
- Camera and Cube processes were stopped before requesting the power cycle.

Pre-cycle boot ID: `017fd627-d49b-49f2-9b25-842c343f4a6b`, also saved in
`reports/cube-boot-before-id.txt`.

## Post-power-cycle test verified

The user reported completing the requested test. The current boot ID is
`b2b03ccc-22f3-46e1-beba-9a99d5bdd8d1`, different from the baseline. It matches
the run's boot-id.txt. The recorded service invocation
`3345265492b342108cfc7eb610022ff6` matches systemd and runs/cube-boot.log.
The service started its Cube run at uptime 13.71 s, emitted ready at 26.96 s,
and ended with Result=success / ExecMainStatus=0. These establish actual
automatic startup after reboot. Physical battery voltage/current were not
independently measured; battery use comes from the user's test context.

- Cube run: `runs/cube-vio/20261024T114522.544520968Z/`.
- Camera/trajectory: `runs/vio/20261024T114523.492116200Z/`.
- QR start at uptime 54.485 s and QR stop at 90.907 s: 36.422 s recording.
- Metadata reports 15W at both start and end; profile_changed=false.
- 876 processed frames; 471 tracked (53.8%); 24.05 processed frames/s.
- 109 IMU-initialized frames; 107 also tracked; first initialization at 4.377 s.
- 10 active-map resets; IMU uninitialized at end; zero confirmed loop closures.
- Tracking call p50/p95: 26.51/46.72 ms; estimated capture-age p95: 82.18 ms.
- 216 pending frame overwrites (processing backlog, not USB-drop evidence).
- 33 eligible producer messages; 19 forwarded by the rate-limited Cube bridge.
- Zero Cube link drops, rejected odometry or serial CRC errors.
- Eight cue transmissions: ready/start/valid/lost/valid/lost/stop/saved. Their
  transmission is logged; acoustic recognition of each cue was not separately
  confirmed by the user.
- Module VDD_IN input median 6.43 W, p95 6.97 W; not total battery consumption.
- Cube returned "VisOdom: reset" during forwarded data, evidence of backend
  handling, without proving receipt/fusion of every packet. EKF status remained
  1024 (uninitialized), with "EKF3 waiting for GPS config data" messages.

Verdict: automatic boot, persistent USB permissions, QR control, report export
and Cube transport passed this run. Sustained VIO and EKF fusion did not pass.
Logs include insufficient acceleration / recent-IMU resets; the cause still
requires calibration and motion/initialization diagnostics. Primary source
configuration and arming checks were not changed during this inspection.

At inspection the current power mode had returned to MAXN_SUPER and
`/var/lib/nvpmodel/status` contained `pmode:0002`; the cause was not established.
This does not change the recorded 15W run, but the next default boot test will
reject MAXN_SUPER. Select 15W again before the next battery power cycle.
