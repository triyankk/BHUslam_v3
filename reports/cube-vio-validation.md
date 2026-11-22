# Cube-connected QR/VIO test, 2026-10-24

The native C++ USB bridge reached the actual Cube Orange+ on its stable if00
USB serial path at 115200. It read back ArduCopter 4.7.1, hash dbe79216, system
1/component 1, disarmed. The user confirmed hearing the two-note ready cue.
`reports/cube-buzzer-ready-smoke/` records one tune, 71 heartbeats, zero serial
CRC errors, zero odometry and zero parameter writes.

## Configuration applied and verified

`reports/cube-vio-config-applied/` preserves the original values, restore file,
requested values, per-write readbacks and final readbacks. Five values changed:

| Parameter | Before | After |
| --- | --- | --- |
| EK3_SRC2_POSXY | 0 | 6 |
| EK3_SRC2_VELXY | 5 | 0 |
| VISO_ORIENT | 25 | 0 |
| VISO_POS_X | 0 | 0.15 m |
| VISO_TYPE | 0 | 3 |

The tool verified a recent disarmed heartbeat before writes and reboot, read
back changes and compared unrelated source-1/rangefinder/UART/buzzer parameters.
`reports/cube-vio-post-reboot.json` verified persistence, a disarmed heartbeat
and zero serial CRC errors. No source-selection or arming command was sent.
External horizontal position is prepared in source set 2; fusion is not active
or validated by this test. VISO is now an enabled receiver and can add visual
odometry health checks to Cube pre-arm reporting.

## Integrated startup and moving test

The six-second QR-idle test reached ARMED with camera and Cube connected, sent
ready/error tunes, exported an empty report, and exited 1 for no test started.
This is expected. Its Cube logs are
`runs/cube-vio/20261024T112456.177462229Z/`; camera logs are
`runs/vio/20261024T112456.312350844Z/`.

The user then authorized a 90-second maximum moving test. Both actual start
and stop were recognized from the QR; the stop occurred after 20.833742 seconds.
The launcher exited 0 and final reports were generated.

- Cube logs: `runs/cube-vio/20261024T112633.212059172Z/`.
- Trajectory/report: `runs/vio/20261024T112633.347433036Z/`.
- Power profile: MAXN_SUPER throughout; this was not a 15W test.
- 518 processed frames, 351 tracked; 24.86 processed frames/s.
- 109 IMU-initialized frames, 108 of them tracked; first initialization at 4.29 s.
- Four active-map resets; IMU uninitialized at end; zero confirmed loop closures.
- Tracking call p50 24.70 ms, p95 45.86 ms; estimated capture-age p95 80.35 ms.
- 106 pending video overwrites; these are processing backlog, not a USB-drop count.
- 21 eligible producer messages, 12 odometry messages sent through the bridge
  under its 10 Hz cap, zero receiver rejections, zero link drops or serial CRC
  errors. Sending is transport evidence, not an autopilot receipt/fusion ACK.
- Six cue transmissions: ready, start, valid, lost, stop, saved. Ready was
  acoustically confirmed by the user; other cue sends are logged.
- Median module VDD_IN input 7.03 W; not complete system/battery power.

Logs identify recent-IMU/insufficient-motion resets, including "not enough
acceleration". Cube status reported "EKF3 waiting for GPS config data" and
EKF flags 1024 (EKF_UNINITIALIZED). Pre-arm reports also included unhealthy
visual odometry. Do not interpret the displayed visual trajectory or brief
odometry stream as sustained VIO, EKF fusion or a flight-readiness result.

## Software checks and headless operation

`reports/cube-integration-tests.txt` records six passing checks: IMU sync,
external-nav protocol, simulated Cube integration, transport, QR/report workflow
and browser trajectory plot. The simulated Cube fixture runs the production
bridge over a PTY, verifies tune target and content, valid ODOMETRY forwarding,
stopping on armed heartbeat, and configuration backups/readback while preserving
source set 1. It rejects unexpected flight commands and parameter names.

`reports/cube-vio-service-verify.txt` records a valid systemd user unit. The unit
is installed as `~/.config/systemd/user/bhu-cube-vio-test.service`, not enabled
at boot. Start it after selecting 15W for a detached, display-free test. The
current power profile remains MAXN_SUPER. The user installed persistent camera
permissions with `bin/vio-test install-usb-rule`; the installed file was verified
against the prepared narrow rule and the USB device now belongs to plugdev with
group read/write access. Login-independent boot activation is not configured.

Next validation: improve camera/IMU calibration and initialization continuity,
then run a longer loop at 15W. Independently validate frame signs, mounting,
origin and Cube receiver/fusion before selecting external navigation.
