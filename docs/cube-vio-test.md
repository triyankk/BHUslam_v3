> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> This legacy ORB-SLAM3 bench procedure is a transport and comparison reference. See the [shared project goal](../PROJECT_GOAL.md).

# Headless VIO bench test through Cube Orange+

This workflow uses native C++ for camera tracking, QR detection, MAVLink
transport, Cube buzzer control and report generation. The separate two-wire
Jetson buzzer is no longer used. Disconnect it from the Jetson header.

The detected Cube USB port is
`/dev/serial/by-id/usb-CubePilot_CubeOrange+_36003F000B51333338373339-if00`.
The default USB baud setting is 115200; the existing ToF TELEM1 UART stays at
57600. Each serial port has one owner. The test requires a disarmed Cube and
stops if it becomes armed or loses its heartbeat. A bridge failure signals the
camera process group to stop, allowing cleanup and report finalization.

## Configuration

`bin/vio-test cube-config` only reads settings and writes a local snapshot.
`bin/vio-test cube-config --apply --reboot` explicitly configures the VOXL
ODOMETRY backend, the assumed upright forward mount and 0.15 m forward camera
offset, and source set 2 for external horizontal position. It preserves source
set 1, compass yaw, barometric height, rangefinders and UART settings. It saves
`before.json`, `restore.param`, `changes.json` and `after.json` under a unique
`runs/cube-config-*` directory, reads back changes, and optionally reboots the
disarmed Cube. Reinspect after reconnecting to verify persistence.

Source set 2 is prepared but **not selected automatically**. Thus the first
integration test checks VIO transport and receiver setup while preserving the
current navigation source. A forwarded packet or an ascending tone does not
prove EKF fusion. Origin, axis signs, initialized VIO continuity and EKF
innovations need to be checked before a source-switching test. The camera's
IMU calibration and 0.15 m mounting offset remain provisional.

## Running without a display

First verify the speaker with `bin/vio-test cube-beep ready`.

```bash
bin/vio-test power-15w
bin/vio-test cube-arm
```

An explicitly labelled MAXN comparison is available with
`bin/vio-test cube-arm --power-profile MAXN_SUPER`. The launcher does not change
power modes. The ready cue occurs only after the verified Cube link, camera,
vocabulary and QR scanner have started. No ready cue means do not begin.

| Sound | Meaning / action |
| --- | --- |
| Two short notes | Ready: present the printed `test_vio` QR |
| One long note | Recording started: remove QR and begin smooth motion |
| Three ascending notes | VIO passed its publication gate; eligible poses are being sent to the bridge |
| Three descending notes | Publication gate lost; invalid VIO is withheld |
| Three short notes | Recording stopped: stand still while results finalize |
| Two longer rising notes | Trajectory report generated |
| Six rapid low notes | Startup/runtime error; inspect saved logs |

Gate transitions have a two-second cue cooldown to limit chatter. Cube-native
warnings can interrupt custom tunes. PLAY_TUNE has no audible acknowledgement;
the first two-note cue was confirmed by the user on the actual Cube.
Link loss or loss of Cube power cannot be reported by its buzzer.

QR triggering is unchanged: two detections start, remove QR for at least two
seconds, then present it again after at least five seconds to stop. The default
QR-wait and recording limits are each 600 seconds. Each invocation performs one
test. Walk a smooth loop through a textured scene and revisit its start to
exercise loop closure. Use the printed QR, since a phone display may have poor
infrared contrast. No display or network is needed while recording.

The user service is installed in `~/.config/systemd/user/`. To start a detached
test, select 15W and run `systemctl --user start bhu-cube-vio-test.service`.
The unit requires 15W and is now enabled at boot with user lingering enabled.
It waits up to 120 seconds for the exact Gemini camera and Cube USB device to
be accessible, then allows 60 seconds for the Cube's bidirectional MAVLink link.
Startup does not require a display, network or login. Failure leaves a log;
the unit does not automatically retry or restart a completed test.
The persistent Gemini
USB permission rule was installed by the user and verified against
`deploy/udev/99-bhu-gemini.rules`. For another system, install it with
`bin/vio-test install-usb-rule`; Linux requires your sudo password. It reloads
udev and reapplies only the matching camera's rule.
Startup messages are appended to `runs/cube-boot.log`. Each Cube run records
`boot-id.txt`, `boot-uptime.txt` and `service-invocation.txt` so a real post-boot
run can be distinguished from a manual restart. The 15W service startup and
clean systemd stop were verified before the battery power cycle. The subsequent
automatic boot and a 36.42-second QR-controlled run at 15W were verified from
boot IDs, service invocation and saved logs; sustained VIO remained unstable.
See `reports/cube-boot-validation.md` for measurements and limitations.

To disable automatic startup later:
`systemctl --user disable --now bhu-cube-vio-test.service`.

For the battery validation, shut down Linux normally before disconnecting
power. Power the Jetson, camera and Cube from their intended battery setup,
leave the display disconnected, and wait for the two-note ready cue. Show QR
to start, move around a textured loop, and show QR again to stop. Wait for the
saved cue, then shut down normally before removing battery power. The saved
cue confirms report generation; it does not shut Linux down automatically.

## Results

`runs/cube-vio/TIMESTAMP/` contains `bridge.console.log`, `live.console.log`
and `bridge/`. The live log identifies the corresponding `runs/vio/TIMESTAMP/`
with `report.html`, `trajectory.png`, `metrics.json`, per-frame timing and
resource measurements, raw and optimized trajectories, and loop-closure events.
The report metadata marks navigation transmission and Cube buzzer feedback.

Bridge `events.jsonl` records sent cues, forwarded odometry timestamps, Cube
status text, EKF status and local position telemetry when the Cube provides it.
`cube.tlog` records inbound MAVLink messages with big-endian 64-bit host monotonic
microsecond timestamps (not UTC). `summary.json` records link and forwarding
counts. The sender and receiver both reject stale/invalid odometry; the receiver
caps transmission at 10 Hz. It never forwards arbitrary commands from UDP.

No source-selection, mode or arming command is sent by this workflow. The
configuration tool alone can write its fixed navigation parameter list and
request a disarmed reboot. Runtime bridge requests telemetry message intervals
on its USB link and sends allowlisted buzzer tunes.
