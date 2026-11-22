# Front ToF: confidence above 50 and Cube reception

The active rule averages every primary zone with distance 10–5000 mm and raw
confidence strictly above 50 (51–255). One qualifying zone is enough; the earlier
four-zone gate is removed. Secondary returns do not enter the mean. Calibration
and freshness checks remain. No eligible zones means no distance packet, and
there is no replay of the previous average. Raw zone readings remain recorded.

Native tests cover the four-zone legacy option, averaging all qualifying zones,
secondary exclusion, confidence 50 versus 51/255, and no eligible zones. A
software-UART capture independently verified actual outgoing MAVLink bytes.

## Hardware handoff and validation

The initial free-UART probe received heartbeats at 115200 but no requested
parameter replies; the second Cube USB interface was silent. The existing
optFlow/JT16 service owned the working primary Cube USB link. The user explicitly
requested leaving JT16 stopped and using ToF instead.

Stopping the service alone did not keep it stopped: it restarted during the
first USB attempt. That first `cube-live-check` run is not reception evidence.
A standard runtime mask was lower priority than the existing user unit. An
effective current-boot mask was therefore placed at
`/run/user/1000/systemd/user.control/optflow-flight-logger.service`, targeting
`/dev/null`, and systemd reloaded. LoadState=masked was verified. This does not
convert startup policy to a permanent ToF service; the runtime masks clear at
reboot. The JT16 service was not restored.

The subsequent `cube-exclusive-check` run had one Cube USB owner and produced:

- 375 calibrated samples and outgoing distance packets at 15 Hz.
- Six qualifying zones in 51 samples and seven in 324 samples.
- 103 Cube front proximity reports, source system 1/component 1, sensor ID 10,
  orientation 0. Every report matched a recent selected average within 1 cm.
- Zero malformed UART/USB packets. Independent pymavlink decoding checked every
  selected mean, confidence gate, distance packet, timestamp, and CRC.

Readback confirmed PRX1_TYPE=2 (MAVLink), so front DISTANCE_SENSOR input needed
no Cube parameter change. RNGFND2_TYPE=0; the older rear rangefinder setup is no
longer the current input path. Outgoing-command audit found only distances,
heartbeats, read-only parameter requests, and message-request command 512.

The final viewer is forwarding the same front mean over primary Cube USB. Its
timestamped output directory retains an API snapshot and executable hash. The
camera/VINS acquisition and VIO observer services remain active. Stopping the
shared JT16 Cube-link owner also paused its profile-guarded VIO-to-Cube relay;
the ToF bridge does not silently replace that navigation relay or change EKF
sources. No flight avoidance or estimator-fusion claim is made here.

## Confidence interpretation

The TMF8820 confidence field is an unsigned 8-bit SNR-based code (0–255), not a
percentage or calibrated probability of correctness. The datasheet describes
linear encoding clipped at 255 and optional logarithmic encoding. The threshold
here applies to the raw code as requested.

[Manufacturer datasheet, confidence section](https://look.ams-osram.com/m/52236c476132a095/original/TMF882X-DS000693.pdf)
