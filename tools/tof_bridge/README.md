> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> This single-sensor rangefinder bridge is supporting bench work; it does not define the selected final obstacle-avoidance architecture. See the [shared project goal](../../PROJECT_GOAL.md).

# Single SparkFun TMF8820 to Cube bench bridge

## Live viewer

The current front ToF path shares the existing Cube USB owner. The native
viewer opens I²C7, sends fresh distance datagrams locally, and receives genuine
Cube feedback. It opens neither GPIO UART nor Cube USB:

```bash
tools/tof_bridge/cpp/build/tof_viewer --orientation front --mux-address none --aggregation zone-mean --min-active-zones 1 --confidence 51 --cube-udp-port 14605 --cube-feedback-port 14606
```

Use the installed `bhu-tof-front.service` for normal startup; do not launch a
second acquisition process. The `optflow-flight-logger.service` remains the
sole USB owner, preserving VIO, reference checks, recording and cues. It accepts
only fresh front ToF measurements, never arbitrary commands, and never replays
old readings. JT16 obstacle publishing remains disabled. See the
[current integrated procedure](../../docs/tof-front-proximity.md).

The earlier GPIO/TELEM2 route remains unverified for sustained reception; its
independent observation included a 16-second gap. Shared USB bypasses that
fault. Historical evidence remains in
`reports/tof/gpio-stream-20261121T083619810533Z`. Do not start a separate USB
viewer while the existing USB owner is active.

The Cube's currently verified backend is PRX1_TYPE=2 (MAVLink); it accepts the
front DISTANCE_SENSOR stream directly and returns front proximity sector ID 10.
Front ToF needs no RNGFND slot; the independent rear/downward slots are preserved. No Cube parameters were
changed to enable the November front-sensor transport. Do not use the October rear
rangefinder configuration below as current Cube configuration.

The subsequently requested BendyRuler bench refinement changed only
OA_BR_LOOKAHEAD from 5 to 3 m and OA_MARGIN_MAX from 1.5 to 2 m. Both persisted
and were re-read on 21 November; AVOID_MARGIN remains 2 m. This is not an exact
2 m turn trigger or evidence of autonomous detours with one front sensor.
See `reports/tof/bendyruler-front-1795179471170190940` for the change record and
`reports/tof/bendyruler-resume-20261121T082120685814Z` for post-restart readback
and live reception verification. No additional parameter writes were needed.

The shared-USB integration supersedes the earlier GPIO/USB-alternative
arrangement. Native `bin/tof-front-boot` now publishes on local port14605 with
feedback on14606. Service ordering starts the existing USB owner first. Camera,
VINS, JT16 raw recording/mapping, arm/disarm recording and GPS-reference cues
retain their roles; no runtime mask or second USB writer is used.

Current single **front** sensor, wired directly to Jetson physical pins 1
(3.3 V), 3 (SDA), 5 (SCL), and 9 (GND), with EN unconnected on the SparkFun
board. Start the sensor-only viewer without opening or transmitting on a Cube
serial link:

```bash
bash tools/tof_bridge/cpp/build.sh
tools/tof_bridge/cpp/build/tof_viewer --sensor-only --orientation front --mux-address none --aggregation zone-mean --min-active-zones 1 --confidence 51
```

This mode needs no Cube connection, does not request or change Cube parameters,
and disables VIO forwarding. Valid readings are graphed and recorded with
`valid=true` and `transmitted=false`; MAVLink captures remain empty. It is a
sensor display, not evidence of front obstacle acceptance by the Cube.

The current front-sensor rule uses **every primary zone whose confidence is
strictly above 50**, i.e. 51–255, with distance 10–5000 mm.
The selected distance is the unweighted arithmetic average of **all** active
primary zones; secondary returns are excluded. Calibration must
be valid and the result counter fresh. If no zones qualify, no distance
packet is emitted and the graph has a gap; no previous average is replayed.
Raw readings for all nine zones continue to be recorded and displayed.

The `--aggregation zone-mean --min-active-zones 1 --confidence 51` defaults apply the same gate
to actual MAVLink output when bridging is later enabled. They do not enable
Cube forwarding by themselves. Legacy selection remains available with
`--aggregation nearest --confidence 20` for historical integration commands. Logs retain
`nearest_mm` separately from `primary_mean_mm`, `active_zone_count`, and the
gated `selected_mm`. Selected millimetres and transmitted centimetres are
truncated down; averaging itself is recorded as a floating-point value.
The previous four-zone requirement has been removed. An average can dilute a
near obstacle with farther returns; it is not evidence that the path is clear.

The following mux/rear setup and Cube settings describe the earlier bench
configuration, retained as historical integration instructions. Recheck current
Cube settings before re-enabling a bridge for the new front mounting.

```bash
bash tools/tof_bridge/cpp/build.sh
tools/tof_bridge/cpp/build/tof_viewer --mux-address 0x70 --mux-channel 0
```

The native **C++17** executable reads Linux I²C directly, loads the sensor's RAM
firmware and saved calibration, handles MAVLink over UART, records captures,
and serves the browser API. It does not launch Python. Building requires native
`g++`, OpenSSL development headers/libcrypto, and pthreads. MAVLink and JSON
headers are vendored with pinned versions in `cpp/vendor/versions.json`.
The dashboard remains HTML/CSS/JavaScript with Canvas in the browser.
The current HW-617/PCA9548A wiring places the rear TMF8820 on channel 0 of mux
`0x70`, on I²C bus 7. The command above selects only that channel before sensor
initialization and each measurement, checks mux readback, and disables all
channels when the capture stops or retries. Firmware loading and serial-bound
calibration work through the selected channel. The API and recording summary
include `mux_address` and `mux_channel`.

Use `--mux-channel 0` through `7` to select another physical channel; each sensor
still needs its own matching `--calibration` file. This viewer currently reads
one selected sensor, rather than cycling through four. Only one process may own
the mux. Channel identities planned for later are 0=rear, 1=left, 2=right, 3=top.
The channel and `--orientation` are explicit independent options; selecting a
different channel does not automatically change mounting direction.
Mux selection follows the [PCA9548A control-byte protocol](https://www.ti.com/lit/ds/symlink/pca9548a.pdf).
For the earlier direct sensor wiring, omit both mux options (or use
`--mux-address none`); mux access is disabled by default.

The previous Python launcher remains available as a fallback:
`python3 tools/tof_bridge/live_viewer.py` for direct sensor wiring; it does not
configure the mux. Run only one launcher at a time.

Open **http://127.0.0.1:8765** in a browser on the Jetson. The viewer runs the
calibrated UART bridge, detects the Cube's actual running baud, and shows the
distance graph, nine primary sensor zones, calibration, temperature, update
rate, and connection freshness. Cube distance points are its own rangefinder
or proximity-sector reports returned over UART, not an echo of the Jetson's outgoing
data. Cube USB remains available for Mission Planner. The C++ viewer uses UART
for returned telemetry. The Python fallback also offers an optional `--usb DEVICE`
observer, for use only when no other program owns that USB port.
Startup baud detection retries if the Cube is temporarily absent.
A failed sensor bridge suppresses
fresh values, reports the error, and retries with backoff. Each retry reloads
the saved serial-specific calibration and writes a new `bridge-002`, etc.
capture segment. Unavailable measurements are never replayed as fresh data.

The chart offers 15/30/60-second windows, pause/resume, and CSV export of the
buffered history. Pausing the chart does not pause capture. Stale/uncalibrated
sensor values and stale Cube reports display as unavailable. All raw bridge
measurements and Cube observations are saved under a new timestamped
`reports/tof/native-live-*` directory (`live-*` for Python). Memory histories are bounded; recording files
continue growing until stopped. No flight commands or parameter writes occur.
The viewer reads the selected rangefinder settings and warns if that slot is
disabled. Default is `--rangefinder-slot 2`. It accepts returned reports only
from Cube system 1/component 1 and the selected mounting orientation. Rear
reports use rangefinder ID 1 (slot 2), or proximity sector ID 14 when the Cube's
RangeFinder proximity backend is enabled. The viewer labels each returned trace
as rangefinder or proximity. Cube status
messages are displayed live, including any existing pre-arm warnings.

Current verified Cube configuration:

```text
RNGFND2_TYPE   = 10     # MAVLink
RNGFND2_ORIENT = 4      # Rear
RNGFND2_MIN    = 0.01   # metres
RNGFND2_MAX    = 5.0    # metres; advertised upper limit, not validated flight range
PRX1_TYPE      = 4      # RangeFinder proximity backend; enabled 28 October 2026
```

Existing downward RNGFND1 remains unchanged. Snapshots and verification are in
`reports/tof/cube-rear-rangefinder-01` through `-03` for the earlier rangefinder-only
setup, and `reports/tof/proximity-enable-20261028-01` for proximity activation.
Only PRX1_TYPE was changed for that activation, followed by a disarmed reboot.
The Cube uses one RangeFinder proximity backend to collect horizontal and upward
rangefinders. Its filtered sector distance can differ from the latest raw ToF
sample; check target response and sensor dropout before relying on avoidance.
Current avoidance settings are AVOID_ENABLE=7, AVOID_BEHAVE=1 (Stop), and
AVOID_MARGIN=2 m. This is a bench configuration, not a flight-readiness claim;
the readiness audit records unresolved estimator, RC, downward-rangefinder,
and other prearm conditions. No arming or avoidance-motion test has been done.

Closing the browser tab leaves capture running. **Ctrl-C in the launcher
terminal stops the viewer and releases its sensor/UART handles.** Do not
run another bridge against the same sensor/UART. All
sensor configuration remains volatile; this tool does not reconfigure EN pinmux.

Only loopback is served. For viewing from another computer over SSH:

```bash
ssh -L 8765:127.0.0.1:8765 atas@JETSON_IP
```

Then open the same localhost URL on that computer. There are no CDN assets,
frontend packages, or runtime Python dependencies for the C++ executable.

Optional Gemini VIO input: `--vio-udp-port 14600` enables a localhost-only,
ODOMETRY-only relay through the same UART owner. It is disabled by default.
See [the external-navigation bench guide](../../docs/cube-external-nav.md)
for frame conventions, freshness/quality gates, bandwidth and current limits.
No VIO producer or EKF parameter change is started automatically.

Validation: `cpp/build.sh` runs the native protocol self-test after compiling.
`python3 tools/tof_bridge/check_bridge.py CAPTURE_SEGMENT` independently checks
the recorded binary MAVLink against the sensor readings.
`node tools/tof_bridge/test_viewer_ui.js API_SNAPSHOT_JSON` executes the actual UI
against a recorded API snapshot, including stale and invalid measurements.
Use `--duration 30` for a bounded capture, `--baud 57600` to skip detection, or
`--help` for the other native options. No Cube parameter writes occur.

Latest validated wiring uses Cube **TELEM1**, with Jetson pin 8 TX to Cube pin
3 RX, Jetson pin 10 RX to Cube pin 2 TX, and common ground. Live Jetson UART
requires the existing **cube-uart-pio** boot entry for the validated setup.
Use **57600 baud** for the Cube's current running configuration. After the
rangefinder configuration reboot, both runtime baud and stored SERIAL1_BAUD=57
were verified. The viewer still auto-detects baud on startup/reconnection.
The earlier [PIO link results](../../reports/tof/telem1-pio-01/findings.md)
predate the current rangefinder-only configuration.
The TELEM2/115200 examples below describe the original intended setup, not the
currently connected port.

This tool reads the nine TMF8820 zones over I²C and sends the selected distance
as MAVLink 2 `DISTANCE_SENSOR`. The current default is the mean of primary
zones with confidence above 50; legacy nearest-primary/secondary selection is
available explicitly with `--aggregation nearest`.
It sends no arming, flight-control, parameter-write, or mode-change commands.
This first bench implementation uses one mounting direction and does not
preserve the nine zones as separate angular sectors.

## Hardware and local dependencies

Existing sensor wiring, physical Jetson header pins:

| Jetson | SparkFun sensor |
| --- | --- |
| 1, 3.3 V | 3.3 V |
| 3, SDA | SDA |
| 5, SCL | SCL |
| 6, ground | GND |

EN was subsequently connected to physical pin 7 and validated using the
temporary pinmux helper. The viewer does not toggle EN; the current single
sensor uses its default address 0x41.
Live device was identified on `/dev/i2c-7`, address `0x41`, serial **23937420**.
The first early probe received an ACK on bus 1, but later acquisition succeeded
on bus 7. Use bus 7 for this configuration and investigate intermittent errors
before relying on it. No scan of unrelated slave addresses is needed.

`/dev/ttyTHS1` was available and opened at 115200 8N1 without hardware flow
control. The header UART uses physical pin 8 TX and pin 10 RX on this developer
kit. Opening a UART or successfully writing bytes does not prove electrical
delivery at its pins; a loopback/receiver is required for that check.

Vendored dependencies (unpacked wheels, licenses retained):

| Package | Version | Wheel SHA256 |
| --- | --- | --- |
| tmf882x-driver | 0.2.0 | `83924421a6157fdc8a5bfe337e44d0ca85227e65696c039b3941c4378881bcee` |
| smbus2 | 0.5.0 | `1a15c3b9fa69357beb038cc0b5d37939702f8bfde1ddc89ca9f17d8461dbe949` |

The existing Python environment supplies pyserial 3.5 and pymavlink 2.4.49.
The vendored driver supplies a 2636-byte RAM firmware image. Sensor firmware
is loaded into volatile RAM when necessary; system packages and Jetson pinmux
are not changed. Calibration is stored in the workspace and reloaded on start.

## Calibration and sensor capture

From `/home/atas/BHUslamV3`, with the optical face uncovered, no object within
40 cm, and low ambient light, create calibration once:

```bash
python3 tools/tof_bridge/bridge.py \
  --calibrate tools/tof_bridge/calibration-23937420-map1.json \
  --duration 5 --output reports/tof/calibration-01
```

Then put a target at a known distance and capture readings:

```bash
python3 tools/tof_bridge/bridge.py \
  --calibration tools/tof_bridge/calibration-23937420-map1.json \
  --duration 30 --output reports/tof/sensor-01
python3 tools/tof_bridge/check_bridge.py reports/tof/sensor-01
```

Output directories and calibration files are never overwritten. Choose a new
directory for each run. The script exclusively locks this tool's sensor access;
unrelated I²C programs must also be stopped. Each sensor command and wait for a
fresh result has a deadline. Failures stop the process; measurements are not
replayed. No distance is emitted for invalid calibration, duplicate frames,
or no confident return within 10–5000 mm. An absent return means unknown,
not guaranteed clear space. Reject counts are available in `summary.json`.

Calibration is bound to sensor serial and SPAD map, with a SHA256 integrity
check. Recalibrate after changing the sensor optics/housing or SPAD layout.

## UART output

After calibrated sensor checks pass, add:

```bash
python3 tools/tof_bridge/bridge.py \
  --calibration tools/tof_bridge/calibration-23937420-map1.json \
  --uart /dev/ttyTHS1 --baud 115200 --orientation rear \
  --duration 30 --output reports/tof/uart-01
python3 tools/tof_bridge/check_bridge.py reports/tof/uart-01
```

UART opens only after a calibrated valid return. It is exclusively opened.
Distances use source system 1/component 191, sensor ID 0, minimum 1 cm,
maximum 500 cm, rear orientation 4, and unknown covariance/quality.
The system ID must match the actual Cube system ID for the intended onboard
component arrangement; `--system-id` changes it. Millimetres are floored to
centimetres. A heartbeat is sent approximately once per second. Received Cube
heartbeats are printed; no Cube parameters are changed automatically.

For the user-assisted Cube test, power down before wiring, then connect:

| Jetson | Cube carrier TELEM2 |
| --- | --- |
| Pin 8 TX | RX |
| Pin 10 RX | TX |
| Ground | GND |

Verify the carrier's connector labels/pinout. Leave telemetry 5 V disconnected
and retain independent power. Start with props removed. With the conventional
TELEM2/SERIAL2 mapping, the intended ArduCopter configuration is:

```text
SERIAL2_PROTOCOL = 2    # MAVLink 2
SERIAL2_BAUD     = 115  # 115200 baud
PRX1_TYPE       = 2    # MAVLink proximity
```

Record firmware version and existing parameter values before changes. Confirm
the port is free and its flow-control configuration matches the three-wire
connection. Reboot as required, inspect `DISTANCE_SENSOR` in Mission Planner's
MAVLink Inspector and verify the rear measurement in Proximity view. Do not
enable flight avoidance based on this bench test alone.

## Verification

```bash
python3 tools/tof_bridge/check_bridge.py
```

Without an argument this tests filtering, primary/secondary return selection,
unit conversion, all five mounting directions, and MAVLink CRC decoding over a
software serial PTY. It does not exercise physical UART pins. With a capture
directory it decodes the raw binary packets and checks their distance,
orientation, timestamp, and counts against the real measurement log.

Sources: [SparkFun board](https://www.sparkfun.com/sparkfun-qwiic-dtof-imager-tmf8820.html),
[sensor datasheet](https://look.ams-osram.com/m/52236c476132a095/original/TMF8820-21-28-Multizone-Time-of-Flight-Sensor.pdf),
[Python driver source](https://github.com/rogiervandergeer/tmf882x-driver),
[ArduPilot proximity backend](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Proximity/AP_Proximity_MAV.cpp).

21November rear-rangefinder update: Cube RNGFND2 is now the H-Flow downward CAN
rangefinder, not the front TMF8820. The front ToF still uses direct MAVLink PRX1.
Separate PRX2 consumes the rear TF03. The viewer's legacy RNGFND-slot parameter
readout must not be confused with ToF configuration. Values beyond ToF's5m range
(including the rear-derived180m unavailable-front sentinel) are excluded from
accepted front reception and counted as unavailable_cube_reports; raw RX remains.
See ../../docs/hflow-down-rear-rangefinder.md.
