# Optical Flow Flight Stack

Cube Orange+ optical-flow flight, JT16 proximity, low-speed native obstacle
avoidance, synchronized sensor recording, and Jetson ground tools for a heavy
quadrotor.

The active architecture is deliberately simple: the Cube owns stabilization,
estimation, modes, missions, landing, and motors. The Jetson sends JT16
proximity and records evidence. It does not send movement, mode, arming,
landing, attitude, or motor commands.

## Read This First

This is flight-critical experimental software. A healthy Jetson service is not
permission to arm.

**2026-09-27 incident:** the installed 4.7.1 firmware omits horizontal
FlowHold/AltHold avoidance. RC7 high and healthy PRX do not restore it.
Obstacle-approach testing is suspended; read the
[investigation and unapplied repair candidate](data/diagnostics/2026-09-27-tree-contact/README.md).

Current runtime restored on **2026-09-26**:

- `optflow-flight-logger.service` is enabled for boot and owns JT16/Cube
  proximity. `optflow-vio-flight-shadow.service` is stopped and disabled.
- Retain the working Cube USB by-id connection in `config/system.yaml`.
  No Cube parameters were changed during this service restoration.
- JT16 ran near 5 Hz, with about 9,600 points per scan. Twenty consecutive
  one-second samples had fresh data, stable Cube proximity health and zero
  reconnects. RC7 was high at 1945 us during the disarmed check.
- VIO/LIO odometry output, ExternalNav fusion and companion movement are off.
  Arm-triggered sensor recording remains enabled.
- Cube still reported RC/GCS failsafe, altitude/EKF/home and `VisOdom: not
  healthy` pre-arm faults. Stopping VIO does not change its Cube backend
  configuration. The restored proximity stream is **not flight clearance**.

Historical bench state on **2026-09-19**, not the current service state:

- Cube upgraded to official ArduCopter 4.7.1 and disarmed.
- JT16 decoding near 5 Hz.
- JT16 and Cube links remain connected with zero service reconnects, but Cube
  proximity health repeatedly alternates between good and `PRX1: No Data`
  despite continuous packets. This is a no-flight state.
- Cube also reports bad EKF attitude, about 99 degrees of yaw inconsistency,
  and a missing altitude estimate on the table. All must clear before flight.
- Copter 4.7.1 allocates the MAVLink VisualOdom backend successfully. A live
  fused D415/IM10A/JT16 shadow run produced 13.13 Hz with nonzero covariance,
  but ExternalNav fusion remains off and true map relocalization is absent.
- After changing TELEM2 to 115200 for a large-frame test, Cube-to-Jetson
  heartbeats continued but Jetson-to-Cube requests stopped and USB failed to
  re-enumerate after reboot. The logger is stopped; physically power-cycle the
  Cube before further bench work. Do not fly in this recovery state.
- The strict stationary H-Flow gate remains open because apparent stationary
  speed p95 was 0.144 m/s against a 0.10 m/s target.
- GPS-disabled `AUTO`, BendyRuler rerouting, and `RTL` have not completed the
  staged field proof in this document.

Never use `STABILIZE` as a fallback on this aircraft. Never use
`GUIDED_NOGPS`. Keep GPS physically connected during initial commissioning.
Never bypass RC, GCS, PRX, EKF, compass, battery, range, or flow warnings just
to arm.

## Current Hardware

Body coordinates use FRD: X forward, Y right, Z down. Negative Z is above the
center of gravity.

| Component | Connection | Position from CG | Active role |
| --- | --- | --- | --- |
| Cube Orange+ | Airframe | `(+0.08, 0.00, -0.08) m`, Yaw270 | EKF3, modes, mission, motors, sole yaw compass after calibration |
| CubePilot Here4 | Cube DroneCAN | External mast | GNSS setup reference only; its DroneCAN compasses are disabled |
| Holybro H-Flow | Cube CAN2 | `(0.00, 0.00, +0.10) m`, downward | Horizontal velocity and downward range |
| Hesai JT16 | Jetson USB-RS485 | `(0.00, 0.00, -0.10) m`, calibrated yaw 180 deg | 360 degree proximity |
| Intel RealSense D415 | Jetson USB | `(+0.19, 0.00, +0.10) m`, forward | RGB, depth range, annotated video |
| Hiwonder IM10A | Jetson USB serial | `(+0.08, 0.00, -0.09) m` | External acceleration, gyro, attitude |
| Four 40 A ESCs | Cube outputs | One per motor | Individual motor drive |
| 6S 22 Ah battery | Power module | Suspended below CG | Main power monitoring |

The opposite motor centers are 0.85 m apart diagonally and the aircraft uses
18 inch propellers. Software protects a conservative 0.75 m airframe radius.
The low-hanging battery makes restrained attitude and acceleration limits
important, but it does not excuse oscillation, vibration, or hot propulsion.

## Active Data Path

```text
H-Flow + range + Cube IMUs/barometer/compass
                       |
                       v
                 Cube EKF3
                       |
               modes and mission
                       |
                 ESCs and motors

JT16 -> Jetson sector extraction -> MAVLink proximity -> Cube avoidance
  |
  +-> Desktop CSV recording

D415 + IM10A + Cube telemetry -> Desktop CSV and annotated MP4
```

The boot service is `optflow-flight-logger.service`. The name is historical;
its permanent flight function is JT16-to-Cube obstacle proximity. Sensor
recording is a tap inside the same process, so it does not open a competing
Cube UART or stop proximity output.

## Desktop Ground Tools

Directly clickable launchers are installed on the Jetson Desktop:

| Desktop icon | Function |
| --- | --- |
| **Drone Ground Tools** | Recording settings, Cube logs, parameter export, Mission Planner, sensor viewers, and VIO Calibration |
| **ArduPilot BIN to CSV** | Select a `.BIN`, choose a destination, and convert it to readable CSV |
| **Depth Camera Viewer** | Open the D415 in the selected RGB/depth layout |
| **JT16 Point Cloud + IMU** | Independent LAN viewer for the JT16 cloud and its built-in IMU; no Cube or IM10A connection |

The same applications can be started from a terminal:

```bash
./optflow ground-tools
./optflow bin-csv
./optflow depth-viewer
./optflow im10a-viewer
./optflow jt16-viewer --open
./optflow vio-calibrate
```

**JT16-only viewer:** open `http://<Jetson-IP>:8776` on another device on the
same trusted network. The desktop launcher runs
`bash scripts/launch_jt16_viewer.sh`; it keeps the viewer available after closing
the browser, but does not enable boot autostart. Stop it with
`systemctl --user stop optflow-jt16-viewer.service`.
It shares the existing lidar feed when available, or owns only JT16 when free.
It never pauses flight services or changes RC/flight parameters. Stop a
direct-mode viewer before starting flight services. See
[JT16 viewer setup, units and limitations](docs/JT16_VIEWER.md).

**VIO calibration:** Ground Tools > Tools > **VIO Calibration**, or the command
above. Open `http://<Jetson-IP>:8773` from another LAN device. The console stays
idle until a props-off, disarmed capture is requested. It provides a printable
target, live D415/IM10A checks, beep/flash-guided capture, camera fitting and
Kalibr export/import. Camera-IMU solving requires a Kalibr environment; captured
data and imported candidates do not enable flight. See the complete
[calibration procedure](docs/VIO_CALIBRATION.md).

Capture finished but analysis interrupted: select it under **Saved capture**,
then **Analyze saved capture**. No sensor reconnection is needed. Results are
saved as `analysis.md`, `analysis.json`, `camera-fit.json` and
`camera-residuals.csv` inside that capture's folder. CLI equivalent:

```bash
./optflow vio-calibrate --analyze data/calibrations/vio/<UTC-timestamp>
```

Completed offline analysis does not mean camera-IMU calibration or flight
approval; the console keeps quality warnings and pending Kalibr work visible.

No printer: choose **Screen target** to open a separate fullscreen-capable
AprilGrid window. Select **14-36 inches** (default **22 inches**), then confirm
one displayed tag's measured width/height. Its physical size is transferred
automatically to the capture and Kalibr export. Resizing/hiding the target
invalidates its scale; keep the screen fixed throughout capture.

### Recording Tab

Recordings default to:

```text
~/Desktop/Drone_Sensor_Recordings/
```

Choose the output root, one video layout, whether to create an annotated MP4,
and whether arm detection should start recording automatically:

- **RGB**: normal color video.
- **Depth**: colorized range video.
- **RGB + Depth**: side-by-side color and range video.

With **Record automatically while armed** enabled, the first armed Cube
heartbeat starts one session. The session remains open for three seconds after
disarm so final telemetry is retained, then it is finalized. A new folder is
created for every later arm/disarm cycle. Repeated armed heartbeats do not make
duplicate folders.

Before every field session, run:

```bash
./optflow obstacle-status
```

Require `READY=true`, `RECORDING_THREAD_ALIVE=true`,
`AUTO_RECORD_ON_ARM=true`, and `RECORDING_LAST_ERROR=None`. A dead recorder now
restarts the complete service instead of leaving OA running without evidence.
An explicit `systemctl --user stop optflow-flight-logger.service` remains
stopped until it is manually started or the Jetson reboots; `is-enabled` alone
does not prove that it is currently running.

For the lowest takeoff-time load, use **Start Recording** while disarmed and
allow ten seconds for the D415 and serial workers to settle. With automatic
recording, arm and hold on the ground for those ten seconds instead. In both
cases, require `PRX_STABLE=True`, fresh JT16 frames, and zero reconnects
before takeoff. Close the D415 viewer before recording so it does not own the
camera.

Each session folder uses this exact format:

```text
dd-mm-yyyy_HH-MM-SS_index-number
13-08-2026_14-05-22_001
```

The suffix is one root-wide sequence: it increases for every session even when
the timestamp changes. Every row contains local time, UTC time, and host
monotonic time.

```text
<session>/
  manifest.json
  sensor_data.csv
  cube_mavlink.csv
  im10a.csv
  depth_camera.csv
  jt16.csv
  depthcam_rgb.mp4             # selected mode only
  depthcam_depth.mp4
  depthcam_combined.mp4
```

| File | Human-readable contents |
| --- | --- |
| `sensor_data.csv` | Unified rows from every active sensor stream, names, SI units, axes, quality, ranges, and details |
| `cube_mavlink.csv` | Every received Cube MAVLink message with source IDs and all fields |
| `im10a.csv` | Raw and corrected body-axis acceleration, gyro, attitude/quaternion, sensor time, checksum counters |
| `depth_camera.csv` | Every D415 frame's center range, center-region median, p05 nearest range, image median/max, valid percentage, and 6 x 8 range grid |
| `jt16.csv` | Every decoded scan's point count, nearest range, valid sectors, and complete 72-sector distance array |
| `depthcam_*.mp4` | Selected view in H.264 MP4 at a time-paced 20 fps, with frame number, timestamp, center range, nearest p05 range, and valid depth percentage overlaid |

The MP4 index is updated once per second while recording. A clean stop still
finalizes the file normally, while an interrupted Jetson process preserves the
already committed video instead of leaving an MP4 with no playable index.

The CSVs preserve every decoded Cube and IM10A measurement plus every D415 and
JT16 frame summary. Full raw lidar points and all 307,200 depth pixels per
frame are intentionally not expanded into CSV because that would be neither
responsive nor human-readable.

Command-line recording control is also available:

```bash
./optflow record start --output-root "$HOME/Desktop" --video-mode combined
./optflow record status
./optflow record stop
```

Only one process may own the D415 or IM10A. Stop recording before launching an
individual D415 or IM10A viewer.

### Cube Logs Tab

This pane defaults to the latest 10 DataFlash logs. Set another count, press
**Scan Cube**, select one or more rows with Ctrl/Shift, choose the destination,
and press **Download Selected**.

Safety and ownership behavior:

1. The current service status must contain a fresh disarmed heartbeat.
2. The pane pauses the boot service and acquires the Cube UART lock.
3. It connects directly to TELEM2 and verifies disarmed again.
4. Each log is streamed with out-of-order packet handling, gap repair, and an
   exact byte-count check.
5. The boot service restarts automatically on success or failure.

Do not scan or download while armed. A real transfer test downloaded log 165
as exactly 529,231 bytes, parsed 12,773 valid messages, and restored healthy
proximity with zero reconnects.

Terminal list command:

```bash
./optflow cube-logs --count 10
```

### Tools Tab

The window launches:

- Mission Planner through Mono.
- The BIN-to-CSV converter.
- The full 3D JT16/drone browser.
- The D415 RGB-depth viewer.
- The IM10A live body-axis plot.
- Current Cube parameter export.

The managed 3D viewer pauses the hardware service only while it is open and
restores it when the viewer closes. Confirm the service is healthy afterward.

## Current Parameter Export

The requested live export is on the Desktop:

```text
Cube_Current_Parameters_13-08-2026_14-24-54.param
Cube_Parameter_Explainer_13-08-2026_14-24-54.csv
```

The `.param` file contains all 1,161 values downloaded from the Cube over
TELEM2 for exact backup. The focused reference CSV contains 281 rows: 228
meaningful values changed from firmware defaults and 53 default-valued settings
that still define this aircraft's flight, safety, sensor, RC, or interface
contract. Volatile runtime counters and boot-time pressure references are
excluded. Each row contains:

- A plain-language reason the parameter is included.
- Current value and decoded meaning.
- Firmware default and changed-from-default flag.
- Group and critical-interface-group flag.
- Display name and detailed purpose.
- Units, valid range, increment, enum values, and bitmask.
- Reboot requirement and user level.
- Exact default and metadata provenance.

The full snapshot identifies 233 values different from their firmware
defaults; five are volatile counters or boot-time pressure references and are
intentionally omitted from the explainer. Defaults primarily come from a real
DataFlash `PARM` set from this aircraft. The OA subgroup values missing from
that older log use the matching ArduCopter 4.6.3 source constants. Descriptions
combine official ArduPilot metadata with Mission Planner's local metadata
cache.

Create a fresh pair at any time while disarmed:

```bash
./optflow cube-params --destination "$HOME/Desktop"
```

The complete parameter stream normally takes 30 to 60 seconds on this Cube.
The OA service pauses only after a fresh disarmed heartbeat and restarts
automatically when the export finishes or fails.

Never load the complete `.param` file blindly onto another aircraft.

## Cube Configuration

Important values verified in the current live export:

| Area | Current values |
| --- | --- |
| Firmware/estimator | ArduCopter 4.7.1, `AHRS_EKF_TYPE=3`, `EK3_PRIMARY=0` |
| Cube mount | `AHRS_ORIENTATION=6` (`Yaw270`) |
| Attitude limit | `ANGLE_MAX=3000` (30 deg) |
| TELEM2 | `SERIAL2_PROTOCOL=2`, `SERIAL2_BAUD=115` (115200 baud) |
| H-Flow | `FLOW_TYPE=6`, `FLOW_POS_Z=0.10` |
| Downward range | `RNGFND1_TYPE=24`, `RNGFND1_POS_Z=0.10` |
| EKF source 1 | POSXY GPS/RTK, VELXY optical flow, POSZ barometer, yaw compass |
| Proximity | `PRX1_TYPE=2` (MAVLink) |
| Reactive OA | `AVOID_ENABLE=7`, 2.00 m stopping/reaction distance, 1.50 m hard boundary/altitude gate |
| Path planner | `OA_TYPE=1`, `OA_BR_LOOKAHEAD=4.00`, `OA_MARGIN_MAX=1.50` |
| OA database | `OA_DB_SIZE=100`, `OA_DB_EXPIRE=5`, `OA_DB_QUEUE_SIZE=80` |
| Autonomous speed | `WPNAV_SPEED/UP/DN=50`, `RTL_SPEED=50`, `LAND_SPEED/HIGH=50` cm/s |
| Terrain | `TERRAIN_ENABLE=0` |
| Battery | `BATT_MONITOR=4`, `BATT_CAPACITY=22000`, arm 23.1 V, low 21.0 V, critical 20.4 V |

A mission `DO_CHANGE_SPEED` item can override `WPNAV_SPEED`. Omit it or keep
it at or below 0.50 m/s.

Battery voltage uses raw pack voltage with a 10 s persistence timer. Below
21.0 V, action 4 attempts SmartRTL and falls back to Land if SmartRTL is not
available; below 20.4 V, action 1 commands Land. The 23.1 V arming threshold is
unchanged. Capacity failsafes remain disabled, so validate voltage/current
scaling against the wattmeter rather than trusting percentage alone.

### Optional RTK Corrections

The Holybro USB RTK connected to the MK15 is the stationary base. The Here4 on
the aircraft remains the rover: `GPS1_TYPE=9`, `GPS1_CAN_NODEID=125`,
`GPS2_TYPE=0`, and `GPS_INJECT_TO=127`. QGroundControl must auto-connect the
base and complete survey-in, or use an accurately known base position, before
it forwards RTCM corrections over the existing MAVLink telemetry link. A
blinking base fix light alone does not prove that the aircraft has RTK.

Confirm `RTK Float`, then `RTK Fixed`, on the aircraft GPS indicator before
claiming corrected navigation. Source set 1 fuses GPS/RTK horizontal position
and H-Flow horizontal velocity. RC9 high still disables GPS measurements; the
flow velocity path remains available for the existing short GPS-disabled test.
Loss of RTCM alone degrades the Here4 to its normal GNSS solution and does not
disable H-Flow. No Lua or automatic EKF source-switching script is enabled.

Keep the base immobile after survey-in and repeat survey-in whenever it moves.
See the [ArduPilot RTK correction guide](https://ardupilot.org/copter/docs/common-rtk-correction.html)
and [QGroundControl RTK setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/general.html).

## MK15 Controls

![MK15 stock channel reference](docs/assets/mk15-control-map.svg)

The SIYI MK15 stock map assigns RC5/6/7 to three-stage switches SA/SB/SC and
RC8/9/10/11 to buttons A/B/C/D. The current Cube options align as follows:

| Channel | Stock MK15 control | Cube function |
| --- | --- | --- |
| RC5 | SA | Six-position flight-mode input |
| RC6 | SB | AltHold mode, option 70 |
| RC7 | SC | Proximity avoidance enable, option 40 |
| RC8 | A | FlowHold, option 71 |
| RC9 | B | Current readback: optical-flow calibration, option 158; NOT GPS disable |
| RC10 | C | Land, option 18 |
| RC11 | D | Motor emergency stop, option 31 |

This is a stock-reference visual, not proof of the transmitter's current
mapping. `RC6_OPTION=70` makes SB an AltHold mode selector. In SIYI Channel
Monitor or Mission Planner Radio Calibration, move one physical control at a
time and verify only the expected channel changes through the low and high
thresholds. The current bench status has no RC input, so perform this check
before arming.

RC11 is not an airborne recovery mode. Motor emergency stop makes the aircraft
fall and is for an impact, entanglement, or immediate propeller hazard.

Latest SIYI documents are available from the
[official MK15 download center](https://www.siyi.biz/en/product/hand-gcs/mk15-industry/download/).

## Obstacle Avoidance

**27 September flight restriction:** the installed official ArduCopter 4.7.1
build omits `AP_AVOIDANCE_ALTHOLD_ENABLED`. FlowHold and AltHold have **no
horizontal proximity response** in this build, even with healthy PRX and RC7
high. Previous instructions claiming otherwise were incorrect for this
firmware. See the [flight investigation](data/diagnostics/2026-09-27-tree-contact/README.md).
No further obstacle-approach flight is approved by these instructions.

JT16 proximity is always sent while the boot service is healthy. RC7 controls
the Cube's simple proximity avoidance in supported modes. It does not disable
BendyRuler's separate AUTO path planner in the inspected 4.7.1 source.

- RC7 low: proximity remains visible; simple avoidance and Jetson obstacle
  audio are disabled. Do not treat this as an AUTO planner-off switch.
- RC7 high above 1.50 m AGL: requests Cube-native proximity avoidance;
  mode support must also be verified.
- Below 1.50 m AGL: Cube suppresses proximity reaction for safe takeoff and
  landing; keep RC7 low as the independent pilot gate.
- 1.50 m to 2.00 m: audio gives one warning beep per second. Audio is
  independent of whether the flight mode can actually avoid obstacles.
- 1.25 m to 1.50 m: three beeps per second.
- Below 1.25 m: beep rate rises toward 10 Hz.
- The logged Loiter profile uses a 2.00 m margin and Slide behavior, not a
  guaranteed stop. FlowHold/AltHold do not react in the installed build.
  The separate 1.50 m value is the never-plan-to-cross hard boundary.
- BendyRuler uses a 4.0 m lookahead and 1.50 m CG-referenced margin in
  `AUTO`, regular `GUIDED`, and `RTL` when all position requirements are valid.

The D415 is recorded and visualized but does not currently drive Cube
proximity. Active flight proximity is JT16 only.

Before enabling RC7, require:

```bash
./optflow obstacle-status
```

The output must show fresh JT16 frames, Cube connected, `READY=true`,
`PRX_STABLE=True`, zero growing reconnect count, and a valid RC7 PWM.
`READY_SCOPE=proximity_transport_only`: these checks prove the data path,
not obstacle detection coverage or flight readiness. Do not enable RC7 to
hide a `PRX1: No Data` condition.

## GPS-Disabled Mission

**Current audit, 27 September:** Cube returns only the sequence-zero home
placeholder, not an executable mission. Home/Origin were not returned in the
bounded check, GPS has no fix, and RC9 is option 158 (flow calibration), not
65 (GPS disable). The historical procedure below is blocked until channel
mapping, position and mission are reviewed. No channel assignment was changed.

### What Is Actually Required

GPS hardware is not the only possible position source for autonomous modes,
but `AUTO` still requires a valid position estimate, EKF Origin, Home, yaw,
altitude, and a mission expressed in global coordinates. In this aircraft,
H-Flow supplies horizontal velocity and the EKF integrates it into local
position. Integration drifts, so this is a short-range experimental path, not
a guarantee of accurate long-distance return.

BendyRuler avoids obstacles along an existing valid route. It does not create
position, fix optical-flow drift, restore a lost origin, or make `AUTO` safe
when flow/range/yaw/PRX is stale.

### Props-Off Gate

1. Confirm the aircraft is disarmed and propellers are removed.
2. Boot Cube, Jetson, transmitter, telemetry radio, and GCS.
3. Verify RC5 and RC7 through RC11 one at a time in the channel monitor.
4. Require valid H-Flow quality and range, correct direction, healthy battery,
   compass consistency, fresh PRX, and no RC or GCS failsafe.
5. Verify the mission contains no terrain altitude and no speed command above
   0.50 m/s.
6. Confirm RC7 low, RC9 low, RC11 released, and the pilot can select FlowHold
   and Land deliberately.

### Plan the First Mission

1. Keep GPS connected and RC9 low.
2. Boot outdoors and wait for a stable 3D GPS fix, EKF Origin, and Home. When
   testing the RTK base, also require the aircraft GPS indicator to reach RTK
   Float and preferably RTK Fixed before takeoff.
3. Confirm the Home marker is at the real launch point. Do not use a stale or
   manually guessed Home.
4. Create a short two-waypoint mission: about 3 m straight out, 3 m back to
   the launch coordinate, both near 2 m relative altitude.
5. End with `LOITER_UNLIM`. Do not use automatic takeoff, automatic landing,
   spline waypoints, terrain altitude, or `RTL` in the first proof.
6. Upload the mission, read it back, and check every coordinate, altitude, and
   command before arming.

### Fly With GPS Disabled by RC9

1. Confirm the service and telemetry are healthy. Leave RC7 low and RC9 low.
2. Take off manually in the already-tested altitude-controlled mode. Never
   select `STABILIZE`.
3. Select FlowHold with RC8 and hold 1.5 to 2 m AGL for at least 10 seconds.
4. Raise RC9 to disable GPS while still in FlowHold. Wait another 10 seconds.
5. Continue only if QGC still shows FlowHold, local position remains valid,
   the aircraft does not accelerate into drift, and no flow, range, EKF, yaw,
   RC, GCS, battery, or PRX warning appears.
6. Start the uploaded mission by selecting regular `AUTO`. The mission starts
   immediately after the mode is accepted.
7. Keep RC7 low for the first three no-obstacle repetitions. Abort for wrong
   direction, growing drift, oscillation, unexpected tilt, lost pilot
   authority, stale position, or telemetry failure.
8. At mission end, reselect FlowHold by cycling RC8 low then high. Lower RC9
   to restore GPS and land manually in the stable altitude-controlled mode.
9. Download and review both Cube and Jetson data after every repetition.

### Add Avoidance

Only after three clean no-obstacle missions:

1. Use one large lightweight obstacle in a wide test area.
2. Confirm fresh JT16 sectors and `proximity_healthy=true`.
3. Raise RC7 before selecting `AUTO`.
4. Confirm the Cube slows, stops, or routes around the obstacle without
   crossing the 1.50 m CG target margin.
5. Lower RC7 after returning to FlowHold.

### Physical GPS Disconnect Demonstration

This proves GPS-denied AUTO during flight, not a GPS-free cold boot. Complete
three clean RC9-disabled missions before using the physical disconnect.

1. Require a calibrated and motor-interference-validated Cube internal compass.
   The Here4 DroneCAN compass driver must remain disabled.
2. With Here4 GNSS connected and RC9 low, boot outdoors and establish a fresh
   EKF Origin and Home. Upload and read back the short out-and-back mission.
3. While disarmed, raise RC9 first and verify GPS is disabled while local
   position remains healthy. Then disconnect Here4 without rebooting the Cube.
   Wait at least 10 seconds and verify the GCS shows no GPS while compass yaw,
   H-Flow, range, local position, EKF, RC, battery, telemetry, and PRX remain
   healthy.
4. Do not arm if Home moved, global position became zero, XY position control
   became unhealthy, or any pre-arm warning remains.
5. Keep RC7 low, take off manually in FlowHold, and hold 1.5 to 2 m AGL for
   10 seconds. Select AUTO only after the hold remains bounded.
6. The first mission is 3 m out and 3 m back at no more than 0.50 m/s, ending
   in `LOITER_UNLIM`. Recover to FlowHold with RC8 and land manually.
7. Record the GCS GPS status and download the Cube log. The evidence must show
   AUTO with no valid GPS input and continuous optical-flow/range/yaw health.

### Later RTL Test

Do not unplug GPS for the first mission. Test a short `RTL` only after the
out-and-back mission passes repeatedly with bounded return error. Keep GPS
hardware connected but disabled through RC9 so it can be restored immediately.

A GPS-free cold boot is a separate test. ArduCopter 4.6.3 does not guarantee a
usable persisted Origin and Home for this setup. The operator must establish
and verify both before expecting global mission or `RTL` behavior.

### Pilot Recovery

If the mission behaves incorrectly:

1. Cancel `AUTO` to the already-proven FlowHold path using RC8.
2. Lower RC7 if avoidance itself is producing an unsafe response.
3. Lower RC9 to restore GPS if the GPS solution was healthy before the test.
4. Stabilize the hover in the altitude-controlled mode and land manually, or
   use RC10 Land only when that is the safer terminal action.
5. Never fall through to `STABILIZE` and never use `GUIDED_NOGPS`.

## Mission Planner

Launch from the Ground Tools window, or directly:

```bash
cd "$HOME/Downloads/mission_planner"
mono MissionPlanner.exe
```

### Download DataFlash Logs in Mission Planner

1. Connect Mission Planner to the Cube. Use USB on the bench when possible;
   radio downloads can be slow.
2. Open **Flight Data**.
3. Open the **DataFlash Logs** tab at the lower left.
4. Press **Download DataFlash Log Via Mavlink**.
5. Refresh/select the required log and start the download.
6. Wait for completion before disconnecting or powering down.
7. This Mono installation reports its user data and log directory as:

```text
~/.local/share/Mission Planner/logs/
```

8. To inspect a saved file, return to **DataFlash Logs**, press
   **Review a Log**, choose the `.BIN`, and graph the required messages.

Mission Planner's official log workflow is documented in
[Downloading and Analyzing Data Logs](https://ardupilot.org/plane/docs/common-downloading-and-analyzing-data-logs-in-mission-planner.html).

### Convert BIN to CSV

Double-click **ArduPilot BIN to CSV**, choose the `.BIN`, choose the output
`.csv`, select the required data categories, and press **Convert**. **Basic
Sensors** is selected by default and includes attitude/IMU/vibration,
height/vertical motion, optical flow (`OF`/`FHLD`/`FHXY`), rangefinder
(`RFND`), proximity and obstacle data (`PRX`/`OA*`), battery/voltage/current,
GPS/origin, and flight modes/events.

Use **Select All** for a complete conversion or **Clear** before building a
small custom export. Each row contains relative time, absolute UTC when
available, message type, matching categories, human-readable `field=value`
data, and machine-readable JSON fields. The proximity category contains the
JT16-derived records stored by Cube; raw JT16 point clouds remain in the
Jetson sensor-session files.

The alphabetized Mission Planner reference generated for this Cube is on the
Desktop as `Mission_Planner_Log_Abbreviations.csv`. It contains message entries
such as `ATT`, `OF`, `RFND`, and `PRX`, plus qualified field entries such as
`ATT.Roll`, with descriptions, units, categories, and whether each record
appeared in the sample log.

## Service Commands

The current boot profile is **JT16 proximity only**. Inspect or restart it
while disarmed:

```bash
systemctl --user restart optflow-flight-logger.service
systemctl --user status optflow-flight-logger.service --no-pager --full
systemctl --user is-enabled optflow-flight-logger.service
./optflow obstacle-status
```

After a Jetson boot, the service waits until uptime reaches 60 seconds before
starting JT16 proximity. Cube owns stabilization and avoidance; this Jetson
profile sends no pose or movement commands. Do not open a second direct
Cube/JT16 reader alongside it.

### Experimental VIO Profile (Disabled)

The earlier VIO/proximity profile and `./optflow install-vio-service` remain
in the repository for development, but **are not the active boot setup**.
`odometry_shadow_to_cube_enabled` is false. Re-enabling the experimental
profile requires an explicit reviewed bench configuration, not just starting
its service. The calibration dataset is not yet approved.

Its historical QGC message `VIO DATA ACCEPTED: EXTNAV DISABLED` describes
packet acceptance, never GPS-denied arming or flight readiness.

### Disarmed ExternalNav Test

On 20 September 2026 (local time), a 90-second bench run temporarily selected
VIO for source-set-1 horizontal position and velocity. Cube reported all
three EKF lanes using external navigation, with GPS reporting no receiver
and zero satellites. The run passed the bench fusion checks: 924 packets at
11.43 Hz, a 0.204 s maximum gap, zero vision-health dropouts, and 807 Cube
local-position samples. Barometer height and compass yaw remained selected.

The latest pre-arm fault was `RC not found`. Startup `PRX1` and `VisOdom`
faults cleared once the streams began. GPS-denied arming was not tested, and
no `GPS_GLOBAL_ORIGIN` was returned. Heading alignment, physical motion,
origin/home setup and flight performance still require validation.

After deliberately restoring the reviewed experimental configuration, with
props removed and Cube disarmed, the bounded experiment is:

```bash
systemctl --user stop optflow-flight-logger.service
./optflow vio-extnav-bench
systemctl --user start optflow-flight-logger.service
```

This command backs up the parameters, temporarily applies
`EK3_SRC1_POSXY=6` and `EK3_SRC1_VELXY=6`, runs for 90 seconds, and restores
the original values with readback even if the sensor run fails. It sends no
arm or movement command. `--duration` permits 30 to 180 seconds. If restoration
fails, inspect the saved backup before restarting the flight service.

Evidence is under
`data/recordings/lio/20260919T212455Z_vio-externalnav-bench/` and the adjacent
`20260919T212452Z_externalnav_bench_parameters.json` contains the verified
rollback. Future runs use their own timestamp. A successful
`MAV_CMD_RUN_PREARM_CHECKS` acknowledgement only confirms that Cube accepted
the request; inspect the subsequent pre-arm messages and sensor state.

Inspect the currently enabled service:

```bash
systemctl --user status optflow-flight-logger.service --no-pager --full
journalctl --user -u optflow-flight-logger.service -n 100 --no-pager
./optflow obstacle-status
```

For a future approved GPS-assisted shadow flight, physically connect GPS, keep RC9 in
the GPS-enabled position, wait for a valid home/EKF lock, and require both
`VisOdom` and `PRX1` to remain healthy. Fly manually in Loiter with RC7 low;
ExternalNav fusion remains disabled. Do not unplug GPS during this test.

Manual full visualizer sequence:

```bash
systemctl --user stop optflow-flight-logger.service
./optflow visualizer --host 0.0.0.0 --no-browser
# After Ctrl+C:
systemctl --user restart optflow-flight-logger.service
```

The Ground Tools **IM10A + JT16 3D** button opens the focused sensor viewer and
manages ownership automatically. It shows only the live IM10A orientation,
acceleration, angular rate, and JT16 3D point cloud. Its fixed top-view guide
labels the body-frame sectors at `0`, `45`, `90`, `135`, `180`, `225`, `270`,
and `315` degrees. Cube telemetry, D415, trajectories, and movement output are
not opened by this viewer. Closing it restores the OA service automatically.

## Troubleshooting

### `PRX1: No Data`

```bash
systemctl --user status optflow-flight-logger.service --no-pager --full
./optflow obstacle-status
journalctl --user -u optflow-flight-logger.service -n 100 --no-pager
```

Require the service to be active, fresh JT16 counts to increase in
`runtime/obstacle_avoidance_status.json`, and no competing sensor viewer. Do not disable
PRX or arming checks to hide the error.

### RC or GCS Failsafe

`rc_pwm=null`, `RC not found`, or `GCS failsafe on` is a real no-flight state.
Power and bind the transmitter, verify receiver output, restore telemetry, and
confirm channel motion before arming.

### Recorder Has No Camera or IMU Rows

- Stop any individual D415 or IM10A viewer.
- Confirm `/dev/imu_usb` exists and D415 USB permissions are installed.
- Open the session `manifest.json` and inspect `source_errors`.
- Stop and restart recording after resolving ownership.

### Cube Log Scan Refused

The log pane requires a recent disarmed heartbeat before it pauses the service.
Confirm the Cube is disarmed, wait for `./optflow obstacle-status` to update,
then scan again.

### Hot Motors or ESCs

Land and disarm. Measure temperatures, then review voltage sag, current,
vibration, motor balance, propellers, commanded thrust, and DataFlash outputs.
Individual 40 A ESCs improve electrical margin but do not make heat acceptable.

## Development

Create the local environment and run all checks:

```bash
./optflow setup
./optflow build
./optflow test
git diff --check
```

Key paths:

| Path | Purpose |
| --- | --- |
| `config/system.yaml` | Hardware geometry, limits, and active safety gates |
| `mav.parm` | Checked-in reference parameter snapshot |
| `src/` | Python implementation |
| `scripts/` | Executable wrappers and hardware tools |
| `visualizer/` | Three.js browser interface |
| `hardware/` | Drivers, udev, systemd, and headless access |
| `tests/` | Unit and contract tests |
| `docs/` | Detailed engineering procedures |
| `data/` | Local calibration and flight evidence |
| `runtime/` | Locks, status files, and transient state |
| `AGENTS.md` | Full engineering and safety context |

## Non-Negotiable Rules

- Never use `STABILIZE` as a fallback.
- Never use `GUIDED_NOGPS` for this aircraft.
- Never treat FlowHold as drift-free global position.
- Never assume a one-time GPS fix guarantees later `RTL`.
- Never run two owners of Cube UART, JT16, D415, or IM10A.
- Never hide RC, GCS, PRX, EKF, compass, battery, range, or flow failures.
- Never invent Home or EKF Origin.
- Never change the disarm or motor emergency-stop assignment without prompting
  the operator first.
- Always preserve a tested pilot takeover path and review logs before expanding
  the envelope.

Opt-in VINS reception-only observation uses the existing Cube UART owner via `OPTFLOW_VIO_OBSERVATION=1`; source set 1 and `EK3_SRC_OPTIONS=0` are preserved. See the current update in `docs/INTERFACES.md`. VIO fusion and switching remain separate tests.
