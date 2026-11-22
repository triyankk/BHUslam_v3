## H-Flow down / TF03 rear range —21November2026

The operator moved TF03 rear and authorized its avoidance use plus H-Flow integrated downward DroneCAN range. Native scoped config applied exactly six changes and rebooted disarmed/props-off: RNGFND1_ORIENT4; RNGFND2_TYPE24,MIN0.08,MAX30,POS_Z0.1; PRX2_TYPE4. Preserve RNGFND1_TYPE27 and existing0.1–180m limits, front PRX1_TYPE2, FLOW_TYPE6/calibration/CAN, primary/backup EKF/GPS/compass/source and avoidance tuning. RNGFND2_ORIENT25/ADDR0 already matched; position copied from flow module. H-Flow read-only audit now includes slot2 range parameters and rear slot1 orientation. No optical-flow scale change or flight calibration was made. Live downID1/orient25 read15–16cm and rearID14/orient4 read17–18cm. Global proximity health can reflect rear while front GPIO is still missing. Keep JT16 recording/mapping only. New down ground-clearance0.2m and moved rear mount offsets were preserved, not remeasured. Native front viewer rejects180m unavailable-front sentinel telemetry outside ToF's5m range. See native docs/hflow-down-rear-rangefinder.md and report.

## Current front-ToF source split —21November2026

The operator selected front ToF over the Jetson GPIO TELEM2 link for Cube proximity; JT16 must be recording/mapping only. Native front ToF service owns I2C7 and ttyTHS1 at115200. The separate Cube USB owner is restored/unmasked with obstacle_avoidance.mavlink_output_enabled=false and lidar_enabled=false. Its native point reader/shared mapping feed and flight raw-recording gate remain; skip JT16 obstacle extraction, scan queueing, distance transmission and obstacle alerts. Do not mask/stop this shared USB owner just to stop JT16 proximity. Preserve VINS forwarding, arm/disarm recording, Cube cues, GPS reference and source settings. Legacy generic JT16 Cube preset now refuses this external-source config, preventing accidental PRX disable/profile overwrite. No Cube parameters were changed; readback already matches PRX1_TYPE2,front orientation0,OA_TYPE1/horizontal3m lookahead/2m margin. GPIO still has ~15s gaps and is not dependable avoidance; no timeout relaxation or stale-clear substitution. Single front coverage does not validate lateral/rear detours. See native docs/tof-front-proximity.md and reports/tof-front-integration-20261121.

## QGC reference text and arming readback —20November2026

The operator requested reference status text for QGC and GPS-free arming confirmation. NavigationReference now emits fixed NavRef STATUSTEXT (minimum2s, unchanged repeat30s) through the existing owner; C++ initialization/pose processing and the narrow incoming whitelist remain. It requires current boot/options before reporting verified state and clears that cue qualification on changed origin/Home. No source/mode/arm command or Cube parameter changed. Existing recording/beeps remain. Connection-time bounded read-only audit exposes ARMING_SKIPCHK/NEED_LOC/FENCE_ENABLE/TYPE; ARMING_CHECK is absent on installed4.7. Current mask304374 already skipped INS/compass/VIO and other checks and was preserved. Do not infer full health from pre-arm true under skipped checks. Current ALT_HOLD pre-arm health passes without GPS fix, but the operator declined GPS disconnection and QGC was unavailable. No unconditional armability, physical-unplug or visual-QGC proof. See native reports/gps-reference-qgc-20261120.

## Fresh-GPS bootstrap authorized — 20 November 2026

The native relay now supports exact vio_optflow_compass_gps_bootstrap: selected hybrid sources6/5/1/6/1, OPTIONS0, GPS1_TYPE9/GPS2_TYPE0, COMPASS_ENABLE1. Only GPS1 driver is changed from the prior selected profile; backup sets and compass device/priority/use flags remain. AHRS_OPTIONS8 keeps previous-origin reuse off. C++ initializes from fresh GPS and Cube origin, requests explicit GPS Home only while disarmed when absent, and requires Home readback. The sole-owner adapter independently validates this narrow COMMAND_INT179 whitelist plus local NAVREF_OK marker. It still rejects mode, source, arm, motion and parameter-write packets. Other profiles retain their original transport behavior. Read-only metadata requests run only for the new profile. Three slow beeps report a verified geographic reference; existing ten rapid recording-ready beeps and arm/disarm recording remain unchanged. No persisted launch-point fallback. Indoors validates waiting behavior only; clear-sky fix and physical disconnect remain unvalidated. See BHUslamV3/docs/gps-bootstrap.md and reports/gps-bootstrap-20261120 for exact evidence.

## Previous-origin reuse disabled —20 November2026

At the operator’s explicit request, native read/write/readback changed only AHRS_OPTIONS24->8, clearing UseRecordedOriginForNonGPS(bit16) while preserving RecordOrigin(bit8) and every other option. No origin/Home was assigned or erased, GPS1/2_TYPE remain0, and the selected VIO/optical-flow/internal-compass profile is unchanged. A fresh reference must be established after every new Cube boot. The proposed GPS plug/fix/disconnect bootstrap is not yet implemented: it needs one-time GPS-driver configuration, an explicit compatible bridge profile and verified origin/Home initialization. Do not silently enable GPS or change EKF sources. D415 remains disabled and normal services were restored. New scoped native --disable-origin-reuse-only mode preserves unrelated values.

## Primary hybrid sources restored — 20 November 2026

After the operator explicitly requested restoring the previous selected configuration and confirmed props removed, native disarmed read/write/readback changed exactly EK3_SRC1_POSXY3->6 and EK3_SRC1_VELXY3->5. Primary remains POSZ1(barometer),VELZ6(VIO),YAW1(internal compass),OPTIONS0,GPS1/2_TYPE0,COMPASS_ENABLE1,VISO_TYPE3. Current source2 backup contains VIO/optical-flow/compass settings and was preserved exactly, as were source3, mount, proximity and all audited unrelated values. No reboot or source-selection command was sent. Both bridge stages again recognize vio_optflow_compass; the previous Unsupported Cube navigation profile block is resolved. D415 remains disabled, Gemini VINS stays warm, and arm/disarm recording and buzzer logic are unchanged. The new native --vio-optflow-compass-primary-only mode guards the other15 canonical fields and writes only the two horizontal source parameters; complete preset cleanup remains a separate mode. Simulation tests validate exactly two writes and backup-profile preservation. Forwarding/health does not establish fusion, Home readiness or flight accuracy.

## D415 disabled globally — 20 November 2026

Read-only activation audit also found a pre-existing unsupported Cube profile: SRC1_POSXY=3, VELXY=3, VELZ=6, YAW=1 with GPS1/2_TYPE=0. It was unchanged before/after D415 disable and continues to block VIO forwarding. Do not automatically restore the older hybrid profile described below.

The operator requested disabling all D415 activity. sensors.depth_camera.enabled=false is now the persisted project-wide gate. The Desktop recorder omits the D415 worker, forces its effective video setting off, and marks depth_camera in manifest.disabled_sources. Direct pipeline startup, manual depth viewer/server, readiness probe, legacy RGB-D/spatial/flight workers and legacy D415 calibration honor this flag before importing/starting RealSense. Camera CLI overrides preserve the disable flag. Saved ground-tool video_enabled=false; auto_record_on_arm and other settings remain unchanged. Old realsense-status-terminal autostart was already disabled; no active D415-named unit was needed by this setup. Do not confuse D415 with Gemini: native Gemini VINS, JT16, Cube owner/relay, recording state machine and MAVLink buzzer logic remain enabled. No Cube parameter or forwarding gate was changed. RealSense SDK is retained for reversibility; disabling does not uninstall it.

## Current operator-selected pose timing policy — 18 November 2026

The operator explicitly requested removing Jetson pose timing limits and delegating freshness to ArduCopter. The live VINS producer now uses zero startup/recovery wait and no pose-age maximum; camera-position finite-difference velocity no longer has a150 ms processed-frame-gap cutoff. The C++ relay and this existing serial-owner adapter have no150 ms pose-age rejection or10 Hz throttle. Both keep original acquisition timestamps, age logging, latest-only queues and one-shot monotonically increasing samples. ArduCopter retains its300 ms reception watchdog and EKF timestamp/delay handling. Feature/quality/IMU/finite-data checks, heartbeat/config-audit freshness, disarmed profile qualification and armed profile lock remain. No Cube parameter, source-selection command, recording state machine or buzzer logic changed. Historical150 ms/500 ms/10 Hz pose limits below are superseded for this live path. This is a timing policy change, not EKF fusion or flight validation.

# optFlow_slam Agent Context

## Current hybrid VIO / optical-flow / internal-compass profile, 2026-11-18

The operator selected optical-flow horizontal velocity and the Cube internal compass for yaw. Primary sources now POSXY=6, VELXY=5, POSZ=1, VELZ=6, YAW=1, OPTIONS=0, GPS1/2_TYPE=0, COMPASS_ENABLE=1, VISO_TYPE=3. Internal primary compass ID1313809 (I2C bus2 address0x0c AK09918) is selected with COMPASS_USE=1 and USE2/3=0; EXTERNAL=0 and priority1ID matches. Enabled global compass and rebooted disarmed using prior props-off confirmation; primary calibration offsets and use flags were preserved. Post-reboot compass and optical-flow health bits are true. The two old DroneCAN compass IDs were disabled and are now zero after boot.

Both C++ relay and existing serial-owner adapter now accept exact vio_optflow_compass in addition to earlier profiles, using the same17 audited keys, disarmed qualification, freshness/quality/mount checks and armed-profile lock. Native --vio-optflow-compass-only applies the complete hybrid contract plus unused-set cleanup. Sets2/3 have POSXY/VELXY/VELZ/YAW=0, POSZ=1; SRC2 had regained nonzero VIO entries and was re-cleared. No source-selection, arm, mode, motor or mission command was sent. The startup/short-recovery comparison showed no measured benefit in its120s25W stationary window; live gate remains500ms and100ms candidate is diagnostic only. Recording/beeps/proximity remain unchanged. GPS-free fusion accuracy, calibration and motor-on performance remain unvalidated; do not relax health gates or restore old profiles automatically. Refresh and re-save the transmitter after this update.


## GPS-free unused-source dependencies cleared, 2026-11-18

The prior GPS-free apply preserved GPS sources in unused sets2/3, causing AHRS EK3 sources require GPS because firmware pre-arm checks every stored set. At the operator's explicit request, cleared unused source2/3 POSXY, VELXY, VELZ and YAW to 0, retaining POSZ=1. Exactly five writes: SRC2_POSXY/YAW and SRC3_POSXY/VELXY/YAW ->0. Readback verifies all ten values and primary source1, VISO_TYPE=3, PRX1_TYPE=2, OPTIONS=0 preservation. No reboot, source-selection, mode or arm command was sent. Backup and rollback were saved. The native GPS-free apply mode and exported profile now include all ten unused-source values, making 27 complete profile values. This supersedes preserving GPS references in unused sets for GPS-free setup. VIO position/fusion health remains separate; no arming check is bypassed.


## GPS-free parameters loaded for transmitter saving, 2026-11-18

The operator explicitly requested applying the GPS-free profile to the Cube to save it on the ground-control transmitter. Exactly six values changed with fresh disarmed backup/readback: COMPASS_ENABLE 1->0, GPS1_TYPE 9->0, EK3_SRC1_POSXY/VELXY/VELZ 3->6 and YAW 1->6. Source1 POSZ=1, EK3_SRC_OPTIONS=0, VISO_TYPE=3, mount/quality settings, PRX1_TYPE=2, recording/beeps and all unrelated scoped settings were preserved. A disarmed reboot was performed using prior props-off confirmation. All 17 actual Cube replies matched gps_free_vio for 25 consecutive post-reboot seconds. This supersedes older GPS-aided state below; do not automatically restore GPS settings. No source-selection command, arm/mode/motor/mission command or arming-check bypass was sent. The GPS rollback was saved. Origin/Home/heading and VIO performance/fusion remain unvalidated; this was bench configuration for saving, not flight validation.


## JT16 backend corrected to MAVLink, 2026-11-18

The operator identified PRX1_TYPE=4 (RangeFinder). That earlier enable setting was an implementation mistake for the existing eight horizontal MAVLink DISTANCE_SENSOR packets. Corrected only PRX1_TYPE 4 -> 2 (MAVLink), read back and rebooted while freshly disarmed with prior props-off confirmation. Post-reboot independent readback confirms 2; all common audited other parameters were preserved, including VISO_TYPE=3, GPS source1 and EK3_SRC_OPTIONS=0. The parameter tool was fixed so enable-proximity-only now selects 2; integration tests verify the single write. Base config already requests proximity_type=2. Do not restore 4 from older notes/rollback evidence for this transport. Recording, beeps, RC and BendyRuler tuning were not changed. Actual post-correction health observation is saved separately; no obstacle direction/braking/flight validation is implied.


## GPS-aided VINS reception activated, 2026-11-18

The operator explicitly superseded the earlier VISO_TYPE=0 override and requested GPS-aided recording plus actual Cube reception verification. Props removed was confirmed. Fresh native disarmed readback preceded VISO_TYPE 0 -> 3, saved/read back and activated by reboot. Primary POSXY/VELXY/VELZ=3, POSZ=1, YAW=1, EK3_SRC_OPTIONS=0, GPS/compass and all unrelated scoped settings were preserved. Both guards now identify gps_observation. No source switch, arm, mode, mission or motor command was sent.

Temporary LOG_DISARMED=1 was restored to 0 after capture. Actual Cube BIN 207 contains 31 accepted VISP and 31 accepted VISV records, every RTimeUS matching an actual companion serial write; all 4,446 recorded XKFS entries have SS=0. Native decoding found no desynchronization or truncated tail. This proves frontend receipt, not VIO EKF fusion. The stationary stream remained intermittent: maximum accepted-packet gap 36.325 s, so genuine VisOdom unhealthy periods remain. Do not weaken gates or disable arming checks to hide this. PRX1 No Data/GCS-failsafe history remains distinct; decoded JT16 scans do not prove proximity health. All five boot units are active/enabled; recording and ten-beep behavior remain unchanged. The mapper was paused for exclusive log transfer and restored; VINS stayed warm. Earlier forwarding-disabled descriptions below are historical and do not override this explicit activation.


## Dual-profile VINS bridge update, 2026-11-18

The operator requested one bridge that supports GPS observation and GPS-free VIO. The existing opt-in adapter now recognizes exact profiles while retaining sole serial ownership. GPS observation: primary POSXY=3, VELXY=3, POSZ=1, VELZ=3, YAW=1, GPS1_TYPE=9, GPS2_TYPE=0, COMPASS_ENABLE=1, VISO_TYPE=3. GPS-free VIO: primary POSXY=6, VELXY=6, POSZ=1, VELZ=6, YAW=6, GPS1/2_TYPE=0, COMPASS_ENABLE=0, VISO_TYPE=3. Both require AHRS_EKF_TYPE=3, EK3_ENABLE=1, EK3_SRC_OPTIONS=0, VISO_QUAL_MIN=40, VISO_ORIENT=0 and mount X/Y/Z=0.13/0/0.05. These 17 keys must all be finite and no older than five seconds. The old GPS/flow hybrid is not an accepted profile.

Current live configuration is GPS recording: same GPS settings but VISO_TYPE=0. It is identified as gps_recording and intentionally sends no poses. This bridge update changes no Cube parameter and does not authorize activating VIO fusion. A profile must qualify while disarmed; a different profile while armed is blocked. Armed startup or reconnection after a heartbeat gap requires disarm again. Non-primary source-selection evidence remains latched off. Profile recognition never sends a source switch, parameter write or arm/mode/mission command. Mount tolerances are 1e-5 for float readback. Quality >=40, age <=150 ms, latest-only queues, 10 Hz cap, packet identity/frames, recording and ten-beep cue remain unchanged.

Status/logs now include profile and pinned_profile. Actual serial writes include profile. GPS-free fusion, geographic origin/Home/heading, clipping mitigation and motor-on accuracy remain separate unvalidated work. Entire Here4 will be removed without an independent compass; VIO must supply yaw. Never infer north from arbitrary VINS startup yaw. See docs/INTERFACES.md for this contract.


## JT16 proximity configuration restored, 2026-11-18

Operator requested restoring JT16/BendyRuler, then explicitly waived scan verification and requested configuration only. Fresh disarmed native readback confirmed OA_TYPE=1, OA_BR_TYPE=1, OA_BR_LOOKAHEAD=4, OA_MARGIN_MAX=1.5. Only PRX1_TYPE changed 0 -> 4 with verified readback and rollback. All audited EKF source sets (including current SRC2 POSXY=3), EK3_SRC_OPTIONS=0, VISO_TYPE=0, RC settings and avoidance tuning were preserved. No Cube reboot, mode, arm, motor or source-selection command was sent. LiDAR power was reported disconnected, and propeller removal was not confirmed. Power JT16 and reboot Cube to load the backend; current scan reception, orientation, proximity health and actual avoidance remain unverified. The Cube owner was restored and remains boot enabled. Do not label this as flight approval or automatically re-enable VIO forwarding. RC7 option 40 does not universally disable BendyRuler; BendyRuler applies to AUTO/GUIDED/RTL, not Loiter/FlowHold.

## Temporary USB transport test, 2026-11-18

The operator requested USB isolation after the GPIO/TELEM2 path stopped answering requests. Thirty-second identical C++ read-only probes received zero matching audit replies over /dev/ttyTHS1 and 201 matching replies over Cube USB. Incoming UART heartbeats continued; its transmit/request direction is unresolved. This does not prove a particular pin or wire is faulty. The runtime now has an explicit USB-test systemd override for the Cube if00 by-id endpoint, with the base UART override preserved. Follow live cube.endpoint status and installed service environment. USB is required while this test override remains installed. No Cube parameter, EKF source, five-second parameter freshness gate, 150 ms pose age limit, recording or buzzer logic was changed. Do not silently restore UART until bidirectional testing passes.


## Gemini mounting update, 2026-11-18

Gemini mount update, 2026-11-18: operator reports 22.8 degrees downward pitch and camera optical centre 0.13 m forward, 0 m lateral (assumed), 0.05 m below the Cube reference. Rotation is removed by the native camera-to-body transform; Cube VISO_POS_X=0.13, Y=0, Z=0.05 corrects translation once. VISO_ORIENT remains 0. Internal camera/IMU calibration is unchanged, and this geometry report is not full calibration validation. Live disarmed receipt after the change cleared the earlier pitch-mismatch pre-arm warning. Source set 1 and EK3_SRC_OPTIONS=0 were preserved.

## Reception-only VINS observation update, 2026-11-18

The operator explicitly authorized gated VINS-Fusion ODOMETRY reception during the GPS-aided observation flight, including while armed. This supersedes older proximity-only/disarmed-only output descriptions solely for this opt-in observation adapter; it does not authorize VIO fusion or source switching. The existing Cube UART owner remains sole writer. OPTFLOW_VIO_OBSERVATION=1 enables the adapter in vio_observation.py; native estimation and upstream packet filtering remain outside the UART owner. UDP 14603 accepts only one validated ODOMETRY packet, not arbitrary MAVLink commands; UDP 14604 exports selected actual Cube telemetry. No sibling code/config is imported.

Live source set 1 is GPS POSXY=3, optical-flow VELXY=5, barometer POSZ=1, VELZ=0, compass YAW=1, with EK3_SRC_OPTIONS=0. The sole approved Cube change was VISO_TYPE 0 to 3 and a disarmed reboot; source sets were preserved. No RC option 90 was assigned, and SCR_ENABLE=0. Firmware defaults to primary source at boot; only Cube BIN XKFS.SS=0 confirms actual source use per core after flight. No automatic source-switch command is sent. Source set 2 retains its pre-existing ExternalNav position configuration.

The adapter requires a disarmed startup heartbeat, later permits armed observation, audits fixed source/backend parameters read-only, blocks on missing/changed replies or stale heartbeat, and preserves recording and buzzer code. It keeps latest-only poses with capture age <=150 ms, quality >=40, and 10 Hz maximum writes. Oldest parameter replies are refreshed first with a five-second freshness limit. Parameter/config mismatch never triggers a write or source change. Frontend health is logged as a proxy, not pose acknowledgment or EKF fusion proof. The existing camera-to-body assumptions are not calibration approval.

Arm/disarm Desktop recording and the fixed ten-beep recording-ready cue remain unchanged. Observation health/serial writes are recorded under data/recordings/vio-observation, indexed by boot ID; status exposes cube.vio_observation. Keep matching Cube BIN VISP/VISV for receipt and XKFS for source selection. Logs and provisional poses are not flight approval. VISO_TYPE=3 may add genuine VisOdom pre-arm faults; never disable checks to hide them.


This file applies to the entire repository. It is the durable engineering and
safety context for coding agents and maintainers. Read it before changing code,
Cube parameters, services, hardware ownership, flight procedures, or claims of
readiness.

## Source of Truth and Precedence

When facts disagree, use this order:

1. Live, disarmed hardware readback collected in the current session.
2. `config/system.yaml` for intended project configuration and safety gates.
3. `mav.parm` for the latest checked-in Cube parameter snapshot.
4. Generated session manifests and reports under `data/recordings/`.
5. The focused documents under `docs/`.
6. This file and `README.md`.
7. Historical comments, old logs, assumptions, or conversation memory.

Do not turn an assumption into a verified flag. If hardware state is cheap and
safe to read while disarmed, read it. If a value may be stale and cannot be
verified, label it as historical.

## Project Objective

Build a staged GPS-denied autonomy stack for a heavy Cube Orange+ quadrotor:

- Reliable manual optical-flow flight without depending on GPS position.
- CG-referenced 360-degree obstacle awareness.
- Timestamped lidar-inertial and visual-inertial local motion estimation.
- Local 3D mapping and fixed-altitude 2.5D planning first.
- A guarded return to the stored launch pose in the same local map.
- Eventual GPS-denied operation with pilot takeover and controlled failure.

The Cube owns stabilization, EKF3, altitude, optical-flow velocity fusion,
flight modes, motor mixing, and motor outputs. The Jetson owns sensor decoding,
SLAM/VIO, mapping, planning, evidence recording, and only future bounded local
targets. Jetson code must never become a replacement flight controller.

## Current Snapshot

Independent JT16 viewer, 2026-10-05:

- `./optflow jt16-viewer` / Desktop **JT16 Point Cloud + IMU** serves the
  read-only standalone page on LAN port 8776. It never opens Cube or IM10A.
- Native bridge optional `--viewer-port 18776` sends bounded, nonblocking
  point-cloud chunks and the SDK's built-in JT16 IMU callbacks over loopback.
  The existing stdout point protocol is unchanged. OA supplies this side feed.
- Auto source mode listens when the JT16 project lock is held; otherwise it
  owns only lidar. No automatic flight-service stop/restart or boot enable.
  Stop a direct-mode viewer before starting another hardware owner.
- Bench reception showed about 5 Hz clouds and 200 Hz internal IMU callbacks.
  SDK duplicate-value suppression means this is not a raw sample-loss audit.
  Sensor timestamp is not verified UTC; no IMU/body calibration, orientation,
  odometry, or flight readiness is inferred. See `docs/JT16_VIEWER.md`.
- User confirmed only Hesai was connected. Cube absence was checked before
  briefly restarting the existing lidar owner to enable the optional feed.
  No RC settings, Cube parameters, firmware or estimator settings changed.

Incident update, 2026-09-27 (takes precedence over older OA claims):

- Downloaded Cube logs 191 and 190 to `data/flight_logs/2026-09-27/`.
  Log 191 is the flight; 190 is an older disarmed 4.6.3 startup log.
- Log 191 is official ArduCopter 4.7.1 `dbe79216`. The checksum-verified
  saved firmware feature list explicitly disables
  `AP_AVOIDANCE_ALTHOLD_ENABLED`. `AVOID_ANG_MAX` is absent from the flight
  parameters. Horizontal FlowHold/AltHold avoidance is compiled out, even
  with RC7 high and healthy PRX. Do not repeat historical claims otherwise.
- RC7 stayed at 1945 us. In FlowHold, a front/right obstacle closed to
  0.76 m at Cube boot time 228.612 s while downward range was about 3.16 m.
  Switching to Loiter at 228.911 s produced `SA.Back=1`; minimum PRX then
  reached 0.75 m. No AUTO/BendyRuler event occurred in this flight.
- `PRX_FILT=0.25` introduces noticeable lag (raw 0.76 m versus filtered
  right-front 1.20 m). `READY` means proximity transport only; it does not
  verify per-mode support, detection coverage, braking, or flight clearance.
- Report and an unapplied Loiter-only early-stop candidate are in
  `data/diagnostics/2026-09-27-tree-contact/`. No incident-related flight
  parameter writes or firmware flash were performed. Field obstacle tests
  remain blocked until firmware support and measured braking are verified.
- Follow-up read-only AUTO audit: firmware/hash confirmed live, 1,169
  parameters, `OA_TYPE=1`, 4 m lookahead, 1.5 m margin, `WP_SPD=0.5` m/s.
  Mission contains only home placeholder, no executable route. No Home or
  Origin returned within bounded requests; GPS has no fix; EKF reports
  relative but not absolute horizontal position. AUTO readiness not proven.
- Live `RC9_OPTION=158` is flow calibration, NOT the historical GPS-disable
  option 65. Do not execute old RC9 GPS-denial instructions. No remap applied.
- RC7 option 40 gates simple `AC_Avoid` only in the inspected 4.7.1 code;
  BendyRuler and its obstacle database are not toggled off by RC7-low.
  Never describe RC7 as universal AUTO planner cancellation or guarantee
  avoidance in every flight mode. Preserve mode/ground safety gates.

Current operator-requested profile, 2026-09-26:

- Restore JT16-only proximity, not VIO navigation. A fresh disarmed heartbeat
  was checked before stopping/disabling `optflow-vio-flight-shadow.service`.
- `optflow-flight-logger.service` is enabled/running, with Cube USB by-id
  retained from `config/system.yaml`. Do not revert it to UART without a new
  link audit. `odometry_shadow_to_cube_enabled` is now false.
- Twenty one-second health samples: all fresh/ready/proximity healthy/stable,
  minimum JT16 4.955 Hz, maximum frame age 184 ms, 95 new frames and zero Cube
  reconnects. RC7 high at 1945 us; Cube disarmed.
- Pre-arm messages still include RC/GCS failsafe, altitude/EKF/home faults
  and `VisOdom: not healthy`. No Cube backend, EKF source or arming parameter
  was changed. Restored proximity is not flight clearance.
- VIO calibration code/evidence is retained for offline development; do not
  re-enable its service or odometry flag merely because it exists in the repo.

Historical disarmed VIO fusion evidence, 2026-09-20 local time:

- At that time the boot service was `optflow-vio-flight-shadow.service`, using stable
  Cube USB from `config/system.yaml`. The older OA service is inactive.
- `./optflow vio-extnav-bench` is an explicitly requested, bounded disarmed
  experiment. It temporarily applies ExternalNav XY position/velocity in
  source set 1, retains barometer Z and compass yaw, and restores parameters
  after the run. This exception does not authorize armed ExternalNav output.
- Run `20260919T212455Z_vio-externalnav-bench` passed: 924 VIO packets at
  11.43 Hz, max gap 0.204 s, zero vision-health dropouts, and 807 Cube local
  positions. All three EKF lanes reported using external navigation while
  GPS reported no receiver and zero satellites.
- Last pre-arm fault was `RC not found`; PRX/VisOdom startup faults cleared.
  No EKF origin was returned. Full pre-arm success, heading/motion alignment,
  global origin/home and flight performance remain unproven.
- Parameter rollback was verified in the adjacent timestamped
  `externalnav_bench_parameters.json`; normal boot still excludes ExternalNav.
- `MAV_CMD_RUN_PREARM_CHECKS` ACK is command acceptance, not pre-arm success.
- QGC must describe this boot stage as
  `VIO DATA ACCEPTED: EXTNAV DISABLED`, without a flight-ready claim.

The historical snapshot below is superseded by the evidence above where they
conflict.

Snapshot consolidated on 2026-08-11:

- Project stage: `hardware_bringup`.
- Active boot profile: JT16 obstacle avoidance only.
- Active systemd unit: `optflow-flight-logger.service`.
- The unit name is historical; its executable is
  `scripts/run_obstacle_avoidance_service.sh`.
- Cube UART: direct `/dev/ttyTHS1`, MAVLink2, 460800 baud.
- Experimental local MAVLink router: disabled and not part of active runtime.
- JT16 proximity: active and live through the MAVLink proximity backend.
- Desired Cube local planner: BendyRuler (`OA_TYPE=1`) with 4.00 m lookahead
  and 1.50 m CG margin; live two-stage apply/readback is pending.
- Desired Cube autonomous speed profile: 0.50 m/s horizontal, climb, descent,
  RTL, and landing; live readback is pending.
- D415 contribution to flight avoidance: disabled.
- FAST-LIO, RGB-D odometry, SLAM navigation, and local return: inactive in the
  boot profile.
- Cube ExternalNav fusion: disabled.
- Jetson companion movement output: disabled.
- Return stage: `locked`; live control false.
- LIO validation approval: false.
- Last successful OA evidence: 150/150 ready/link/proximity-health samples
  across 75 s, zero reconnects, no fresh `PRX1: No Data`.
- A 2026-08-11 read found the old Cube process disarmed, JT16 stale, PRX
  unhealthy, and RC absent. Restarting the watchdog-enabled runtime recovered
  fresh JT16 frames, `READY=true`, `PRX_HEALTH=True`, and RC7 low at 1045 us.
  The latest volatile pre-arm text then reported `GPS 1: Bad fix`.
- Latest two flight logs contain ordinary `OF` records but no official `OFCA`
  calibration samples; no flow-scale recommendation was accepted.
- Full repository validation after these changes: 259 Python tests passed and
  the visualizer production build completed.

This proves a bench data path, not obstacle-avoidance flight performance and
not GPS-denied readiness.

## Non-Negotiable Safety Contract

### VIO Calibration Console

- `./optflow vio-calibrate` / Ground Tools > Tools > VIO Calibration starts an
  idle browser console on LAN port 8773; never autostart sensor capture.
- Implementation: `vio_calibration.py` (target/fits/export/import),
  `vio_calibration_capture.py` (disarmed acquisition/service ownership),
  `vio_calibration_server.py` (HTTP/CLI), and `visualizer/src/vio-calibration.*`.
- Sessions: `data/calibrations/vio/<UTC timestamp>/`; procedure:
  `docs/VIO_CALIBRATION.md`. AprilGrid is 6x6, 36h11, black border 2,
  nominal tag edge 20 mm, spacing ratio 0.3, ID0 bottom left. Kalibr tag bit
  orientation is 180 degrees from OpenCV's canonical drawMarker output.
- Capture requires props-off confirmation and live disarmed Cube; only a
  previously active managed VIO/OA service is paused/restored. Stale cached
  disarmed status must never authorize stopping a flight runtime.
- Save raw IM10A axes/SI units and sensor/host times; do not configure IMU baud.
  Export uses one fitted clock map per sensor; residual offset is still unknown.
- OpenCV camera fit is local; full camera-IMU fitting requires external Kalibr
  and measured IMU noise YAML. Never invent noise densities to unlock solving.
- Offline `--analyze SESSION` and the console's Analyze saved capture reuse
  immutable Kalibr exports, never acquire hardware or pause flight services.
  `analysis.json`/`analysis.md` preserve progress, stage quality and blockers.
  Fit uses a strict 80-view cap and OpenCV QR plus held-out checks; the old
  stride-based cap could silently keep more frames and stall the dense SVD fit.
  The 20260920T081432.888446Z capture was recovered: 0.561 px fit RMS,
  0.796 px held-out p95; 41.2% target detection and marginal tilt diversity
  still fail existing gates. No calibration applied. Kalibr and a measured
  IMU noise/bias model remain required; old zero-gyro noise reports are inadequate.
- All results are candidates, not calibration verification or flight approval.
  No Cube parameter writes, EKF changes, movement commands, or automatic
  application of extrinsics. PLAY_TUNE is the only capture MAVLink output.
- The console's **Screen target** opens `vio-target.html`, with a 14-36-inch
  diagonal selector defaulting to 22 before displaying tags. Bitmap geometry
  is 95 cells square: 10-cell tags, 3-cell gaps, 10-cell white margins. Physical
  size estimated from diagonal/CSS screen dimensions requires a width/height
  ruler check. Never infer verified physical millimeters from diagonal alone.
- `calibration_display.py` validates scale/liveness. Confirmed target size is
  frozen into `manifest.target.tagSize` (meters), plus `target_display` metadata,
  and copied unchanged into Kalibr export. Configuration/remeasurement is
  blocked while sensor capture owns hardware. Resize/zoom/fullscreen/movement,
  hidden targets and expired target heartbeat invalidate capture. VNC-side
  scaling may be invisible to browser APIs: document the need to remeasure.

### Flight Safety

- Never select, recommend, or automate `STABILIZE` as a fallback. On this heavy
  aircraft an inadequate throttle position can cause immediate altitude loss.
- Never use `GUIDED_NOGPS` for this architecture.
- Never arm, take off, land, select a mode, send raw attitude, mix motors, or
  command motor outputs from Jetson code during bring-up.
- Never treat standard ArduPilot RTL as guaranteed by a one-time GPS fix. Its
  experimental optical-flow use requires valid local position, yaw, range,
  EKF Origin, Home, and staged logged proof throughout the flight.
- Never remove GPS because a visualizer trajectory looks plausible.
- Never send ExternalNav into Cube EKF3 without approved estimator evidence,
  physical calibration gates, and an explicit digest-bound approval artifact.
- Never fabricate an approval file or mark assumed data as verified.
- Never hide PRX, RC, compass, battery, vibration, EKF, or other pre-arm faults
  by disabling checks.
- Never invent an EKF origin. ArduCopter 4.6 does not guarantee a persistent
  GPS-free origin after a cold boot.
- Never let two processes own the Cube UART, IM10A, D415, or JT16.
- Never treat optical flow as globally drift-free position.
- Never claim obstacle flanking or path planning from native proximity stop or
  lean-away behavior.
- Never change the RC disarm or emergency-stop assignment without prompting the
  operator once and observing the intended transmitter channel.
- Never use hand motion, assumed distances, or a bench carry test as flight
  validation.
- GPS remains physically connected during commissioning unless an approved
  phase explicitly requires denial.
- Props are removed for bench configuration, calibration, direct UART work,
  and estimator carry tests.
- Field changes are made one variable at a time and reviewed from both Cube and
  Jetson evidence before progression.

## Airframe and Propulsion

- Aircraft class: approximately 6 kg heavy quadrotor.
- Opposite motor-center diagonal: 0.85 m.
- Propellers: 18 inch diameter.
- Physical rotating radius: `0.425 + 0.2286 = 0.6536 m`.
- Configured protected radius: 0.75 m.
- Hard obstacle clearance: 1.50 m measured horizontally from the CG.
- Battery: 6S, 22,000 mAh, mounted below the CG.
- Full-charge pack voltage: approximately 25.2 V.
- Former propulsion: one shared 55 A 4-in-1 ESC.
- Current propulsion: four individual 40 A ESCs, one per motor.
- Bench current evidence: about 11 A for one motor at 75 percent throttle;
  15 A per motor is the conservative planning case.
- Individual ESC flights showed hot motors/ESCs but no observed thrust loss.
  Heat remains a fault signal requiring measurement and review.

The low battery creates a pendulum-like mechanical response when attitude
commands are aggressive. Do not mask mechanical CG or thrust imbalance with
arbitrary controller limits. Verify CG, battery restraint, motor alignment,
propellers, thrust balance, and vibration before tuning attitude response.

Relevant Cube limits from the checked-in snapshot and prepared profile:

- `ANGLE_MAX=3000` or 30 degrees.
- Checked-in snapshot: `WPNAV_SPEED=200`, `WPNAV_SPEED_DN=100`, and
  `WPNAV_SPEED_UP=150` cm/s.
- Prepared autonomous profile: `WPNAV_SPEED=50`, `WPNAV_SPEED_DN=50`,
  `WPNAV_SPEED_UP=50`, `RTL_SPEED=50`, `LAND_SPEED=50`, and
  `LAND_SPEED_HIGH=50` cm/s. Do not call these live until exact readback passes.
- `PILOT_SPD_DN=1.0` m/s.
- `PILOT_SPD_UP=1.5` m/s.
- `LAND_SPEED=50` cm/s.
- `MOT_THST_HOVER` last snapshot: approximately 0.2443.
- `MOT_SPIN_ARM=0.15`, `MOT_SPIN_MIN=0.18`, `MOT_SPIN_MAX=0.95`.

Power monitoring:

- `BATT_MONITOR=4`.
- `BATT_CAPACITY=22000` mAh.
- `BATT_ARM_VOLT=23.1` V.
- `BATT_LOW_VOLT=21.0` V, with a 10 s persistence timer.
- `BATT_CRT_VOLT=20.4` V, below the low threshold as required.
- `BATT_FS_LOW_ACT=4` (SmartRTL or Land) and `BATT_FS_CRT_ACT=1` (Land).
- `BATT_LOW_MAH=0` and `BATT_CRT_MAH=0`; capacity failsafes are disabled.
- Voltage pin 14, current pin 15 in the current Cube setup.
- Current and voltage scaling are aircraft-specific and must be checked against
  a wattmeter. GCS percentage is not primary evidence until scaling and
  capacity estimation are validated.

See `docs/POWER_AND_ESC.md` before propulsion changes or field progression.

## Coordinate Frames

Unless explicitly stated otherwise, all rigid-body positions and rotations use
body FRD:

- X forward.
- Y right.
- Z down.

Consequences:

- A negative mount Z is above the CG.
- A positive mount Z is below the CG.
- Horizontal bearing zero is forward.
- Positive horizontal bearing rotates toward aircraft right.

Other frames:

- Cube local flight position: NED.
- Cube body velocity/attitude contracts: FRD where specified by MAVLink.
- Browser world and launch-local trajectory: FLU for Three.js display.
- FAST-LIO backend frame follows its resolved ROS configuration and is
  explicitly transformed before comparison or Cube transport.
- Copter local velocity targets use local NED after applying launch-frame yaw.

Never change a frame conversion without updating `docs/INTERFACES.md`, adding a
test, and checking all signs with a known physical movement.

## Mechanical Sensor Geometry

All positions below are relative to the aircraft CG in body FRD.

| Device | Position | Orientation and notes |
| --- | --- | --- |
| Cube Orange+ | `(+0.08, 0.00, -0.08) m` | Mounted Yaw270; `AHRS_ORIENTATION=6` |
| Holybro H-Flow | `(0.00, 0.00, +0.10) m` | Downward on CAN2; connectors-rear yaw assumption |
| H-Flow range | `(0.00, 0.00, +0.10) m` | Downward range paired with flow |
| Hesai JT16 | `(0.00, 0.00, -0.10) m` | Centered, 0.10 m above CG, body rotation yaw 180 deg |
| Intel D415 | `(+0.19, 0.00, +0.10) m` | Forward, 0.19 m ahead and 0.10 m below CG |
| IM10A | `(+0.08, 0.00, -0.09) m` | Directly above Cube IMU, approximately 0.01 m higher |

Calibration flags:

- Camera intrinsics: verified.
- Camera-to-body extrinsics: not verified for control.
- IMU-to-body extrinsics: verified from physical position and dynamic axis
  correlation.
- JT16-to-body extrinsics: verified from cardinal targets.
- IMU noise profile: not verified.
- End-to-end sensor time synchronization: not verified.
- JT16 unit correction: verified.
- JT16 live baud: verified.
- Airframe geometry: verified from measured dimensions.

## Cube Hardware and UART

Flight controller: Cube Orange+ running ArduCopter 4.7.1 with EKF3 enabled.

Jetson TELEM2 wiring uses only 3.3 V signals and ground:

- Jetson physical pin 6: GND.
- Jetson physical pin 8: TX to Cube RX.
- Jetson physical pin 10: RX from Cube TX.
- Do not connect a TELEM power pin to Jetson GPIO power.

Current serial contract:

- TELEM1/QGC: `SERIAL1_PROTOCOL=2`, `SERIAL1_BAUD=57` (57,600 baud).
- TELEM2/Jetson: `SERIAL2_PROTOCOL=2`, `SERIAL2_BAUD=115` (115,200 baud).
- Project endpoint: `/dev/ttyTHS1`.
- MAVLink2 is required for extended ODOMETRY and proximity fields.
- USB MAVLink is not required by the active runtime.

UART history and decision:

- 921600 passed some short messages but lost long 179-byte proximity frames.
- 115200 saturated under broad high-rate telemetry requests.
- 460800 passed a 15/15 bidirectional disarmed parameter audit.
- A small standalone sender held proximity health when the combined runtime
  did not.
- The active path therefore uses direct UART, minimal incoming requests, short
  eight-face messages, latest-only data, and paced writes.
- `mavlink_router.py` and its systemd unit are experimental/inactive. Do not
  silently re-enable them.

Use the process lock at `runtime/locks/cube_mavlink.lock`. A second project
reader must fail rather than compete. Stop the active service before direct
MAVProxy or a live-UART diagnostic.

## Cube EKF and Flight Configuration

Core values:

- `AHRS_EKF_TYPE=3`.
- `EK3_ENABLE=1`.
- `AHRS_ORIENTATION=6`.
- `ARMING_SKIPCHK=0` (all configured checks retained).
- `GUID_OPTIONS=64` so Guided XY position/velocity stabilization remains
  enabled.
- `GUID_TIMEOUT=0.5` s, verified 2026-08-01.
- `TERRAIN_ENABLE=0`.

EKF source sets from the current parameter snapshot:

- Source set 1: GPS POSXY, POSZ barometer, VELXY optical flow, VELZ none,
  compass yaw. RTCM-corrected Here4 position is used when available.
- Source set 2: POSXY none, POSZ barometer, VELXY optical flow, VELZ none,
  compass yaw.
- Source set 3: GPS POSXY, barometer POSZ, GPS VELXY, VELZ none, compass yaw.
- No source set selects ExternalNav.
- Automatic EKF source switching from this project is disabled.

RTK correction contract:

- The MK15 USB Holybro unit is the stationary base and QGroundControl forwards
  its RTCM stream over MAVLink.
- The Here4 rover is `GPS1_TYPE=9` on DroneCAN node 125; `GPS2_TYPE=0` and
  `GPS_INJECT_TO=127` sends received corrections to all configured GPS units.
- `GPS_DRV_OPTIONS=0` is intentional; extended RTCM parsing is diagnostic and
  is not required for normal correction forwarding.
- RTK loss degrades GPS accuracy but does not remove H-Flow velocity aiding.
- RC9 option 65 is the explicit GPS-denial control. No Lua source switch is
  permitted without a separate props-off and flight commissioning cycle.

H-Flow and range:

- `FLOW_TYPE=6`.
- `FLOW_POS_Z=+0.10` m.
- `FLOW_ORIENT_YAW=0` in the current snapshot.
- `RNGFND1_TYPE=24`.
- `RNGFND1_ORIENT=25` downward.
- `RNGFND1_MIN_CM=8`, `RNGFND1_MAX_CM=3000`.
- `RNGFND1_POS_Z=+0.10` m.

Flight-mode snapshot:

- `FLTMODE_CH=5`.
- Slots 1 and 2: mode id 5.
- Slots 3 and 4: mode id 16.
- Slots 5 and 6: mode id 6.

Do not infer a safe field switch layout from numeric ids alone. Confirm the GCS
mode label, transmitter switch, active EKF source set, and pilot intent before
flight. `GUIDED_NOGPS` remains forbidden. Standard RTL may only enter the
staged Cube-owned optical-flow experiment after Home, Origin, local position,
range, flow, yaw, speed limits, and abort behavior have all passed readback and
short-range commissioning.

## RC Channel Contract

- RC5: Cube flight-mode selection.
- RC7: proximity avoidance, `RC7_OPTION=40`.
- RC8: FlowHold, `RC8_OPTION=71`.
- RC9: GPS Disable test switch, `RC9_OPTION=65`.
- RC10: Cube Land, `RC10_OPTION=18`.
- RC11: motor emergency stop, `RC11_OPTION=31`.
- RC12: reserved for the locked future companion-return request and currently
  unassigned in Cube.
- RC7 project thresholds: engage at or above 1700 us, disengage at or below
  1300 us.
- RC12 future-return thresholds: same low/high values.

ArduCopter 4.6.3 cannot map `FS_EKF_ACTION` directly to FlowHold. For deliberate
GPS denial, RC8 must select and confirm FlowHold before RC9 is raised; RC9 low
restores GPS. Jetson remains PRX-only and does not automate this mode change.

The locked return controller requires RC12 to be observed low after arm and
then moved high deliberately. RC12 low cancels immediately. A flight-mode
change blocks output. The Jetson does not select Land; RC10 remains
pilot/Cube-owned.

## Holybro H-Flow Contract

- Transport: DroneCAN on Cube CAN2.
- Mount: downward, 0.10 m below CG.
- Optical flow supplies local horizontal velocity, not absolute position.
- Downward range scales optical flow and supports altitude-dependent validity.
- Flow can drift or fail with poor texture, low light, repeating surfaces,
  excess height, motion blur, vibration, bad calibration, or range failure.
- FlowHold may maintain useful local XY without GPS, but it is not globally
  anchored and may drift like AltHold over time.
- Position-hold naming such as "GPS assist" must not be interpreted as
  GPS-denied global position.

Flight progression must verify flow quality, range freshness, pilot authority,
and mode behavior with GPS still available before any GPS denial.

Flow scale rules:

- Stationary data can audit rate, quality, false velocity, and range stability;
  it cannot identify `FLOW_FXSCALER` or `FLOW_FYSCALER`.
- Exact offline fitting uses the Cube's official `OFCA` samples produced by an
  activated ArduPilot FlowCal run, not reconstructed normal-flight `OF` data.
- `./optflow hflow-log-cal` requires two independent logs, 50 official samples
  per axis in each log, valid ArduPilot fit limits, and no more than 15 percent
  disagreement before it emits the average.
- Logs 163 and 164 from 2026-08-04 contain zero `OFCA` samples. Keep both flow
  scalers at zero until two valid official runs exist.

## JT16 Contract

- Model: Hesai JT16.
- Transport: serial RS485 through PL2303 adapter.
- Stable symlink: `/dev/jt16_usb`.
- USB VID/PID: `0x067B:0x23A3`.
- Unit serial identifier: `DCCEb114J19`.
- Active baud: 3,000,000.
- Legacy baud retained for recovery: 3,125,000.
- Official SDK revision:
  `534c707846a810e8211b93446f878dbf415f7000`.
- Native bridge: `build/jt16_bridge/optflow-jt16-bridge`.
- Correction file: `data/calibrations/jt16/JT16_sample_angle-2.csv`.
- Correction and cardinal orientation are verified.
- Native SDK XYZ uses +Y at zero azimuth, +X right, +Z up.
- Conversion to lidar-forward FRD is `(Y, X, -Z)` before mount rotation and
  CG translation.
- Final configured body rotation is roll 0, pitch 0, yaw 180 degrees.

The C++ bridge emits a binary header plus packed point records. It can preserve
raw serial packets for evidence, but the OA-only service intentionally does not
record raw data continuously.

## D415 Contract

- Model: Intel RealSense D415.
- Stable selected serial: `327322062285`.
- Configured stream: 640 x 480 at 30 FPS.
- RGB HTTP port: 8770.
- Mount: forward, +0.19 m X, +0.10 m Z-down.
- Intrinsics and depth scale passed measured-wall checks at 1.50 m and 2.50 m.
- Final camera-to-body rotation is not control-verified.
- Outdoor depth was intermittent in the 2026-08-03 flight evidence.
- D415 is excluded from active Cube proximity.
- D415 remains available for RGB streaming, point clouds, mapping diagnostics,
  and shadow RGB-D odometry.

Invalid depth must remain unknown. Do not turn zero/invalid outdoor depth into
clear space for flight control.

## IM10A Contract

- Model: Hiwonder IM10A.
- Stable symlink: `/dev/imu_usb`.
- USB VID/PID: `0x1A86:0x7523`.
- Current validated profile: 230400 baud, approximately 200 Hz.
- Enabled records: sensor TIME, acceleration, and angular velocity.
- Body sign map: X `+1`, Y `-1`, Z `-1`.
- Axis map was dynamically verified against Cube gyro motion.
- Position: +0.08 m X, 0 Y, -0.09 m Z.
- Sensor time is a relative device clock, not UTC.
- A 2026-07-31 audit measured approximately 199.84 Hz with no observed
  checksum, payload, ordering, or drop errors.

Historical note: the sensor originally ran the factory 9600-baud, 10 Hz WIT
stream. Do not revert or change baud/rate casually. Any configuration change
must use the reversible configurator with automatic baud recovery and must
re-test rate, jitter, checksum, ordering, and drops.

Noise covariance remains unapproved until the stationary noise procedure
passes. Do not tune estimator covariance from a short hand-held sample.

## Active Obstacle-Avoidance-Only Runtime

Systemd unit:

```text
optflow-flight-logger.service
  -> scripts/run_obstacle_avoidance_service.sh
  -> python -m optflow_slam.obstacle_avoidance_service
  -> native JT16 bridge
```

Active resources:

- Cube `/dev/ttyTHS1`.
- JT16 `/dev/jt16_usb`.

Explicitly inactive resources/features:

- D415.
- IM10A.
- ROS 2 and FAST-LIO.
- RGB-D odometry.
- SLAM map and trajectory.
- Navigation and return controller.
- ExternalNav/ODOMETRY output.
- Companion movement commands.

Status file: `runtime/obstacle_avoidance_status.json`.

The service requests only RC channels at 5 Hz and `SYS_STATUS` at 2 Hz, plus
normal heartbeats. It does not request all streams or high-rate raw IMU.

### OA Point Filtering

From `config/system.yaml`:

- Stage `active`.
- MAVLink output true.
- JT16 input true.
- D415 input false.
- Sector increment 5 degrees, 72 internal sectors.
- Horizontal distance 0.30 m to 8.00 m.
- Body vertical slice -0.40 m to +0.40 m around CG.
- Protected self-filter radius: max of 0.30 m and 0.75 m, therefore 0.75 m.
- Minimum points per sector: 8.
- Distance statistic: 10th percentile.
- Temporal window: 3 scans.
- Temporal selection: conservative minimum.
- Source stale timeout: 0.45 s.
- Target transport cycle: 10 Hz.

### MAVLink Proximity Transport

- Internal 72-sector scan is reduced to eight 45-degree faces.
- Each face uses the minimum known five-degree sector.
- A fresh healthy face with no in-range return sends 801 cm, which is
  `max_distance + 1` for the configured 800 cm sensor range.
- All eight `DISTANCE_SENSOR` packets are sent per cycle.
- Face id and orientation both use values 0 through 7.
- Face zero is forward; values increase clockwise by 45 degrees.
- Packet gap is 12 ms to prevent a serial burst.
- Latest-only scan scheduling prevents stale queue buildup.
- A stale JT16 scan stops transport instead of replaying old clear/obstacle
  data.
- After an 8 s startup grace, a missing or more-than-2 s stale JT16 point
  stream terminates the process so systemd restarts the complete OA runtime.
- ArduPilot's MAVLink proximity timeout is 0.5 s.
- Health monitoring uses `MAV_SYS_STATUS_SENSOR_PROXIMITY`, not the laser
  rangefinder bit.

Earlier bugs and their causes:

- Full `OBSTACLE_DISTANCE` frames were lost at some link rates.
- The combined runtime requested enough telemetry to create queue/scheduling
  pressure and intermittent PRX expiry.
- A tuple passed to `pymavlink.recv_match(type=...)` was treated as one invalid
  type and discarded post-connect heartbeats; it now uses a list.
- Fresh JT16 scans with zero accepted sectors originally sent zero packets,
  causing `PRX1: No Data`; they now send healthy no-return faces.
- Initial status watched `MAV_SYS_STATUS_SENSOR_LASER_POSITION`; it now watches
  the real proximity bit.

Never "fix" PRX by setting `PRX1_TYPE=0`, disabling avoidance, or weakening
arming checks. Fix data freshness and verify direction/distance.

### Native Cube Avoidance

Current desired/read-back parameters:

- `RC7_OPTION=40`.
- `PRX1_TYPE=2`.
- `AVOID_ENABLE=7`.
- `OA_TYPE=1` for BendyRuler.
- `OA_BR_LOOKAHEAD=4.00` m.
- `OA_MARGIN_MAX=1.50` m.
- `OA_DB_SIZE=100`, `OA_DB_EXPIRE=5`, `OA_DB_QUEUE_SIZE=80`.
- `AVOID_MARGIN=1.50` m.
- `AVOID_DIST_MAX=1.50` m.
- `AVOID_BEHAVE=0`.
- `AVOID_BACKUP_SPD=0.50` m/s.
- `AVOID_ACCEL_MAX=1.00` m/s2.
- `WPNAV_SPEED=50`, `WPNAV_SPEED_UP=50`, and `WPNAV_SPEED_DN=50` cm/s.
- `RTL_SPEED=50`, `LAND_SPEED=50`, and `LAND_SPEED_HIGH=50` cm/s.

The parameter tool first writes `AVOID_ENABLE=0`, applies other values, then
restores the configured enable mask and verifies readback. It is disarmed-only.
When changing `OA_TYPE` from zero, the first apply initializes BendyRuler and
returns status 3. Changing the database or queue allocation also returns status
3. Reboot the Cube and run the apply again to verify the active planner and
database allocation.

Reactive avoidance remains the pilot-mode short-range layer. BendyRuler adds a
local path planner in `AUTO`, regular `GUIDED`, and `RTL`; it can route around
simple obstacles but is not global A*, Dijkstra, SLAM relocalization, or a
shortest-path guarantee. It may fail in concave traps or with incomplete
vertical coverage. The 1.50 m boundary remains a target, not a physical
guarantee; braking distance, mode behavior, latency, mass, and approach speed
still matter.

### OA Audio

- Startup: one rising tune after the first successful Cube heartbeat.
- No companion arming tune.
- No companion mode-change tune.
- 1.50 to 2.00 m: 1 beep/s, warning only.
- 1.25 to 1.50 m: 3 beeps/s.
- Below 1.25 m: linear rise toward 10 beeps/s at 0.75 m.
- Beeps require fresh data, RC7 high, and an armed aircraft because
  `only_when_armed=true`.
- RC7 low or unknown, disarmed, stale, or clear states are silent.

## Runtime Ownership and Services

Exactly one process owns each hardware endpoint.

| Runtime/tool | Cube UART | IM10A | D415 | JT16 | Control output |
| --- | --- | --- | --- | --- | --- |
| OA-only boot service | Owns | No | No | Owns | Proximity only |
| Full hardware visualizer | Owns | Owns | Optional | Optional | Diagnostic/tunes; no movement |
| Passive flight service | Owns | Owns | Armed capture | Armed capture | None |
| LIO shadow | Reference owner | Owns | Optional by mode | Owns | None unless explicit disarmed ODOMETRY shadow |
| SLAM proof | Reference owner | Owns | Owns | Owns | Ready tune only |
| SLAM navigation runtime | Owns | Owns | Owns | Owns | Locked/monitor-only today |
| Camera stream | No | No | Owns | No | None |
| `obstacle-status` | Cache only | No | No | No | None |
| Trajectory monitor | Cache/stream only | No | No | No | None |

Use `RuntimeResourceLock` for Cube and JT16 ownership. Existing direct readers
must honor the lock. Do not weaken the lock because a tool is "read-only";
serial reads still consume bytes from the owner.

Boot persistence uses a per-user systemd service and user lingering. The
installed unit is a symlink to the repository-owned service definition.

Useful operations:

```bash
systemctl --user restart optflow-flight-logger.service
systemctl --user status optflow-flight-logger.service --no-pager --full
systemctl --user is-enabled optflow-flight-logger.service
./optflow obstacle-status
```

## FAST-LIO2 Shadow Runtime

The LIO stack is project-local and does not depend on `/opt/ros` as a mutable
system workspace.

Pinned components are in `third_party/lio.lock.json`:

- ROS 2 Humble binary archive `release-humble-20260220` with checked SHA256.
- PCL revision `e8ed4be802f7d0b1acff2f8b01d7c5f381190e05`.
- `pcl_msgs` revision `8a925a7c4626df52dba7ccc5bda5900d63678880`.
- `pcl_conversions` revision
  `67a5c2ba4c4de3ca21c5cd495812a01ced3fb69a`.
- Hesai FAST-LIO2 revision
  `bb2842d34990761eebbd4cc3188e94c7c662a673`.
- ikd-Tree revision `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`.
- Local user-runtime patch:
  `third_party/patches/fast_lio_hesai-user-runtime.patch`.

Build root: `runtime/lio/`.

ROS contract:

- JT16 topic `/optflow/jt16/points`.
- IM10A topic `/optflow/im10a/imu`.
- Odometry topic `/Odometry`.
- Diagnostics topic `/optflow/lio/diagnostics`.
- Map output false by default.
- Pose output to Cube false.
- Disarmed ODOMETRY shadow transport true as an explicit bench-only path.

### Time Synchronization

- Common reference: Jetson monotonic time.
- IM10A sensor time is mapped affinely into host monotonic time.
- JT16 point/frame time is mapped separately.
- Window sample count: 2000.
- Maximum IMU fit span: 20 s.
- Maximum lidar fit span: 60 s.
- Minimum IMU samples: 200.
- Minimum lidar samples: 20.
- Minimum fitted span: 2 s.
- Current lidar-to-IMU offset: +0.010 s.
- Maximum allowed drift: 5000 ppm during current bring-up.
- Maximum IMU residual p95: 5 ms.
- Maximum lidar residual p95: 15 ms.

Five dynamic sessions supported the +10 ms offset. A matched yaw run improved
Cube/LIO attitude p95 from approximately 2.84 deg at zero offset to 2.27 deg.
Global sensor-time verification remains false because D415 alignment is not
closed.

### LIO Validation Gates

Current approval is false. Required report limits:

- Minimum duration: 60 s.
- Minimum odometry rate: 4 Hz.
- Stationary window: 5 s.
- Maximum stationary drift: 0.15 m.
- Maximum return-to-start error: 0.35 m.
- Maximum position jump: 0.50 m.
- Maximum speed: 3.0 m/s.
- Maximum attitude jump: 10 deg.
- Maximum clock resets: zero.
- Minimum Cube reference samples: 100.
- Minimum Cube reference path: 2.0 m.
- Maximum Cube horizontal RMSE: 0.50 m.
- Maximum Cube vertical RMSE: 0.40 m.
- Maximum Cube attitude p95: 10 deg.
- Cube/reference path ratio: 0.70 through 1.30.

Cube local position is an independent comparison signal, not ground truth and
never an input to FAST-LIO.

Do not set `validation.approved=true` manually. Approval requires a generated
passing report and matching digest.

## RGB-D Odometry and SLAM Proof

`rgbd_odometry.py` performs metric D415 RGB-D odometry with a time-matched
IM10A gyro rotation prior. It runs beside FAST-LIO in proof and shadow modes.

Known characteristics:

- D415 input is approximately 30 FPS.
- Dense RGB-D odometry measured around 0.87 Hz on the flight Jetson.
- Navigation visual freshness timeout is therefore 2.0 s.
- Visual/LIO disagreement limit is 0.35 m.
- RGB-D is an independent consistency source, not the fast primary estimator.

`slam-poc` is the shortest visible proof:

- It can guide a props-off carry sequence.
- It flashes the browser twice before each new movement instruction.
- It records LIO and RGB-D trajectories and maps.
- It captures a shadow local-return proposal.
- It sends no pose, mode, arm, navigation, or movement command.
- Its only normal Cube output is a one-shot ready tune.
- Default browser: `http://<jetson>:8767/slam-poc`.

A good visual trail is not sufficient. Reports must score timing, drift,
return-to-start, jumps, source agreement, and Cube reference behavior.

Current estimator remains local odometry/mapping rather than proven
loop-closed/relocalizing SLAM. Do not claim persistent global mapping.

## Cube ODOMETRY Shadow Boundary

`cube-odom-shadow` is the only explicit estimator-to-Cube bench transport path
currently enabled in config.

It must:

- Run disarmed with props removed.
- Audit `AHRS_EKF_TYPE`, `EK3_ENABLE`, and all source axes in all three EKF
  source sets.
- Refuse to send if any Cube source selects ExternalNav.
- Stop immediately if Cube arms or heartbeat becomes stale.
- Send MAVLink2 ODOMETRY with local-FRD/body-FRD contracts.
- Write no Cube parameter.
- Cause no EKF fusion.

Historical bench proof on 2026-07-31 sent 112 packets near 4.999 Hz with zero
parameter writes while ExternalNav was absent. Re-verify before relying on it.

## Mapping and Planning Contract

Version 1 architecture is fixed-altitude 2.5D:

1. Build a local occupancy/cost map from JT16 geometry and validated D415
   depth.
2. Use A* for global route selection.
3. Use a proven collision-aware local planner to validate and adjust immediate
   motion.
4. Send conservative local velocity targets through regular Guided only.
5. Retain launch pose and breadcrumbs for local return.

Current config:

- Architecture `fixed_altitude_2_5d_first`.
- Global planner `astar`.
- Local planner `proven_collision_aware_library_tbd`.
- Autonomous control false.
- ExternalNav output false.
- Target rate 10 Hz.
- Initial horizontal speed 0.5 m/s for general navigation design.
- Initial vertical speed 0.3 m/s.
- Initial yaw rate 20 deg/s.
- Local pose stale timeout 0.45 s.
- Command stale timeout 0.50 s.

Do not hand-roll the collision-aware local planner when a proven ROS-compatible
library can meet the contract. A* does not consume raw camera frames directly;
it consumes a validated map/costmap.

The current OA-only runtime is not this planner. It provides reactive Cube
proximity only.

## Cube-Owned Optical-Flow Mission Contract

This is the current shortest point-to-point and return experiment. It is
separate from SLAM Return:

- Jetson publishes fresh JT16 proximity only.
- Cube EKF3 integrates H-Flow velocity and range into drift-limited local
  position.
- Cube owns regular `GUIDED`, `AUTO`, BendyRuler, and standard `RTL` targets.
- `GUIDED_NOGPS` remains forbidden because attitude targets cannot guarantee
  the 0.50 m/s ground-speed ceiling.
- Jetson does not send pose, velocity, waypoints, mode changes, arm, land, or
  motor commands in this profile.

Required prepared parameters are the Native Cube Avoidance list above. Mission
items must omit `DO_CHANGE_SPEED` above 0.50 m/s because a mission speed command
can override `WPNAV_SPEED`. Terrain-relative mission altitude is forbidden
while terrain support is disabled.

Every autonomous run requires all of the following continuously:

- Active source set 1 using optical flow for horizontal velocity. GPS/RTK
  horizontal position is allowed normally and must be explicitly disabled with
  RC9 for a GPS-denied run.
- Healthy optical flow, downward range, attitude, compass yaw, local position,
  battery, RC, and Cube controllers.
- A real EKF Origin and Home that match the launch site.
- Fresh JT16 proximity and PRX health before RC7 is raised.
- Exact BendyRuler and 0.50 m/s parameter readback.
- Active obstacle database with `OA_DB_SIZE=100`, `OA_DB_EXPIRE=5`, and
  `OA_DB_QUEUE_SIZE=80`; zero database size disables path rerouting.
- A tested pilot cancellation into the currently stable altitude-controlled
  mode. Never cancel into `STABILIZE`.

Source-set parameter values do not prove which source set is active. No new RC
EKF-source switch is assigned by this change. An `RCx_OPTION=90` assignment
requires an observed spare transmitter channel and an explicit channel-contract
update; do not overwrite RC7, RC8, RC9, RC10, RC11, or RC12.

Commission in this order with GPS hardware connected: 1 to 2 m regular-Guided
move without an obstacle, 3 to 5 m AUTO route without an obstacle, BendyRuler
route with one lightweight obstacle and RC7 high, then short RTL while GPS is
excluded from active horizontal fusion. Require three clean logged repetitions
before intentional denial. For a GPS-free cold boot on ArduCopter 4.6.3, the
operator must set and verify a real Origin and Home in the GCS before arming.

## Guarded Local Return Contract

`SLAM RETURN` is a future Jetson local-map breadcrumb return. It remains
separate from the Cube-owned optical-flow RTL experiment above.

Intended behavior:

1. Capture launch pose after estimator initialization.
2. Record outbound breadcrumbs in the same map revision.
3. On deliberate RC12 request in regular Guided, traverse collision-checked
   breadcrumbs in reverse.
4. Send bounded velocity-only `SET_POSITION_TARGET_LOCAL_NED` targets.
5. Stop and release authority on any stale or failed gate.

Current config:

- Stage `locked`.
- Live control false.
- Approval file `runtime/approvals/slam_return_live.json`.
- Status file `runtime/slam_navigation_status.json`.
- Required mode `GUIDED`.
- RC request channel 12.
- Pilot Land channel 10.
- Engage 1700 us, disengage 1300 us.
- EKF source set 1.
- Altitude 1.0 m through 8.0 m.
- Minimum flow quality 50.
- Telemetry stale timeout 0.35 s.
- Battery stale timeout 2.0 s.
- Minimum voltage 22.20 V.
- Command rate 10 Hz.
- Commissioning horizontal speed 0.30 m/s.
- Absolute loader-enforced speed ceiling 0.75 m/s.
- Horizontal acceleration 0.50 m/s2.
- Arrival radius 0.20 m.
- Breadcrumb spacing 0.15 m.
- Waypoint radius 0.12 m.
- Visual stale timeout 2.0 s.
- Visual disagreement limit 0.35 m.

Output gates include:

- Fresh Cube heartbeat.
- Disarmed/armed lifecycle correctly observed.
- Required regular Guided mode.
- RC12 low observed after arm and deliberate high request.
- Fresh RC, local position, optical flow, range, battery, LIO, RGB-D, obstacle,
  and command state.
- Valid EKF origin.
- Exact Cube parameters.
- Approved physical calibrations.
- Approved LIO report with matching revision/digest.
- Digest-bound live approval file.
- Collision-free reverse breadcrumb path.
- No 1.50 m clearance breach.

Any failure latches live return off for the arm cycle and commands a
zero-velocity handoff before authority release where a command path exists.
This version stops for unexpected obstacles; it does not flank or replan.

The project must never automatically switch to Stabilize. Pilot takeover uses
the already-tested altitude-controlled mode or manual landing strategy.

## GPS-Denied Origin and Return Reality

- Optical flow plus range can support local drift-limited hold without GPS.
- It does not provide a geographic home.
- Standard RTL can be experimentally driven by the Cube's optical-flow local
  position only while a valid real EKF Origin and Home exist and the complete
  estimator remains healthy. It is not made reliable by setting Home once.
- BendyRuler consumes position plus proximity; it does not repair localization
  drift, create an origin, or guarantee recovery after a flow/range/yaw fault.
- Local return needs launch pose and current pose in the same live map.
- A cold boot without GPS may lack a usable EKF local origin.
- The operator may explicitly set a valid origin in the GCS, or the system may
  initialize while GPS is available before denial.
- Software must never silently create fake latitude/longitude.
- If localization is lost, correct responses are hold, pilot takeover, or
  controlled landing, not blind dead reckoning.

## Flight Evidence Logger

The active OA service owns the integrated Desktop sensor recorder. Sessions are
written under `~/Desktop/Drone_Sensor_Recordings` using local time plus one
root-wide increasing index.

Current arm-triggered behavior:

- `./optflow obstacle-status` must report `RECORDING_THREAD_ALIVE=true`,
  `AUTO_RECORD_ON_ARM=true`, and no recording error before field work.
- Recorder-thread death fails the complete OA runtime so systemd restarts it;
  OA must not remain apparently ready while sensor recording is unavailable.
- A manually stopped user service stays stopped until an explicit start or a
  reboot even when the unit is enabled.
- The first disarmed-to-armed transition creates one new session; repeated
  armed heartbeats do not create duplicates.
- Disarm retains a three-second tail before finalizing the CSV and MP4 files.
- Manual Start/Stop commands use the same recorder and settings file.

Session evidence can include:

- Unified human-readable sensor CSV.
- Cube MAVLink CSV.
- IM10A CSV.
- D415 depth-summary CSV and selected annotated MP4 mode.
- JT16 sector-summary CSV.
- Session manifest with source errors and finalization state.

This Jetson evidence is independent of Cube DataFlash logging. If the service
is stopped or misses an armed transition, Cube BIN files cannot reconstruct
the absent camera, IM10A, or raw JT16 recording afterward.

DataFlash interpretation must inspect vibration, clipping, attitude, desired
versus achieved rates, motor outputs, thrust loss flags, voltage/current,
failsafes, mode transitions, EKF, optical flow, range, and crash endpoint. Do
not infer a single root cause from one symptom.

## Visualizer Contract

Main visualizer default: port 8765.

- Primary view always shows the drone using Cube attitude/IMU.
- Small corner view shows external IM10A motion.
- IM10A display uses measured X/-Y/-Z body mapping and startup reference
  alignment.
- Browser transport targets 60 Hz; source rates remain visible.
- 3D SCAN shows measured drone geometry, rolling D415/JT16 clouds, Cube motion,
  trajectory layers, and protected radius.
- 3D SCAN is diagnostic; a route line is not proof of SLAM.
- In the spatial UI, trajectory is on by default, JT16 is off by default, and
  D415/JT16 are independently toggleable.
- LIO, RGB-D, Cube, breadcrumbs, and return target are independently
  toggleable.
- Rejected raw LIO points may remain visible for diagnosis but must never feed
  accepted navigation state.
- A red SLAM fault freezes the guarded navigation path.

Other ports:

- LIO assist: 8766, `/lio-assist`.
- SLAM proof: 8767, `/slam-poc`.
- D415 RGB: 8770.

Use `--host 0.0.0.0 --no-browser` for LAN access. Another device on the same
network opens `http://<jetson-ip>:<port>`.

Headless VNC is project-owned and should start at boot after installation.
Verify headless operation without a physical HDMI display before enclosing the
Jetson. See `docs/HEADLESS_ACCESS.md`.

## Audio Contract

Useful project tones are retained:

- OA service rising initialization tone.
- Obstacle-distance warning tones.
- LIO/SLAM proof ready tone.
- User-intervention prompt tones in guided calibration/proof sequences.

Do not add companion arming or generic mode-change tones because ArduPilot
already owns those audible events. Avoid overlapping or indistinguishable tune
sequences.

## Data and Artifact Policy

Project boundary:

- `.venv/`: project Python runtime.
- `vendor/python/`: offline aarch64 wheel mirror.
- `visualizer/node_modules/` and `visualizer/dist/`: generated frontend state.
- `data/calibrations/`: measured calibration artifacts.
- `data/logs/`: application logs.
- `data/maps/`: versioned local maps and metadata.
- `data/recordings/`: sessions and reports.
- `data/flight_logs/`: imported raw Cube evidence; local and Git-ignored.
- `runtime/`: transient status, locks, point frames, and local LIO build.
- `ros_ws/`: only ROS workspace source boundary.
- `third_party/`: pinned external sources/revisions and patches.
- `hardware/`: host configuration source of truth.

Do not import code, configuration, maps, logs, binaries, or Python environments
from sibling workspaces. `tests/test_project_boundary.py` enforces this.

Git must not contain generated flight logs, tlogs, raw recordings, maps,
runtime status, local ROS builds, frontend builds, node modules, or fetched
third-party source trees. Small reproducible calibration/configuration files,
the pinned wheel mirror, native source, tests, and parameter reference snapshot
belong in source control.

The 2026-09-26 ignore policy keeps small VIO session reports/manifests and
candidates, but excludes images, raw timestamp streams, generated targets and
Kalibr duplicates. Raw noise/point-cloud arrays and bridge logs were untracked
without deleting local copies. Diagnostic BIN/partial/log streams and firmware
binaries are ignored; parameter snapshots, small D415 calibration tables,
curated images and the measured JT16 correction CSV remain versioned. Keep
this distinction; do not blanket-ignore every image, CSV or calibration file.

`mav.parm` is a reference snapshot, not an automatic install profile. Never
apply it wholesale to another aircraft.

## Command Safety Classification

Cache/read-only beside active service:

```bash
./optflow obstacle-status
./optflow flight-status
./optflow slam-return-status
```

Offline and no flight hardware:

```bash
./optflow build
./optflow test
./optflow analyze <session> --cube-log <BIN>
./optflow slam-timing <session>
./optflow rtl-shadow <session>
./optflow lio-validate <session>
./optflow hflow-log-cal <flowcal-a.BIN> <flowcal-b.BIN>
```

Hardware-owning, stop active service first:

```bash
./optflow preflight --profile fc_bench
./optflow sensor-check
./optflow obstacle-check
./optflow hflow-audit --seconds 10
./optflow lidar-status
./optflow visualizer --host 0.0.0.0 --no-browser
./optflow flight-service
./optflow lio-shadow
./optflow slam-poc
```

Disarmed parameter tools:

```bash
./optflow cube-mount
./optflow cube-avoidance
./optflow slam-return-setup
./optflow im10a
```

An `--apply` flag changes the safety category. Confirm Cube disarmed, props
removed, current readback, intended scope, and backup behavior before use.

Host-changing tools may require sudo:

```bash
sudo ./optflow install-ch341
sudo ./optflow install-pl2303
sudo ./optflow install-realsense-rules
sudo ./optflow install-headless-vnc
```

Do not ask for or handle a password unless the current operation actually
needs sudo. Let the terminal prompt the user.

## Staged Bring-Up Order

1. Mechanical frame, CG, battery restraint, wiring, propellers, motors, ESCs,
   and power-module acceptance.
2. Props-off Cube orientation, accelerometer, compass, RC, emergency stop,
   battery, UART, H-Flow, range, and PRX checks.
3. GPS-connected manual flight with OA off in the already-tested
   altitude-controlled mode.
4. GPS-connected low-speed Loiter and FlowHold reactive-OA tests, RC7
   deliberate.
5. Props-off two-stage BendyRuler/speed apply, Cube reboot, exact readback, and
   fresh PRX watchdog recovery test.
6. Keep GPS connected while commissioning a 1 to 2 m regular-Guided move and a
   3 to 5 m AUTO route with optical flow as active horizontal source.
7. Add one lightweight obstacle and commission BendyRuler with RC7 high.
8. Commission short Cube RTL with valid Origin/Home and GPS excluded from
   active horizontal fusion; require three clean logged repetitions.
9. Only then test intentional GPS denial while retaining pilot authority and a
   verified GCS Origin/Home procedure.
10. In parallel, continue passive LIO/VIO/OA shadow flights, trajectory
    validation, and future SLAM return work with zero Jetson movement output.

For every field run:

- One-shot plan before leaving Jetson access.
- Services autostart and write QGC status where required.
- Keep a clear area and a second person when practical.
- Approach test obstacles at no more than the procedure's limit.
- Abort on wrong sector, unexpected lean, oscillation, stale proximity,
  estimator fault, loss of telemetry, loss of pilot authority, motor
  saturation, abnormal sound, or excessive heat.
- Do not switch to Stabilize during abort.
- Review Cube and Jetson evidence before the next attempt.

## Known Gaps and Blockers

- Last volatile pre-arm errors included RC not found and compasses
  inconsistent.
- D415 body extrinsics are not control-verified.
- Outdoor D415 depth reliability is not approved.
- IM10A stationary noise profile is not approved.
- End-to-end D415/JT16/IM10A clock synchronization is not approved.
- LIO validation approval is false.
- FAST-LIO remains local odometry without proven loop closure or
  relocalization.
- Persistent maps and cross-flight localization are not approved.
- ExternalNav-to-Cube fusion is disabled.
- Live SLAM return is locked.
- Collision-aware local planner is TBD.
- Reactive OA alone does not flank obstacles; BendyRuler is the prepared local
  planner but does not guarantee globally optimal paths.
- BendyRuler live parameter readback and obstacle-routing flight proof are
  pending.
- JT16/PRX recovered after service restart; repeat the watchdog/reboot evidence
  before each field progression because this is bench transport proof only.
- No RC EKF-source selector is assigned; active source selection must be
  observed explicitly before every mission test.
- The latest two ordinary flight logs contain no official FlowCal samples, so
  exact H-Flow scale averaging remains unavailable.
- GPS-free cold-boot EKF origin handling is unresolved operationally.
- Hot motors/ESCs require continued thermal and current evidence.
- GPS-denied RTL has no flight proof.

Do not collapse these into one score or wave them away. Each has a separate
test and failure consequence.

## Next Engineering Sequence

The quickest defensible route from the current state is:

1. Props off: recover fresh JT16/PRX, verify RC7, clear RC/compass pre-arm
   faults, and run the stationary H-Flow health audit.
2. Apply `OA_TYPE=1` and the 0.50 m/s profile, reboot, rerun the apply for
   dynamic BendyRuler parameters, and require exact readback.
3. Complete props-off JT16 four-direction and empty-space PRX checks on the
   final assembled aircraft.
4. Perform low-speed GPS-connected Loiter then FlowHold reactive-OA
   commissioning.
5. Commission short regular-Guided, AUTO, BendyRuler, and Cube RTL tests in the
   exact order in the Cube mission contract.
6. Review three clean repetitions before deliberate GPS denial.
7. Separately continue IM10A/D415 timing, passive LIO/VIO validation, SLAM
   mapping, and future breadcrumb-return work.

## Source Map

Configuration and shared infrastructure:

- `config/system.yaml`: all intended geometry, stages, limits, and gates.
- `src/optflow_slam/config.py`: strict loader and cross-field safety checks.
- `src/optflow_slam/paths.py`: canonical project-local paths.
- `src/optflow_slam/models.py`: shared data models.
- `src/optflow_slam/runtime_lock.py`: exclusive hardware ownership.
- `src/optflow_slam/mavlink_compat.py`: pinned pymavlink guards.
- `optflow`: project launcher.

Cube and proximity:

- `cube_mount.py`: Cube orientation audit/apply.
- `cube_avoidance.py`: native OA parameter audit/apply.
- `mavlink_proximity.py`: 8-face proximity encoding.
- `obstacles.py`: point filtering, sectors, fusion, clearance, alerts.
- `obstacle_check.py`: no-Cube bench sector check.
- `obstacle_avoidance_service.py`: active minimal runtime.
- `mavlink_router.py`: inactive experimental raw UART/UDP relay.
- `slam_return_setup.py`: Cube return contract audit.

Sensors and calibration:

- `im10a.py`: WIT/IM10A decoder.
- `im10a_config.py`: reversible serial profile audit/apply/recovery.
- `imu_noise_calibration.py`: stationary noise capture/report.
- `imu_alignment.py`: dynamic Cube/IMU timing and scale comparison.
- `jt16_calibration.py`: guided four-cardinal calibration.
- `jt16_plane_calibration.py`: wall/floor/ring validation.
- `clock_sync.py`: robust affine device-to-host time mapping.
- `camera_server.py`: D415 RGB service.
- `sensor_check.py`: Cube-independent sensor check.
- `native/jt16_bridge/`: C++ official SDK bridge.

Flight evidence:

- `flight_logger.py`: synchronized session recording.
- `flight_service.py`: arm-triggered orchestration and status.
- `flight_analysis.py`: reports and DataFlash integration.
- `flight_guide.py`: QGC-guided shadow-flight prompts.
- `flight_supervisor.py`: one guided shadow run per boot plus logger resume.
- `pointcloud.py`: PLY helpers.
- `slam_timing.py`: rate/jitter/drop/clock analysis.

Estimation and SLAM:

- `lio_bridge.py`: synchronized JT16/IM10A ROS 2 publishing.
- `lio_shadow.py`: shared LIO/RGB-D/Cube shadow orchestration.
- `lio_validation.py`: trajectory pass/fail report.
- `lio_visual_assist.py`: browser-guided carry sequence.
- `rgbd_odometry.py`: D415 metric visual odometry.
- `slam_poc.py`: proof launcher.
- `slam_poc_visual.py`: proof dashboard state/report.
- `cube_odometry.py`: disarmed transport-only ODOMETRY sender.

Return and navigation:

- `rtl_shadow.py`: breadcrumbs, bounded proposals, offline replay.
- `slam_navigation.py`: gates, controller, transport, status.
- `slam_navigation_service.py`: full hardware-owning monitor/live runtime.

Visualization:

- `visualizer_server.py`: Cube/IMU/point-cloud HTTP/SSE backend.
- `spatial_stream.py`: bounded atomic live point-cloud frames.
- `visualizer/src/main.js`: main drone-motion UI.
- `visualizer/src/drone.js`: reusable measured drone scene.
- `visualizer/src/spatial.js`: 3D clouds and trajectories.
- `visualizer/src/lio-assist.js`: LIO guidance UI.
- `visualizer/src/slam-poc.js`: proof UI.

Host support:

- `hardware/kernel/ch341/`: IM10A USB serial kernel support.
- `hardware/kernel/pl2303/`: JT16 USB-RS485 kernel support.
- `hardware/udev/`: stable sensor symlinks and D415 permissions.
- `hardware/systemd/`: OA service, experimental router, VNC unit.
- `hardware/headless/`: EDID and Xorg configuration.
- `scripts/bootstrap_lio_runtime.sh`: project-local ROS/PCL/FAST-LIO build.
- `third_party/lio.lock.json`: pinned revisions and hashes.

## Documentation Map

- `README.md`: operator/developer overview and command reference.
- `docs/ARCHITECTURE.md`: responsibility and failure boundaries.
- `docs/INTERFACES.md`: frames, clocks, proximity, odometry, ExternalNav, and
  command messages.
- `docs/BRINGUP.md`: staged bench/field progression.
- `docs/OBSTACLE_AVOIDANCE.md`: active JT16/Cube path and field sequence.
- `docs/SENSOR_CALIBRATION.md`: D415/JT16 measured-target procedure.
- `docs/LIO_SHADOW.md`: IMU profile, timing, shadow sequence, validation.
- `docs/SLAM_RETURN.md`: output gates, RC12 behavior, current lock.
- `docs/FLIGHT_LOGGER.md`: recording contract and analysis.
- `docs/POWER_AND_ESC.md`: propulsion acceptance.
- `docs/HEADLESS_ACCESS.md`: VNC/headless setup.
- `hardware/README.md`: host changes and install commands.
- `runtime/README.md`: transient-state policy.
- `data/README.md`: persistent evidence policy.

## Testing Contract

Run before committing:

```bash
./optflow test
git diff --check
```

Current Python suite count at this snapshot: 212 tests.

Frontend visual contracts use Playwright and include:

- Main responsive/visual check.
- Live spatial 3D check.
- LIO assist check.
- SLAM proof guide check.

For 3D changes, verify desktop and mobile screenshots, nonblank canvas pixels,
camera framing, trajectory visibility, layer toggles, no overlap, and live
movement. A successful frontend build alone is insufficient.

Test scope must increase with safety impact:

- Pure parser/math change: focused unit tests.
- Frame/sign/timing change: unit tests plus recorded replay.
- MAVLink change: packet-level test plus disarmed live health observation.
- Parameter tool: backup, disarmed guard, write/readback test.
- Service ownership: lock and systemd restart tests.
- Flight behavior: staged field evidence, never unit tests alone.

## Engineering Rules for Agents

- Read surrounding code and use existing dataclasses, parsers, status files,
  locks, and command wrappers.
- Keep changes within this repository and preserve project-local/offline setup.
- Use structured MAVLink, YAML, JSON, ROS, and point-cloud APIs rather than
  ad-hoc strings where possible.
- Use latest-only queues for safety-sensitive live sensor data.
- Stale or missing data is unknown and must fail closed. A protocol-defined
  fresh no-return value is distinct from stale input.
- Keep shadow output and active control physically and logically separate.
- Every active output needs an explicit config enable and runtime gate.
- Preserve the rule that no approved evidence means no live output.
- Do not silently broaden telemetry stream requests.
- Do not add beeps for events ArduPilot already signals.
- Keep comments short and explain only non-obvious safety or frame logic.
- Update README, AGENTS, focused docs, tests, and config together when a
  contract changes.
- Do not commit generated logs, maps, recordings, runtime state, fetched SDK
  trees, local builds, or secrets.
- Do not revert unrelated user changes in a dirty worktree.
- Before a hardware diagnostic, verify Cube disarmed from a filtered
  ArduPilot autopilot heartbeat, not the first arbitrary component heartbeat.
- After any live service change, observe it long enough to cross relevant
  watchdog/pre-arm intervals and inspect actual `STATUSTEXT`.

## Definition of Done by Change Type

OA service change:

- Focused transport/service tests pass.
- Full suite passes.
- Service is active and boot-enabled.
- Cube remains disarmed during validation.
- JT16 freshness stays inside 0.45 s.
- Real proximity health bit remains true.
- Packet counter advances.
- No fresh `PRX1: No Data` across multiple pre-arm cycles.
- No competing hardware process is active.

Estimator change:

- Timing, frame, and covariance contracts are explicit.
- Shadow report is generated from real data.
- No Cube pose or movement output occurs unless explicitly testing the
  disarmed transport boundary.
- Validation remains false unless every threshold passes.

Navigation change:

- Replay and shadow behavior pass first.
- Speed/acceleration/freshness limits are enforced in config and runtime.
- RC cancellation, mode loss, stale command, stale pose, obstacle breach, and
  estimator disagreement fail closed.
- No live approval is created automatically.

Documentation/release change:

- Current active runtime is described accurately.
- Locked/shadow/active states are not conflated.
- Commands identify hardware ownership.
- Generated artifacts are ignored.
- `./optflow test` and `git diff --check` pass.
