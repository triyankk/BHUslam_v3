# Front ToF / GPIO integration —21November2026

The operator selected ToF front proximity over Jetson GPIO, with JT16 exclusively
for flight point recording and mapping. The linked ToF chat was read, including
its25sGPIO trial and16s reception gaps. October rear-rangefinder memory was not
used as current configuration.

Cube4.7.1 dbe79216 readback already matches the selected front-ToF profile:
PRX1_TYPE2, RNGFND2_TYPE0, SERIAL2_PROTOCOL2/BAUD115/RTSCTS0,
AVOID_ENABLE7/BEHAVE1/MARGIN2m/ALT_MIN2m, OA_TYPE1/BR_TYPE1/LOOKAHEAD3m,
OA_MARGIN_MAX2m, OA_DB_SIZE100/EXPIRE5s/ALT_MIN1.5m and PRX_IGN_GND1.
Downward RNGFND1_TYPE27 and all EKF/GPS/compass/RC/source settings were preserved.
No Cube parameters, reboot, arm, mode, motor, mission or source command was sent.

Implemented/activated:

- Native existing tof_viewer runs at boot through bhu-tof-front.service and
  bin/tof-front-boot; retains orientation0,confidence51,zone-mean,min1zone,
  direct I2C7,GPIO ttyTHS1 at115200, VIO UDP0. Unique capture/binary identity.
- USB VIO/telemetry/recording owner was unmasked after ToF moved to GPIO.
  Its JT16 obstacle extraction/output/alert paths are disabled in persisted
  config and independently guarded, including queued scans. The native reader
  still publishes full point frames and the shared mapping feed.
- Raw flight packet recording remains gated by actual Cube arming, with the
  existing disarm tail. No fake arm state was injected. Disarmed manual
  recording validates metadata, not actual armed raw-packet creation.
- Existing VINS, observation relay, recording/cues, fresh-GPS workflow and
  FAST-LIO mapping are retained. Five services active/enabled; user linger yes.
- Legacy JT16 Cube preset refuses this external-source config rather than
  disabling/overwriting ToF. Native read-only inspector gained an optional
  proximity audit that captures forwarded raw and Cube-processed front packets.

Evidence:

- Independent45sUSB observer while ToF used GPIO:10 processed front-sector
  reports,8 forwarded rawToF packets; largest report gap15.477s and raw gap15.931s.
-25sample disarmed recording check:120newJT16 frames,zeroJT16 proximity packets,
  zeroJT16 obstacle-alert beeps,32VIO serial writes and no source-guard block.
  Proximity healthy only1/25samples; maximum UI front-report age127.14s.
- Recording finalized with127JT16metadata records and4342Cube records; D415
  remained disabled. Optional IM10A USB input is absent (manifest records that
  source error); Gemini built-in IMU/VINS is separate. Desktop output location
  restored after this temporary test and auto_record_on_arm remains true.
- A temporary read-only ROS subscriber received3live native FAST-LIO odometry
  messages. Flight map/raw file creation is arm gated and was not simulated.
-47 focused service/VIO tests passed. Full suite returns only the same two
  pre-existing failures: .gitignore generated-data expectations and
  docs/JT16_VIEWER.md external-workspace text. Legacy-output tests now select
  legacy inputs explicitly; default tests assert current recording-only source.
- Changed-file whitespace checks passed. Single owners shown in ownership.txt.

The GPIO link is NOT dependable for avoidance. The matching backend's timeout
is500ms, far below observed gaps. The local ToF sensor remains fresh; loss is
between GPIO transmission and Cube ingestion, not absence of JT16 obstacle data.
No timeouts were relaxed or stale/fabricated clear ranges sent. TX/GND physical
checking was requested. Configuration alone cannot make this safe to rely on.

Single forward coverage cannot establish side/rear clearance for autonomous
BendyRuler detours. Prior installed-firmware evidence also excludes horizontal
AltHold/FlowHold avoidance; Loiter reactive and AUTO/GUIDED/RTL BendyRuler need
separate mode, localization, Home/origin and physical-response validation.

Procedure: ../../docs/tof-front-proximity.md.

After operator-confirmed TX/GND reseat, a further45.37s check still failed:681local frames,0newGPIO front-sector readbacks, proximity healthy1/46samples, current healthfalse. JT16 advanced225frames, VIO forwarded564poses, zeroJT16proximity and noVIOsource block. Valid/transmitted measurement-log verification is in gpio-reseat-valid-transmit-check.json. The link remains an avoidance blocker; next isolate GPIO TX versus Cube RX with loopback/3.3VUART-adapter instrumentation. No fallback transport or Cube parameter change was made.
