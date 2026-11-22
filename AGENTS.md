# BHUslamV3 agent instructions

## Current ToF transport: shared USB — 21 November 2026

The operator explicitly replaced unreliable GPIO transport with shared USB.
Native C++ tof_viewer owns I2C7 only and publishes fresh front DISTANCE_SENSOR
on loopback14605; the existing optflow-flight-logger owns Cube USB and returns
genuine front feedback on14606. No second Cube serial owner or GPIO writer.
Preserve confidence>50 mean, VIO/GPS-reference whitelists, recording/cues and
rear/down rangefinders. JT16 remains raw recording/mapping only with obstacle
extraction/output/alerts disabled. This supersedes older GPIO transport wording
below. No Cube parameters or flight modes changed for this handoff.

## Project motive and end goal

**Build a drone that autonomously flies a guided mission without GPS while
detecting and avoiding obstacles along the way.**

This is the project owner's confirmed end goal. Read
[PROJECT_GOAL.md](PROJECT_GOAL.md) for the shared objective, workstream roles
and validation progression. Use it when planning changes, explaining tradeoffs
and maintaining project documentation. Localization-only demos, mapping and
sensor benchmarks are milestones toward the complete mission capability.

## Implementation and documentation context

- VINS-Fusion is the existing primary localization implementation. Preserve its
  role while evaluating the explicitly selected joint JT16/Gemini LIVO work;
  do not describe an offline experiment as a validated live replacement.
- Keep ORB-SLAM3 and VINS-Mono as short references/comparisons in shared operator
  documentation unless a task specifically concerns those tools.
- Follow the current front-ToF/shared-USB selection in `docs/tof-front-proximity.md`;
  JT16 is recording/mapping only. Retain the Gemini measured-depth plan in
  `docs/gemini-depth-avoidance-plan.md` as future/alternative work.
- Treat “guided mission” as autonomous progress toward operator goals. Do not
  infer a specific ArduPilot mode or implemented mission controller from that
  phrase alone; verify the actual firmware/command path for the task.
- Connect configuration, testing, architecture and visualization documentation
  to the project goal. Distinguish planned capabilities, implemented behavior
  and demonstrated results. Keep dated reports as evidence of their original
  test conditions; a newer plan does not rewrite past results.

## Working on hardware and tests

- Read the applicable current workflow/configuration and latest saved evidence
  before changing hardware behavior. Historical port, baud, timing, EKF and
  proximity settings may have been superseded; verify relevant live state
  when the task requires current hardware facts.
- Preserve one owner per camera, Cube serial link and sensor acquisition path.
  Reuse existing feeds/owners when adding consumers.
- Keep runtime additions native C++ where the existing pipeline does so. Source
  `scripts/env.sh` for the workspace ARM64 toolchain.
- Use unique output directories and retain effective settings, logs, timestamps
  and executable identity for repeatable operator tests.
- A documentation goal is not authorization to change flight parameters,
  arming, modes, EKF sources or mission execution. Keep implementation within
  the user's requested scope and preserve unrelated settings.
- Match validation claims to evidence: initialized VIO, received MAVLink,
  sparse maps and GPS-aided flights do not individually prove autonomous
  GPS-denied mission flight or obstacle avoidance.

## Fresh-GPS bootstrap, 20 November 2026

The operator authorized the Here4 plug/fix/disconnect startup workflow. See docs/gps-bootstrap.md. The new exact vio_optflow_compass_gps_bootstrap profile differs from the selected hybrid only by GPS1_TYPE=9. Preserve all EKF sources, backup sets and internal-compass priority/use flags. AHRS_OPTIONS reuse bit16 stays clear. Native C++ owns initialization; the existing sole Cube writer independently whitelists only fresh-GPS disarmed Home requests and the verified-reference cue. No source switching or automatic old-origin restore. Keep ten rapid recording-ready beeps unchanged; three slow beeps indicate verified fresh reference, not flight readiness. Indoor tests do not validate physical GPS-disconnect behavior, fusion or RTL.

## Front ToF selected; JT16 recording/mapping only —21November2026

The operator explicitly replaced current JT16 obstacle detection with front ToF over GPIO TELEM2. See docs/tof-front-proximity.md. This supersedes the previous active-source selection for the current setup; Gemini depth remains documented future/alternative work. Preserve confidence>50 mean and orientation0 unless separately authorized. Native bhu-tof-front owns I2C7 and /dev/ttyTHS1 at115200, VIO UDP disabled. Restore the separate Cube USB owner only with JT16 obstacle extraction/output/alerts disabled; retain raw acquisition, arm/disarm recording and shared mapping feed, VINS, GPS-reference status/cues. Never mix JT16 proximity with ToF. The generic old JT16 Cube preset must refuse this external-source config. Live backend/settings may already match; verify before writing. Intermittent GPIO reception (previous16s gaps,500ms Cube timeout) is a blocking avoidance-health issue, not permission to extend timeouts or claim flight-ready avoidance. Front-only sensing does not prove a side/rear path clear. No automatic mode, arm, mission or source changes.

## H-Flow downward range / TF03 rear proximity —21November2026

Operator moved the180m TF03 to rear and authorized H-Flow integrated CAN range for optical flow plus rear avoidance. With props removed/disarmed, six scoped changes set RNGFND1_ORIENT4, RNGFND2_TYPE24/MIN0.08/MAX30/POS_Z0.1 and PRX2_TYPE4, then rebooted. RNGFND2_ORIENT25/ADDR0 already matched; its X/Y/Z copy existing flow offsets. Keep TF03 TYPE27 and0.1–180m limits, FLOW_TYPE6/CAN setup, front PRX1_TYPE2, all EKF sources and avoidance tuning. Firmware selects only downward orientation25 from its first two range slots for flow; rear slot1 is excluded, down slot2 supplies range. Actual down sensorID1 and rear proximityID14 were read after reboot. Do not count global proximity health as proof the broken front GPIO input is fixed. Front viewer now ignores rear180m out-of-ToF-range telemetry sentinels. RNGFND2_GNDCLR0.2 and moved rear offsets remain unmeasured; no new calibration claim. See docs/hflow-down-rear-rangefinder.md and its report; use native --hflow-down-rear-only with backups/guards for this scope, not old generic presets.
