# VIO position / optical-flow velocity / internal-compass yaw

Configured on 18 November 2026 after the operator explicitly chose optical-flow horizontal velocity and the Cube's internal compass for yaw. This supersedes the earlier all-VIO position/velocity/yaw configuration for current operation.

## Current primary sources

| Parameter | Value | Meaning |
|---|---:|---|
| EK3_SRC1_POSXY | 6 | VIO/ExternalNav horizontal position |
| EK3_SRC1_VELXY | 5 | Optical-flow horizontal velocity |
| EK3_SRC1_POSZ | 1 | Barometer height |
| EK3_SRC1_VELZ | 6 | Existing VIO vertical velocity, unchanged |
| EK3_SRC1_YAW | 1 | Compass yaw |
| EK3_SRC_OPTIONS | 0 | Keep velocity fusion within the selected source set |
| COMPASS_ENABLE | 1 | Enable compass subsystem |
| GPS1_TYPE / GPS2_TYPE | 0 / 0 | GPS disabled |
| VISO_TYPE | 3 | VINS pose reception enabled |

Unused sets 2/3 retain POSXY/VELXY/VELZ/YAW=0 and POSZ=1. The new explicit `vio_optflow_compass` bridge profile requires the same mount (0.13/0/0.05 m), orientation0, quality40 and EKF3 settings as the other supported profiles. It is recognized by both the C++ relay and existing sole-owner serial adapter, qualifies while disarmed and blocks a profile change while armed. It never switches sources automatically. The existing pure-VIO and GPS-observation profiles remain available but are not active.

## Internal compass verification

Fresh disarmed audit identified primary COMPASS_DEV_ID=1313809 (0x140c11): I2C bus2, address0x0c, device type0x14 AK09918. COMPASS_EXTERNAL=0, COMPASS_PRIO1_ID=1313809, COMPASS_USE=1 and USE2/USE3=0. Two historical secondary IDs were DroneCAN devices; they were not enabled for yaw. This follows the official firmware device-ID encoding, not inference from a compass label alone.

The missing setting was COMPASS_ENABLE=0. It was set to1 and the disarmed Cube rebooted with the same-session props-off confirmation retained. Post-reboot audit confirms the primary internal AK09918 and enabled use flag; secondary device IDs are now zero, and the compass and optical-flow health bits are true. Primary compass calibration offsets (-520.434,155.808,-495.612 mG approximately) were preserved exactly; no new calibration or priority assignment was performed. Health does not establish calibration or motor-on magnetic immunity.

## Applied changes and preservation

At the latest pre-write readback, source1 VELXY=5/YAW=1 was already set. Actual changes were COMPASS_ENABLE 0->1 and four unused SRC2 values POSXY/VELXY/VELZ/YAW ->0 (they had reappeared as 6/6/6/1 since the previous cleanup). The explicit complete hybrid profile retains the agreed unused-set cleanup. SRC3 was already clear. Before/requested/after/changes/rollback and independent before/after compass audits are saved. All scoped non-target parameters were checked unchanged, including proximity type2, rangefinders, BendyRuler, RC, notification/log settings and calibration/use flags. No arm, mode, mission, motor or source-selection command was sent.

The native configuration tool adds --vio-optflow-compass-only using the same canonical profile as the C++ guard plus the unused-source settings. The existing serial-owner adapter adds the identical 17-key profile without introducing a new process. Both profile maps match within1e-6. The Cube owner and mapper were paused/restored for exclusive parameter access; VINS stayed warm. Native relay and existing owner were reloaded for the new profile. Recording state machine and recording-ready ten-beep cue were not edited.

## Tests and live check

Native Cube integration and external-navigation transport tests passed, including hybrid full readback, unused-source cleanup, preserved parameters, armed-profile blocking and packet checks. The existing adapter's 12 focused tests passed, including actual fake-connection writes under the new hybrid profile. Its full suite had 382 passes and two unchanged failures in .gitignore generated-data rules and an external workspace reference in docs/JT16_VIEWER.md; neither file was changed by this work.

Post-reboot readback proves internal compass selection and both compass/optical-flow health. A separate 65-second disarmed observation records actual hybrid serial writes, Cube vision health, proximity and recorder status. VIO accuracy/fusion, origin/Home, motor-on tracking and obstacle-response validation remain separate. Genuine VisOdom unhealthy periods must not be hidden by disabling checks.

## Recovery-gate comparison retained

The preceding comparison remains valid at the publisher-policy level: 120.235 s at 25 W, 1569 shared live samples, identical baseline/candidate decisions (1552 accepted each, 98.9165%), modeled maximum packet gap900ms and zero resets. The two interruptions returned valid poses after333.730 and367.104ms, both exceeding the candidate's200ms short-interruption limit. No benefit was measured and the actual gate remains500ms. Candidate100ms recovery is diagnostic only. During that run actual forwarding was blocked by the then-mismatched compass-disabled hybrid settings; its modeled receipt coverage must not be presented as actual Cube health. Full report: ../gate-recovery-comparison-20261118/review.md.

Files: config/ durable parameter evidence; compass-before.json/compass-after.json; live-check.jsonl/live-summary.json; profile table exports; native-tests.log, adapter-tests.log, full-adapter-tests.log; preservation.json and release.sha256. vio-optflow-internal-compass.param exports current primary/backend/unused-source/use settings for this Cube. Refresh/download and re-save the transmitter parameters after this update. Other Desktop pure-VIO files describe alternative profiles, not the current selected one.

Firmware identity reference: [Copter 4.7.1 device-ID decoder](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/Tools/scripts/decode_devid.py).

Final live result: 65 samples, continuously disarmed; 216 actual pose writes during the check. Internal compass and optical flow healthy in all 55 steady samples; proximity healthy in 55/55. Cube vision health true in 36/65 one-second samples. Remaining pre-arm history: PreArm: Radio failsafe on; PreArm: VisOdom: not healthy. The bridge identifies vio_optflow_compass, recording thread remains healthy and the Desktop export matches the saved profile.
