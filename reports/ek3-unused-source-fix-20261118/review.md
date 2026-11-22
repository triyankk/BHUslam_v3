# GPS-free EKF unused-source correction — 18 November 2026

The previous GPS-free profile correctly configured primary source1 but left GPS position/velocity references in unused sets2/3. Copter 4.7.1's source pre-arm check iterates all stored sets, so GPS1_TYPE=0 caused AHRS: EK3 sources require GPS. This was an incomplete profile and is corrected at the operator's explicit request.

## Applied

Only five values changed: EK3_SRC2_POSXY 3->0, EK3_SRC2_YAW 1->0, EK3_SRC3_POSXY 3->0, EK3_SRC3_VELXY 3->0, EK3_SRC3_YAW 1->0. Both unused sets now have POSXY=0, VELXY=0, POSZ=1, VELZ=0, YAW=0. They have no horizontal position, velocity or yaw source; existing barometric height selection is retained. These sets are not fallback navigation profiles.

Primary source1 stays POSXY=6, VELXY=6, POSZ=1, VELZ=6, YAW=6. OPTIONS=0, GPS1/2_TYPE=0, COMPASS_ENABLE=0, VISO_TYPE=3, VISO mount/quality, PRX1_TYPE=2, BendyRuler/RC/rangefinders, notification, logging and arm/disarm recording remain unchanged. Native before/after preservation checks cover all scoped non-target parameters. No reboot, arming, mode, mission, source-selection command or bypass of a check was used. Parameter changes are saved by PARAM_SET and read back; fresh periodic pre-arm messages test the running configuration.

## Tools and profile correction

The existing native C++ tool has --clear-unused-sources-only, which requires the complete primary gps_free_vio contract before applying just the unused-source cleanup. The existing --gps-free-only mode now includes the same ten unused-source values, so a future apply cannot leave this GPS dependency. Both use durable before/requested/after/changes/summary/rollback and disarmed guards. Simulated-Cube integration scenarios passed in 22.16 s; tests cover the five-write cleanup, complete GPS-free application from the old GPS backup sets, all source readbacks and unrelated-parameter/proximity preservation.

The corrected GPS-free parameter file now contains 27 values: the existing 17 primary/backend/mount values plus ten unused-source settings. The canonical workspace candidate and Desktop export are updated. The historical earlier applied 17-value report remains evidence of that earlier action, not the corrected current profile. Refresh/re-download and re-save the ground-control transmitter's full parameters after this correction.

## Verification

Fresh Cube disarm and inactive recording were checked before pausing the existing sole owner/mapper for exclusive native application/readback. The owner and mapper were restored automatically; VINS stayed warm. Independent after-audit confirms all three source sets and unchanged primary VIO/proximity/source options. Its fresh pre-arm texts no longer included EK3 sources require GPS; it reported Need Position Estimate, VisOdom not healthy and the expected transient PRX1 No Data while the proximity owner was paused. A separate post-restore observation covers multiple periodic pre-arm cycles and confirms recovered proximity health. Remaining VIO/origin/position-estimate errors are not fixed by removing a GPS source dependency and are not masked.

Evidence: config/ before/requested/after/changes/summary and restore.param; after-audit.json; preservation.json; live-check.jsonl; live-summary.json; integration.log; release.sha256; gps-free-vio-corrected.param; previous candidate/Desktop copies where applicable. No flight-fusion or flight-readiness validation is implied.

Firmware basis: [Copter 4.7.1 source pre-arm check scans every configured source set](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/AP_NavEKF/AP_NavEKF_Source.cpp).

Final observation: 65 samples with Cube continuously disarmed; no new EK3 sources require GPS messages in 8 actual pre-arm messages. Remaining messages: PreArm: VisOdom: not healthy. Proximity was healthy in 55/55 steady samples. Primary gps_free_vio and healthy recording thread were preserved. The earlier Desktop filename was absent, so the corrected export was written to /home/atas/Desktop/gps-free-vio-corrected.param.
