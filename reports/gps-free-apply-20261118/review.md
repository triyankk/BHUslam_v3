# GPS-free parameter profile applied for ground-controller saving

Applied on 18 November 2026 at the operator's explicit request to load the GPS-free parameters onto the Cube so they can be saved on the ground-control transmitter. This supersedes the preceding GPS-aided navigation configuration; it does not establish GPS-free flight performance.

The exported candidate was compared with the bridge's canonical gps_free_vio contract: all 17 keys match within float tolerance. Added a narrow --gps-free-only mode to the existing C++ parameter tool using that same canonical map. No arbitrary parameter-file execution, new Python process, firmware update, arm/motor/mode/mission command or source-selection command was introduced. Source set 1's parameter configuration deliberately changes to ExternalNav as requested; that differs from sending a source-selection command.

Fresh filtered native Cube readback confirmed disarm. Propeller-removal confirmation from the same ground session remained in effect. The sole Cube owner and shared JT16 mapper were paused for exclusive access and restored automatically; VINS stayed warm. Durable before/requested/after/changes/summary and rollback were written. Exactly six values changed:

| Parameter | Before | Applied |
|---|---:|---:|
| COMPASS_ENABLE | 1 | 0 |
| EK3_SRC1_POSXY | 3 | 6 |
| EK3_SRC1_VELXY | 3 | 6 |
| EK3_SRC1_VELZ | 3 | 6 |
| EK3_SRC1_YAW | 1 | 6 |
| GPS1_TYPE | 9 | 0 |

Other profile values remain AHRS_EKF_TYPE=3, EK3_ENABLE=1, EK3_SRC1_POSZ=1 (barometer), EK3_SRC_OPTIONS=0, GPS2_TYPE=0, VISO_TYPE=3, VISO_ORIENT=0, VISO_POS_X/Y/Z=0.13/0/0.05 and VISO_QUAL_MIN=40. PRX1_TYPE=2, BendyRuler/RC/rangefinder settings, log settings and recording/beeps were preserved. All scoped non-profile parameters were checked unchanged before/after; see preservation.json.

A disarmed reboot activates the disabled compass/GPS configuration. Post-reboot actual owner parameter replies verified all 17 expected values fresh and the recognized gps_free_vio profile for 25 consecutive one-second samples; Cube remained disarmed and recording thread stayed alive. No automatic restoration to GPS is performed. Refresh/download the parameters on the ground controller after reconnecting, then save/export its current full parameter set; avoid saving a cached GPS-aided list.

Native simulated-Cube integration scenarios, including the new mode, passed in 19.12 s. The new test verifies the exact six differences from GPS observation, full desired-profile readback, MAVLink proximity preservation and unrelated-setting preservation. Real hardware write/readback/reboot evidence is separate from this simulation.

The current saved geographic origin is zero and no origin, Home or north-heading value was invented. VIO publication/fusion accuracy remains unvalidated and has been intermittent in this stationary scene. No arming check or fail-safe was bypassed. This is the requested bench configuration for saving and subsequent ground/simulation validation.

Files: config/ contains before/requested/after/changes/summary and restore.param; gps-free-applied.param contains the actual requested values; gps-aided-restore.param restores the preceding GPS/compass profile; status-before.json and status-final.json record owner state; post-reboot-check.jsonl records persistence checks; before-audit.json is the independent read-only audit. The generic source_selected=false summary field means no source-selection command was sent, not that source1 parameter values stayed unchanged.
