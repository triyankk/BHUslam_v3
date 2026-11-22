# Gemini mounting correction — 2026-11-18

Operator reported a real 22.8-degree downward camera pitch, optical centre 13 cm forward and 5 cm below the Cube. Lateral offset and additional mount roll/yaw are assumed zero. These are operator-reported physical dimensions, not full camera/IMU calibration.

Updated configs/vins-fusion-extnav.yaml: R_body_optical = R_y(-22.8 deg) * upright_optical_to_FRD. Optical +Z now maps to body (0.9218631516,0,0.3875155865), correctly pointing forward/down. Existing C++ conversion removes this mounting angle from the outgoing vehicle attitude and rotates velocity/angular rates consistently. Internal camera/IMU geometry and image processing are unchanged.

Changed only Cube VISO_POS_X 0.15 -> 0.13 m and VISO_POS_Z 0 -> 0.05 m. Y=0 and VISO_ORIENT=0 preserved. Translation stays at the camera origin in ODOMETRY, so the Cube applies the lever arm exactly once. Cube reference is used as supplied by the operator; no additional airframe/CG displacement is inferred.

Cube was freshly disarmed before pausing the existing UART owner. Native mount-only tool saved rollback and read back both changes. Independent UART audit confirmed offsets and primary source configuration. No Cube reboot was required/performed; VINS and its observation relay restarted to load the new rotation. All other readable audited parameters, including EK3_SRC1 GPS position / optical-flow velocity and EK3_SRC_OPTIONS=0, were preserved. No arming, mode or source-switch commands.

C++ frame tests passed for downward sign, corrected level-body attitude, preservation of actual body pitch, consistent angular rates, and absence of duplicate translation correction. Native mock Cube test passed for mount-only parameter writes and unrelated-parameter preservation. Actual YAML rotation/translation matched the measured values and orthogonality check.

Final live snapshot: Cube armed=false; vision healthy=false; actual serial writes since restart=880; VINS initialized=true, gate eligible=true. Fresh pre-arm messages after reload:

- PreArm: GPS 1: Bad fix

The previous roll/pitch mismatch warning cleared during the live check, with actual gated VINS output reaching the Cube. This does not quantify residual attitude error, establish VIO fusion, or validate flight performance. GPS bad fix remains. Source sets and options remain unchanged; per-core active-source proof still requires the matching Cube BIN. Recording and buzzer logic were not modified or physically armed/tested in this update.

Boot runs now save camera.yaml and navigation-mount.yaml alongside run metadata so future pose interpretation uses the correct mount. Evidence: cube-mount/, independent-readback.json, geometry.json, tests.log, parameter-preservation.json, owner-final.json, vins-final.json and run-mount.yaml.
