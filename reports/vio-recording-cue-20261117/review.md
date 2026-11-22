# Automatic flight recording and ten-beep cue

Configured on 2026-11-17. This supersedes the earlier continuous-between-flights recording behaviour; estimator processing remains continuous.

- Boot warms VINS-Fusion and JT16 FAST-LIO after the existing Cube startup condition. Neither supplies navigation to Cube.
- Fresh armed state opens new camera/IMU, pose, map and raw LiDAR flight files. Fresh disarm finalizes after three seconds; re-arm during that tail continues the same session. Link loss is not a synthetic disarm.
- VINS uses atomic shared recorder ownership so IMU callbacks remain valid during session changes. A bounded asynchronous writer confirms a flushed stereo pair plus both IMU streams before requesting sound. Pending images before the session's IMU start are omitted to preserve replay brackets.
- Native C++ recording request goes over loopback 14602 to the existing Cube serial owner. It can request only the fixed recording cue, not arbitrary MAVLink. Fresh requests are deduplicated and acknowledged after the owner sends one MAVLink PLAY_TUNE containing exactly ten C notes with rests, approximately 1.25 s. ACK is not audible confirmation. Producer retries expire at 30 s; cues occur per new recording, not per processed frame.
- The existing Python Cube owner has a small cue/arm-status hook because it alone owns the serial link. No additional Python process or serial reader was introduced. Recording/session management remains C++. A flat arm-state file drives native raw-packet recording without restarting JT16 acquisition.
- Per-flight maps include only the flight and tail; backend estimation remains warm. Post-flight review now discovers separate flight folders, with a common boot ID and host monotonic times for association. Lightweight boot diagnostics remain separate and can continue while disarmed.

Validation: seven native Fusion tests passed; twelve existing-owner tests passed, including duplicate ACK behaviour, stale/arbitrary-request rejection and real MAVLink encode/decode of the exact ten-note tune. A real shared JT16 feed with synthetic **file-only** arm/disarm status produced two finalized ~6-second map recordings, 30 poses/30 scans each. No Cube arm command or other flight command was sent. The first integration attempt exposed a legacy root-only HTML export assumption; the launcher was corrected and the second complete test passed. Review fixture verified per-flight discovery, truncated CSV handling and checkpoint recovery.

Installed environment includes cue port 14602 and the native arm-state file. All observation units are enabled. Current camera/Cube are absent, so VIO/mapping are waiting. Native JT16 remains the sole LiDAR reader; no new armed raw-flight files were created while Cube was absent. `config/system.yaml` checksum remains unchanged. Cube settings, navigation/source selection and Jetson power mode were not changed. Actual audible sound, connected VINS session cycling and a combined 15 W workload still need hardware validation.

The mapping unit now stops its full cgroup so an idle readiness helper cannot hold up shutdown. Configuration rollback evidence is under `runs/flight-shadow/install-20261117T111928.645166544Z/`. Original owner source and pre-arm-gate native source are retained beside this report.

No Jetson access is required between normally completed flights: arm, wait for recording confirmation, fly with existing GPS aid, land/disarm, allow finalization, then repeat. Power loss before completion can still lose buffered data. No loop closure, mapping accuracy, EKF fusion or flight readiness is established by these checks.

See [operating procedure](../../docs/flight-shadow-test.md).
