> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> Continuous estimator startup supports the localization required by that mission capability. See the [shared project goal](../PROJECT_GOAL.md).

# Continuous VINS-Fusion boot startup

Current transport override (2026-11-18): the operator requested USB testing after TELEM2 requests stopped receiving replies. Installed USB-test drop-ins make both the existing Cube owner and future VINS startup wait use Cube USB if00. USB must remain connected. The base UART profile is retained; see [current transport details](vins-observation-forwarding.md#temporary-usb-transport-2026-11-18).

Configured on 2026-11-17 at the user's request. Start only when **both** Gemini CPC2963000JJ and the Cube are ready. No QR trigger or fixed recording duration.

The later [GPS-aided shadow-flight profile](flight-shadow-test.md) adds independent shared-feed JT16 mapping, raw LiDAR recording, camera input recording and performance evidence. It retains this startup condition. The later [reception-only observation profile](vins-observation-forwarding.md) enables gated pose forwarding while preserving GPS/flow source set 1.

Latest behaviour: estimators stay running, while recordings open on Cube arm and finalize three seconds after disarm. Each flight gets separate output folders and a ten-beep recording-start cue. No Jetson interaction is required between flights; see the shadow-flight procedure for details and limits.

Connected audit update (2026-11-18): the service sets `BHU_CUBE_DEVICE=/dev/ttyTHS1`; the existing Cube owner uses the same verified TELEM2 UART at 115200 through its runtime environment override. Startup no longer needs Cube USB. The boot ordering cycle is corrected, and missing JT16 no longer terminates the Cube owner or blocks VIO. The later reception-only update sets `VISO_TYPE=3` and forwards gated poses through the existing UART owner, with `EK3_SRC_OPTIONS=0` and source set 1 unchanged.

## Startup sequence

`bhu-vins-fusion.service` starts in the user systemd session. Existing `Linger=yes` permits startup without login. It requests the already installed `optflow-flight-logger.service`, preserving that service as the only Cube UART owner.

1. Wait for read/write access to the exact Gemini USB device and configured Cube serial endpoint (currently `/dev/ttyTHS1`). Resolve the Gemini USB bus/device number from sysfs each time.
2. Require the existing Cube owner's `runtime/obstacle_avoidance_status.json` to change after the C++ readiness guard starts. Require running service, connected Cube, heartbeat age <=1500 ms and disarmed state for three successive polls. Old saved ready status cannot release startup. This is **link/startup readiness**, not EKF fusion or flight readiness. It does not require the LiDAR transport's `ready` flag or a healthy VIO EKF, which could otherwise prevent estimator initialization.
3. Recheck device access, create a unique run directory and start the native C++ stereo-inertial VINS-Fusion adapter continuously.

With either device absent, the service remains waiting and does not open the camera. While waiting for a fresh disarmed heartbeat it does not open Cube serial. Once VINS starts, arming does not restart it or change its map. A Cube link loss after startup does not terminate this compute-only estimator. Crash/replug failures can restart this service with a new run/map; this policy must be revisited before connecting its output to live flight navigation.

## Current output boundary

**Gated observation reception.** This service passes `--udp-port 14600` to the native observation relay. The existing UART owner transmits validated ODOMETRY while retaining source set 1 and `EK3_SRC_OPTIONS=0`. No source, mode, arming, mission or movement commands are sent. See [transport, logging and limits](vins-observation-forwarding.md). The old bench bridge remains disarmed-only and is not used.

Factory/provisional calibration remains explicit via `--allow-provisional`. This flag is not calibration approval. EKF fusion, GPS/VIO alignment and transition, per-mode obstacle avoidance and combined 15 W performance have not been validated by this boot configuration. No Cube parameters or Jetson power settings are changed. No online loop closure is enabled.

## Operation

```bash
cd /home/atas/BHUslamV3
scripts/build-vins-fusion.sh
bin/vins-fusion-boot install
bin/vins-fusion-boot status
journalctl --user -u bhu-vins-fusion.service -f
bin/vins-fusion-boot stop
```

Installation saves prior unit files/enabled states in `runs/vins-fusion-boot/install-*`, disables competing BHU recorder/legacy ORB camera services and enables/starts the Fusion unit. It does not disable or modify the existing JT16 service. To disable future startup: `systemctl --user disable --now bhu-vins-fusion.service`. Older camera services can be re-enabled individually using the saved enabled-state list; do not run them alongside Fusion.

## Logs and diagnostics

Runs appear in `runs/vins-fusion-boot/YYYYMMDDTHHMMSS.NNNNNNNNNZ/`; `latest` points to the most recently started estimator run. Each contains boot ID/uptime, power-mode snapshot, binary checksum, startup Cube status and `live/` outputs:

- `tracking.csv`: timing, feature support, gate decisions and drop counters.
- `pose_history.csv`: initialized native camera poses, including withheld samples.
- `status.json`: atomic snapshot approximately every second while frames are processed. Check `host_monotonic_us` against host monotonic time; stale files, including those after a crash, are not current health. `gate_eligible` does not establish accuracy or Cube fusion.
- `summary.json`: generated after normal shutdown. Diagnostics retain only the most recent up to 8192 samples in memory; cumulative frame counters cover the entire process. CSVs continue to disk and are flushed about once per second, without fsync/power-loss durability guarantees.
- `input/`: asynchronous recording of selected rectified stereo pairs before CLAHE, plus delivered SI IMU samples and replay calibration. See the shadow-flight procedure for coverage and disk limits.

The existing profile remains mild CLAHE clip 2, four OpenCV threads, 0.5 s / 40 persistent-feature publication gate and 150 ms freshness limit. The service journals console output. Data accumulates until explicitly archived/deleted; inspect free disk space before extended unattended operation.

## Without attached hardware

Software checks can validate startup guards, fixed-memory diagnostics and service installation/waiting. Actual camera capture, boot-through-camera initialization, tracking and Cube fusion require a later connected test. An enabled/active systemd unit can mean **waiting for devices**, not that VIO is producing poses.
