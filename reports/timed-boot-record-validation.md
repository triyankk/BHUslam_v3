# Automatic two-minute boot recording validation — 2026-10-25

The installed/enabled user unit `bhu-vio-record.service` now runs
`bin/vio-test cube-record --seconds 120`. QR triggering and intermediate
movement cues are removed from this boot mode. Device waiting, lingering,
exclusive camera/Cube ownership and persistent camera USB permissions are
retained. The competing Cube/VIO-test boot unit remains disabled.

The native recorder starts its timer and cue on the first valid stereo frame.
The Cube start and stop events both use `MFT150L8O5CP8CP8C`: three notes with
pauses. The stop cue follows capture-file flushing, filesystem synchronization
and final metadata fsync. The Jetson stays powered on after capture.

## Verification

- All eight existing native/UI checks passed:
  `reports/timed-boot-record-tests.txt`. Socket/PTY checks required normal host
  device access; their initial sandbox-denied run was repeated successfully.
- The simulated Cube fixture exercised the new start cue through feedback-only
  mode and continued to reject navigation forwarding in that mode.
- Systemd verified the updated unit:
  `reports/timed-boot-record-unit-verify.txt`.
- The actual installed service completed a full 120-second manual-start check
  using the same ExecStart and device-wait path as boot. No user-directed motion
  was requested during this setup check.
- Camera dataset: `recordings/diagnostic/20261025T100658.190042160Z/`.
- Cube run: `runs/cube-record/20261025T100658.056026398Z/`.
- Duration 120.012977 s, stop_reason=duration_limit, qr_trigger=false,
  mode=boot_timed, timed_duration_complete=true, complete=true.
- 3,597 stereo pairs, 24,376 accelerometer and 24,376 gyro measurements, zero
  timestamp-skew rejections and zero combined frame-index gaps.
- Native integrity check completed. Metadata records
  filesystem_synced_before_stop_cue=true.
- The only normal Cube cue events were record_start and record_stop, at host
  monotonic 2764.902011 and 2885.383349 s. Their transmission was verified;
  a separate human acoustic confirmation was not requested.
- Power profile 15W at both start and end; no SLAM or navigation transmission.
- Service Result=success, ExecMainStatus=0 and ActiveState=inactive afterward;
  UnitFileState=enabled for the next boot. No camera/Cube capture process remains.

The full 120-second capture was tested through a manual service start. A new
physical power cycle with this updated mode has not been performed; automatic
boot of the same user unit/device-wait path was previously verified.

Outputs remain lossless native/rectified stereo frames and timestamped IMU
files for replay, under `recordings/diagnostic/`. No MP4 conversion is required
for VIO analysis. Shut Linux down normally before removing power.
