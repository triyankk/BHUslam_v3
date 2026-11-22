# Fresh-GPS bootstrap setup — 20 November 2026

Operator authorized GPS plug/fix/disconnect startup, with no previous-origin reuse.

Implemented a native C++ state machine in the existing relay, an exact compatible
hybrid profile, a narrow independently validated Home-command boundary in the
existing sole serial owner, GPS/reference status/logging and a three-slow-beep
reference cue. No new process, serial owner, runtime Python estimator, source
switching, mode, arm or motion output was introduced. Ten rapid recording-ready
beeps and arm/disarm recording remain unchanged. D415 stays disabled.

Applied exactly GPS1_TYPE0->9 and rebooted while disarmed, using the existing
props-removed confirmation. Fresh before/requested/after/restore artifacts are
in applied/. An independent post-reboot audit in post-reboot-readback/ confirms
all67 audited values, with exactly that one difference from before. AHRS_OPTIONS8,
primary6/5/1/6/1, OPTIONS0, all backup sources and selected internal compass
ID1313809/use flags are preserved. See preservation.json.

The Cube reports GPS1 detected as DroneCAN1-125. Indoor GPS currently reports
fix_type1 (no fix), zero satellites and zero reported accuracy. The state machine
waits for stable GPS; no origin/Home is fabricated or restored, no Home command
is sent and no reference-ready cue is emitted. Both relay stages identify
vio_optflow_compass_gps_bootstrap and accept ordinary validated VINS packets
independently of this waiting state.

Verification:

- Native protocol/bootstrap/transport/pseudo-Cube configuration suites passed.
  The GPS state-machine suite passed again after testing poor-to-good GPS recovery.
-26 focused sole-owner tests passed, including Home whitelist, malformed/CRC
  packet rejection, armed/source/quality guards, readback-only confirmation,
  one cue, Cube reboot, duplicate timestamps and poor-fix recovery.
- Full owner suite retains the same two unrelated pre-existing failures:
  generated-data .gitignore expectations and docs/JT16_VIEWER.md external-path
  policy. Changed-file whitespace checks pass; repository-wide diff check also
  reports pre-existing hardware/systemd/x11vnc.service trailing blank line.
- Both bridge stages, VINS and mapper are boot-enabled and active at final check.
- Saved live samples/status/summaries provide the exact settled-state observation.

One VINS process exit/restart occurred during the initial observation. Its last
saved sample was eligible; the existing error path did not persist an exception
reason, and no journal was available to identify it. Do not attribute that gap to
GPS gating: both bridge source guards were unblocked. A temporary proximity
No Data pre-arm message also occurred during service recovery; retain current
status/readbacks rather than interpreting a historical warning as current health.

Remaining: clear-sky fresh fix, matching Cube origin/Home and audible cue, physical
Here4 disconnect with Cube continuously powered/disarmed, internal-compass and
EKF continuity, then separate fusion/RTL validation. Indoor setup is not a flight
readiness or GPS-disconnected validation claim.

Operator procedure: ../../docs/gps-bootstrap.md.

Final settled45s observation: 465 VINS ODOMETRY serial writes, zero bridge blocks/rejections, 46/46 sampled Cube vision-health reports healthy, maximum sampled heartbeat age923ms. Cube remained disarmed. GPS remained no-fix, reference waiting, zero Home writes. The pre-arm history retained PRX1: No Data from recovery; final current proximity health=True, scan age=40ms. This does not validate flight fusion.
