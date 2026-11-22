# Native front ToF through the existing USB owner — 21 November 2026

Implemented the operator-authorized transport change from unreliable GPIO to
shared Cube USB. This supports the GPS-denied mission project as a verified
sensor-transport milestone, not proof of autonomous obstacle avoidance.

## Running architecture

- Native C++ `tof_viewer`: sole TMF8820 owner on I²C7; confidence >50,
  arithmetic mean of all qualifying primary zones, minimum one, front orientation.
  No GPIO UART or USB serial descriptor is opened.
- Local datagrams on 14605: one original MAVLink2 DISTANCE_SENSOR per eligible
  frame. The existing USB owner accepts only the tested front-distance whitelist.
- Existing optFlow USB owner: sole /dev/ttyACM0 owner, same thread forwards ToF,
  VIO and existing recording/reference/cue functions. JT16 obstacle publishing
  remains disabled; its raw acquisition/mapping/recording stays enabled.
- Local feedback on 14606: genuine Cube front sector ID 10, heartbeat and
  limited status/parameter telemetry. Unavailable-front sentinels are excluded.
- Both services are enabled for startup, and ToF starts after/wants the USB owner.

No Cube parameter, mode, arm, source-selection, mission or reboot command was
added or executed for this handoff. Existing VIO/profile/Home whitelists and
ten-rapid/three-slow cue logic are unchanged. The ToF adapter cannot carry those
commands. The Cube remained disarmed throughout observed activation/validation.

## Validation

Native build/self-tests passed. Five protocol tests exercised CRC and identity
checks, wrong-message/command rejection, 150 ms freshness, future/repeated/
out-of-order data, 32-bit timestamp wrap, latest-only delivery, no replay and
feedback/sentinel filtering. Focused owner and VIO regression files passed.
The actual dashboard JavaScript passed stale/invalid/pause/disconnect and shared
USB-label checks.

The final 60-second window recorded 900 eligible sensor frames and 293 valid
Cube front reports. Every report matched a recent ToF average within 1 cm;
the largest front-feedback gap including window edges was 0.269 s, with zero
feedback CRC errors. All front values were within the ToF's 1–500 cm range.
Owner counters showed zero new rejected/superseded inputs during the window,
1089 additional VIO writes, and zero JT16 proximity writes. Aggregate Cube
health was healthy in all sampled polls, but **front-sector freshness and value**
are the stronger evidence because an independent rear sensor also contributes
to global proximity health. Cached owner-counter snapshot boundaries differ
slightly from the exact native-log window; deltas are not an exact packet-loss
or duplication estimate.

A controlled 2.5-second ToF-only pause produced no replayed USB ToF packets and
correctly aged the front report past one second. VIO wrote 47 packets while ToF
was paused. The ToF service was restored automatically and fresh front feedback
was reverified. No flight or avoidance-motion test was performed.

## Changes and reproducibility

Canonical native transport is `tools/tof_bridge/cpp/cube_udp.hpp`, used by the
existing main loop. The narrow Python adapter canonical copy is
`tools/tof_bridge/usb_owner_relay.py`, installed as
`optFlow_slam/src/optflow_slam/tof_transport.py`; its final hash matches the copy.
The existing owner's six small hook changes are retained in owner.patch with
before/after files and hash guard. Tests and artifacts are preserved here.

Service configuration adds only OPTFLOW_TOF_UDP_PORT=14605 and
OPTFLOW_TOF_FEEDBACK_PORT=14606 to the existing USB owner. The ToF boot launcher
uses --cube-udp-port14605 / --cube-feedback-port14606, with VIO relay disabled.
Final process descriptors verified ToF has only I²C7 and the USB owner has
/dev/ttyACM0. Camera/VINS, VIO observer and JT16 mapping services were active.

Evidence: minute-validation-final/, independent-packet-verification.json,
dropout-validation.json, final-state.json, effective-binaries-and-sources.json,
install-manifest.json, owner.patch, and saved previous boot/unit/source files.
The first minute-validation/ captures the initial successful run; the final
run also includes the additional unavailable-front guard for rear rangefinders.

Rollback should be scoped and disarmed: stop ToF, restore the saved native
boot/unit and owner-before.py if undoing the hooks, remove the ToF-specific
owner drop-in, reload/restart in one-owner order. Reverting to GPIO restores an
unreliable transport and must not be presented as healthy obstacle reception.
