# Forwarding verification and USB workaround — 2026-11-18

The user authorized verifying the forwarding fault and fixing it if confirmed, then explicitly requested USB testing to isolate the GPIO path. Testing was disarmed in actual Jetson 15 W mode. No Cube parameter, source selection, arming check, pose gate, recording or beep logic was changed.

## Verified cause and comparison

The earlier explanation that the five-second parameter freshness policy alone caused the fault was not established. A read-only native C++ audit running exclusively on TELEM2 also timed out on EK3_SRC2_POSXY. A dedicated C++ transaction probe then requested one named audit parameter at a time, retrying at 1/2/4 seconds rather than flooding requests.

- TELEM2 /dev/ttyTHS1 at 115200: 30 seconds, 30 Cube heartbeats, zero malformed packets, 14 requests, zero matching parameter replies, zero complete audits. One unsolicited STAT_RUNTIME parameter was received; it was not counted as a requested reply.
- Additional UART probe tested companion/GCS sender identities and read-only autopilot-version requests. Zero matching parameter replies, zero version replies, zero command acknowledgments. Incoming telemetry continued normally. The identity fixture exercised source system 1/components 197 and 196, and source 254/component 190; it did not exercise system 255 despite a reserved array entry.
- Same transaction probe on exact Cube USB if00 by-id endpoint: 30 seconds, 202 requests, 201 matching replies, 20 complete ten-parameter audit cycles, zero timeouts. The final request was still outstanding at test cutoff. Matching reply RTT median about 11.5 ms, p95 about 90.2 ms. One malformed packet was counted during this USB probe; its cause was not isolated, so this is not a zero-framing-error claim.

This isolates the operational failure to the TELEM2/UART request path, while proving the same Cube/parameter protocol responds over USB. Incoming UART telemetry alone does not prove bidirectional operation. It does not identify a particular defective GPIO pin, wire, mux setting, or Cube per-port behaviour. Physical UART repair remains unverified. Independent USB readback showed SERIAL2_PROTOCOL=2, SERIAL2_BAUD=115, SERIAL2_OPTIONS=0 and BRD_SER2_RTSCTS=0.

## Applied workaround

Installed two explicit, reversible user-systemd test drop-ins:

- Existing Cube owner: optflow-flight-logger.service.d/70-bhu-cube-usb-test.conf sets OPTFLOW_CUBE_ENDPOINT to `/dev/serial/by-id/usb-CubePilot_CubeOrange+_36003F000B51333338373339-if00`.
- VINS future startup: bhu-vins-fusion.service.d/70-bhu-vins-usb-test.conf sets BHU_CUBE_DEVICE to that same USB endpoint.

Repository sources are in deploy/systemd/. Existing base UART configuration remains available. The existing owner is still the only serial writer. Only that owner was restarted after fresh disarmed confirmation; the running VINS estimator/map was kept warm. Future startup waits for USB and Gemini. USB must remain connected, including after battery boot, while these drop-ins are installed.

The proposed audit-deadline/scheduler change was **not applied**: USB restored replies using the existing code. Byte comparison confirmed the adapter is unchanged from the pre-test copy. The five-second audit freshness limit, 1.5-second heartbeat guard, 150 ms pose freshness gate, native VINS publication policy, 10 Hz cap, quality filter and frame transforms were retained.

EKF source set 1 remains GPS horizontal position (3), optical-flow horizontal velocity (5), barometer height (1), no vertical velocity (0), compass yaw (1); EK3_SRC_OPTIONS=0. No source switching was sent. VISO_TYPE=3 and measured mount settings were preserved. Frame/source configuration readback is in usb-settings.json; active per-core source/fusion proof still requires the matching Cube BIN and remains a separate validation.

## Three-minute actual forwarding test

`usb-monitor.jsonl` contains 360 half-second samples over 180.23 seconds at 15 W. The Cube stayed disarmed throughout.

- 1,258 actual ODOMETRY serial writes in the monitored interval: approximately 7.0 Hz overall. This includes VINS withholding periods.
- Zero audit-blocked monitor samples in either the native relay or final owner, zero stale-heartbeat/disconnected samples, zero owner reconnects.
- Maximum oldest parameter-reply age 2.096 seconds, safely within the unchanged five-second limit.
- Pose age at write p95 146.34 ms, maximum 149.95 ms. This is host-estimated capture-to-write age, not calibrated exposure-to-Cube latency. Serial write is enqueue/call evidence, not per-pose acknowledgment.
- Cube vision health true in 314/360 samples; VINS gate good in 293/360 samples. These snapshots have different cadence/phase and are not identical measures.
- Recording thread alive in every sample; recording remained inactive because the aircraft remained disarmed; ten-beep recording cue count stayed zero. Existing arm/disarm and cue logic was preserved, not physically tested by arming.

A 8.999-second write gap remained. `longest-gap-analysis.json` found 150 VINS frames inside that gap, **zero gate-eligible frames**, median stereo support 20 and median frame age 85.6 ms; audit guards stayed open. This is a real input/publication continuity problem rather than the previous transport audit stall. Median support at the threshold does not imply sustained 500 ms good support. No stale or fabricated pose was sent to conceal it. VisOdom can therefore still report not healthy during genuine input gaps even though the USB request path works.

At the final immediate check, the Cube vision-health flag was true and forwarding guard open. Recent pre-arm history contained GPS bad fix and one VisOdom not healthy during an input gap. No full pre-arm success or flight readiness is claimed. No Cube BIN was downloaded; vision health is a frontend reception proxy, not proof of fusion. No live armed test or GPS/VIO source switch was performed.

## Startup and evidence

Effective systemd environment confirms both owner and future VINS startup use USB. Camera/USB access guard and the fresh-disarmed Cube startup guard passed. default.target transaction verification succeeded without a dependency cycle. A new physical cold boot of this exact override remains untested.

Evidence: transport-comparison.json, transaction-probe/, identity-probe/, usb-transaction-probe/, usb-settings.json, usb-validation-summary.json, longest-gap-analysis.json, usb-monitor.jsonl, usb-owner-events.jsonl, effective-environment.txt, services.txt and power-mode.txt. C++ probe sources are retained beside this report for reproducibility; these are read-only diagnostic tools, not new flight services.

Rollback only while freshly disarmed: remove the two named USB-test drop-ins, reload user systemd, and restart the existing Cube owner **after** validating UART bidirectionally. Do not change flight-controller settings or weaken watchdogs to hide a one-direction link.

Primary protocol references: [MAVLink parameter transactions](https://mavlink.io/en/services/parameter.html), [ArduPilot parameter request/reply implementation](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.1/libraries/GCS_MAVLink/GCS_Param.cpp).
