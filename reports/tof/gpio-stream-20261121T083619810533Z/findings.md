# GPIO ToF stream selected — 21 November 2026

At the operator's explicit request, moved the sole front ToF sender from
primary Cube USB to /dev/ttyTHS1, Jetson header pins 6/8/10 to Cube TELEM2,
at 115200 8N1 with no hardware flow control. The sensor remains directly on
I²C7. The mean of primary zones with confidence >50 and no four-zone minimum
is unchanged. No Cube parameters, boot configuration or pinmux were changed.

Before handoff, a fresh disarmed Cube heartbeat was verified from the existing
USB owner's capture. That owner was stopped. USB then had only a diagnostic
observer sending read-only parameter and message requests, never ToF distances.
It confirmed SERIAL2_PROTOCOL=2, SERIAL2_BAUD=115, BRD_SER2_RTSCTS=0,
SERIAL2_OPTIONS=0, PRX1_TYPE=2. Baseline front proximity reports were zero after
the former USB ToF stream stopped.

The bounded GPIO run recorded 372 calibrated sensor frames and outgoing front
distance packets at 15 Hz. Independent pymavlink checks matched all encoded
distances to confidence-gated zone averages, units, timestamps and CRCs.
The USB observer received 31 raw companion ToF packets forwarded by Cube and
28 processed front proximity-sector reports. Thus actual GPIO-origin reception
is demonstrated independently of the sender's write log.

Reception is not reliable: the largest gap between observed processed front
reports was about 16 seconds. There was one malformed packet in USB observation
and zero in GPIO return capture. These counts are not a precise packet-loss
estimate because raw MAVLink forwarding and processed-report schedules differ.
They do establish that fresh proximity feedback was not continuous. No
avoidance-motion or flight test was performed.

The bounded sender and USB observer closed cleanly. A new continuous sender
was started at live-gpio/ using GPIO as requested. USB is not a distance-data
fallback in this running configuration. The viewer flags stale Cube reports;
the sender being Running does not establish healthy Cube reception. JT16
remains stopped/runtime-masked, and the shared VIO-to-Cube relay remains paused.

Evidence: observer-ready.json, usb-observer.json, usb-rx.bin,
gpio-bridge/bridge-001/, verification.json, live-command.json and live-gpio/.
