# Rear ToF proximity enabled — 28 October 2026

Changed only PRX1_TYPE from 0 to 4 (RangeFinder), verified the write, and
requested a reboot after a fresh disarmed Cube heartbeat. Full before/after
readbacks are in configuration.json. Avoidance tuning, other proximity
backends, serial settings, RC7 assignment, and downward/rear rangefinder
settings were preserved. No arm or flight-mode commands were sent.

The first post-reboot heartbeat was still in boot state, and the immediate
parameter read timed out. Later independent readback over the unowned Cube USB
port confirmed PRX1_TYPE=4 persisted and every other audited setting was
unchanged. UART parameter readback subsequently worked in the live viewer too.
The initial timeout and recovery are retained in configuration.json.

## Bench verification

- 501 calibrated TMF8820 samples at 15 Hz over PCA9548A 0x70, channel 0.
- All 501 outgoing MAVLink distance packets matched the sensor logs, including
  units, orientation, timestamps, and CRC under independent pymavlink decoding.
- Cube returned 130 rear proximity-sector reports: source system 1/component 1,
  sensor ID 14, orientation 4. No malformed UART packets were observed.
- After the input stream stopped, a six-second query observed no stale rear
  sector reports. The backend must receive fresh sensor data to advertise it.
- Native self-tests and actual browser-script tests passed, including rear
  proximity labeling and stale/disconnected states.

The installed ArduPilot source suppresses horizontal raw rangefinder telemetry
when a RangeFinder proximity backend is enabled and sends proximity-sector
telemetry instead. Rear rangefinder ID 1 therefore becomes proximity ID 14
(PROXIMITY_SENSOR_ID_START=10 plus rear sector 4). The C++ viewer now recognizes
both report types while retaining source, orientation, and ID filtering; it
labels proximity reports explicitly. Parameter polling is paced to avoid
overrunning ArduPilot's small pending-reply queue.

## Limits and next physical test

This establishes rear proximity ingestion and telemetry, not flight avoidance
performance. The sensor still sees a near return of about 4 cm. Clear wiring,
housing, and other obstructions from the optical face, and compare controlled
flat-target distances over the intended operating range. The Cube's filtered
proximity reading need not equal each instantaneous raw sample.

AVOID_ENABLE=7, AVOID_BEHAVE=1 (Stop), and AVOID_MARGIN=2 m remain set. Existing
horizontal backup speed is 1 m/s and vertical backup speed is 0.75 m/s; Stop
does not disable backing away from a close obstacle. No avoidance motion was
tested. Prior RC, EKF/altitude, visual-odometry, battery, and downward-rangefinder
prearm problems remain unresolved. AVOID_ALT_MIN=1.5 m and PRX_IGN_GND=1 also
need a valid downward rangefinder for their intended checks. The readiness audit
records the AltHold support uncertainty for this firmware build.

Sources:

- https://github.com/ArduPilot/ardupilot/blob/dbe79216/libraries/GCS_MAVLink/GCS_Common.cpp
- https://github.com/ArduPilot/ardupilot/blob/dbe79216/libraries/AP_Proximity/AP_Proximity.h
- https://ardupilot.org/copter/docs/common-rangefinder-setup.html
