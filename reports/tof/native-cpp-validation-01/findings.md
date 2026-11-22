Native C++ viewer validation
============================

The native C++17 executable directly read TMF8820 serial 23937420 over I²C7
and sent MAVLink DISTANCE_SENSOR through /dev/ttyTHS1 at 57600 baud. No Python
process was used by the executable. The browser dashboard uses the existing
HTML/CSS/JavaScript and the same HTTP API.

The 35-second capture produced 523 calibrated frames at 15.000 Hz, with no
duplicate results, missing sensor result counters, rejected readings, or bad
UART packets. Independent pymavlink decoding matched all 523 outgoing distance
packets against the sensor logs, including units, orientation, timestamps, and CRC.

The Cube returned 135 rear RANGEFINDER2 reports (system 1/component 1, sensor ID 1,
orientation 4). Every report matched a transmitted distance from the preceding
second. Readback confirmed TYPE=10, ORIENT=4, MIN=0.01 m, MAX=5 m. No parameter
writes or proximity processing were performed.

Existing prearm messages concerned GPS/EKF/AHRS, battery voltage, RC, and downward
Rangefinder 1. No rear Rangefinder 2 error appeared in the captured status messages.
These checks validate bench data transport, not obstacle-avoidance behavior or flight.

Native protocol self-tests passed; the actual dashboard script passed fresh,
stale, invalid-calibration, disabled-rangefinder, pause, and disconnected-API checks
using api-snapshot.json. Timed shutdown completed cleanly and released the hardware.
See verification.json and bridge-001/summary.json for machine-readable results.
