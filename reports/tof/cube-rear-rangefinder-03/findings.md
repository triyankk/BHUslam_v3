# Rear ToF as a Cube rangefinder

Final requested mode is **rangefinder only**, without proximity processing.
The prior proximity change was undone (`PRX1_TYPE=0`) before the final reboot.

Verified after a reboot of the disarmed Cube:

| Parameter | Value |
| --- | --- |
| RNGFND2_TYPE | 10, MAVLink |
| RNGFND2_ORIENT | 4, rear |
| RNGFND2_MIN | 0.01 m |
| RNGFND2_MAX | 5.0 m |
| PRX1_TYPE | 0, disabled |
| SERIAL1_PROTOCOL | 2, MAVLink 2 |
| SERIAL1_BAUD | 57, 57600 baud, also verified at runtime |

RNGFND1 was already configured as a downward-facing MAVLink rangefinder
(type 10, orientation 25, minimum 0.08 m, maximum 30 m); those settings were
read before and after and left unchanged. Original settings for slot 2 and
the early partial attempts are recorded in `../cube-rear-rangefinder-01` and
`../cube-rear-rangefinder-02`; all final values are in `configuration.json`.

The live viewer now requests and plots the Cube's own **rangefinder** reports.
It accepts source system 1/component 1, sensor ID 1 (zero-based ID for slot 2),
orientation 4. Forwarded Jetson data and proximity-sector IDs do not count as
rangefinder confirmation. Raw UART decoding and the monitoring outcome are
recorded in `rangefinder-validation.json`.

The monitored period confirmed fresh and changing rear rangefinder returns,
valid sensor streaming, and no rear-rangefinder error messages. The Cube also
reports existing pre-arm issues: RC not found, altitude estimate unavailable,
EKF attitude/AHRS configuration, battery below minimum arming voltage,
downward Rangefinder 1 No Data, and waiting for GPS configuration. These are
not silently suppressed or fixed by this rangefinder integration.

The viewer remains running at http://127.0.0.1:8765 with a live telemetry
message panel. Neither avoidance configuration nor arming checks were changed.
The 5 m maximum is the sensor's advertised upper limit, not a validated usable
flight range or an absolute ranging accuracy claim.
