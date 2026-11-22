# Cube proximity disabled — 2026-11-18

Operator requested removal of the disconnected proximity sensor. The battery warning was absent in both fresh 36-second checks; battery-monitor configuration was preserved.

Changed only PRX1_TYPE from 4 to 0 using the native C++ configuration tool, after fresh disarmed telemetry. Saved before/after snapshots and rollback in config/. All other readable audited values were preserved. Rebooted the disarmed Cube to unload the existing backend, then independently read back PRX1_TYPE=0 over TELEM2 /dev/ttyTHS1 at 115200. No arming, mode or source-selection commands.

Post-reboot: armed=false; 41 heartbeats; 0 malformed packets. GPS fix type 1, satellites 0.

Fresh status messages:

- Initialising ArduPilot
- Calibrating barometer
- Barometer 1 calibration complete
- Barometer 2 calibration complete
- ArduPilot Ready
- AHRS: DCM active
- RCOut: PWM:1-14
- GPS 1: specified as DroneCAN1-125
- GPS 1: detected DroneCAN1-125
- EKF3 IMU0 initialised
- EKF3 IMU1 initialised
- EKF3 IMU2 initialised
- AHRS: EKF3 active
- EKF3 IMU0 fusing optical flow
- EKF3 IMU1 fusing optical flow
- EKF3 IMU2 fusing optical flow
- EKF3 IMU0 tilt alignment complete
- EKF3 IMU0 started relative aiding
- EKF3 IMU1 tilt alignment complete
- EKF3 IMU1 started relative aiding
- EKF3 IMU2 tilt alignment complete
- EKF3 IMU2 started relative aiding
- EKF3 IMU0 MAG0 initial yaw alignment complete
- EKF3 IMU1 MAG0 initial yaw alignment complete
- EKF3 IMU2 MAG0 initial yaw alignment complete
- PreArm: RC not found
- PreArm: Gyros inconsistent
- PreArm: Check mag field: 1126, max 875, min 185
- PreArm: Rangefinder 1: No Data

UART owner was automatically restored after each exclusive serial session. VIO service remained enabled/running. Rangefinder parameters were left unchanged. Proximity sensing is disabled; reconnecting JT16 alone will not re-enable the Cube backend. Simulated Cube integration test passed, including the single-write proximity case and unrelated-parameter preservation.
