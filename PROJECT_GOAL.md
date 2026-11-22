# Project end goal: autonomous GPS-denied missions with obstacle avoidance

Confirmed by the project owner on **20 November 2026**:

> We are building a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.

This is the end goal of BHUslamV3. Localization, mapping, sensor integration,
visualization and benchmarking support this complete mission capability.
A successful individual estimator or sensor test is an intermediate milestone.

## What the completed system must do

- Estimate vehicle motion and maintain navigation without depending on GPS
  for mission execution. Any map, reference frame, heading or origin needed
  for navigation must have an explicit initialization method.
- Execute the operator's intended mission or goals autonomously, with the Cube
  providing stabilization and flight control.
- Detect obstacles in the relevant flight volume and avoid collisions while
  continuing toward the mission goal when a usable route is available.
- Handle localization loss, stale/missing obstacle observations, blind regions
  and blocked routes through explicit, tested behavior. Unknown space must not
  silently become clear space.
- Produce repeatable evidence of mission progress, navigation health, obstacle
  clearance and recovery behavior so multiple operators can assess results.

“Guided mission” describes the desired autonomous goal-directed behavior. It
is not a declaration that the implementation must use only ArduPilot GUIDED
or only AUTO mode. The supported mode, goal commands and replanning behavior
must be chosen and verified for the installed firmware.

## How the current work contributes

| Workstream | Contribution to the end goal | Scope / status reference |
| --- | --- | --- |
| VINS-Fusion stereo + IMU | Existing primary localization implementation and comparison baseline for GPS-denied navigation | [Startup](docs/vins-fusion-boot.md), [observation transport](docs/vins-observation-forwarding.md) |
| Joint JT16/Gemini LIVO evaluation | Investigate more robust localization/mapping with LiDAR, images and one IMU | [Offline experiment](docs/livo-offline-replay.md); evaluation does not establish a live replacement |
| Gemini depth processing | Planned navigation obstacle detection, coverage/health reporting and local avoidance/replanning input | [Selected depth avoidance plan](docs/gemini-depth-avoidance-plan.md); plan is not an implemented capability claim |
| JT16 mapping | 3D mapping and recorded evidence; joint-estimator evaluation where explicitly selected | [Mapping](docs/hesai-jt16-mapping.md), [isolated mapping](docs/jt16-only-mapping.md) |
| Cube integration | Accept localization/obstacle observations and execute verified flight-control and mission behavior | [EKF verification](docs/ekf-verification-deferred.md), [flight test stages](docs/flight-shadow-test.md) |
| Replay, visualization and operator procedures | Diagnose failures and make configuration, timing and test results reproducible | [Operator guide](docs/operator-testing-guide.md), [Fusion viewer](docs/vins-fusion-viewer.md) |

VINS-Fusion remains the existing primary implementation while the explicitly
selected LIVO experiment is evaluated. ORB-SLAM3 and VINS-Mono are reference
and comparison tools. The selected avoidance direction uses Gemini measured
depth; earlier JT16 proximity proposals and single-ToF rangefinder tests are
historical or supporting work, rather than a definition of the final system.

## Evidence and progression

Progress from checked sensor inputs and calibration to sustained localization,
verified Cube fusion, goal-directed GPS-denied control, obstacle response and
finally combined autonomous mission trials. At each stage, record what was
actually tested, the applicable configuration and remaining failures. Offline
agreement, initialization, packet reception, a map or a successful GPS-aided
observation flight alone does not demonstrate the complete end goal.

GPS-aided shadow flights and disarmed/offline tests remain useful development
methods. Their narrower scope does not redefine the project's GPS-free target.
Dated documentation and saved reports retain the conditions of their own tests;
use the relevant latest readbacks and evidence before operating hardware.

This goal update changes documentation and agent guidance. It does not apply
flight parameters, enable an obstacle source, select an EKF source, or issue
mission, mode, movement or arming commands.

## Current test-source selection —21November2026

The operator selected a front ToF sensor over Jetson GPIO/UART as the Cube
proximity source. JT16 is now for raw point recording and flight mapping only;
it must not publish obstacle distances. See docs/tof-front-proximity.md for the
source/serial-owner split and live evidence. This supersedes earlier active
source assumptions for the current test setup. Gemini-depth avoidance remains
an alternative development plan. Front-only coverage and intermittent GPIO
reception do not satisfy the complete all-around mission-avoidance objective.
