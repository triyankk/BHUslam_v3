> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> This historical design is an intermediate proposal; the selected Gemini-depth plan is linked below. See the [shared project goal](../PROJECT_GOAL.md).

# VINS-Fusion navigation with JT16 obstacle avoidance

**Superseded design, 20 November 2026:** the selected direction is Gemini
depth for obstacle detection and local replanning, with JT16 reserved for 3D
mapping. See the [Gemini depth avoidance plan](gemini-depth-avoidance-plan.md).
The historical proposal below is retained for context; this documentation
change does not switch live obstacle sources.

Proposed integration, 2026-11-17. No flight parameters, boot services or live transmission changed by this assessment.

## Responsibilities

Gemini stereo/IMU -> existing C++ VINS-Fusion -> publication gate -> MAVLink ODOMETRY -> Cube EKF3.

JT16 -> one acquisition owner -> body-frame obstacle filtering -> MAVLink proximity -> Cube avoidance controller.

VINS-Fusion estimates motion; its tracked sparse features do not establish collision-free space. Use measured JT16 returns for the initial obstacle source. Mapping is an optional consumer of the same LiDAR acquisition, not a prerequisite for reactive proximity. Keep the existing estimator implementation and avoid adding an independent Jetson flight controller for this increment.

## Reuse and boundaries

The existing implementation in `/home/atas/vscode/optFlow_slam` has a native C++ JT16 reader and an obstacle service. Its saved profile enables LiDAR proximity and disables camera depth contribution. These are inspected configuration facts, not proof of current Cube configuration or flight readiness. Reuse its acquisition/proximity path after reviewing its ownership and aircraft-specific settings; do not launch a competing serial reader or copy historical mount/airframe dimensions into this project.

The BHU isolated mapping launcher pauses that service and refuses a connected Cube. It is not the combined-flight launcher. Shared mapping needs the existing acquisition owner to expose its full mapping feed on port 18777. VINS and proximity must share a single owner per Cube serial link, or use independently verified separate physical links. Preserve freshness gating and avoid repeating old measurements as new observations.

New runtime additions should be C++; existing Python orchestration in the other checkout remains unchanged unless a separately justified migration is needed.

## Validation order

1. Connect the Cube disarmed. Snapshot current firmware, parameters, active EKF source and link ownership. Verify origin and actual VINS measurement fusion using `docs/ekf-verification-deferred.md`; saved BHU tests do not establish it.
2. Verify actual JT16 mounting, coverage and aircraft envelope. With props removed, move a target around observed directions and confirm correct Cube proximity directions/ranges. Treat blind sectors and stale input as unknown, not clear. Check ground/self returns and deliberate sensor interruption.
3. Validate the exact firmware/mode combination. The other checkout's September flight evidence identifies ArduCopter 4.7.1 dbe79216 with horizontal AltHold/FlowHold avoidance compiled out. Healthy PRX cannot enable missing code. Re-read current firmware; do not infer current support from a generic parameter preset.
4. Start with verified position-controlled reactive stopping/sliding in Loiter, subject to successful VINS EKF fusion. Autonomous detours are a later increment: ArduPilot documents BendyRuler for AUTO, GUIDED and RTL. Verify the chosen command type and firmware implementation before relying on it.
5. Benchmark VINS + proximity + optional mapping concurrently at 15 W. Record pose age/gaps, LiDAR observation age, link queueing, CPU/thermal load and rejection/fault events. JT16 supplies about 5 Hz clouds in prior bench captures; a 10 Hz sender does not produce 10 Hz independent observations.

Set speed and clearance from measured worst-case observation/control delay and braking, plus the physical aircraft envelope and uncertainty. A starting model is reaction distance >= v * delay + v^2 / (2 * effective braking acceleration) + clearance. Do not promote unmeasured acceleration or a universal distance into a flight setting. Define and validate VIO-loss and proximity-loss responses before enabling flight; position hold cannot be assumed available after localization loss.

## References

- https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
- https://ardupilot.org/copter/docs/common-simple-object-avoidance.html
- https://ardupilot.org/copter/docs/common-oa-bendyruler.html
- Existing implementation/evidence: `/home/atas/vscode/optFlow_slam/AGENTS.md`, `config/system.yaml` and `data/diagnostics/2026-09-27-tree-contact/`.
- BHU saved EKF audit: `reports/ekf-saved-run-audit-20261117/review.md`.
