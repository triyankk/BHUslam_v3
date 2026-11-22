# Front proximity boot repair and response tuning —21November2026

Observed: ToF valid~15Hz, USB owner inactive, stale runtime status; systemd
reported a dependency cycle and deleted the owner start job. Removed front
unit After=optflow-flight-logger.service, retaining Wants. Installed unit updated,
daemon reloaded, existing owner started; no parallel Cube writer. Dependency
verification passed. Future actual boot validation is still required.

Front-only genuine Cube reports returned, not just global rear-supported health.
Recovery samples and final-health.jsonl retain fresh runtime timestamps,
forwarded/report counters and VIO write counts. No synthetic clear data or
freshness extension was used. Acquisition/aggregation/confidence remain native
and unchanged. Services retain recording/cues, GPS-reference and mapping roles.

Approved response candidate: PRX_FILT2Hz, AVOID_ACCEL_MAX1m/s². Actual readback
found PRX_FILT already2Hz and acceleration0.75; exactly one write0.75->1 was
needed. applied/{before,after,requested,changes,summary}.json and restore.param
retain the scoped audit and rollback. Stop1, margin2m, altitude gate2m,
RNGFND_FILT0.5 and all other audited values were preserved. Cube remained
disarmed; no reboot, mode/source/arming/mission command.

Tests: responsive_avoidance_config and rangefinder_config passed. Cube integration
initial sandbox run could not bind its test UDP socket; rerun with host loopback
passed31.24s. Binary SHA256:
81b79cbd0249359f405a505ae2baa1e41b6dbdae71a06973426347feb80da939

This validates configuration and stationary transport only. Controlled braking,
clearance and actual reboot validation remain separate.

## Actual reboot check

Boot ID39f51815-f44a-4eb5-9875-9936c1daee1f differs from repaired
boot6d2caa31-b95a-4215-a383-3b5554fe3a4e. Before any intervention, both front
ToF and USB owner were active automatically, genuine front feedback advanced,
and171/171 accumulated proximity health samples were healthy. VINS, relay and
JT16 mapper were also active. The installed owner retains its60s-uptime startup
wait, so readiness is not immediate at power-on.

Read-only exclusive post-boot audit verified PRX_FILT2, AVOID_ACCEL_MAX1,
AVOID_BEHAVE1 and AVOID_MARGIN2; zero writes and no reboot command. Owner restored
on exit. See reboot-check/. A pre-arm EKF position-variance warning and missing
origin/Home were observed independently; this does not validate navigation or
flight avoidance performance.
