# Brief recovery policy replay — 18 November 2026

Replayed existing diagnostic decisions made by the actual C++ DurationGate on identical captured VINS samples. Candidate:500 ms startup/fault recovery,100 ms brief recovery only if the first good return is within200 ms of the last published capture, no pending discontinuity, unchanged150 ms freshness/feature/IMU checks. This does not run the estimator again or send data to the Cube.

Fresh stationary45-second window:443 frames,267 accepted under each policy,0 additional candidate frames. The modeled latest-only10 Hz transport produces231 packets for each,19 gaps>300 ms, maximum2030 ms, modeled receipt coverage64.377% each. These modeled packets differ from the174 observed actual serial writes in the stationary report; simulation omits actual scheduling/transport jitter and must not be presented as hardware reception proof.

Full five-minute window:3026 frames,2059 accepted under each policy,0 additional candidate frames. Both modeled transports:1757 packets,94 gaps>300 ms, maximum5935 ms, modeled receipt coverage70.109%. One modeled reset change in each. No candidate100 ms target samples in this five-minute window.

Decision:do not promote this candidate based on these results. It never reaches its short-recovery path under the recorded interruptions, so enabling it would not resolve the measured unhealthy periods. Keep live500 ms gate and all source settings unchanged. Address processing latency and velocity validity across processed frame gaps next; widening the candidate interruption bound would constitute a different policy requiring its own comparison.

No code/service/Cube configuration changes, estimator restart, recording interruption or runtime Python addition. Outputs saved under replay/ and five-minute-replay/.
