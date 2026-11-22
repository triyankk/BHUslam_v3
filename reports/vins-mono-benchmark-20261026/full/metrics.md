# Recorded-data comparison

No ground truth; these are availability, runtime and stationary-tail consistency metrics. Native health flags differ.

| Run | Frames | Native initialized poses | Init (s) | Longest later native gap (s) | Active pipeline p95 (ms) | Peak RSS (MiB) | Quiet-tail drift (m) |
|---|---:|---:|---:|---:|---:|---:|---:|
| two-minute-vins | 3597 | 3545 | 1.7353960000000015 | 0.0 | 78.673278 | 102.398 | 0.0019922687533596778 |
| two-minute-orb-mono | 3597 | 458 | 6.507736000000001 | 65.04399313383504 | 34.586663 | 723.625 | unavailable |
| two-minute-vins-native-noise | 3597 | 3017 | 1.7353960000000015 | 17.620948370788337 | 76.830999 | 87.2812 | unavailable |
| two-minute-orb-stereo | 3597 | 2196 | 4.338490999999998 | 29.067890210145286 | 63.750082 | 713.438 | unavailable |
| eighty-second-vins | 2399 | 2350 | 1.6352779999999996 | 0.0 | 75.794961 | 80.8555 | unavailable |
| eighty-second-orb-mono | 2399 | 728 | 6.808093999999997 | 17.487456337676303 | 34.250652 | 605.223 | unavailable |
