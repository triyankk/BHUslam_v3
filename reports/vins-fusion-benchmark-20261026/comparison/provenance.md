# Comparison inputs

VINS-Fusion: `../full/`, including power/boot/commit/checksum manifests and per-run configurations.

Existing VINS-Mono and ORB-SLAM3: `../../vins-mono-benchmark-20261026/full/`, with their original manifests. These historical results are reused, not rerun or relabelled as new Fusion-session measurements. Both sessions verified 15 W mode. Input directories here are relative symlinks; the generated HTML embeds all plotted data and opens independently.

All variants use the same source recordings, original factory extrinsics and zero time offset. Loop closure and online calibration are disabled. Fusion's upstream failure detector is entirely disabled; its native initialized flag must not be interpreted as successful visual tracking.
