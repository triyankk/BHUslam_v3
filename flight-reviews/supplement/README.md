# Optical-flow and remaining-evidence supplement

[Download the dataset release](https://github.com/triyankk/BHUslam_v3/releases/tag/flight-data-20261122).

**22 November recorder sessions; 7980 readable optical-flow messages.** These are numerical MAVLink measurements, not H-Flow camera images.

`optical-flow/<Desktop session>/optical-flow.csv` preserves timestamps, source IDs, flow fields, quality, ground distance, and the original fields JSON. `summary.json` records message counts, corrupt tails and metadata/count discrepancies. Empty sessions remain explicit. No automatic VIO/telemetry flight match is asserted.

## Downloadable archives

| Asset | Bytes |
| --- | ---: |
| [optical-flow-and-cube-telemetry-20261122.tar.zst](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/optical-flow-and-cube-telemetry-20261122.tar.zst) | 15,560,594 |
| [flight-runtime-diagnostics-20261122.tar.zst](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/flight-runtime-diagnostics-20261122.tar.zst) | 21,488,013 |
| [post-review-report-evidence-20261122.tar.zst](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/post-review-report-evidence-20261122.tar.zst) | 55,223,098 |
| [effective-settings-and-unmatched-evidence-20261122.tar.zst](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/effective-settings-and-unmatched-evidence-20261122.tar.zst) | 230,058,538 |
| [tof-transport-bytes-20261122.tar.zst](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/tof-transport-bytes-20261122.tar.zst) | 3,335,131 |

Download `supplement-catalog.json` and `SUPPLEMENT-SHA256SUMS`; verify the sums before extraction. Each archive has one top-level directory with a member hash manifest. Snapshot provenance lists the original source paths.

The telemetry archive preserves the full available November CSVs and recorder manifests. Other archives preserve forwarding/observation logs, estimator and performance diagnostics, front-ToF measurements and Cube feedback, report JSON/CSV/log evidence, sanitized installed settings, and the separately labelled unmatched JT16 capture.

Installed settings are a current snapshot, not a verified historical flight configuration. Credential-setting lines are redacted. Recent/changing files were left local; no services or sensors were stopped.

## What remains local

[Remaining local data inventory](remaining-local-data.json) lists sizes, examples and exclusions. Older calibration/baseline recordings, historical external experiments, generated viewers/images, duplicate map checkpoints and code/binary backups remain local. Builds/dependencies are rebuildable. Useful older experiments have not been silently declared backed up.

See [storage and restoration instructions](../../docs/flight-data-storage.md).
