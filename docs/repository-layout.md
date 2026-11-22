# Repository scope and local data

The Git root is `/home/atas/BHUslamV3`, the common parent of the VIO source,
configuration, launchers, and supporting tools. The initial commits capture the
existing workspace; they do not activate services, change Cube parameters, or
establish hardware or flight validation.

## Tracked project files

- `tools/vins_fusion/`: native VINS-Fusion integration and live camera/IMU path.
- `src/`, `include/`, and `tools/`: native acquisition, integration, diagnostics,
  offline experiments, and supporting utilities. Small vendored ToF libraries,
  licenses, and sensor firmware under `tools/tof_bridge/` are retained.
- `configs/`, `bin/`, `scripts/`, `deploy/`, `patches/`, and `CMakeLists.txt`:
  settings, launchers, build scripts, deployment definitions, and source patches.
- `tests/`, `web/`, and `assets/`: checks, operator interfaces, and design assets.
- Root documentation, `docs/`, and written reports under `reports/`: procedures,
  architecture, and dated evidence. Report YAML/parameter snapshots and SHA-256
  identity files are retained alongside Markdown findings.

## Files intentionally kept outside Git

`.gitignore` excludes build trees, downloaded packages and dependency checkouts,
recordings, runtime sessions, logs, caches, local locks, credentials, and generated
report data. These files remain on the original machine. In particular,
`reports/` images, telemetry, JSON/CSV datasets, generated HTML viewers, maps,
and historical source/binary backups are not an archive supplied by a clone.
References in dated reports can therefore point to local-only evidence.
Back up those datasets separately when transferring or preserving a test.

Selected arm-session evidence is now preserved separately through
[flight review packages and release archives](flight-data-storage.md).
`flight-reviews/` contains committed summaries, small diagnostics and sampled
map previews; complete selected input/map/raw files are downloadable release
assets, with local archive copies under the ignored `release-assets/` directory.

`bin/` contains launch scripts and is tracked; compiled programs in `build/`
are rebuilt locally. The small ToF firmware asset is a required vendor input,
not a generated executable, and remains tracked.

## External dependencies and rebuild limits

`configs/dependency-sources.json` records the observed upstream URLs and exact
commits of dependency checkouts on 22 November 2026. Dependencies are not Git
submodules. `scripts/bootstrap.sh` stages its documented SDK, system packages,
and ORB-SLAM3/Pangolin dependencies; VINS and LIVO also require the staging
steps in their existing benchmark/replay documentation. The manifest records
source identity, not a complete reproducible dependency installer or SDK lock.

`patches/dependency-snapshots/ORB_SLAM3.patch` captures the complete observed
tracked-file delta against the manifest's ORB-SLAM3 commit. Apply it only to a
clean checkout at that commit, as an alternative to the overlapping individual
ORB patches; do not apply both sets. Other manifest checkouts had no tracked
changes at capture time. Installed SDK/sysroot trees are not captured.

Some existing checks consume local recordings or generated reports. Those
checks need their documented datasets after a fresh clone. Repository setup
validation checks Git contents and ignore rules; it does not rerun hardware
tests or prove the current VIO system's performance.
