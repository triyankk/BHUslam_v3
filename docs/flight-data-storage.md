# Flight review and replay data storage

Flight recordings support the project's GPS-denied mission and obstacle-avoidance
evaluation. An arm-triggered recording does not by itself establish takeoff,
accurate localization, Cube fusion, or autonomous flight. VINS-Fusion remains
the primary VIO; the JT16 maps here are independent FAST-LIO2 results, not joint
VIO/LiDAR estimates.

## Where the data lives

| Layer | Location | Contents |
| --- | --- | --- |
| Git review packages | `flight-reviews/<run>_flight-<host-us>/` | README, manifest, input-integrity findings, poses, tracking, saved boot settings, optional JT16 trajectory and map preview, per-file hashes, download links |
| Downloadable replay datasets | [GitHub release flight-data-20261122](https://github.com/triyankk/BHUslam_v3/releases/tag/flight-data-20261122) | One losslessly compressed archive per saved arm-triggered session; large archives are split into ordered parts |
| Local copies of release assets | `/home/atas/BHUslamV3/release-assets/flight-data-20261122/` | The uploaded archives, `catalog.json`, and `SHA256SUMS`; ignored by Git |
| Original VIO files | `/home/atas/BHUslamV3/runs/vins-fusion-boot/<run>/live/flights/flight-*/` | `pose_history.csv`, `tracking.csv`, `flight.json`, and full `input/` folder |
| Original JT16 mapping | `/home/atas/BHUslamV3/runs/flight-shadow/<boot-uuid>/mapping-*/data/flights/flight-*/` | Maps, trajectories, scan timing, IMU, completed checkpoints |
| Original JT16 raw input | `/home/atas/vscode/optFlow_slam/data/recordings/jt16-shadow/<boot-uuid>/flight-*.raw` | Original serial packet bytes |
| Original Cube telemetry | `/home/atas/Desktop/Drone_Sensor_Recordings/<session>/cube_mavlink.csv` | Desktop recorder output; not automatically attributed to a flight |
| Audited Cube DataFlash files | `/home/atas/BHUslamV3/reports/latest-flights-20261120/cube-logs/` | Original Cube `.BIN` logs; only independently matched logs enter per-flight archives |

Start with [the review catalog](../flight-reviews/README.md). A normal Git clone
includes the lightweight reviews, not the large raw recordings. Archives are
separate release assets. The original recordings are not moved or deleted.

## What each archive contains

All paths are relative to a single `<run>_flight-<host-us>/` directory:

```text
<session>/
  manifest.json
  files.json                 # member size, SHA-256, and original absolute path
  boot/                      # saved boot settings, status, executable hashes
  vio/
    flight.json
    pose_history.csv
    tracking.csv
    input/
      left/*.pgm
      right/*.pgm
      frames.csv
      capture.csv
      accel.csv
      gyro.csv
      orb.yaml
      recording.json
  jt16/                      # only when associated evidence exists
    trajectory.csv
    clouds.csv
    imu.csv
    summary.json
    map.ply                  # full selected map, including empty maps as evidence
    raw/*.raw
    calibration/             # installed angle-table copy and provenance
  cube/                      # independently matched DataFlash/observation logs
    cube-log-<id>.BIN
    observation/*.jsonl
```

Missing files remain missing. Unfinalized recordings, damaged images, empty
maps, checkpoint recovery and unavailable Cube evidence are explicitly listed
in each review's `manifest.json` and README. The image check validates PGM
headers and exact payload length; it does not establish image quality,
stereo calibration, timestamp synchronization, or VIO accuracy. Original bytes
are preserved rather than silently trimming invalid tails.

`input-inventory.json` in each Git review separately counts every saved PGM
file, lists structurally invalid files, and identifies files absent from the
readable frame-index prefix. The catalog-level `input-inventory.json` combines
these audits and is also supplied as a release asset. Metadata's
`frames_written` can exceed the readable index row count after corruption;
unindexed images are retained in the archive but are not claimed to have a
trustworthy recovered pairing/timestamp index. This audit is an external
addendum and does not change the immutable archives or their internal manifests.

The VIO frames are the selected processed, rectified stereo pairs before
CLAHE. They do not include overwritten camera pairs or native unrectified
images. Each replay archive retains the complete available input directory.

The Git map preview contains at most 5,000 deterministically sampled vertices.
It is a subset of the full exported map solely to keep reviews small. The full
selected `map.ply` is in the archive. If the final export is absent, the latest
completed numbered checkpoint is used and identified; `.tmp` generations are
excluded. Duplicate checkpoints, generated HTML viewers, and duplicate
`map-data.json` exports are not archived.

## How recordings are associated

- A package ID combines the original VIO run folder and monotonic arm-start ID.
  These names identify files; wall clocks and modification times do not prove
  chronological flight order. Older test boots reset their host wall clocks.
- JT16 mapping is associated only when a unique session in the same boot UUID
  starts within two seconds of VIO recording start. The exact offset is saved.
  This is an arm-session association, not proof of sensor-clock synchronization.
- Raw JT16 captures must share the boot UUID and have a recorded filename
  start within the VIO arm interval, allowing two seconds before its start.
  Multiple raw segments can belong to one session after acquisition restart.
  Unassociated raw files are listed in the catalog and remain local.
- Cube logs 211–214 use the existing exact VISP/companion timestamp and boot
  matching evidence in `reports/latest-flights-20261120/`. Their logs can span
  other arm cycles. Other sessions are marked unmatched; Desktop telemetry is
  not assigned using folder dates or file modification times.
- The included JT16 sample-angle table is the currently installed copy. Its
  historical use and the lidar/IMU extrinsics are not newly validated.
- `source_code_commit` identifies the workspace source snapshot at packaging.
  Saved boot binary checksums are the available executable identity; the source
  commit alone does not prove those executables were built from that commit.

## Download, verify, and restore

Install the GitHub CLI, `zstd`, and GNU tar. From a fresh directory:

```bash
gh release download flight-data-20261122 --repo triyankk/BHUslam_v3 \
  --pattern catalog.json --pattern SHA256SUMS \
  --pattern input-inventory.json --pattern METADATA-SHA256SUMS
sha256sum --check METADATA-SHA256SUMS
```

Choose the session in the review catalog, then download the filenames linked
in its `downloads.json`. For example, set `session` to an exact catalog ID:

```bash
session=20261121T113541.628563736Z_flight-189338136
gh release download flight-data-20261122 --repo triyankk/BHUslam_v3 \
  --pattern "${session}.tar.zst*"
# Check only assets present in this directory. For the complete release,
# download all assets and use: sha256sum --check SHA256SUMS
grep -F "  ${session}.tar.zst" SHA256SUMS | sha256sum --check -
mkdir restored
tar --zstd -xf "${session}.tar.zst" -C restored
```

If the archive is split, concatenate its parts in numeric order before
extracting (use this in place of the final `tar` command above):

```bash
cat "${session}.tar.zst".part[0-9][0-9][0-9] | zstd -dc | tar -xf - -C restored
```

`METADATA-SHA256SUMS` verifies the downloaded catalog and input-inventory
addendum; `SHA256SUMS` covers the 22 archive assets in this snapshot.
`files.json` inside the extracted session lists hashes and sizes of the original
members. `SHA256SUMS` authenticates the compressed bytes against the committed
manifest, while member hashes support checking the restored originals. Do not
overwrite the original run folders during review. See the existing VINS and
JT16 replay documentation for required libraries, calibration and clock limits;
packaging does not establish that a replay will initialize or track correctly.

## Creating another snapshot

With recording stopped/finalized and sufficient free disk, use new output
directories and a new release tag:

```bash
python3 tools/package_flight_reviews.py \
  --reviews flight-reviews-next \
  --archives release-assets/flight-data-NEXT \
  --tag flight-data-NEXT
python3 tools/audit_flight_review_inputs.py flight-reviews-next
```

The script refuses existing output directories and detects files that change
while being archived. It reads saved data and never opens sensors, modifies
services, or changes aircraft settings. Compression uses two low-priority zstd
threads. It tests every compressed stream and records SHA-256 hashes. Archives
at or above 1 GiB are split into smaller release assets; this stays below
[GitHub's per-asset release limit](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases).

Review the catalog, completeness warnings, source associations and file list;
commit the review directory, tooling, and documentation. Create a release at
that commit, upload all assets plus `catalog.json`, `SHA256SUMS`, the combined
`input-inventory.json`, and `METADATA-SHA256SUMS` for the metadata hashes, and verify
the remote asset names, sizes, and hashes. Keep archives under the ignored
`release-assets/` directory. Do not use `git add -f` on bulk recordings.
