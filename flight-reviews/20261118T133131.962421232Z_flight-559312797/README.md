# Review: 20261118T133131.962421232Z_flight-559312797

Boot UUID: `eacff06f-b284-456b-ba9b-ce8b58c41fd0`. Arm-triggered recording; finalized: **True**.

Stereo pairs indexed: **620**; invalid pairs: **0**.

This package preserves existing evidence toward GPS-denied navigation and obstacle-avoidance evaluation. It does not establish flight accuracy, Cube fusion, or mission readiness.

## Review files

See [the file-level input inventory](input-inventory.json) for invalid/unindexed images and metadata count differences.

`vio/` contains recorded poses, tracking diagnostics and input metadata. `boot/` contains saved settings and executable identity. `manifest.json` records source paths, associations, completeness and limitations. `files.json` gives SHA-256 hashes of every original file included in the archive.

A `jt16/map-preview.ply`, when present, is a deterministic subset of at most 5,000 exported map vertices. The full selected map is `jt16/map.ply` inside the replay archive.

## Findings and limitations

- JT16 map recovered from the latest completed checkpoint; final export is missing.
- JT16 sample-angle table/provenance is the currently installed copy, not proven historical calibration.
- No independently verified Cube DataFlash association is available for this session.

## Complete replay evidence

- [20261118T133131.962421232Z_flight-559312797.tar.zst](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/20261118T133131.962421232Z_flight-559312797.tar.zst) (146,022,140 bytes, SHA-256 `4d51200cf118f87e1e2d25c74425edd4d3eb0257acf384f66d5460b20512395b`)

See [storage and restoration instructions](../../docs/flight-data-storage.md). Original files are retained byte-for-byte, including incomplete or damaged inputs. VIO images are selected rectified processed pairs before CLAHE; overwritten/unrectified pairs are absent.
