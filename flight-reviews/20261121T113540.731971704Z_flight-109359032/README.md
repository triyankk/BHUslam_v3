# Review: 20261121T113540.731971704Z_flight-109359032

Boot UUID: `0163353c-b471-4f05-99c8-8b00668f49f0`. Arm-triggered recording; finalized: **True**.

Stereo pairs indexed: **108**; invalid pairs: **0**.

This package preserves existing evidence toward GPS-denied navigation and obstacle-avoidance evaluation. It does not establish flight accuracy, Cube fusion, or mission readiness.

## Review files

See [the file-level input inventory](input-inventory.json) for invalid/unindexed images and metadata count differences.

`vio/` contains recorded poses, tracking diagnostics and input metadata. `boot/` contains saved settings and executable identity. `manifest.json` records source paths, associations, completeness and limitations. `files.json` gives SHA-256 hashes of every original file included in the archive.

A `jt16/map-preview.ply`, when present, is a deterministic subset of at most 5,000 exported map vertices. The full selected map is `jt16/map.ply` inside the replay archive.

## Findings and limitations

- NUL corruption in frames.csv; only complete preceding rows inspected.
- JT16 sample-angle table/provenance is the currently installed copy, not proven historical calibration.
- No independently verified Cube DataFlash association is available for this session.

## Complete replay evidence

- [20261121T113540.731971704Z_flight-109359032.tar.zst](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/20261121T113540.731971704Z_flight-109359032.tar.zst) (44,690,333 bytes, SHA-256 `9dd3ed38170af810f3331d875930cf5f70bf9e64ac5b745ae3a579ba22145550`)

See [storage and restoration instructions](../../docs/flight-data-storage.md). Original files are retained byte-for-byte, including incomplete or damaged inputs. VIO images are selected rectified processed pairs before CLAHE; overwritten/unrectified pairs are absent.
