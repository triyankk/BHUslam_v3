# Review: 20261118T141315.611685968Z_flight-159185104

Boot UUID: `6fc5d417-8f80-442b-850f-e3df9a4140cd`. Arm-triggered recording; finalized: **True**.

Stereo pairs indexed: **1352**; invalid pairs: **156**.

This package preserves existing evidence toward GPS-denied navigation and obstacle-avoidance evaluation. It does not establish flight accuracy, Cube fusion, or mission readiness.

## Review files

See [the file-level input inventory](input-inventory.json) for invalid/unindexed images and metadata count differences.

`vio/` contains recorded poses, tracking diagnostics and input metadata. `boot/` contains saved settings and executable identity. `manifest.json` records source paths, associations, completeness and limitations. `files.json` gives SHA-256 hashes of every original file included in the archive.

A `jt16/map-preview.ply`, when present, is a deterministic subset of at most 5,000 exported map vertices. The full selected map is `jt16/map.ply` inside the replay archive.

## Findings and limitations

- 156 indexed stereo pairs are missing or invalid; original bytes are preserved.
- JT16 map recovered from the latest completed checkpoint; final export is missing.
- JT16 sample-angle table/provenance is the currently installed copy, not proven historical calibration.
- Cube DataFlash association comes from the saved four-flight audit; its log may span other arm cycles.

## Complete replay evidence

- [20261118T141315.611685968Z_flight-159185104.tar.zst](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/20261118T141315.611685968Z_flight-159185104.tar.zst) (525,231,719 bytes, SHA-256 `be2e27b88aa8fa3bc71124850b5f8006c2b32dc197fe3e3402530fa493e39eb7`)

See [storage and restoration instructions](../../docs/flight-data-storage.md). Original files are retained byte-for-byte, including incomplete or damaged inputs. VIO images are selected rectified processed pairs before CLAHE; overwritten/unrectified pairs are absent.
