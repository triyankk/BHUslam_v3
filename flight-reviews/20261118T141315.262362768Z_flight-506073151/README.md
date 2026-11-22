# Review: 20261118T141315.262362768Z_flight-506073151

Boot UUID: `b9909dc0-5245-4740-b31a-a4c880c12fb3`. Arm-triggered recording; finalized: **True**.

Stereo pairs indexed: **237**; invalid pairs: **0**.

This package preserves existing evidence toward GPS-denied navigation and obstacle-avoidance evaluation. It does not establish flight accuracy, Cube fusion, or mission readiness.

## Review files

See [the file-level input inventory](input-inventory.json) for invalid/unindexed images and metadata count differences.

`vio/` contains recorded poses, tracking diagnostics and input metadata. `boot/` contains saved settings and executable identity. `manifest.json` records source paths, associations, completeness and limitations. `files.json` gives SHA-256 hashes of every original file included in the archive.

A `jt16/map-preview.ply`, when present, is a deterministic subset of at most 5,000 exported map vertices. The full selected map is `jt16/map.ply` inside the replay archive.

## Findings and limitations

- No unique JT16 map session matched within 2 seconds in the same boot.
- JT16 sample-angle table/provenance is the currently installed copy, not proven historical calibration.
- Cube DataFlash association comes from the saved four-flight audit; its log may span other arm cycles.

## Complete replay evidence

- [20261118T141315.262362768Z_flight-506073151.tar.zst](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/20261118T141315.262362768Z_flight-506073151.tar.zst) (109,206,113 bytes, SHA-256 `e81e67d6a97fd173d9df9ae0a6e892ccd70420bce7bed3f0b40b969e8e739a9e`)

See [storage and restoration instructions](../../docs/flight-data-storage.md). Original files are retained byte-for-byte, including incomplete or damaged inputs. VIO images are selected rectified processed pairs before CLAHE; overwritten/unrectified pairs are absent.
