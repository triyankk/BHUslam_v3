# Review: 20261121T113541.628563736Z_flight-271195708

Boot UUID: `6162f008-07f1-4e70-840f-62a25fc69ccd`. Arm-triggered recording; finalized: **True**.

Stereo pairs indexed: **465**; invalid pairs: **0**.

This package preserves existing evidence toward GPS-denied navigation and obstacle-avoidance evaluation. It does not establish flight accuracy, Cube fusion, or mission readiness.

## Review files

See [the file-level input inventory](input-inventory.json) for invalid/unindexed images and metadata count differences.

`vio/` contains recorded poses, tracking diagnostics and input metadata. `boot/` contains saved settings and executable identity. `manifest.json` records source paths, associations, completeness and limitations. `files.json` gives SHA-256 hashes of every original file included in the archive.

A `jt16/map-preview.ply`, when present, is a deterministic subset of at most 5,000 exported map vertices. The full selected map is `jt16/map.ply` inside the replay archive.

## Findings and limitations

- JT16 selected map has zero vertices; no map preview is supplied.
- JT16 sample-angle table/provenance is the currently installed copy, not proven historical calibration.
- No independently verified Cube DataFlash association is available for this session.

## Complete replay evidence

- [20261121T113541.628563736Z_flight-271195708.tar.zst](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/20261121T113541.628563736Z_flight-271195708.tar.zst) (188,871,484 bytes, SHA-256 `0476c06681df39bc2d8ce14166e03f8f748e24b2ac6579211bab2f556c9e0bd4`)

See [storage and restoration instructions](../../docs/flight-data-storage.md). Original files are retained byte-for-byte, including incomplete or damaged inputs. VIO images are selected rectified processed pairs before CLAHE; overwritten/unrectified pairs are absent.
