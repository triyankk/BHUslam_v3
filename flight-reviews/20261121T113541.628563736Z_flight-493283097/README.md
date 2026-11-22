# Review: 20261121T113541.628563736Z_flight-493283097

Boot UUID: `6162f008-07f1-4e70-840f-62a25fc69ccd`. Arm-triggered recording; finalized: **True**.

Stereo pairs indexed: **917**; invalid pairs: **49**.

This package preserves existing evidence toward GPS-denied navigation and obstacle-avoidance evaluation. It does not establish flight accuracy, Cube fusion, or mission readiness.

## Review files

See [the file-level input inventory](input-inventory.json) for invalid/unindexed images and metadata count differences.

`vio/` contains recorded poses, tracking diagnostics and input metadata. `boot/` contains saved settings and executable identity. `manifest.json` records source paths, associations, completeness and limitations. `files.json` gives SHA-256 hashes of every original file included in the archive.

A `jt16/map-preview.ply`, when present, is a deterministic subset of at most 5,000 exported map vertices. The full selected map is `jt16/map.ply` inside the replay archive.

## Findings and limitations

- 49 indexed stereo pairs are missing or invalid; original bytes are preserved.
- JT16 selected map has zero vertices; no map preview is supplied.
- JT16 sample-angle table/provenance is the currently installed copy, not proven historical calibration.
- No independently verified Cube DataFlash association is available for this session.

## Complete replay evidence

- [20261121T113541.628563736Z_flight-493283097.tar.zst](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/20261121T113541.628563736Z_flight-493283097.tar.zst) (358,551,875 bytes, SHA-256 `6aba201785f1d50087672493ec47e827536d1f3106d7709c1b96ad15d94ca326`)

See [storage and restoration instructions](../../docs/flight-data-storage.md). Original files are retained byte-for-byte, including incomplete or damaged inputs. VIO images are selected rectified processed pairs before CLAHE; overwritten/unrectified pairs are absent.
