# Review: 20261118T141315.557799248Z_flight-803834094

Boot UUID: `0a389dc9-98e1-4ac9-ae84-a30fe55892cd`. Arm-triggered recording; finalized: **True**.

Stereo pairs indexed: **827**; invalid pairs: **109**.

This package preserves existing evidence toward GPS-denied navigation and obstacle-avoidance evaluation. It does not establish flight accuracy, Cube fusion, or mission readiness.

## Review files

See [the file-level input inventory](input-inventory.json) for invalid/unindexed images and metadata count differences.

`vio/` contains recorded poses, tracking diagnostics and input metadata. `boot/` contains saved settings and executable identity. `manifest.json` records source paths, associations, completeness and limitations. `files.json` gives SHA-256 hashes of every original file included in the archive.

A `jt16/map-preview.ply`, when present, is a deterministic subset of at most 5,000 exported map vertices. The full selected map is `jt16/map.ply` inside the replay archive.

## Findings and limitations

- 109 indexed stereo pairs are missing or invalid; original bytes are preserved.
- JT16 timing/IMU/trajectory logs exist, but no complete map export was found.
- JT16 sample-angle table/provenance is the currently installed copy, not proven historical calibration.
- Cube DataFlash association comes from the saved four-flight audit; its log may span other arm cycles.

## Complete replay evidence

- [20261118T141315.557799248Z_flight-803834094.tar.zst](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/20261118T141315.557799248Z_flight-803834094.tar.zst) (320,512,694 bytes, SHA-256 `124d774c30f76caec04cf193647386908aa71f381d7a706a486136e74c56d34d`)

See [storage and restoration instructions](../../docs/flight-data-storage.md). Original files are retained byte-for-byte, including incomplete or damaged inputs. VIO images are selected rectified processed pairs before CLAHE; overwritten/unrectified pairs are absent.
