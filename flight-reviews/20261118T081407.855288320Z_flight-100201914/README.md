# Review: 20261118T081407.855288320Z_flight-100201914

Boot UUID: `b805ee5b-6a12-4758-b903-ac4f638ff468`. Arm-triggered recording; finalized: **False**.

Stereo pairs indexed: **1200**; invalid pairs: **368**.

This package preserves existing evidence toward GPS-denied navigation and obstacle-avoidance evaluation. It does not establish flight accuracy, Cube fusion, or mission readiness.

## Review files

See [the file-level input inventory](input-inventory.json) for invalid/unindexed images and metadata count differences.

`vio/` contains recorded poses, tracking diagnostics and input metadata. `boot/` contains saved settings and executable identity. `manifest.json` records source paths, associations, completeness and limitations. `files.json` gives SHA-256 hashes of every original file included in the archive.

A `jt16/map-preview.ply`, when present, is a deterministic subset of at most 5,000 exported map vertices. The full selected map is `jt16/map.ply` inside the replay archive.

## Findings and limitations

- VIO recording was not finalized; preserve as incomplete evidence.
- NUL corruption in frames.csv; only complete preceding rows inspected.
- 368 indexed stereo pairs are missing or invalid; original bytes are preserved.
- No unique JT16 map session matched within 2 seconds in the same boot.
- No JT16 raw capture could be associated by same boot and recording-start interval.
- No independently verified Cube DataFlash association is available for this session.

## Complete replay evidence

- [20261118T081407.855288320Z_flight-100201914.tar.zst](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/20261118T081407.855288320Z_flight-100201914.tar.zst) (291,012,652 bytes, SHA-256 `5c207fa27328b72802ec0e9d0823ba9557e177c54d5e4a6e3e1ee8f4b3e8a177`)

See [storage and restoration instructions](../../docs/flight-data-storage.md). Original files are retained byte-for-byte, including incomplete or damaged inputs. VIO images are selected rectified processed pairs before CLAHE; overwritten/unrectified pairs are absent.
