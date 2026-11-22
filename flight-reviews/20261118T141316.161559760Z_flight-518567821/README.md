# Review: 20261118T141316.161559760Z_flight-518567821

Boot UUID: `85f03d02-ec4c-4f9d-b518-57d9820692ec`. Arm-triggered recording; finalized: **True**.

Stereo pairs indexed: **2941**; invalid pairs: **0**.

This package preserves existing evidence toward GPS-denied navigation and obstacle-avoidance evaluation. It does not establish flight accuracy, Cube fusion, or mission readiness.

## Review files

See [the file-level input inventory](input-inventory.json) for invalid/unindexed images and metadata count differences.

`vio/` contains recorded poses, tracking diagnostics and input metadata. `boot/` contains saved settings and executable identity. `manifest.json` records source paths, associations, completeness and limitations. `files.json` gives SHA-256 hashes of every original file included in the archive.

A `jt16/map-preview.ply`, when present, is a deterministic subset of at most 5,000 exported map vertices. The full selected map is `jt16/map.ply` inside the replay archive.

## Findings and limitations

- JT16 map recovered from the latest completed checkpoint; final export is missing.
- JT16 sample-angle table/provenance is the currently installed copy, not proven historical calibration.
- Cube DataFlash association comes from the saved four-flight audit; its log may span other arm cycles.

## Complete replay evidence

- [20261118T141316.161559760Z_flight-518567821.tar.zst.part000](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/20261118T141316.161559760Z_flight-518567821.tar.zst.part000) (1,073,741,824 bytes, SHA-256 `0992ce9d366d4020ab58669ff4cf017af3af58339b25b83defea02b70736ed22`)
- [20261118T141316.161559760Z_flight-518567821.tar.zst.part001](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/20261118T141316.161559760Z_flight-518567821.tar.zst.part001) (103,981,549 bytes, SHA-256 `6df93be88cc6d1bbb04ddcd9b07699e66afa56961fa159ef00c2d740274d1080`)

See [storage and restoration instructions](../../docs/flight-data-storage.md). Original files are retained byte-for-byte, including incomplete or damaged inputs. VIO images are selected rectified processed pairs before CLAHE; overwritten/unrectified pairs are absent.
