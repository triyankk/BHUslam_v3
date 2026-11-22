# Review: 20261118T091238.639064632Z_flight-331497527

Boot UUID: `6794f621-9b99-4f37-9e09-11d6930f03bb`. Arm-triggered recording; finalized: **True**.

Stereo pairs indexed: **5567**; invalid pairs: **0**.

This package preserves existing evidence toward GPS-denied navigation and obstacle-avoidance evaluation. It does not establish flight accuracy, Cube fusion, or mission readiness.

## Review files

See [the file-level input inventory](input-inventory.json) for invalid/unindexed images and metadata count differences.

`vio/` contains recorded poses, tracking diagnostics and input metadata. `boot/` contains saved settings and executable identity. `manifest.json` records source paths, associations, completeness and limitations. `files.json` gives SHA-256 hashes of every original file included in the archive.

A `jt16/map-preview.ply`, when present, is a deterministic subset of at most 5,000 exported map vertices. The full selected map is `jt16/map.ply` inside the replay archive.

## Findings and limitations

- No unique JT16 map session matched within 2 seconds in the same boot.
- No JT16 raw capture could be associated by same boot and recording-start interval.
- No independently verified Cube DataFlash association is available for this session.

## Complete replay evidence

- [20261118T091238.639064632Z_flight-331497527.tar.zst.part000](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/20261118T091238.639064632Z_flight-331497527.tar.zst.part000) (1,073,741,824 bytes, SHA-256 `4f8bf385b0a8d52dd118f9dbc1eeae2c5dd805109b7155cbc95ec8962bba771a`)
- [20261118T091238.639064632Z_flight-331497527.tar.zst.part001](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/20261118T091238.639064632Z_flight-331497527.tar.zst.part001) (1,073,741,824 bytes, SHA-256 `4f6cc7b6ef9b562fa4959e0253aa5dde5194b23bf21a0ffed1e62ed8d16a598c`)
- [20261118T091238.639064632Z_flight-331497527.tar.zst.part002](https://github.com/triyankk/BHUslam_v3/releases/download/flight-data-20261122/20261118T091238.639064632Z_flight-331497527.tar.zst.part002) (126,964,050 bytes, SHA-256 `0843ae7726379971f4c9552056007bbe531d0b57d620af9a10ec028b447f9414`)

See [storage and restoration instructions](../../docs/flight-data-storage.md). Original files are retained byte-for-byte, including incomplete or damaged inputs. VIO images are selected rectified processed pairs before CLAHE; overwritten/unrectified pairs are absent.
