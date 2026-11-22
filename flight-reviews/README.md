# Saved flight review catalog

[Published asset verification and preservation checks](release-verification.json).

[Optical-flow telemetry and remaining-evidence supplement](supplement/README.md)
includes the remaining-local-data audit.

The [complete image inventory](input-inventory.json) lists invalid and unindexed
PGM files, including images beyond corrupted frame-index tails. Saved image
counts and readable indexed-pair counts are distinct.

[Replay dataset release](https://github.com/triyankk/BHUslam_v3/releases/tag/flight-data-20261122)

Sessions are indexed by recorded run name and arm-start ID, not claimed chronological flight order. Folder wall clocks reset in older sessions. See [storage instructions](../docs/flight-data-storage.md).

| Session | Indexed pairs | Invalid pairs | JT16 map vertices | Cube log |
| --- | ---: | ---: | ---: | --- |
| [20261118T081407.855288320Z_flight-100201914](20261118T081407.855288320Z_flight-100201914/README.md) | 1200 | 368 | unavailable | unmatched |
| [20261118T091238.639064632Z_flight-331497527](20261118T091238.639064632Z_flight-331497527/README.md) | 5567 | 0 | unavailable | unmatched |
| [20261118T122431.721549104Z_flight-894534460](20261118T122431.721549104Z_flight-894534460/README.md) | 664 | 0 | 1522 | unmatched |
| [20261118T133131.962421232Z_flight-312765354](20261118T133131.962421232Z_flight-312765354/README.md) | 688 | 0 | unavailable | unmatched |
| [20261118T133131.962421232Z_flight-559312797](20261118T133131.962421232Z_flight-559312797/README.md) | 620 | 0 | 1537 | unmatched |
| [20261118T141315.262362768Z_flight-163287986](20261118T141315.262362768Z_flight-163287986/README.md) | 844 | 0 | 4302 | unmatched |
| [20261118T141315.262362768Z_flight-324882084](20261118T141315.262362768Z_flight-324882084/README.md) | 1279 | 0 | unavailable | unmatched |
| [20261118T141315.262362768Z_flight-506073151](20261118T141315.262362768Z_flight-506073151/README.md) | 237 | 0 | unavailable | 211 |
| [20261118T141315.557799248Z_flight-803834094](20261118T141315.557799248Z_flight-803834094/README.md) | 827 | 109 | unavailable | 212 |
| [20261118T141315.611685968Z_flight-159185104](20261118T141315.611685968Z_flight-159185104/README.md) | 1352 | 156 | 4992 | 214 |
| [20261118T141316.161559760Z_flight-518567821](20261118T141316.161559760Z_flight-518567821/README.md) | 2941 | 0 | 5054 | 213 |
| [20261121T112159.087745155Z_flight-386227867](20261121T112159.087745155Z_flight-386227867/README.md) | 68 | 0 | 2904 | unmatched |
| [20261121T112159.087745155Z_flight-394427917](20261121T112159.087745155Z_flight-394427917/README.md) | 192 | 0 | 3005 | unmatched |
| [20261121T112159.087745155Z_flight-417675015](20261121T112159.087745155Z_flight-417675015/README.md) | 54 | 0 | 3266 | unmatched |
| [20261121T113540.731971704Z_flight-109359032](20261121T113540.731971704Z_flight-109359032/README.md) | 108 | 0 | 250000 | unmatched |
| [20261121T113541.628563736Z_flight-189338136](20261121T113541.628563736Z_flight-189338136/README.md) | 341 | 0 | 250000 | unmatched |
| [20261121T113541.628563736Z_flight-271195708](20261121T113541.628563736Z_flight-271195708/README.md) | 465 | 0 | 0 | unmatched |
| [20261121T113541.628563736Z_flight-404311163](20261121T113541.628563736Z_flight-404311163/README.md) | 461 | 0 | 0 | unmatched |
| [20261121T113541.628563736Z_flight-493283097](20261121T113541.628563736Z_flight-493283097/README.md) | 917 | 49 | 0 | unmatched |

Unassociated raw recordings are listed in `catalog.json`; they were not silently attached to a flight.
