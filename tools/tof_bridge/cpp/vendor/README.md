Vendored build inputs
====================

`json.hpp` is nlohmann/json 3.11.3, downloaded from:
https://raw.githubusercontent.com/nlohmann/json/v3.11.3/single_include/nlohmann/json.hpp
Its MIT license is retained in `json-LICENSE.MIT`.

`mavlink/` contains the official generated C headers from:
https://github.com/mavlink/c_library_v2/tree/6a7760569ce6d46c56f93163703543d4345e940a
The ardupilotmega dialect and its included dialects are retained so incoming
ArduPilot messages can be checked with the official message definitions and CRCs.
Unused dialects and generated tests are omitted.

Exact versions and downloaded-file SHA256 digests are in `versions.json`.
Sensor firmware and serial-specific calibration remain in the parent bridge
directory and are checked for integrity before use.
