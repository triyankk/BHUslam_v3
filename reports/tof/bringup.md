# TMF8820 single-board bring-up

Current configuration is now [rear RNGFND2 only](cube-rear-rangefinder-03/findings.md),
with PRX1_TYPE=0 and the existing downward RNGFND1 preserved. The viewer plots
Cube-generated rangefinder returns over UART. Earlier proximity test results
below are historical.

Latest Cube test: [TELEM1 with PIO](telem1-pio-01/findings.md) passed sustained
sensor conversion, valid bidirectional UART traffic, and Cube-generated rear
proximity reports. Runtime baud is 57600; stored Cube SERIAL1_BAUD is 115200,
so recheck runtime baud after a Cube restart. Earlier findings below are retained
as historical test evidence.

Current state: calibration, sensor acquisition, MAVLink conversion, and writes
to the physical UART device passed. Cube receipt and electrical delivery at the
header pins remain to be checked. Cube remains disconnected.

- Jetson identifies as Orin Nano Engineering Reference Developer Kit Super.
- SparkFun TMF8820 is wired to physical pins 1/3/5/6.
- Sensor responded on I²C bus 7, address 0x41; serial 23937420.
- RAM ranging app loaded: app ID 0x03, firmware minor 32, SPAD map 1 (3x3).
- Initial five reads returned approximately 11–14 mm, confidence 255, and
  calibration status 0x31 (no factory calibration). These are not accepted as
  validated obstacle distances.
- `uncalibrated-01/` captures a subsequent run: four frames, zero emitted
  distance packets, one heartbeat, then I²C remote-I/O error (errno 121).
- A later 50-read enable-register check on bus 7 had 50 acknowledgements and
  zero errors. This does not establish sustained ranging stability.
- Actual `/dev/ttyTHS1` opened exclusively at 115200 8N1, flow control off.
  No physical UART receiver/loopback has been checked.
- Software PTY test passed primary/secondary filtering, threshold rejection,
  mm-to-cm flooring, all five orientations, byte-for-byte transport, and
  fragmented MAVLink 2 packet decoding with valid CRCs.
- Decoding `uncalibrated-01/mavlink.bin` matched measurement-log counts and
  confirmed that uncalibrated distance reports were suppressed.

After the user prepared an open calibration scene:

- Calibration was created and saved as
  `tools/tof_bridge/calibration-23937420-map1.json`, tied to serial 23937420 and
  SPAD map 1. Calibration payload SHA256:
  `d76918cadd57e5c2be443f9d4ac992f5eca197e0364cbb781c56ad7b72e756a4`.
- `calibration-01/`: 75/75 calibrated frames in five seconds, no errors,
  counter gaps, duplicates, or rejected frames. The scene had not been
  independently measured; two near returns were also observed.
- `uart-01/`: 450/450 calibrated frames in 30 seconds, 14.9972 Hz, no I²C or
  UART write errors, counter gaps, duplicates, or rejected frames. Nearest
  confident returns ranged from 825 to 888 mm in this scene.
- UART `/dev/ttyTHS1`, 115200 8N1, rear orientation, source system 1/component
  191: 450 DISTANCE_SENSOR plus 29 HEARTBEAT packets, 12309 bytes total.
- Binary MAVLink capture decoded with valid CRCs and matched all 450 logged
  sensor distances, orientations, and timestamps exactly.
- Inter-frame interval standard deviation was 1.424 ms, min 54.386 ms,
  max 82.868 ms. This is a bench transport check, not a real-time guarantee.
- The test exited normally and UART was closed. No bridge is running now.

Next: connect the Cube telemetry UART with power off, then resume the bridge
for receipt verification and a target-distance change test. Absolute ranging
accuracy, physical UART reception, firmware version, Cube parameter mapping,
and obstacle avoidance behaviour have not yet been verified.

Implementation and commands: [bridge README](../../tools/tof_bridge/README.md).

## EN wiring test

The user added sensor EN to physical header pin 7. NVIDIA's installed GPIO
mapping identifies it as PAC.06, gpiochip0 line 144; it was initially unused.
Software GPIO output/readback toggled 0/1 successfully, but sensor responses
did not reliably follow the requested EN state. Initial high/low/high tests
received no ACK; the subsequent saved `en-identity-01/` test received an ACK
at 0x41 even with EN commanded low, then hit an I²C remote-I/O error during
cleanup. No sensor address-change command was reached.

GPIO handles were released with the requested output high. Physical EN voltage
and pinmux output configuration remain unverified. Reading kernel pinctrl
debugfs and NVIDIA's header configuration tool requires root here; passwordless
sudo is unavailable. Do not interpret software readback as electrical proof.
The saved identity test is `tools/tof_bridge/check_en_identity.py`; rerun only
after resolving wiring or pinmux. The sensor has not been confirmed restored
to the previously working calibrated state after this EN test.
