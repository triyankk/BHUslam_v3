# PCA9548A channel 0 C++ bridge validation

The connected HW-617/PCA9548A at 0x70 on I²C7 successfully routed TMF8820
serial 23937420 at 0x41 on channel 0. The sensor was in bootloader mode at the
initial wiring check; native initialization loaded its RAM firmware through
the mux and restored its serial-specific calibration.

The 35-second run recorded 521 calibrated samples at 15.000 Hz, no counter gaps
or duplicates, and no malformed UART packets. Independent pymavlink decoding
matched all 521 transmitted DISTANCE_SENSOR packets to the sensor logs. All
147 returned Cube rear rangefinder reports matched a recently transmitted
distance (source system 1/component 1, sensor ID 1, orientation 4).

The sensor's nearest return was 38–39 mm during this bench capture. This
checks data transport and calibration status; it does not establish accuracy
at other distances or flight suitability.

Readback confirmed RNGFND2_TYPE=10, ORIENT=4, MIN=0.01 m, MAX=5 m. Existing
Cube status messages are listed in verification.json; none identifies rear
rangefinder 2. No Cube parameters were written, and the VIO relay stayed off.

An empty channel 7 correctly produced sensor I/O errors, no measurements, and
bounded retries. After both successful and failed runs, independent mux control
readback was 0x00: all channels disconnected. Invalid mux addresses/channels
were rejected before starting acquisition. The native protocol self-test and
the actual browser dashboard tests passed with the real API snapshot.

The bridge selects one channel before sensor initialization and every sample,
verifies readback, and holds sensor/mux ownership locks through capture. A
normal stop or retry shuts down the sensor before disconnecting mux channels.
An abrupt SIGKILL or power loss cannot run cleanup; startup explicitly selects
one channel again. Hardware RESET is not driven by this implementation.

Run the current rear sensor with:

```bash
tools/tof_bridge/cpp/build/tof_viewer --mux-address 0x70 --mux-channel 0
```
