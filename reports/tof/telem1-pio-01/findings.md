# TELEM1 bridge with Jetson UART PIO

Live device tree confirmed `/bus@0/serial@3100000` has neither dmas nor dma-names.
The user selected existing `cube-uart-pio` boot entry; it is now the boot default.

## Baud rate

The Cube's SERIAL1_BAUD readback is now 115 (115200), but runtime TELEM1 responds
at 57600. A direct read-only parameter probe received six PARAM_VALUE replies
and six heartbeats at 57600 with no malformed frames; at 115200 it received only
malformed data and no parameter replies. This is consistent with a pending
Cube baud change requiring restart. This test did not change Cube parameters.
After restarting the Cube, determine its running rate again before bridging.

## Thirty-second real sensor test

- Sensor serial 23937420, I²C 7, address 0x41, calibration restored, SPAD map 1.
- 450 frames and 450 distance packets at 14.99955 Hz; no calibration rejections,
  duplicate sensor frames, result-counter gaps, I²C errors, or UART write errors.
- UART RX: 1836 bytes, 60 heartbeats (30 from autopilot component 1),
  1 PARAM_VALUE, 13 STATUSTEXT, 3 TIMESYNC; no BAD_DATA frames.
- All 450 locally captured DISTANCE_SENSOR packets passed CRC decoding and
  matched sensor-log distances/timestamps/orientation.
- USB independently observed 407 forwarded component-191 distance packets.
  Every observed forwarded packet matched the original sensor timestamp and
  centimetre distance exactly. The forwarding count is not proof of lossless
  physical delivery; do not label all 450 as independently confirmed received.
- USB independently observed 204 Cube-generated rear proximity reports from
  component 1, sensor ID 14, orientation 4. This confirms the Cube is processing
  the data through its proximity backend, beyond simply forwarding packets.
- Reported distances and exact-match verification are in `validation.json`.
- Inter-frame interval standard deviation 1.062 ms, range 62.876–70.367 ms.

Compared with the DMA test, malformed RX traffic disappeared and sustained
forwarded/proximity reports became available. These results support retaining
PIO for this bench setup; they do not establish flight readiness or calibrated
absolute ranging accuracy against a known target.

The preliminary PING probe recorded no PING replies, but decoded valid Cube
telemetry. Bidirectional communication was instead confirmed by the subsequent
six direct UART parameter replies and the sustained autopilot heartbeats.

All test processes exited normally. The bridge is not currently running.
