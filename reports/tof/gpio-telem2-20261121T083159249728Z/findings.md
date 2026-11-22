# Jetson GPIO UART to Cube TELEM2 check — 21 November 2026

The operator reports physical Jetson pins 6/8/10 connected to Cube TELEM2.
The GPIO UART is /dev/ttyTHS1, mapped to serial@3100000. The active device tree
has status=okay and no DMA properties; the saved boot default is cube-uart-pio.
No pinmux, baud parameter, flow-control parameter, or boot setting was changed.
The working ToF-to-Cube USB owner remained running throughout.

At 115200 8N1 without host RTS/CTS, the initial probe received 111 valid MAVLink
packets and 8 autopilot heartbeats, with no malformed bytes. One targeted
SERIAL2_BAUD reply reported 115 (115200 baud). That shows some request/reply
communication, but is not sustained bidirectional validation.

The following sequential test sent eight individual parameter reads, allowing
1.5 seconds per request. Zero of eight received the expected reply. It still
received 12 Cube heartbeats and 183 forwarded companion DISTANCE_SENSOR packets,
with zero malformed bytes. No diagnostic parameter replies appeared in the
USB owner's capture either. Thus Cube-to-Jetson reception works, but a reliable
Jetson-to-Cube request/reply path was not established. This does not isolate
the fault to TX wiring: electrical wiring, Cube input handling/configuration,
and routing would need further diagnosis before attributing the cause.

The companion distance packets arriving on the GPIO UART are the current USB
ToF input forwarded by the Cube; they are not proof of ToF transmission from
the Jetson GPIO pins. ToF forwarding remains on primary Cube USB and stayed
fresh with zero malformed USB packets during the test.

Expected signal wiring, using physical connector pin numbers:

- Jetson 8 TX -> Cube TELEM2 pin 3 RX.
- Jetson 10 RX <- Cube TELEM2 pin 2 TX.
- Jetson 6 GND -> Cube TELEM2 pin 6 GND.

TELEM2 power pin 1 is not part of this three-wire signal connection. Do not
replace the working USB stream with this UART based solely on clean heartbeats.
Evidence: probe.json, sustained.json, raw RX binaries, usb-crosscheck.json and
verification.json. Only parameter reads and firmware-message requests were sent.
