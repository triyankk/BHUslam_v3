# Cube TELEM1 bridge check

Connected Cube identified over USB as ArduCopter 4.7.1 (release type 255),
system 1/component 1, quadcopter, disarmed heartbeat.

Read-back configuration:

- SERIAL1_PROTOCOL=2 (MAVLink2)
- SERIAL1_BAUD=57 (57600 baud)
- SERIAL1_OPTIONS=0
- BRD_SER1_RTSCTS=2 (automatic)
- PRX1_TYPE=2 (MAVLink)
- PRX1_ORIENT=0, PRX1_YAW_CORR=0; first two ignored sectors have zero width.

`../telem1-01/`: 225 valid calibrated ToF frames, 225 outgoing distance
messages, 15 heartbeats, 14.991 Hz, no I²C/write errors. Binary capture CRCs
and sensor-log correspondence passed. An observer on USB saw one forwarded
DISTANCE_SENSOR packet from Jetson component 191, orientation 4, distance 63 cm.
That proves some Jetson TX data physically reached and was parsed by the Cube;
it does not measure sustained delivery rate.

`../telem1-no-flow-01/`: temporarily set BRD_SER1_RTSCTS=0 with acknowledgement,
then sent 180 valid measurements over 12 seconds. USB saw one forwarded Jetson
packet (20 cm) and one Cube-generated proximity report from component 1,
sensor ID 14, orientation 4, 63 cm. The latter confirms the proximity backend
had accepted a rear distance, but does not demonstrate fresh tracking of every
subsequent sample. Restored BRD_SER1_RTSCTS=2 with acknowledgement. No other
Cube parameter was written. No arming/mode commands were sent.

Passive UART RX checks at 115200, 57600, and 921600 produced malformed data and
no valid heartbeats. A separate PING test is saved in `uart-ping.json` and
`uart-ping-rx.bin`. Stable bidirectional communication remains unverified.

The active Jetson device tree retains dmas/dma-names for UART 0x3100000
(`/dev/ttyTHS1`). An existing extlinux entry named `cube-uart-pio`, menu label
`Cube UART PIO kernel`, selects an existing alternative DTB. Comparing decoded
DTBs verified its only change is deletion of dmas and dma-names for this UART.
Boot default is still `primary`; no Jetson boot files were modified here.
Testing this alternative is a diagnostic step, not a confirmed fix.

All test processes exited. There is no persistent bridge running. Physical
UART timing, signal integrity, sustained delivery, and known-target ranging
accuracy remain to be established before flight use.
