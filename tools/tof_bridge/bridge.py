#!/usr/bin/env python3
"""Single TMF8820 -> MAVLink 2 DISTANCE_SENSOR, with bounded I2C waits.

Bench integration: nearest confident return across nine zones, one direction.
No autopilot parameter changes, control commands, or automatic calibration.
"""
import argparse
from collections import deque
from contextlib import ExitStack
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
from pathlib import Path
import signal
import statistics
import sys
import time
import traceback

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "vendor"))
from smbus2 import SMBus, i2c_msg
from tmf882x import TMF882x, TMF882xException
from tmf882x.measurement import TMF882xMeasurement
from pymavlink.dialects.v20 import common as mav

ORIENTATIONS = {"front": 0, "right": 2, "rear": 4, "left": 6, "up": 24}


class Sensor(TMF882x):
    """Use the pinned driver's RAM firmware/configuration, with bounded waits."""
    def _send_command(self, command):
        self.bus.write_byte_data(self.address, 0x08, command)
        deadline = time.monotonic() + 3
        while True:
            status = self._read_status()
            if status < 0x10:
                if status > 1:
                    raise TMF882xException(f"Command {command:#x} failed: {status:#x}")
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Sensor command {command:#x} timed out")
            time.sleep(self.poll_delay)

    def next_measurement(self, spad_map):
        # One-shot operation avoids torn reads between successive result frames.
        self.bus.write_byte_data(self.address, 0xE1, 0xFF)
        self._send_command(0x10)
        deadline = time.monotonic() + 1
        try:
            while not (self.bus.read_byte_data(self.address, 0xE1) & 2):
                if time.monotonic() >= deadline:
                    raise TimeoutError("No fresh TMF8820 result within one second")
                time.sleep(self.poll_delay)
            data = i2c_msg.read(self.address, 132)
            self.bus.i2c_rdwr(i2c_msg.write(self.address, [0x20]), data)
            raw = list(data)
            if raw[0] != 0x10 or int.from_bytes(raw[2:4], "little") != 128:
                raise ValueError(f"Unexpected result page header: {raw[:4]}")
            status = self.bus.read_byte_data(self.address, 0x07)
            return TMF882xMeasurement.from_bytes(raw, spad_map), status
        finally:
            self._send_command(0xFF)


def nearest_return(measurement, threshold=20, minimum_mm=10, maximum_mm=5000):
    candidates = []
    for row in measurement.grid:
        for zone in row:
            for distance, confidence in ((zone.distance, zone.confidence),
                                         (zone.secondary_distance, zone.secondary_confidence)):
                if confidence >= threshold and minimum_mm <= distance <= maximum_mm:
                    candidates.append((distance, confidence))
    return min(candidates, default=None)


class Output:
    def __init__(self, capture=None, uart=None):
        self.capture, self.uart = capture, uart
        self.packets = 0
        self.bytes = 0

    def write(self, data):
        if self.uart is not None:
            if self.uart.write(data) != len(data):
                raise IOError("Incomplete UART write")
        if self.capture is not None:
            self.capture.write(data)
            self.capture.flush()
        self.packets += 1
        self.bytes += len(data)
        return len(data)


def send_distance(link, distance_mm, orientation, stamp_ms):
    # Floor to cm so rounding never increases the reported clearance.
    link.distance_sensor_send(stamp_ms & 0xFFFFFFFF, 1, 500,
                              distance_mm // 10, mav.MAV_DISTANCE_SENSOR_LASER,
                              0, ORIENTATIONS[orientation], 255,
                              signal_quality=0)  # unknown; sensor confidence is not a probability


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bus", type=int, default=7)
    p.add_argument("--address", type=lambda x: int(x, 0), default=0x41)
    p.add_argument("--orientation", choices=ORIENTATIONS, default="rear")
    p.add_argument("--duration", type=float, default=30, help="Seconds; 0 runs until Ctrl-C")
    p.add_argument("--hz", type=float, default=15)
    p.add_argument("--confidence", type=int, default=20)
    p.add_argument("--uart", help="e.g. /dev/ttyTHS1; omitted means capture/read only")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--request-rangefinder", "--request-proximity", dest="request_reports", action="store_true",
                   help="Request Cube rangefinder reports over the UART at 5 Hz")
    p.add_argument("--rangefinder-slot", type=int, default=2)
    p.add_argument("--system-id", type=int, default=1)
    p.add_argument("--component-id", type=int, default=191)
    p.add_argument("--output", type=Path, required=True, help="New output directory")
    p.add_argument("--calibration", type=Path, help="Calibration JSON tied to serial and SPAD map")
    p.add_argument("--calibrate", type=Path, help="Create new calibration JSON; open scene required")
    return p


def main():
    args = parser().parse_args()
    if not (0 < args.hz <= 30) or not math.isfinite(args.hz):
        raise ValueError("--hz must be between 0 and 30")
    if args.duration < 0 or not math.isfinite(args.duration):
        raise ValueError("--duration must be finite and nonnegative")
    if not 1 <= args.confidence <= 255:
        raise ValueError("--confidence must be 1..255")
    if not (1 <= args.system_id <= 255 and 1 <= args.component_id <= 255):
        raise ValueError("MAVLink IDs must be 1..255")
    if args.calibrate and args.calibration:
        raise ValueError("Select calibration creation or loading, not both")
    if args.calibrate and args.uart:
        raise ValueError("Calibration must run without UART output")
    args.output.mkdir(parents=True, exist_ok=False)
    stopped = False

    def stop(_sig, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    summary = {"utc": datetime.now(timezone.utc).isoformat(), "bus": args.bus,
               "address": hex(args.address), "orientation": args.orientation,
               "uart": args.uart, "baud": args.baud, "confidence_threshold": args.confidence,
               "frames": 0, "sent_distances": 0, "rejected": 0, "result_counter_gaps": 0,
               "duplicates": 0, "calibration_status_counts": {}, "error": None}
    summary.update(uart_rx_bytes=0, uart_rx_messages={}, autopilot_heartbeats=0)
    # Bound timing history for continuous live viewing. Distance extrema below
    # are retained across the whole run without storing every measurement.
    frame_times = deque(maxlen=8192)
    output = None
    sensor = None
    started = None
    try:
        with ExitStack() as stack:
            lock = stack.enter_context(open(HERE / f".i2c-{args.bus}-{args.address:02x}.lock", "a"))
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            bus = stack.enter_context(SMBus(args.bus))
            sensor = Sensor(bus, args.address)
            sensor.enable()
            stack.callback(sensor.standby)
            if sensor.app_id != 3:
                raise RuntimeError(f"Ranging firmware not running: app_id={sensor.app_id:#x}")
            sensor._send_command(0xFF)
            sensor.spad_map = 1
            serial_number = sensor.serial_number
            summary.update(sensor_serial=serial_number, app_id=sensor.app_id,
                           firmware_minor=sensor.minor, spad_map=1,
                           measurement_period_ms=sensor.measurement_period,
                           kilo_iterations=sensor.kilo_iterations)
            print(json.dumps(summary), flush=True)
            if args.calibrate:
                if args.calibrate.exists():
                    raise FileExistsError(args.calibrate)
                payload = sensor.calibrate()
                record = {"sensor_serial": serial_number, "spad_map": 1,
                          "created_utc": datetime.now(timezone.utc).isoformat(),
                          "data_hex": payload.hex(), "sha256": hashlib.sha256(payload).hexdigest()}
                args.calibrate.parent.mkdir(parents=True, exist_ok=True)
                with args.calibrate.open("x") as f:
                    json.dump(record, f, indent=2)
                args.calibration = args.calibrate
                print(f"Calibration saved: {args.calibrate}", flush=True)
            if args.calibration:
                record = json.loads(args.calibration.read_text())
                if record["sensor_serial"] != serial_number or record["spad_map"] != 1:
                    raise ValueError("Calibration belongs to another sensor/SPAD map")
                payload = bytes.fromhex(record["data_hex"])
                if hashlib.sha256(payload).hexdigest() != record["sha256"]:
                    raise ValueError("Calibration checksum mismatch")
                sensor.write_calibration(payload)
                summary["calibration_file"] = str(args.calibration.resolve())
            # Open UART only after a valid calibrated first sample, below.
            capture = stack.enter_context((args.output / "mavlink.bin").open("xb"))
            rx_capture = stack.enter_context((args.output / "uart-rx.bin").open("xb"))
            peer_log = stack.enter_context((args.output / "cube-reports.jsonl").open("x"))
            log = stack.enter_context((args.output / "measurements.jsonl").open("x"))
            output = Output(capture=capture)
            link = mav.MAVLink(output, srcSystem=args.system_id, srcComponent=args.component_id)
            receiver = mav.MAVLink(None)
            receiver.robust_parsing = True
            uart = None
            started = time.monotonic()
            next_tick = started
            last_heartbeat = started - 1
            last_proximity_request = started - 1
            last_configuration_request = started - 1
            last_print = started - 1
            previous_result = None
            while not stopped and (not args.duration or time.monotonic() - started < args.duration):
                measurement, status = sensor.next_measurement(1)
                now = time.monotonic()
                frame_times.append(now)
                summary["frames"] += 1
                key = hex(status)
                summary["calibration_status_counts"][key] = summary["calibration_status_counts"].get(key, 0) + 1
                delta = None if previous_result is None else (measurement.result_number - previous_result) % 256
                if delta == 0:
                    summary["duplicates"] += 1
                elif delta is not None:
                    summary["result_counter_gaps"] += delta - 1
                previous_result = measurement.result_number
                nearest = nearest_return(measurement, args.confidence)
                valid = status == 0 and nearest is not None and delta != 0
                row = {"elapsed_s": now - started, "monotonic_s": now,
                       "result_number": measurement.result_number, "system_tick": measurement.system_tick,
                       "temperature_c": measurement.temperature, "calibration_status": status,
                       "distance_mm": measurement.primary_grid,
                       "confidence": measurement.primary_grid_confidence,
                       "secondary_mm": measurement.secondary_grid,
                       "secondary_confidence": measurement.secondary_grid_confidence,
                       "nearest_mm": nearest[0] if nearest else None, "transmitted": valid}
                if valid:
                    if args.uart and uart is None:
                        import serial
                        uart = stack.enter_context(serial.Serial(args.uart, args.baud, timeout=0,
                                                                 write_timeout=1, exclusive=True,
                                                                 rtscts=False, dsrdtr=False))
                        # Kernel exclusion also blocks non-cooperating subsequent opens.
                        import termios
                        fcntl.ioctl(uart.fileno(), termios.TIOCEXCL)
                        output.uart = uart
                        if args.request_reports:
                            for name in (f'RNGFND{args.rangefinder_slot}_TYPE', f'RNGFND{args.rangefinder_slot}_ORIENT', f'RNGFND{args.rangefinder_slot}_MIN', f'RNGFND{args.rangefinder_slot}_MAX'):
                                link.param_request_read_send(args.system_id, 1, name.encode(), -1)
                    send_distance(link, nearest[0], args.orientation, int(now * 1000))
                    summary["sent_distances"] += 1
                    summary['nearest_min_mm'] = min(summary.get('nearest_min_mm', nearest[0]), nearest[0])
                    summary['nearest_max_mm'] = max(summary.get('nearest_max_mm', nearest[0]), nearest[0])
                else:
                    summary["rejected"] += 1
                log.write(json.dumps(row) + "\n")
                log.flush()
                if now - last_heartbeat >= 1:
                    link.heartbeat_send(mav.MAV_TYPE_ONBOARD_CONTROLLER, mav.MAV_AUTOPILOT_INVALID,
                                        0, 0, mav.MAV_STATE_ACTIVE)
                    last_heartbeat = now
                if now - last_print >= 1:
                    print(json.dumps(row), flush=True)
                    last_print = now
                if args.request_reports and uart is not None and now-last_proximity_request >= .2:
                    link.command_long_send(args.system_id, 1, mav.MAV_CMD_REQUEST_MESSAGE,
                                           0, 132, 0, 0, 0, 0, 0, 0)
                    last_proximity_request = now
                if args.request_reports and uart is not None and now-last_configuration_request >= 2:
                    link.param_request_read_send(args.system_id, 1, f'RNGFND{args.rangefinder_slot}_TYPE'.encode(), -1)
                    last_configuration_request = now
                # Log autopilot heartbeats without sending commands or changing parameters.
                if uart is not None and uart.in_waiting:
                    incoming = uart.read(uart.in_waiting)
                    rx_capture.write(incoming)
                    rx_capture.flush()
                    summary['uart_rx_bytes'] += len(incoming)
                    for msg in receiver.parse_buffer(incoming) or []:
                        name = msg.get_type()
                        summary['uart_rx_messages'][name] = summary['uart_rx_messages'].get(name, 0) + 1
                        if msg.get_srcSystem() == args.system_id and msg.get_srcComponent() == 1:
                            stamp = time.monotonic()
                            if name == 'HEARTBEAT':
                                peer_log.write(json.dumps({'type':'heartbeat','monotonic_s':stamp})+'\n')
                                peer_log.flush()
                            elif name == 'DISTANCE_SENSOR' and msg.orientation == ORIENTATIONS[args.orientation] and msg.id == args.rangefinder_slot-1:
                                peer_log.write(json.dumps({'type':'distance','monotonic_s':stamp,
                                                           'm':msg.current_distance/100,
                                                           'sensor_id':msg.id,'cube_boot_ms':msg.time_boot_ms})+'\n')
                                peer_log.flush()
                            elif name == 'STATUSTEXT':
                                peer_log.write(json.dumps({'type':'status_text','monotonic_s':stamp,
                                                           'severity':msg.severity,'text':msg.text})+'\n')
                                peer_log.flush()
                            elif name == 'PARAM_VALUE' and msg.param_id.startswith(f'RNGFND{args.rangefinder_slot}_'):
                                peer_log.write(json.dumps({'type':'parameter','monotonic_s':stamp,
                                                           'name':msg.param_id,'value':msg.param_value})+'\n')
                                peer_log.flush()
                        if msg.get_type() == "HEARTBEAT":
                            if msg.autopilot != mav.MAV_AUTOPILOT_INVALID:
                                summary['autopilot_heartbeats'] += 1
                            print(f"RX heartbeat system={msg.get_srcSystem()} component={msg.get_srcComponent()}", flush=True)
                next_tick += 1 / args.hz
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()
    except Exception as exc:
        summary["error"] = f"{type(exc).__name__}: {exc}"
        (args.output / "error.txt").write_text(traceback.format_exc())
        print(summary["error"], file=sys.stderr, flush=True)
    finally:
        if started is not None:
            summary["elapsed_s"] = time.monotonic() - started
        if len(frame_times) > 1:
            times = list(frame_times)
            gaps = [b-a for a, b in zip(times, times[1:])]
            summary.update(observed_hz=1/statistics.mean(gaps),
                           timing_statistics_window_frames=len(times),
                           interval_min_ms=min(gaps)*1000, interval_max_ms=max(gaps)*1000,
                           interval_stddev_ms=statistics.pstdev(gaps)*1000)
        if output is not None:
            summary.update(mavlink_packets=output.packets, mavlink_bytes=output.bytes)
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary), flush=True)
    return 1 if summary["error"] or not summary["sent_distances"] else 0


if __name__ == "__main__":
    sys.exit(main())
