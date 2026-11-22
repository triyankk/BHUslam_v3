#!/usr/bin/env python3
"""Exercise physical pin 7 EN, volatile address assignment, and serial identity.

Leaves the sensor enabled at default address 0x41 with saved calibration loaded.
Does not modify pinmux, boot files, or autopilot parameters.
"""
import argparse
from datetime import datetime, timezone
import errno
import fcntl
import hashlib
import json
from pathlib import Path
import time
import traceback
import sys

from bridge import HERE, Sensor, nearest_return
from smbus2 import SMBus, i2c_msg


def probe(bus, address):
    try:
        value = i2c_msg.read(address, 1)
        bus.i2c_rdwr(i2c_msg.write(address, [0xE0]), value)
        return list(value)[0]
    except OSError as exc:
        if exc.errno in (errno.EREMOTEIO, errno.ENXIO):
            return None
        raise


def assign(sensor, new_address):
    # This command changes the bus address immediately. Polling the old address,
    # as the vendored helper does, can NACK even when the change succeeds.
    with sensor._configuration_mode():
        sensor.bus.write_byte_data(sensor.address, 0x3B, new_address << 1)
        sensor.bus.write_byte_data(sensor.address, 0x3E, 0)
    sensor.bus.write_byte_data(sensor.address, 0x08, 0x21)
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        if probe(sensor.bus, new_address) is not None:
            sensor.address = new_address
            status = sensor.bus.read_byte_data(new_address, 0x08)
            if status < 0x10:
                if status > 1:
                    raise RuntimeError(f"Address-change command status {status:#x}")
                return
        time.sleep(.01)
    raise TimeoutError("Sensor did not acknowledge the newly assigned address")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    import Jetson.GPIO as gpio
    result = {'utc': datetime.now(timezone.utc).isoformat(), 'physical_pin': 7,
              'gpio': 'PAC.06', 'bus': 7, 'cycles': [], 'passed': False,
              'error': None, 'restored_default': False}
    configured = False
    calibration = json.loads((HERE / 'calibration-23937420-map1.json').read_text())
    payload = bytes.fromhex(calibration['data_hex'])
    if hashlib.sha256(payload).hexdigest() != calibration['sha256']:
        raise ValueError('Calibration integrity failure')
    try:
        with open(HERE / '.i2c-7-41.lock', 'a') as lock, SMBus(7) as bus:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            gpio.setmode(gpio.BOARD)
            gpio.setup(7, gpio.OUT, initial=gpio.HIGH, consumer='tmf8820-en-identity')
            configured = True
            try:
                for cycle in range(3):
                    row = {'cycle': cycle+1}
                    result['cycles'].append(row)
                    gpio.output(7, gpio.LOW)
                    time.sleep(.1)
                    row['low_readback'] = gpio.input(7)
                    row['low_default_ack'] = probe(bus, 0x41) is not None
                    row['low_assigned_ack'] = probe(bus, 0x42) is not None
                    if row['low_default_ack'] or row['low_assigned_ack']:
                        raise RuntimeError('EN low did not isolate/reset the sensor')
                    gpio.output(7, gpio.HIGH)
                    time.sleep(.1)
                    row['high_readback'] = gpio.input(7)
                    row['high_default_ack'] = probe(bus, 0x41) is not None
                    if not row['high_default_ack']:
                        raise RuntimeError('Sensor does not acknowledge 0x41 with EN high; check physical pinmux, wiring, and power')
                    sensor = Sensor(bus, 0x41)
                    row['reset_app_id'] = sensor.app_id
                    if row['reset_app_id'] != 0x80:
                        raise RuntimeError('EN reset did not return sensor to bootloader')
                    sensor.enable()
                    row['serial_before'] = sensor.serial_number
                    if row['serial_before'] != calibration['sensor_serial']:
                        raise RuntimeError('Unexpected sensor serial number')
                    if probe(bus, 0x42) is not None:
                        raise RuntimeError('0x42 is already occupied; refusing address assignment')
                    assign(sensor, 0x42)
                    row['serial_after'] = sensor.serial_number
                    row['old_address_ack'] = probe(bus, 0x41) is not None
                    if row['serial_after'] != row['serial_before'] or row['old_address_ack']:
                        raise RuntimeError('Identity or address exclusivity check failed')
                    sensor.spad_map = 1
                    sensor.write_calibration(payload)
                    measurement, status = sensor.next_measurement(1)
                    row['calibration_status'] = status
                    row['nearest_return'] = nearest_return(measurement)
                    if status != 0:
                        raise RuntimeError('Calibration did not reload at the assigned address')
                    print(json.dumps(row), flush=True)
                result['passed'] = True
            finally:
                # Return to the original single-board configuration, even after
                # an unsuccessful test. Keep GPIO high, then release its handle.
                active_error = sys.exc_info()[0] is not None
                try:
                    gpio.output(7, gpio.LOW)
                    time.sleep(.1)
                    gpio.output(7, gpio.HIGH)
                    time.sleep(.1)
                    if probe(bus, 0x41) is not None:
                        sensor = Sensor(bus, 0x41)
                        sensor.enable()
                        if sensor.serial_number == calibration['sensor_serial']:
                            sensor.spad_map = 1
                            sensor.write_calibration(payload)
                            result['restored_default'] = True
                except Exception as exc:
                    result['cleanup_error'] = f'{type(exc).__name__}: {exc}'
                    if not active_error:
                        raise
                finally:
                    gpio.output(7, gpio.HIGH)
                    gpio.cleanup(7)
                    configured = False
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
        (args.output / 'error.txt').write_text(traceback.format_exc())
    finally:
        if configured:
            gpio.output(7, gpio.HIGH)
            gpio.cleanup(7)
        (args.output / 'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] and result['restored_default'] and not result['error'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
