#!/usr/bin/env python3
"""Temporary Orin Nano physical-pin-7 output setup and EN identity test.

Uses NVIDIA's documented runtime PADCTL procedure. Writes only pin 7's register,
saves the original value before writing, and restores it when the test exits.
No boot/device-tree files are changed. Requires root access to /dev/mem.
"""
import argparse
from datetime import datetime, timezone
import json
import mmap
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys

HERE = Path(__file__).resolve().parent
REGISTER = 0x02448030  # NVIDIA jetson-gpio Orin Nano physical pin 7/PAC.06
OUTPUT_MASK = (1 << 10) | (1 << 6) | (1 << 4)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--test', action='store_true', help='Apply temporary output setting and run test')
    args = p.parse_args()
    model = Path('/proc/device-tree/model').read_bytes().rstrip(b'\x00').decode()
    if 'Orin Nano' not in model:
        raise RuntimeError(f'Refusing pinmux access on another platform: {model}')
    if os.geteuid() != 0:
        raise PermissionError('Run with sudo; /dev/mem requires root')
    import Jetson.GPIO.gpio_pin_data as pins
    definition = next(x for x in pins.JETSON_ORIN_NX_PIN_DEFS if x[3] == 7)
    if definition[:3] != (144, 'PAC.06', 'tegra234-gpio'):
        raise RuntimeError(f'Unexpected installed pin-7 mapping: {definition}')
    lines = subprocess.check_output(['gpioinfo', 'gpiochip0'], text=True)
    line = next(x for x in lines.splitlines() if 'line 144:' in x)
    if '"PAC.06"' not in line or '[used]' in line:
        raise RuntimeError(f'Pin 7 is unavailable: {line}')
    output = HERE.parents[1] / 'reports' / 'tof' / ('en-pinmux-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    output.mkdir(parents=True, exist_ok=False)
    record = {'model': model, 'register': hex(REGISTER), 'applied': False,
              'restored': False, 'test_exit_code': None}
    page = REGISTER & ~(mmap.PAGESIZE - 1)
    offset = REGISTER - page

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    with open('/dev/mem', 'r+b', buffering=0) as device:
        with mmap.mmap(device.fileno(), mmap.PAGESIZE, offset=page,
                       flags=mmap.MAP_SHARED, prot=mmap.PROT_READ | mmap.PROT_WRITE) as memory:
            original = struct.unpack_from('<I', memory, offset)[0]
            proposed = original & ~OUTPUT_MASK
            record.update(original=hex(original), proposed=hex(proposed))
            # Persist rollback data before any hardware modification.
            record_path = output / 'pinmux.json'
            record_path.write_text(json.dumps(record, indent=2)+'\n')
            print(json.dumps(record, indent=2), flush=True)
            print('Report directory:', output, flush=True)
            if not args.test:
                print('Read-only: no register write requested.', flush=True)
                return 0
            if original == 0xFFFFFFFF or original & (1 << 7):
                raise RuntimeError('Unexpected/locked PADCTL value; refusing modification')
            try:
                struct.pack_into('<I', memory, offset, proposed)
                record['applied'] = True
                if struct.unpack_from('<I', memory, offset)[0] != proposed:
                    raise RuntimeError('Pinmux write readback mismatch')
                import check_en_identity
                sys.argv = [str(HERE / 'check_en_identity.py'), '--output', str(output / 'identity')]
                code = check_en_identity.main()
                record['test_exit_code'] = code
                return code
            finally:
                struct.pack_into('<I', memory, offset, original)
                record['restored'] = struct.unpack_from('<I', memory, offset)[0] == original
                record_path.write_text(json.dumps(record, indent=2)+'\n')
                print('Original pinmux restored:', record['restored'], flush=True)
                if not record['restored']:
                    raise RuntimeError('Pinmux restoration failed; original value is recorded in pinmux.json')


if __name__ == '__main__':
    raise SystemExit(main())
