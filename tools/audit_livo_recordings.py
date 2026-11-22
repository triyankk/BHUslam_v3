#!/usr/bin/env python3
"""Read-only LIVO input inventory; packet header checks are NOT SDK/CRC validation."""
import calendar
import csv
import datetime
import json
from pathlib import Path
import struct
import subprocess
import argparse

ROOT = Path(__file__).resolve().parents[1]
RAW = Path('/home/atas/vscode/optFlow_slam/data/recordings/jt16-shadow')


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def raw_inventory(path):
    data = path.read_bytes()
    position = skipped = 0
    times = {0: [], 1: []}
    seq = {0: [], 1: []}
    while position + 16 <= len(data):
        kind = data[position + 5]
        if data[position:position + 4] != b'\xee\xff\x01\x08' or kind not in times:
            position += 1
            skipped += 1
            continue
        size = 80 if kind == 0 else 34
        if position + size > len(data):
            break
        utc = data[position + 6:position + 12]
        try:
            if utc[0]:
                stamp = calendar.timegm(datetime.datetime(1900 + utc[0], *utc[1:]).timetuple())
            else:
                stamp = int.from_bytes(utc[2:6], 'big')
            stamp += struct.unpack_from('<I', data, position + 12)[0] * 1e-6
            times[kind].append(stamp)
            seq[kind].append(struct.unpack_from('<H', data, position + size - 6)[0])
        except (ValueError, OverflowError):
            skipped += size
        position += size
    result = {'path': str(path), 'bytes': len(data), 'skipped_bytes': skipped,
              'trailing_bytes': len(data) - position, 'crc_validated': False}
    for kind, name in [(0, 'point_packets'), (1, 'imu_packets')]:
        stamps = times[kind]
        deltas = [b - a for a, b in zip(stamps, stamps[1:])]
        jumps = [((b-a) % 65536) for a, b in zip(seq[kind], seq[kind][1:])]
        result[name] = {'count': len(stamps), 'sensor_start_s': stamps[0] if stamps else None,
                        'sensor_end_s': stamps[-1] if stamps else None,
                        'span_s': stamps[-1]-stamps[0] if stamps else 0,
                        'max_gap_s': max(deltas, default=0),
                        'nonincreasing_timestamps': sum(x <= 0 for x in deltas),
                        'sequence_discontinuities': sum(x != 1 for x in jumps)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--crc-auditor', type=Path, help='Built tools/jt16/audit_raw_crc.cpp executable')
    args = parser.parse_args()
    flights = json.loads((ROOT/'reports/latest-flights-20261120/flights-by-cube.json').read_text())
    result = {'scope': 'Latest four matched Cube flights; offline input audit only',
              'livo_run': False, 'accuracy_improvement_established': False,
              'warning': 'Raw packet counts/timing are header-only, without CRC or XYZ decoding. Sensor clocks are not verified UTC.',
              'flights': []}
    for flight in flights:
        recording = Path(flight['path'])/'input'
        frames = rows(recording/'frames.csv')
        captures = rows(recording/'capture.csv')
        host = [float(x['host_capture_us'])*1e-6 for x in captures]
        start = flight['recording_start_host_us']*1e-6
        stop = flight['recording_stop_host_us']*1e-6
        mapping = []
        for path in sorted((ROOT/'runs/flight-shadow'/flight['boot']).glob('mapping-*/data/flights/*/clouds.csv')):
            clouds = rows(path)
            selected = [x for x in clouds if start <= float(x['host_ns'])*1e-9 <= stop]
            mapping.append({'path': str(path), 'clouds_logged': len(clouds),
                            'clouds_during_camera_recording': len(selected)})
        result['flights'].append({
            'cube_log': flight['cube_log_expected'], 'boot': flight['boot'],
            'recording': str(recording), 'recording_duration_s': stop-start,
            'images': len(frames), 'selected_image_rate_hz': (len(host)-1)/(host[-1]-host[0]) if len(host)>1 else 0,
            'max_selected_image_gap_s': max((b-a for a,b in zip(host,host[1:])), default=0),
            'accel_samples': len(rows(recording/'accel.csv')),
            'gyro_samples': len(rows(recording/'gyro.csv')),
            'mapping_logs': mapping,
            'raw_captures': [raw_inventory(p) for p in sorted((RAW/flight['boot']).glob('*.raw'))]})
    output = ROOT/'reports/livo-replay-readiness-20261120'
    output.mkdir(exist_ok=True)
    for flight in result['flights']:
        captures = rows(Path(flight['recording'])/'capture.csv')
        for raw in flight['raw_captures']:
            if args.crc_auditor:
                raw['sdk_crc_audit'] = json.loads(subprocess.check_output(
                    [str(args.crc_auditor.resolve()), raw['path']], text=True))
            opening = float(Path(raw['path']).stem.split('-')[1])/1e6
            raw['coarse_file_open_host_s'] = opening
            raw['coarse_overlap_images'] = sum(opening <= float(v['host_capture_us'])/1e6
                <= opening+raw['point_packets']['span_s'] for v in captures)
            raw['coarse_timing_warning'] = 'Filename file-open time plus sensor span only; not validated capture synchronization.'
    (output/'inventory.json').write_text(json.dumps(result, indent=2)+'\n')
    for flight in result['flights']:
        print(f"Cube {flight['cube_log']}: {flight['images']} images, {flight['selected_image_rate_hz']:.1f} Hz, max gap {flight['max_selected_image_gap_s']:.3f}s")
        for raw in flight['raw_captures']:
            print(f"  {Path(raw['path']).name}: {raw['point_packets']['count']} point packets, span {raw['point_packets']['span_s']:.3f}s, max gap {raw['point_packets']['max_gap_s']:.3f}s; skipped={raw['skipped_bytes']}")


if __name__ == '__main__':
    main()
