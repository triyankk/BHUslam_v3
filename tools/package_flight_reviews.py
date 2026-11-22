#!/usr/bin/env python3
"""Snapshot saved arm-triggered sessions for Git review and release archives.

Read-only with respect to source recordings. No hardware/service operations.
"""
import argparse
import csv
import hashlib
import io
import json
import math
import re
import shutil
import subprocess
import tarfile
from datetime import datetime, timezone
from pathlib import Path


def load(path):
    return json.loads(path.read_text()) if path.is_file() else {}


def dump(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + '\n')


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def valid_pgm(path):
    if not path.is_file():
        return False
    try:
        with path.open('rb') as f:
            tokens = []
            while len(tokens) < 4:
                line = f.readline(1024)
                if not line:
                    return False
                tokens.extend(line.split(b'#', 1)[0].split())
            magic, width, height, maximum = tokens
            width, height, maximum = int(width), int(height), int(maximum)
            return (magic == b'P5' and width > 0 and height > 0
                    and 0 < maximum <= 65535
                    and path.stat().st_size == f.tell() + width * height * (1 if maximum < 256 else 2))
    except (ValueError, OSError):
        return False


def frame_check(folder):
    index = folder / 'frames.csv'
    result = {'indexed_pairs': 0, 'invalid_pairs': 0, 'invalid_row_indices_zero_based': [],
              'valid_prefix_pairs': 0, 'check': 'PGM header and exact payload length; no photometric/geometry test'}
    if not index.is_file():
        result['error'] = 'frames.csv missing'
        return result
    data = index.read_bytes()
    nul = data.find(b'\0')
    if nul >= 0:
        # Interrupted files can contain zero-filled tails. Inspect complete rows
        # before corruption; retain the original index unchanged in the archive.
        boundary = data.rfind(b'\n', 0, nul) + 1
        result['error'] = 'NUL corruption in frames.csv; only complete preceding rows inspected'
        result['unparsed_index_bytes'] = len(data) - boundary
        data = data[:boundary]
    with io.StringIO(data.decode('utf-8', errors='replace')) as stream:
        for i, row in enumerate(csv.DictReader(stream)):
            result['indexed_pairs'] += 1
            valid = True
            for side in ['left', 'right']:
                name = row.get(side)
                p = (folder / name).resolve() if name else folder
                if not p.is_relative_to(folder.resolve()) or not valid_pgm(p):
                    valid = False
            if not valid:
                result['invalid_pairs'] += 1
                result['invalid_row_indices_zero_based'].append(i)
            elif result['invalid_pairs'] == 0:
                result['valid_prefix_pairs'] += 1
    return result


def map_vertices(path):
    if not path.is_file():
        return None
    with path.open() as f:
        for line in f:
            if line.startswith('element vertex '):
                return int(line.split()[-1])
            if line.strip() == 'end_header':
                break
    return None


def preview(source, destination, count, limit=5000):
    step = max(1, math.ceil(count / limit))
    with source.open() as f:
        for line in f:
            if line.strip() == 'end_header':
                break
        points = [line for i, line in enumerate(f) if i % step == 0]
    destination.write_text('ply\nformat ascii 1.0\nelement vertex ' + str(len(points))
                           + '\nproperty float x\nproperty float y\nproperty float z\nend_header\n'
                           + ''.join(points))
    return len(points)


class HashReader:
    def __init__(self, stream):
        self.stream = stream
        self.hash = hashlib.sha256()

    def read(self, size):
        data = self.stream.read(size)
        self.hash.update(data)
        return data


def archive(output, package_id, entries):
    """Write exact source bytes plus per-member hashes; reject changing sources."""
    temporary = output / (package_id + '.tar.zst.partial')
    final = output / (package_id + '.tar.zst')
    if temporary.exists() or final.exists() or list(output.glob(final.name + '.part*')):
        raise RuntimeError('Archive already exists: ' + str(final))
    proc = subprocess.Popen(['nice', '-n', '10', 'zstd', '-q', '-T2', '-3', '-o', str(temporary)], stdin=subprocess.PIPE)
    members = []
    try:
        with tarfile.open(fileobj=proc.stdin, mode='w|') as tar:
            for source, relative in entries:
                if source.is_symlink() or not source.is_file():
                    raise RuntimeError('Expected regular file: ' + str(source))
                before = source.stat()
                info = tar.gettarinfo(str(source), arcname=package_id + '/' + relative)
                info.uid = info.gid = 0
                info.uname = info.gname = ''
                with source.open('rb') as stream:
                    reader = HashReader(stream)
                    tar.addfile(info, reader)
                after = source.stat()
                if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                    raise RuntimeError('Source changed while packaging: ' + str(source))
                members.append({'path': relative, 'source': str(source), 'bytes': before.st_size,
                                'sha256': reader.hash.hexdigest()})
            payload = (json.dumps({'files': members}, indent=2) + '\n').encode()
            info = tarfile.TarInfo(package_id + '/files.json')
            info.size = len(payload)
            tar.addfile(info, io.BytesIO(payload))
        proc.stdin.close()
        if proc.wait() != 0:
            raise RuntimeError('zstd compression failed')
        subprocess.run(['zstd', '-q', '--test', str(temporary)], check=True)
        temporary.rename(final)
    except BaseException:
        proc.stdin.close()
        proc.kill()
        proc.wait()
        raise
    # Keep each release asset below 1 GiB, including large future sessions.
    cap = 1024 ** 3
    paths = [final]
    if final.stat().st_size >= cap:
        paths = []
        with final.open('rb') as source:
            i = 0
            remaining_total = final.stat().st_size
            while remaining_total:
                part = output / (final.name + f'.part{i:03d}')
                remaining_part = min(cap, remaining_total)
                with part.open('wb') as destination:
                    while remaining_part:
                        block = source.read(min(8 * 1024 * 1024, remaining_part))
                        if not block:
                            raise RuntimeError('Unexpected EOF while splitting archive')
                        destination.write(block)
                        remaining_part -= len(block)
                        remaining_total -= len(block)
                paths.append(part)
                i += 1
        final.unlink()
    return members, [{'name': p.name, 'bytes': p.stat().st_size, 'sha256': digest(p)} for p in paths]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--reviews', type=Path, required=True, help='New Git-tracked review directory')
    parser.add_argument('--archives', type=Path, required=True, help='New ignored archive directory')
    parser.add_argument('--tag', required=True)
    parser.add_argument('--repo', default='triyankk/BHUslam_v3')
    args = parser.parse_args()
    root = args.workspace.resolve()
    reviews, archives = args.reviews.resolve(), args.archives.resolve()
    if reviews.exists() or archives.exists():
        parser.error('Output directories must not exist; use a new snapshot name')
    reviews.mkdir(parents=True)
    archives.mkdir(parents=True)
    release = f'https://github.com/{args.repo}/releases/tag/{args.tag}'
    download = f'https://github.com/{args.repo}/releases/download/{args.tag}/'
    raw_root = Path('/home/atas/vscode/optFlow_slam/data/recordings/jt16-shadow')
    calibration_root = Path('/home/atas/vscode/optFlow_slam/data/calibrations/jt16')
    known = load(root / 'reports/latest-flights-20261120/flights-by-cube.json')
    known = {str(Path(row['path']).resolve()): row for row in known} if known else {}
    revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    catalog = {'schema_version': 1, 'source_code_commit': revision,
               'packaged_utc': datetime.now(timezone.utc).isoformat(), 'release_tag': args.tag,
               'release_url': release, 'flights': [], 'assets': []}
    used_raw = set()
    for flight in sorted((root / 'runs/vins-fusion-boot').glob('*/live/flights/flight-*')):
        if not flight.is_dir():
            continue
        run = flight.parents[2]
        package_id = run.name + '_' + flight.name
        review = reviews / package_id
        review.mkdir()
        boot = (run / 'boot-id.txt').read_text().strip()
        meta = load(flight / 'flight.json')
        start = meta['recording_start_host_us']
        stop = meta.get('recording_stop_host_us', start)
        notes = []
        if not meta.get('finalized'):
            notes.append('VIO recording was not finalized; preserve as incomplete evidence.')
        check = frame_check(flight / 'input')
        if check.get('error'):
            notes.append(check['error'] + '.')
        if check['invalid_pairs']:
            notes.append(f"{check['invalid_pairs']} indexed stereo pairs are missing or invalid; original bytes are preserved.")
        entries = []

        def add(source, relative):
            if source.is_file():
                entries.append((source, relative))

        def copy(source, relative):
            if source.is_file():
                destination = review / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, destination)

        for name in ['flight.json', 'pose_history.csv', 'tracking.csv']:
            add(flight / name, 'vio/' + name)
            copy(flight / name, 'vio/' + name)
        for source in sorted((flight / 'input').rglob('*')):
            if source.is_file():
                add(source, 'vio/input/' + source.relative_to(flight / 'input').as_posix())
        for name in ['recording.json', 'orb.yaml']:
            copy(flight / 'input' / name, 'vio/input/' + name)
        for name in ['boot-id.txt', 'boot-uptime.txt', 'camera.yaml', 'navigation-mount.yaml',
                     'power-mode.txt', 'binary.sha256', 'runtime-profile.sha256', 'cube-startup-status.json']:
            add(run / name, 'boot/' + name)
            copy(run / name, 'boot/' + name)

        candidates = list((root / 'runs/flight-shadow' / boot).glob('mapping-*/data/flights/flight-*'))
        candidates = [p for p in candidates if abs(int(p.name.split('-')[1]) - start) <= 2_000_000]
        mapping = None
        if len(candidates) == 1:
            folder = candidates[0]
            chosen = folder if (folder / 'summary.json').exists() else None
            if chosen is None:
                checkpoints = [p.parent for p in folder.glob('checkpoints/*/summary.json')
                               if p.parent.name.isdigit() and (p.parent / 'map.ply').is_file()]
                chosen = max(checkpoints, key=lambda p: int(p.name)) if checkpoints else None
            for name in ['trajectory.csv', 'clouds.csv', 'imu.csv']:
                add(folder / name, 'jt16/' + name)
            copy(folder / 'trajectory.csv', 'jt16/trajectory.csv')
            mapping = {'source': str(folder), 'association': 'same boot UUID; unique arm-start within 2 seconds',
                       'start_delta_us': int(folder.name.split('-')[1]) - start,
                       'selected_export': str(chosen) if chosen else None}
            if chosen:
                count = map_vertices(chosen / 'map.ply')
                mapping.update({'vertices': count, 'summary': load(chosen / 'summary.json'),
                                'checkpoint_recovery': chosen != folder})
                add(chosen / 'summary.json', 'jt16/summary.json')
                add(chosen / 'map.ply', 'jt16/map.ply')
                copy(chosen / 'summary.json', 'jt16/summary.json')
                if count:
                    mapping['preview_vertices'] = preview(chosen / 'map.ply', review / 'jt16/map-preview.ply', count)
                else:
                    notes.append('JT16 selected map has zero vertices; no map preview is supplied.')
                if chosen != folder:
                    notes.append('JT16 map recovered from the latest completed checkpoint; final export is missing.')
            else:
                notes.append('JT16 timing/IMU/trajectory logs exist, but no complete map export was found.')
        else:
            notes.append('No unique JT16 map session matched within 2 seconds in the same boot.')

        raw_files = []
        for source in sorted((raw_root / boot).glob('*.raw')):
            match = re.match(r'flight-(\d+)-', source.name)
            if match and start - 2_000_000 <= int(match[1]) <= stop:
                add(source, 'jt16/raw/' + source.name)
                raw_files.append(str(source))
                used_raw.add(source.resolve())
        if not raw_files:
            notes.append('No JT16 raw capture could be associated by same boot and recording-start interval.')
        if raw_files:
            for name in ['JT16_sample_angle-2.csv', 'source.json']:
                add(calibration_root / name, 'jt16/calibration/' + name)
            notes.append('JT16 sample-angle table/provenance is the currently installed copy, not proven historical calibration.')

        cube = known.get(str(flight.resolve()))
        cube_record = None
        if cube:
            log_id = cube['cube_log_expected']
            binary = root / f'reports/latest-flights-20261120/cube-logs/cube-log-{log_id}.BIN'
            add(binary, 'cube/' + binary.name)
            for source in map(Path, cube.get('observation_logs', [])):
                add(source, 'cube/observation/' + source.name)
            cube_record = {'log_id': log_id, 'binary_found': binary.is_file(),
                           'association_evidence': 'reports/latest-flights-20261120/boot-cube-matches.json',
                           'association': 'existing exact VISP/companion timestamp and boot-UUID review'}
            notes.append('Cube DataFlash association comes from the saved four-flight audit; its log may span other arm cycles.')
        else:
            notes.append('No independently verified Cube DataFlash association is available for this session.')

        record = {'id': package_id, 'boot_id': boot, 'original_vio_folder': str(flight),
                  'flight': meta, 'input_integrity': check, 'jt16': mapping, 'jt16_raw': raw_files,
                  'cube': cube_record, 'notes': notes, 'source_code_commit': revision,
                  'time_warning': 'Run folder timestamps and mtimes are not reliable flight order; use boot UUID and host-monotonic times.',
                  'validation_limit': 'Recorded arm sessions, not proof of takeoff, trajectory accuracy, EKF fusion or autonomous avoidance.'}
        dump(review / 'manifest.json', record)
        add(review / 'manifest.json', 'manifest.json')
        print('Packaging', package_id, 'pairs', check['indexed_pairs'], 'invalid', check['invalid_pairs'], flush=True)
        members, assets = archive(archives, package_id, entries)
        dump(review / 'files.json', {'files': members})
        for asset in assets:
            asset['url'] = download + asset['name']
            asset['flight_id'] = package_id
        dump(review / 'downloads.json', {'release': release, 'assets': assets})
        links = '\n'.join(f"- [{a['name']}]({a['url']}) ({a['bytes']:,} bytes, SHA-256 `{a['sha256']}`)" for a in assets)
        readme = (f'# Review: {package_id}\n\n'
                  f"Boot UUID: `{boot}`. Arm-triggered recording; finalized: **{meta.get('finalized', False)}**.\n\n"
                  f"Stereo pairs indexed: **{check['indexed_pairs']}**; invalid pairs: **{check['invalid_pairs']}**.\n\n"
                  'This package preserves existing evidence toward GPS-denied navigation and obstacle-avoidance evaluation. '
                  'It does not establish flight accuracy, Cube fusion, or mission readiness.\n\n'
                  '## Review files\n\n'
                  '`vio/` contains recorded poses, tracking diagnostics and input metadata. `boot/` contains saved settings '
                  'and executable identity. `manifest.json` records source paths, associations, completeness and limitations. '
                  '`files.json` gives SHA-256 hashes of every original file included in the archive.\n\n'
                  'A `jt16/map-preview.ply`, when present, is a deterministic subset of at most 5,000 exported map vertices. '
                  'The full selected map is `jt16/map.ply` inside the replay archive.\n\n'
                  '## Findings and limitations\n\n' + '\n'.join('- ' + n for n in notes) + '\n\n'
                  '## Complete replay evidence\n\n' + links + '\n\n'
                  'See [storage and restoration instructions](../../docs/flight-data-storage.md). '
                  'Original files are retained byte-for-byte, including incomplete or damaged inputs. '
                  'VIO images are selected rectified processed pairs before CLAHE; overwritten/unrectified pairs are absent.\n')
        (review / 'README.md').write_text(readme)
        catalog['flights'].append(record)
        catalog['assets'].extend(assets)
        print('Ready', package_id, sum(a['bytes'] for a in assets), 'compressed bytes', flush=True)

    catalog['unassociated_raw_files'] = [str(p) for p in sorted(raw_root.rglob('*.raw')) if p.resolve() not in used_raw]
    dump(reviews / 'catalog.json', catalog)
    dump(archives / 'catalog.json', catalog)
    checksum = ''.join(f"{a['sha256']}  {a['name']}\n" for a in catalog['assets'])
    (archives / 'SHA256SUMS').write_text(checksum)
    (reviews / 'SHA256SUMS').write_text(checksum)
    lines = ['# Saved flight review catalog', '', f'[Replay dataset release]({release})', '',
             'Sessions are indexed by recorded run name and arm-start ID, not claimed chronological flight order. '
             'Folder wall clocks reset in older sessions. See [storage instructions](../docs/flight-data-storage.md).', '',
             '| Session | Indexed pairs | Invalid pairs | JT16 map vertices | Cube log |',
             '| --- | ---: | ---: | ---: | --- |']
    for r in catalog['flights']:
        vertices = r['jt16'].get('vertices', 'unavailable') if r['jt16'] else 'unavailable'
        lines.append(f"| [{r['id']}]({r['id']}/README.md) | {r['input_integrity']['indexed_pairs']} | "
                     f"{r['input_integrity']['invalid_pairs']} | {vertices} | {r['cube']['log_id'] if r['cube'] else 'unmatched'} |")
    lines += ['', 'Unassociated raw recordings are listed in `catalog.json`; they were not silently attached to a flight.', '']
    (reviews / 'README.md').write_text('\n'.join(lines))
    print('Complete:', len(catalog['flights']), 'sessions;', len(catalog['assets']), 'assets', flush=True)


if __name__ == '__main__':
    main()
