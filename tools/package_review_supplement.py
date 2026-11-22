#!/usr/bin/env python3
"""Preserve November optical-flow telemetry and omitted review diagnostics."""
import argparse
import csv
from collections import Counter
from datetime import datetime, timezone
import io
import json
from pathlib import Path
import re
import shutil
import subprocess
import time

from package_flight_reviews import archive, digest, dump, load


def readable_csv(data):
    nul = data.find(b'\0')
    if nul >= 0:
        data = data[:data.rfind(b'\n', 0, nul) + 1]
    return csv.DictReader(io.StringIO(data.decode('utf-8', errors='replace'))), nul >= 0


def flow_rows(rows):
    output = []
    counts = Counter()
    fields = ['flow_x', 'flow_y', 'flow_comp_m_x', 'flow_comp_m_y', 'flow_rate_x',
              'flow_rate_y', 'quality', 'ground_distance', 'sensor_id', 'time_usec']
    for row in rows:
        name = row.get('message_type', '')
        counts[name] += 1
        if name not in ['OPTICAL_FLOW', 'OPTICAL_FLOW_RAD']:
            continue
        try:
            values = json.loads(row.get('fields', '{}'))
        except (ValueError, TypeError):
            values = {}
        item = {k: row.get(k, '') for k in ['timestamp_local', 'timestamp_utc', 'monotonic_ns',
                                           'message_type', 'source_system', 'source_component']}
        item.update({k: values.get(k, '') for k in fields})
        item['original_fields_json'] = row.get('fields', '')
        output.append(item)
    return output, counts


def extend_context(reviews, archives):
    """Snapshot current settings and otherwise-unassociated evidence separately."""
    root = Path(__file__).resolve().parents[1]
    output = reviews / 'supplement'
    result = load(output / 'catalog.json')
    name = 'effective-settings-and-unmatched-evidence-20261122'
    folder = archives / 'supplement-staging' / name
    folder.mkdir()
    covered = {r['source'] for p in reviews.glob('*/files.json') for r in load(p)['files']}
    covered.update(r['original_source'] for p in output.glob('*-files.json') for r in load(p)['files'])
    sources = []
    for p in (root / 'runs/tof-front').rglob('*'):
        if p.is_file() and p.suffix in ['.json', '.jsonl', '.csv', '.log', '.txt', '.yaml', '.sha256']:
            sources.append((p, p.relative_to(root).as_posix(), False))
    # Preserve all saved performance/adapter logs, including bench boots, while
    # retaining their original boot directories instead of assigning flights.
    for p in (root / 'runs/flight-shadow').rglob('*'):
        if p.is_file() and p.name in ['tegrastats.log', 'adapter.log', 'lio.log']:
            sources.append((p, p.relative_to(root).as_posix(), False))
    for item in load(reviews / 'catalog.json')['unassociated_raw_files']:
        p = Path(item)
        sources.append((p, 'unassociated-jt16/' + p.parent.name + '/' + p.name, False))
    units = Path('/home/atas/.config/systemd/user')
    for p in sorted(units.glob('bhu-*.service')) + sorted((units / 'optflow-flight-logger.service.d').glob('*.conf')):
        sources.append((p, 'effective-settings/systemd/' + p.relative_to(units).as_posix(), True))
    p = Path('/home/atas/vscode/optFlow_slam/config/system.yaml')
    sources.append((p, 'effective-settings/optFlow_slam/system.yaml', True))
    entries = []
    provenance = []
    for source, relative, sanitize in sorted(sources):
        if str(source.resolve()) in covered:
            continue
        before = source.stat()
        if not sanitize and before.st_mtime > time.time() - 120:
            result['excluded_or_unstable'].append({'path': str(source), 'reason': 'recent live evidence; left local'})
            continue
        target = folder / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if sanitize:
            lines = source.read_text().splitlines(keepends=True)
            redacted = sum(bool(re.search(r'token|secret|password|credential|api[_-]?key|private[_-]?key', line, re.I)) for line in lines)
            text = ''.join('# REDACTED sensitive-setting line\n' if re.search(r'token|secret|password|credential|api[_-]?key|private[_-]?key', line, re.I) else line for line in lines)
            text = re.sub(r'(https?://)[^\s/@]+:[^\s/@]+@', r'\1REDACTED@', text)
            target.write_text(text)
            destination = output / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(target, destination)
        else:
            redacted = 0
            shutil.copy2(source, target)
        after = source.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            target.unlink()
            result['excluded_or_unstable'].append({'path': str(source), 'reason': 'changed during context snapshot'})
            continue
        entries.append((target, relative))
        provenance.append({'path': relative, 'original_source': str(source), 'bytes': target.stat().st_size,
                           'sha256': digest(target), 'sanitized': sanitize, 'redacted_lines': redacted,
                           'capture_note': 'Current installed file; not proven historical flight configuration' if sanitize else 'Unassociated or boot-level evidence, not forced into a flight'})
    dump(folder / 'provenance.json', {'captured_utc': datetime.now(timezone.utc).isoformat(), 'files': provenance})
    entries.append((folder / 'provenance.json', 'provenance.json'))
    print('Packaging context supplement', len(provenance), 'files', flush=True)
    members, assets = archive(archives, name, entries)
    dump(output / (name + '-files.json'), {'files': provenance})
    for asset in assets:
        asset['url'] = f"https://github.com/triyankk/BHUslam_v3/releases/download/{result['release_tag']}/{asset['name']}"
    result['archives'].extend(assets)
    for session in result['sessions']:
        session['manifest_count_mismatch'] = session['manifest_cube_rows'] is not None and session['manifest_cube_rows'] != session['readable_cube_rows']
        dump(output / 'optical-flow' / session['session'] / 'summary.json', session)
    dump(output / 'catalog.json', result)
    dump(archives / 'supplement-catalog.json', result)
    sums = ''.join(f"{a['sha256']}  {a['name']}\n" for a in result['archives'])
    sums += digest(archives / 'supplement-catalog.json') + '  supplement-catalog.json\n'
    (output / 'SHA256SUMS').write_text(sums)
    (archives / 'SUPPLEMENT-SHA256SUMS').write_text(sums)
    print('Context complete:', len(assets), 'assets;', sum(a['bytes'] for a in assets), 'compressed bytes', flush=True)


def append_transport(reviews, archives):
    root = Path(__file__).resolve().parents[1]
    output = reviews / 'supplement'
    result = load(output / 'catalog.json')
    name = 'tof-transport-bytes-20261122'
    entries = []
    for p in sorted((root / 'runs/tof-front').rglob('*.bin')):
        if p.stat().st_mtime > time.time() - 120:
            result['excluded_or_unstable'].append({'path': str(p), 'reason': 'recent live transport capture; left local'})
            continue
        entries.append((p, p.relative_to(root).as_posix()))
    provenance = [{'path': relative, 'original_source': str(p), 'bytes': p.stat().st_size,
                   'sha256': digest(p)} for p, relative in entries]
    metadata = archives / 'supplement-staging' / (name + '-provenance.json')
    dump(metadata, {'files': provenance})
    members, assets = archive(archives, name, entries + [(metadata, 'provenance.json')])
    dump(output / (name + '-files.json'), {'files': provenance})
    for asset in assets:
        asset['url'] = f"https://github.com/triyankk/BHUslam_v3/releases/download/{result['release_tag']}/{asset['name']}"
    result['archives'].extend(assets)
    dump(output / 'catalog.json', result)
    dump(archives / 'supplement-catalog.json', result)
    sums = ''.join(f"{a['sha256']}  {a['name']}\n" for a in result['archives'])
    sums += digest(archives / 'supplement-catalog.json') + '  supplement-catalog.json\n'
    (output / 'SHA256SUMS').write_text(sums)
    (archives / 'SUPPLEMENT-SHA256SUMS').write_text(sums)
    print('Preserved', len(entries), 'closed ToF binary transport captures', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reviews', type=Path, default=Path('flight-reviews'))
    parser.add_argument('--archives', type=Path, required=True)
    parser.add_argument('--tag', default='flight-data-20261122')
    parser.add_argument('--context-only', action='store_true', help='Extend an existing three-archive supplement')
    parser.add_argument('--transport-only', action='store_true', help='Add closed ToF transport captures')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    reviews = args.reviews.resolve()
    output = reviews / 'supplement'
    if args.transport_only:
        append_transport(reviews, args.archives.resolve())
        return
    if args.context_only:
        extend_context(reviews, args.archives.resolve())
        return
    if output.exists():
        parser.error('Supplement output already exists; use a new reviews directory')
    output.mkdir(parents=True)
    archives = args.archives.resolve()
    staging = archives / 'supplement-staging'
    if staging.exists():
        parser.error('Staging directory already exists')
    staging.mkdir()
    base = load(reviews / 'catalog.json')
    boots = {f['boot_id'] for f in base['flights']}
    already = {r['source'] for p in reviews.glob('*/files.json') for r in load(p)['files']}
    tracked = set(subprocess.check_output(['git', '-C', str(root), 'ls-files', '-z'], text=True).split('\0'))
    cutoff = time.time() - 120
    result = {'created_utc': datetime.now(timezone.utc).isoformat(), 'release_tag': args.tag,
              'sessions': [], 'archives': [], 'excluded_or_unstable': [],
              'selection': 'November Desktop CSVs; diagnostics from the 19 VIO session boots; ignored report evidence. Historical external recordings remain inventoried locally.'}
    signature = re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----|\b(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,}|AKIA[0-9A-Z]{16}|sk-proj-[A-Za-z0-9_-]{30,})\b')

    def make_group(name, sources):
        folder = staging / name
        folder.mkdir()
        entries = []
        provenance = []
        for source, relative in sources:
            if str(source.resolve()) in already:
                continue
            before = source.stat()
            if before.st_mtime > cutoff or source.is_symlink():
                result['excluded_or_unstable'].append({'path': str(source), 'reason': 'recently changing file or symlink'})
                continue
            target = folder / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            after = source.stat()
            if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                target.unlink()
                result['excluded_or_unstable'].append({'path': str(source), 'reason': 'changed during snapshot'})
                continue
            if source.suffix != '.BIN' and signature.search(target.read_bytes()):
                raise RuntimeError('Credential signature requires review: ' + str(source))
            provenance.append({'path': relative, 'original_source': str(source), 'bytes': before.st_size,
                               'sha256': digest(target)})
            entries.append((target, relative))
        dump(folder / 'provenance.json', {'files': provenance})
        entries.append((folder / 'provenance.json', 'provenance.json'))
        print('Packaging supplement', name, len(provenance), 'source files', flush=True)
        members, assets = archive(archives, name, entries)
        dump(output / (name + '-files.json'), {'files': provenance})
        for asset in assets:
            asset['url'] = f"https://github.com/triyankk/BHUslam_v3/releases/download/{args.tag}/{asset['name']}"
        result['archives'].extend(assets)
        return folder

    desktop = Path('/home/atas/Desktop/Drone_Sensor_Recordings')
    selected = sorted(p for p in desktop.glob('*11-2026*') if p.is_dir())
    telemetry = [(p, session.name + '/' + p.name) for session in selected
                 for p in sorted(session.iterdir()) if p.is_file() and (p.suffix == '.csv' or p.name == 'manifest.json')]
    folder = make_group('optical-flow-and-cube-telemetry-20261122', telemetry)
    columns = ['timestamp_local', 'timestamp_utc', 'monotonic_ns', 'message_type', 'source_system',
               'source_component', 'flow_x', 'flow_y', 'flow_comp_m_x', 'flow_comp_m_y', 'flow_rate_x',
               'flow_rate_y', 'quality', 'ground_distance', 'sensor_id', 'time_usec', 'original_fields_json']
    for session in selected:
        snapshot = folder / session.name
        if not (snapshot / 'cube_mavlink.csv').exists():
            continue
        rows, corrupt = readable_csv((snapshot / 'cube_mavlink.csv').read_bytes())
        extracted, counts = flow_rows(rows)
        metadata = load(snapshot / 'manifest.json')
        review = output / 'optical-flow' / session.name
        review.mkdir(parents=True)
        with (review / 'optical-flow.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=columns, lineterminator='\n')
            writer.writeheader()
            writer.writerows(extracted)
        record = {'session': session.name, 'original_folder': str(session),
                  'started_utc_recorded': metadata.get('started_utc'), 'state_recorded': metadata.get('state'),
                  'readable_cube_rows': sum(counts.values()), 'manifest_cube_rows': metadata.get('row_counts', {}).get('cube'),
                  'optical_flow_rows': len(extracted), 'message_counts': dict(counts), 'nul_corruption': corrupt,
                  'vio_association': 'Not automatically assigned; verify boot/monotonic alignment independently.',
                  'limit': 'Decoded recorded MAVLink values, not raw optical images or a new flow accuracy/fusion result. Invalid index tails are not reconstructed.'}
        dump(review / 'summary.json', record)
        shutil.copy2(snapshot / 'manifest.json', review / 'original-manifest.json')
        result['sessions'].append(record)

    runtime = []
    allowed = {'.csv', '.json', '.jsonl', '.log', '.txt', '.yaml', '.param', '.sha256'}
    for name in ['runs/vins-forwarding', 'runs/flight-shadow', 'runs/tof-front']:
        for boot in boots:
            for p in (root / name / boot).rglob('*'):
                if p.is_file() and p.suffix in allowed and not any(s in str(p) for s in ['/flights/', '/checkpoints/']) and p.name not in ['map-data.json', 'viewer-data.json']:
                    runtime.append((p, p.relative_to(root).as_posix()))
    for flight in base['flights']:
        run = Path(flight['original_vio_folder']).parents[2]
        for p in run.glob('live/*'):
            if p.is_file() and p.suffix in allowed:
                runtime.append((p, p.relative_to(root).as_posix()))
    observation = Path('/home/atas/vscode/optFlow_slam/data/recordings/vio-observation')
    # Source timestamps alone do not prove a boot match; preserve November logs
    # together at their original recording level rather than force a flight join.
    for p in observation.glob('202611*.jsonl'):
        runtime.append((p, 'external-vio-observation/' + p.name))
    runtime = list({str(p): (p, rel) for p, rel in runtime}.values())
    make_group('flight-runtime-diagnostics-20261122', sorted(runtime))

    evidence = []
    for p in (root / 'reports').rglob('*'):
        if not p.is_file() or p.suffix not in {'.json', '.jsonl', '.csv', '.log', '.txt', '.tlog', '.BIN'}:
            continue
        relative = p.relative_to(root).as_posix()
        if relative in tracked or any(s in relative for s in ['/input/', 'map-data.json', 'viewer-data.json', 'sparse-map.json', 'failed-full-map.json', 'comparison.json']):
            continue
        evidence.append((p, relative))
    make_group('post-review-report-evidence-20261122', sorted(evidence))
    dump(output / 'catalog.json', result)
    dump(archives / 'supplement-catalog.json', result)
    sums = ''.join(f"{a['sha256']}  {a['name']}\n" for a in result['archives'])
    sums += digest(archives / 'supplement-catalog.json') + '  supplement-catalog.json\n'
    (output / 'SHA256SUMS').write_text(sums)
    (archives / 'SUPPLEMENT-SHA256SUMS').write_text(sums)
    print('Complete:', len(selected), 'telemetry sessions;', sum(s['optical_flow_rows'] for s in result['sessions']),
          'flow rows;', sum(a['bytes'] for a in result['archives']), 'compressed bytes', flush=True)
    extend_context(reviews, archives)
    append_transport(reviews, archives)


if __name__ == '__main__':
    main()
