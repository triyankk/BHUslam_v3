#!/usr/bin/env python3
"""Add a file-level input inventory without rewriting immutable replay archives."""
import argparse
import csv
import io
from pathlib import Path

from package_flight_reviews import dump, load, valid_pgm


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reviews', type=Path)
    args = parser.parse_args()
    audits = []
    for manifest in sorted(args.reviews.glob('*/manifest.json')):
        record = load(manifest)
        folder = Path(record['original_vio_folder']) / 'input'
        metadata = load(folder / 'recording.json')
        raw_index = (folder / 'frames.csv').read_bytes()
        nul = raw_index.find(b'\0')
        prefix = raw_index[:raw_index.rfind(b'\n', 0, nul) + 1] if nul >= 0 else raw_index
        referenced = {'left': set(), 'right': set()}
        for row in csv.DictReader(io.StringIO(prefix.decode('utf-8', errors='replace'))):
            for side in referenced:
                if row.get(side):
                    referenced[side].add(row[side])
        audit = {'id': record['id'], 'metadata_frames_written': metadata.get('frames_written'),
                 'inspected_index_pairs': record['input_integrity']['indexed_pairs'],
                 'index_has_nul_corruption': nul >= 0,
                 'limit': 'File structure only. Unindexed files lack a trustworthy association from frames.csv; no reconstructed timing is supplied.',
                 'sides': {}}
        for side in referenced:
            files = sorted((folder / side).glob('*.pgm'))
            unindexed = [p.relative_to(folder).as_posix() for p in files
                         if p.relative_to(folder).as_posix() not in referenced[side]]
            invalid = [p.relative_to(folder).as_posix() for p in files if not valid_pgm(p)]
            audit['sides'][side] = {'files_on_disk': len(files), 'invalid_pgm_files': invalid,
                                   'unindexed_pgm_files': unindexed}
        dump(manifest.parent / 'input-inventory.json', audit)
        audits.append(audit)
    dump(args.reviews / 'input-inventory.json', {'sessions': audits})
    print('Inventoried', len(audits), 'sessions; original files and replay archives unchanged.')


if __name__ == '__main__':
    main()
