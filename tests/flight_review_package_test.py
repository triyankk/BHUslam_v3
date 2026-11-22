#!/usr/bin/env python3
"""Offline checks for corrupted-input handling and byte-exact release packaging."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    'package_flight_reviews', Path(__file__).resolve().parents[1] / 'tools/package_flight_reviews.py')
package = importlib.util.module_from_spec(spec)
spec.loader.exec_module(package)


class FlightReviewPackageTest(unittest.TestCase):
    def test_damaged_and_missing_frames_are_reported_without_changing_originals(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'left').mkdir()
            (root / 'right').mkdir()
            good = b'P5\n2 2\n255\n\x00\x01\x02\x03'
            (root / 'left/a.pgm').write_bytes(good)
            (root / 'right/a.pgm').write_bytes(good)
            (root / 'left/b.pgm').write_bytes(good[:-1])
            index = (b'timestamp_s,left,right\n1,left/a.pgm,right/a.pgm\n'
                     b'2,left/b.pgm,right/missing.pgm\n' + b'\0' * 100)
            (root / 'frames.csv').write_bytes(index)
            result = package.frame_check(root)
            self.assertEqual(result['indexed_pairs'], 2)
            self.assertEqual(result['invalid_pairs'], 1)
            self.assertEqual(result['valid_prefix_pairs'], 1)
            self.assertEqual(result['invalid_row_indices_zero_based'], [1])
            self.assertIn('NUL corruption', result['error'])
            self.assertEqual((root / 'frames.csv').read_bytes(), index)

    def test_archive_roundtrip_and_member_hashes_include_binary_corruption(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'input.raw'
            original = b'unchanged binary data\0\xff\n' * 100
            source.write_bytes(original)
            output = root / 'out'
            output.mkdir()
            members, assets = package.archive(output, 'test-session', [(source, 'jt16/raw/input.raw')])
            asset = output / assets[0]['name']
            self.assertEqual(assets[0]['sha256'], hashlib.sha256(asset.read_bytes()).hexdigest())
            stream = subprocess.check_output(['zstd', '-dc', str(asset)])
            with tarfile.open(fileobj=io.BytesIO(stream)) as tar:
                self.assertEqual(tar.extractfile('test-session/jt16/raw/input.raw').read(), original)
                manifest = json.load(tar.extractfile('test-session/files.json'))
            self.assertEqual(manifest['files'], members)
            self.assertEqual(members[0]['sha256'], hashlib.sha256(original).hexdigest())
            self.assertEqual(source.read_bytes(), original)
            with self.assertRaises(RuntimeError):
                package.archive(output, 'test-session', [(source, 'jt16/raw/input.raw')])


if __name__ == '__main__':
    unittest.main()
