#!/usr/bin/env python3
"""Check flow extraction preserves original payloads and stops at corruption."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from package_review_supplement import flow_rows, readable_csv


class ReviewSupplementTest(unittest.TestCase):
    def test_corrupt_tail_is_reported_and_not_invented(self):
        original = b'message_type,fields\nHEARTBEAT,{}\n\0OPTICAL_FLOW,{}\n'
        reader, corrupt = readable_csv(original)
        extracted, counts = flow_rows(reader)
        self.assertTrue(corrupt)
        self.assertEqual(counts, {'HEARTBEAT': 1})
        self.assertEqual(extracted, [])

    def test_zero_values_and_unflattened_fields_survive(self):
        fields = json.dumps({'flow_x': 0, 'quality': 0, 'ground_distance': 0.25, 'extra': 42})
        rows, counts = flow_rows([{'message_type': 'OPTICAL_FLOW', 'fields': fields},
                                  {'message_type': 'HEARTBEAT', 'fields': '{}'},
                                  {'message_type': 'OPTICAL_FLOW_RAD', 'fields': '{"integrated_x":0.01}'}])
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]['flow_x'], 0)
        self.assertEqual(rows[0]['quality'], 0)
        self.assertEqual(rows[0]['ground_distance'], 0.25)
        self.assertEqual(json.loads(rows[0]['original_fields_json'])['extra'], 42)
        self.assertEqual(json.loads(rows[1]['original_fields_json'])['integrated_x'], 0.01)
        self.assertEqual(counts['HEARTBEAT'], 1)


if __name__ == '__main__':
    unittest.main()
