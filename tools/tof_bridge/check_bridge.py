#!/usr/bin/env python3
"""Check real captures, or exercise encoding over a software serial terminal."""
import argparse
import io
import json
import os
from pathlib import Path
import pty
import time

import bridge
from tmf882x.measurement import TMF882xMeasurement, TMF882xSpadResult
from pymavlink.dialects.v20 import common as mav


def self_test():
    import serial
    zones = [TMF882xSpadResult(0, 0, 0, 0) for _ in range(18)]
    zones[0] = TMF882xSpadResult(19, 100, 100, 743)
    zones[1] = TMF882xSpadResult(100, 1299, 0, 0)
    m = TMF882xMeasurement(1, 20, 2, 0, 0, 0, 123, zones, 1)
    assert bridge.nearest_return(m) == (743, 100)
    assert bridge.nearest_return(m, 101) is None
    master, slave = pty.openpty()
    try:
        with serial.Serial(os.ttyname(slave), 115200, timeout=0.2) as uart:
            capture = io.BytesIO()
            output = bridge.Output(capture, uart)
            link = mav.MAVLink(output, srcSystem=1, srcComponent=191)
            for orientation in bridge.ORIENTATIONS:
                bridge.send_distance(link, 743, orientation, 1000)
            expected = capture.getvalue()
            received = bytearray()
            deadline = time.monotonic() + 2
            while len(received) < len(expected) and time.monotonic() < deadline:
                import select
                if select.select([master], [], [], 0.1)[0]:
                    received.extend(os.read(master, 4096))
            assert bytes(received) == expected
            # Decode fragmented bytes, including MAVLink's checksum validation.
            decoder = mav.MAVLink(None)
            messages = []
            for b in received:
                msg = decoder.parse_char(bytes([b]))
                if msg:
                    messages.append(msg)
            assert len(messages) == 5
            assert [m.orientation for m in messages] == list(bridge.ORIENTATIONS.values())
            for msg in messages:
                assert msg.get_type() == "DISTANCE_SENSOR"
                assert msg.current_distance == 74
                assert msg.min_distance == 1 and msg.max_distance == 500
                assert msg.get_srcSystem() == 1 and msg.get_srcComponent() == 191
                assert msg.covariance == 255
        print("PASS: confidence filtering, secondary returns, mm->cm conversion, five orientations,")
        print("      MAVLink 2 CRC decoding, and serial transport over a software PTY.")
        print("This does not test the physical Jetson UART pins.")
    finally:
        os.close(master)
        os.close(slave)


def check_capture(directory):
    summary = json.loads((directory / "summary.json").read_text())
    rows = [json.loads(line) for line in (directory / "measurements.jsonl").read_text().splitlines()]
    decoder = mav.MAVLink(None)
    messages = decoder.parse_buffer((directory / "mavlink.bin").read_bytes()) or []
    distances = [m for m in messages if m.get_type() == "DISTANCE_SENSOR"]
    transmitted = [r for r in rows if r["transmitted"]]
    assert len(rows) == summary["frames"]
    assert len(distances) == len(transmitted) == summary["sent_distances"]
    assert len(messages) == summary["mavlink_packets"]
    if summary.get('aggregation') == 'zone-mean':
        threshold = summary.get('confidence_threshold', 20)
        minimum = summary['minimum_active_zones']
        for row in rows:
            active = [d for distances, qualities in zip(row['distance_mm'], row['confidence'])
                      for d, c in zip(distances, qualities) if 10 <= d <= 5000 and c >= threshold]
            assert row['active_zone_count'] == len(active)
            expected = sum(active) // len(active) if len(active) >= minimum else None
            assert row['selected_mm'] == expected
            if expected is None:
                assert not row['valid'] and not row['transmitted']
    for row, msg in zip(transmitted, distances):
        assert row["calibration_status"] == 0
        assert msg.current_distance == row.get("selected_mm", row["nearest_mm"]) // 10
        assert msg.orientation == bridge.ORIENTATIONS[summary["orientation"]]
        assert msg.time_boot_ms == int(row["monotonic_s"] * 1000) & 0xFFFFFFFF
    heartbeats = sum(m.get_type() == 'HEARTBEAT' for m in messages)
    print(json.dumps({"frames": len(rows), "distance_packets": len(distances),
                      "heartbeat_packets": heartbeats,
                      "other_packets": len(messages)-len(distances)-heartbeats,
                      "crc_and_log_match": True, "run_error": summary["error"]}, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("directory", nargs="?", type=Path)
    args = p.parse_args()
    if args.directory:
        check_capture(args.directory)
    else:
        self_test()
