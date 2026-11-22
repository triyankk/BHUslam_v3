import socket
import time
import unittest
from types import SimpleNamespace
from pymavlink.dialects.v20 import ardupilotmega as mav
from usb_owner_relay import TofUsbTransport


def distance(stamp, cm=100, system=1, component=191, orientation=0, sensor_id=0):
    link = mav.MAVLink(None, srcSystem=system, srcComponent=component)
    return link.distance_sensor_encode(stamp & 0xFFFFFFFF, 1, 500, cm,
        mav.MAV_DISTANCE_SENSOR_LASER, sensor_id, orientation, 255).pack(link)


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.feedback = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.feedback.bind(("127.0.0.1", 0))
        self.feedback.settimeout(.1)
        self.relay = TofUsbTransport(0, self.feedback.getsockname()[1])
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.relay.close(); self.feedback.close(); self.sender.close()

    def test_valid_fresh_and_replay_ordering(self):
        now = 1_000_000_000
        self.assertIsNotNone(self.relay.accept(distance(990), now))
        self.assertIsNone(self.relay.accept(distance(990), now))
        self.assertIsNone(self.relay.accept(distance(989), now))
        self.assertIsNone(self.relay.accept(distance(1001), now))
        self.assertIsNone(self.relay.accept(distance(800), now))

    def test_millisecond_wrap_is_valid(self):
        now = (0x100000000 + 10) * 1_000_000
        self.assertIsNotNone(self.relay.accept(distance(0xFFFFFFFF), now))
        self.assertIsNotNone(self.relay.accept(distance(5), now))

    def test_no_other_sensor_command_or_identity_can_pass(self):
        now = 1_000_000_000
        for args in ({"component": 197}, {"system": 2}, {"orientation": 4}, {"sensor_id": 1}, {"cm": 0}, {"cm": 501}):
            self.assertIsNone(self.relay.accept(distance(990, **args), now))
        link = mav.MAVLink(None, srcSystem=1, srcComponent=191)
        arm = link.command_long_encode(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0).pack(link)
        self.assertIsNone(self.relay.accept(arm, now))
        packet = bytearray(distance(990)); packet[-1] ^= 1
        self.assertIsNone(self.relay.accept(bytes(packet), now))
        self.assertIsNone(self.relay.accept(distance(990)*2, now))

    def test_latest_only_and_no_replay_on_next_poll(self):
        writes, commands, parameters = [], [], []
        connection = SimpleNamespace(write=lambda data: writes.append(data),
            mav=SimpleNamespace(command_long_send=lambda *a: commands.append(a),
                param_request_read_send=lambda *a: parameters.append(a)))
        now = time.monotonic_ns(); ms = now//1_000_000
        for delta, cm in [(10, 100), (9, 200), (8, 300)]:
            self.sender.sendto(distance(ms-delta, cm), ("127.0.0.1", self.relay.port))
        self.relay.poll(connection, 1, 1, now)
        self.assertEqual(len(writes), 1)
        self.assertEqual(mav.MAVLink(None).parse_char(writes[0]).current_distance, 300)
        self.assertEqual(self.relay.superseded, 2)
        self.relay.poll(connection, 1, 1, time.monotonic_ns())
        self.assertEqual(len(writes), 1)
        self.assertTrue(all(c[2] == 512 for c in commands))
        self.assertTrue(all(p[2] == b"PRX1_TYPE" for p in parameters))
        self.sender.sendto(distance(ms-500, 400), ("127.0.0.1", self.relay.port))
        self.relay.poll(connection, 1, 1, time.monotonic_ns())
        self.assertEqual(len(writes), 1)

    def test_feedback_only_from_autopilot_front_sector(self):
        link = mav.MAVLink(None)
        msg = link.parse_char(distance(1000, component=1, sensor_id=10))
        self.relay.observe(msg, time.monotonic_ns())
        data, _ = self.feedback.recvfrom(1024)
        self.assertEqual(mav.MAVLink(None).parse_char(data).id, 10)
        wrong = link.parse_char(distance(1001, component=191, sensor_id=10))
        self.relay.observe(wrong, time.monotonic_ns())
        unavailable = link.parse_char(distance(1002, component=1, sensor_id=10, cm=18000))
        self.relay.observe(unavailable, time.monotonic_ns())
        self.assertEqual(self.relay.unavailable_cube_reports, 1)
        self.assertEqual(self.relay.cube_reports, 1)
        with self.assertRaises(socket.timeout):
            self.feedback.recvfrom(1024)


if __name__ == "__main__":
    unittest.main()
