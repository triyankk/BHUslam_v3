"""Narrow ToF adapter for the existing Cube USB owner's thread.

Native C++ retains acquisition, calibration and zone aggregation. This adapter
accepts only fresh front DISTANCE_SENSOR packets; it cannot relay flight commands.
"""
import socket
import time
from pymavlink.dialects.v20 import ardupilotmega as mav


class TofUsbTransport:
    def __init__(self, port=14605, feedback_port=14606, system=1):
        if not 0 <= port <= 65535 or not 1 <= feedback_port <= 65535:
            raise ValueError("Invalid ToF handoff ports")
        self.system = system
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", port))
        self.socket.setblocking(False)
        self.port = self.socket.getsockname()[1]
        self.feedback = ("127.0.0.1", feedback_port)
        self.received = self.rejected = self.forwarded = self.superseded = 0
        self.cube_reports = self.feedback_packets = 0
        self.unavailable_cube_reports = 0
        self.last_capture_ms = None
        self.last_input_ns = self.last_write_ns = self.last_cube_ns = 0
        self.last_distance_cm = None
        self.next_distance_request_ns = self.next_parameter_request_ns = 0
        self.last_error = None

    def close(self):
        self.socket.close()

    def accept(self, data, now_ns):
        self.received += 1
        try:
            if not 12 <= len(data) <= 280 or data[0] != 253 or data[2] != 0 or len(data) != data[1] + 12:
                raise ValueError("Expected one unsigned MAVLink2 packet")
            message = mav.MAVLink(None).parse_char(data)
            if (message is None or message.get_type() != "DISTANCE_SENSOR"
                    or message.get_srcSystem() != self.system or message.get_srcComponent() != 191
                    or message.orientation != 0 or message.id != 0
                    or message.type != mav.MAV_DISTANCE_SENSOR_LASER
                    or message.min_distance != 1 or message.max_distance != 500
                    or not 1 <= message.current_distance <= 500):
                raise ValueError("Not a permitted front ToF measurement")
            now_ms = (now_ns // 1_000_000) & 0xFFFFFFFF
            age = (now_ms - message.time_boot_ms) & 0xFFFFFFFF
            if age > 150:
                raise ValueError("Stale or future ToF measurement")
            if self.last_capture_ms is not None:
                delta = (message.time_boot_ms - self.last_capture_ms) & 0xFFFFFFFF
                if not 0 < delta < 0x80000000:
                    raise ValueError("Repeated or out-of-order ToF measurement")
            self.last_capture_ms = message.time_boot_ms
            self.last_input_ns = now_ns
            return message
        except Exception:
            self.rejected += 1
            return None

    def poll(self, connection, target_system, target_component, now_ns):
        newest = None
        for _ in range(64):
            try:
                data, address = self.socket.recvfrom(1024)
            except BlockingIOError:
                break
            if address[0] != "127.0.0.1":
                self.received += 1
                self.rejected += 1
                continue
            message = self.accept(data, time.monotonic_ns())
            if message is not None:
                if newest is not None:
                    self.superseded += 1
                newest = (data, message)
        now_ns = time.monotonic_ns()
        if newest is not None:
            data, message = newest
            if (((now_ns // 1_000_000) - message.time_boot_ms) & 0xFFFFFFFF) <= 150:
                count = connection.write(data)
                if count is not None and count != len(data):
                    raise OSError("Partial ToF USB write")
                self.forwarded += 1
                self.last_write_ns = time.monotonic_ns()
                self.last_distance_cm = message.current_distance
                self.last_error = None
            else:
                self.rejected += 1
        # Read-only diagnostics, emitted by the sole owner, not accepted via UDP.
        if now_ns >= self.next_distance_request_ns:
            connection.mav.command_long_send(target_system, target_component,
                mav.MAV_CMD_REQUEST_MESSAGE, 0, 132, 0, 0, 0, 0, 0, 0)
            self.next_distance_request_ns = now_ns + 200_000_000
        if now_ns >= self.next_parameter_request_ns:
            connection.mav.param_request_read_send(target_system, target_component, b"PRX1_TYPE", -1)
            self.next_parameter_request_ns = now_ns + 2_000_000_000

    def observe(self, message, now_ns):
        if message.get_srcSystem() != self.system or message.get_srcComponent() != 1:
            return
        kind = message.get_type()
        permitted = kind in {"HEARTBEAT", "STATUSTEXT", "SYS_STATUS"}
        if kind == "PARAM_VALUE":
            permitted = message.param_id == "PRX1_TYPE"
        if kind == "DISTANCE_SENSOR":
            permitted = message.id == 10 and message.orientation == 0
            if permitted and not (1 <= message.current_distance <= 500
                    and message.min_distance <= message.current_distance <= message.max_distance
                    and message.signal_quality != 1):
                # Another (e.g. rear) backend may keep global PRX healthy while
                # ArduPilot emits an unavailable-front sentinel. Never call it fresh.
                self.unavailable_cube_reports += 1
                permitted = False
            if permitted:
                self.cube_reports += 1
                self.last_cube_ns = now_ns
        if permitted:
            try:
                self.socket.sendto(message.get_msgbuf(), self.feedback)
                self.feedback_packets += 1
            except OSError as exc:
                self.last_error = str(exc)

    def disconnect(self):
        self.last_write_ns = self.last_cube_ns = 0

    def snapshot(self, now_ns):
        def age(stamp):
            return None if not stamp else max(0, (now_ns-stamp)/1e6)
        return {"enabled": True, "transport": "native ToF -> localhost -> sole Cube USB owner",
            "port": self.port, "feedback_port": self.feedback[1], "received": self.received,
            "rejected": self.rejected, "forwarded": self.forwarded, "superseded": self.superseded,
            "last_input_age_ms": age(self.last_input_ns), "last_serial_write_age_ms": age(self.last_write_ns),
            "last_distance_cm": self.last_distance_cm, "Cube_front_reports": self.cube_reports,
            "unavailable_Cube_front_reports": self.unavailable_cube_reports,
            "Cube_front_report_age_ms": age(self.last_cube_ns), "feedback_packets": self.feedback_packets,
            "max_input_age_ms": 150, "last_error": self.last_error}
