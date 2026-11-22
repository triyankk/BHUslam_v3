#!/usr/bin/env python3
"""Local live ToF/Cube viewer. Standard-library HTTP server; no web packages.

Launches the existing calibrated bridge, tails its measurement log, and observes
Cube-generated rangefinder reports returned over UART. Does not write Cube parameters.
"""
import argparse
from collections import deque
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

HERE = Path(__file__).resolve().parent


class State:
    def __init__(self, args, output):
        self.args, self.output = args, output
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.start = time.monotonic()
        self.sensor = deque(maxlen=1800)
        self.cube = deque(maxlen=1200)
        self.latest = None
        self.last_heartbeat = None
        self.sensor_count = self.cube_count = self.bad_usb = 0
        self.baud = None
        self.bridge_status = 'Starting'
        self.bridge_error = None
        self.cube_error = None
        self.cube_parameters = {}
        self.status_messages = deque(maxlen=30)
        self.process = None

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            sensor_age = None if self.latest is None else now-self.latest['monotonic_s']
            cube_age = None if not self.cube else now-self.cube[-1]['monotonic_s']
            heartbeat_age = None if self.last_heartbeat is None else now-self.last_heartbeat
            recent = [r for r in self.sensor if r['t'] > now-self.start-5]
            hz = (len(recent)-1)/(recent[-1]['t']-recent[0]['t']) if len(recent)>1 else 0
            return {'elapsed_s': now-self.start, 'baud': self.baud,
                    'orientation': self.args.orientation, 'uart': self.args.uart,
                    'usb': self.args.usb, 'bridge_status': self.bridge_status,
                    'bridge_error': self.bridge_error, 'cube_error': self.cube_error,
                    'sensor_age_s': sensor_age, 'cube_age_s': cube_age,
                    'heartbeat_age_s': heartbeat_age, 'hz': hz,
                    'cube_parameters': dict(self.cube_parameters),
                    'rangefinder_slot': self.args.rangefinder_slot,
                    'telemetry_messages': list(self.status_messages),
                    'sensor_count': self.sensor_count, 'cube_count': self.cube_count,
                    'bad_usb_packets': self.bad_usb,
                    'latest': self.latest,
                    'sensor': [r for r in self.sensor if r['t'] > now-self.start-65],
                    'cube': [r for r in self.cube if r['t'] > now-self.start-65],
                    'output': str(self.output)}


def detect_baud(args):
    import serial
    from pymavlink.dialects.v20 import ardupilotmega as mav
    if args.baud:
        return args.baud
    # Read-only requests prove the actual running rate, even if the Cube stores
    # a different rate pending restart. Do this before starting the bridge.
    for baud in (57600, 115200, 921600):
        with serial.Serial(args.uart, baud, timeout=.05, write_timeout=1,
                           exclusive=True, rtscts=False) as port:
            link = mav.MAVLink(port, srcSystem=254, srcComponent=190)
            link.robust_parsing = True
            end = time.monotonic()+1.8
            next_request = 0
            heartbeats = 0
            while time.monotonic() < end:
                now = time.monotonic()
                if now >= next_request:
                    link.param_request_read_send(args.system_id, 1, b'MAV_SYSID', -1)
                    next_request = now+.4
                for msg in link.parse_buffer(port.read(4096)) or []:
                    if (msg.get_type() == 'HEARTBEAT' and msg.get_srcSystem() == args.system_id
                            and msg.get_srcComponent() == 1 and msg.autopilot != mav.MAV_AUTOPILOT_INVALID):
                        heartbeats += 1
                        if heartbeats >= 2:
                            return baud
                    if (msg.get_type() == 'PARAM_VALUE' and msg.param_id == 'MAV_SYSID'
                            and msg.get_srcSystem() == args.system_id and msg.get_srcComponent() == 1):
                        return baud
    raise RuntimeError('No Cube parameter reply at 57600, 115200, or 921600 baud. Check power and TX/RX wiring.')


def tail_measurements(state, partial):
    with state.lock:
        for line in partial.splitlines():
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            state.latest = row
            state.sensor_count += 1
            state.sensor.append({'t': row['monotonic_s']-state.start,
                                 'm': (row['nearest_mm']//10)/100 if row['transmitted'] else None})


def run_bridge_session(state, session_index):
    args = state.args
    try:
        while not state.stop.is_set():
            try:
                baud = detect_baud(args)
                break
            except Exception as exc:
                with state.lock:
                    state.bridge_status, state.bridge_error = 'Waiting for link', str(exc)
                state.stop.wait(2)
        if state.stop.is_set():
            return
        with state.lock:
            state.baud = baud
            state.bridge_error = None
        if state.stop.is_set():
            return
        session_name = 'bridge' if session_index == 1 else f'bridge-{session_index:03d}'
        command = [sys.executable, str(HERE/'bridge.py'), '--bus', str(args.bus),
                   '--uart', args.uart, '--baud', str(baud), '--orientation', args.orientation,
                   '--system-id', str(args.system_id), '--calibration', str(args.calibration),
                   '--duration', '0', '--request-rangefinder', '--rangefinder-slot', str(args.rangefinder_slot), '--output', str(state.output/session_name)]
        with (state.output/f'{session_name}-console.txt').open('x') as console:
            process = subprocess.Popen(command, stdout=console, stderr=subprocess.STDOUT)
            state.process = process
            with state.lock:
                state.bridge_status = 'Running'
            path = state.output/session_name/'measurements.jsonl'
            position, pending = 0, ''
            peer_path = state.output/session_name/'cube-reports.jsonl'
            peer_position, peer_pending = 0, ''
            while True:
                if state.stop.is_set() and process.poll() is None:
                    process.terminate()
                if path.exists():
                    with path.open() as log:
                        log.seek(position)
                        pending += log.read()
                        position = log.tell()
                    if '\n' in pending:
                        complete, pending = pending.rsplit('\n', 1)
                        tail_measurements(state, complete)
                if peer_path.exists() and not args.usb:
                    with peer_path.open() as log:
                        log.seek(peer_position)
                        peer_pending += log.read()
                        peer_position = log.tell()
                    if '\n' in peer_pending:
                        complete, peer_pending = peer_pending.rsplit('\n', 1)
                        for line in complete.splitlines():
                            event = json.loads(line)
                            with state.lock:
                                if event['type'] == 'heartbeat':
                                    state.last_heartbeat = event['monotonic_s']
                                elif event['type'] == 'distance':
                                    event['t'] = event['monotonic_s']-state.start
                                    state.cube.append(event)
                                    state.cube_count += 1
                                elif event['type'] == 'status_text':
                                    state.status_messages.append(event)
                                elif event['type'] == 'parameter':
                                    state.cube_parameters[event['name']] = event['value']
                if process.poll() is not None:
                    summary_path = state.output/session_name/'summary.json'
                    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
                    with state.lock:
                        state.bridge_status = 'Stopped' if process.returncode == 0 else 'Error'
                        state.bridge_error = summary.get('error') or (f'Bridge exited ({process.returncode})' if process.returncode else None)
                    break
                time.sleep(.05)
    except Exception as exc:
        with state.lock:
            state.bridge_status, state.bridge_error = 'Error', str(exc)
    finally:
        if state.process is not None and state.process.poll() is None:
            state.process.terminate()
            try:
                state.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                state.process.kill()
                state.process.wait()


def run_bridge(state):
    session_index, delay = 1, 2
    while not state.stop.is_set():
        started = time.monotonic()
        run_bridge_session(state, session_index)
        if state.stop.is_set():
            break
        with state.lock:
            state.bridge_status = 'Reconnecting sensor'
        if time.monotonic()-started > 10:
            delay = 2
        state.stop.wait(delay)
        delay = min(30, delay*2)
        session_index += 1


def observe_cube(state):
    import serial
    from pymavlink.dialects.v20 import ardupilotmega as mav
    from bridge import ORIENTATIONS
    while not state.stop.is_set():
        try:
            with serial.Serial(state.args.usb, 115200, timeout=.05, write_timeout=1,
                               exclusive=True) as port, (state.output/'cube.jsonl').open('a') as log:
                decoder = mav.MAVLink(port, srcSystem=255, srcComponent=190)
                decoder.robust_parsing = True
                next_request = 0
                with state.lock:
                    state.cube_error = None
                while not state.stop.is_set():
                    now = time.monotonic()
                    if now >= next_request:
                        decoder.command_long_send(state.args.system_id, 1, mav.MAV_CMD_REQUEST_MESSAGE,
                                                  0, 132, 0, 0, 0, 0, 0, 0)
                        next_request = now+.2
                    for msg in decoder.parse_buffer(port.read(4096)) or []:
                        received = time.monotonic()
                        if msg.get_type() == 'BAD_DATA':
                            with state.lock:
                                state.bad_usb += 1
                            continue
                        if msg.get_srcSystem() != state.args.system_id or msg.get_srcComponent() != 1:
                            continue  # Do not mistake forwarded Jetson packets for Cube processing.
                        if msg.get_type() == 'HEARTBEAT':
                            with state.lock:
                                state.last_heartbeat = received
                        elif (msg.get_type() == 'DISTANCE_SENSOR'
                              and msg.orientation == ORIENTATIONS[state.args.orientation]
                              and msg.id == state.args.rangefinder_slot-1):
                            row = {'t': received-state.start, 'monotonic_s': received,
                                   'm': msg.current_distance/100, 'sensor_id': msg.id,
                                   'cube_boot_ms': msg.time_boot_ms}
                            log.write(json.dumps(row)+'\n')
                            log.flush()
                            with state.lock:
                                state.cube.append(row)
                                state.cube_count += 1
        except Exception as exc:
            with state.lock:
                state.cube_error = str(exc)
            state.stop.wait(2)


def serve(state, port):
    page = (HERE/'viewer.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/':
                data, mime = page, 'text/html; charset=utf-8'
            elif path == '/api/state':
                data, mime = json.dumps(state.snapshot(), allow_nan=False).encode(), 'application/json'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.timeout = .2
    threads = [threading.Thread(target=run_bridge, args=(state,), daemon=True)]
    if state.args.usb:
        threads.append(threading.Thread(target=observe_cube, args=(state,), daemon=True))
    for thread in threads:
        thread.start()
    print(f'Live viewer: http://127.0.0.1:{port}', flush=True)
    print(f'Recordings: {state.output}', flush=True)
    print('Ctrl-C stops the viewer and its UART bridge.', flush=True)
    try:
        while not state.stop.is_set():
            server.handle_request()
    finally:
        state.stop.set()
        server.server_close()
        for thread in threads:
            thread.join(timeout=8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--bus', type=int, default=7)
    parser.add_argument('--uart', default='/dev/ttyTHS1')
    parser.add_argument('--usb', help='Optional separate USB observer port; default uses UART returns only')
    parser.add_argument('--baud', type=int, help='Omit to detect the running Cube UART baud')
    parser.add_argument('--rangefinder-slot', type=int, default=2)
    parser.add_argument('--system-id', type=int, default=1)
    parser.add_argument('--orientation', choices=['front','rear','left','right','up'], default='rear')
    parser.add_argument('--calibration', type=Path, default=HERE/'calibration-23937420-map1.json')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--duration', type=float, default=0, help='Optional automatic stop, seconds')
    args = parser.parse_args()
    output = args.output or HERE.parents[1]/'reports/tof'/('live-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    state = State(args, output)
    signal.signal(signal.SIGINT, lambda *_: state.stop.set())
    signal.signal(signal.SIGTERM, lambda *_: state.stop.set())
    if args.duration:
        timer = threading.Timer(args.duration, state.stop.set)
        timer.daemon = True
        timer.start()
    serve(state, args.port)


if __name__ == '__main__':
    main()
