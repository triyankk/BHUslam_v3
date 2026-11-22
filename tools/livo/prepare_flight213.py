#!/usr/bin/env python3
"""Prepare initial, explicitly uncalibrated flight-213 replay geometry/time."""
import csv
import hashlib
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'reports/livo-replay-readiness-20261120'
RUN = ROOT/'runs/vins-fusion-boot/20261118T141316.161559760Z'
CAM = RUN/'live/flights/flight-518567821/input'
ANCHOR = ROOT/'runs/flight-shadow/85f03d02-ec4c-4f9d-b518-57d9820692ec/mapping-20261118T141315.059744432Z/data/flights/flight-518422613/clouds.csv'


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def ry(degrees):
    angle = np.deg2rad(degrees)
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def main():
    mount = yaml.safe_load((ROOT/'configs/livo-mount-initial.yaml').read_text())
    assert mount['previous_flight_mount_unchanged_confirmed']
    assert mount['jt16_mount']['connector_direction_aircraft'] == 'rear'
    camera_mount = cv2.FileStorage(str(RUN/'navigation-mount.yaml'), cv2.FILE_STORAGE_READ)
    rbc = camera_mount.getNode('Mount.R_body_optical').mat()
    camera_mount.release()
    # Vendor figure 2/3: +Y toward connector, +Z along upright rotation axis.
    # Rear-facing connector: +Y rear, +X left, +Z up, before measured tilt.
    rbl = ry(-mount['jt16_mount']['tilt_forward_from_aircraft_platform_upright_deg']) @ np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
    rcl = rbc.T @ rbl
    tcl = rbc.T @ np.array(mount['jt16_centre_relative_to_left_lens_body_m'])
    np.testing.assert_allclose(rcl.T@rcl, np.eye(3), atol=1e-12)
    assert abs(np.linalg.det(rcl)-1) < 1e-12
    np.testing.assert_allclose(rbc@tcl, mount['jt16_centre_relative_to_left_lens_body_m'], atol=1e-12)
    anchor = rows(ANCHOR)[0]
    lidar_host = float(anchor['host_ns'])*1e-9-float(anchor['sensor_end_s'])
    captures = rows(CAM/'capture.csv')
    camera_host = float(np.median([float(x['host_capture_us'])*1e-6-float(x['device_s']) for x in captures]))
    storage = cv2.FileStorage(str(OUT/'flight213-initial.yaml'), cv2.FILE_STORAGE_WRITE)
    storage.write('recording', str(CAM))
    storage.write('decoded', str(OUT/'flight213-decoded'))
    storage.write('lidar_to_camera_time_offset_s', lidar_host-camera_host)
    storage.write('R_camera_lidar', rcl)
    storage.write('t_camera_lidar_m', tcl)
    storage.release()
    manifest = {
        'status': 'initial_not_calibrated', 'connector': 'rear',
        'basis_source': 'Hesai JT16 manual figure 2/3: +Y toward connector, +Z up',
        'R_body_lidar': rbl.tolist(), 'R_camera_lidar': rcl.tolist(),
        't_camera_lidar_m': tcl.tolist(),
        'lidar_host_offset_single_anchor_s': lidar_host,
        'camera_host_offset_median_s': camera_host,
        'lidar_to_camera_offset_s': lidar_host-camera_host,
        'clock_drift_estimated': False,
        'source_revisions': {},
    }
    for name in ['FAST-LIVO2', 'fast-livo-vikit', 'fast-livo-sophus']:
        manifest['source_revisions'][name] = subprocess.check_output(['git', '-C', str(ROOT/'third_party'/name), 'rev-parse', 'HEAD'], text=True).strip()
    manifest['mount_file_sha256'] = hashlib.sha256((ROOT/'configs/livo-mount-initial.yaml').read_bytes()).hexdigest()
    (OUT/'initial-transform-and-clock.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print('Initial transform and single-anchor time mapping prepared; not a calibration result.')


if __name__ == '__main__':
    main()
