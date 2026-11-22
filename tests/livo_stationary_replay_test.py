#!/usr/bin/env python3
"""Known stationary scene checks adapter timing, frames and empty LiDAR slices."""
import csv
import json
import subprocess
import tempfile
from pathlib import Path
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('timestamp', '<f8'), ('ring', '<u2'), ('intensity', 'u1'), ('confidence', 'u1')])


def write_csv(path, header, rows):
    with path.open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def main():
    with tempfile.TemporaryDirectory(prefix='bhu-livo-stationary-') as temporary:
        directory = Path(temporary)
        camera, decoded = directory/'camera', directory/'decoded'
        camera.mkdir()
        (decoded/'scans').mkdir(parents=True)
        rng = np.random.default_rng(213)
        image = rng.integers(0, 255, (400, 640), dtype=np.uint8)
        image = cv2.GaussianBlur(image, (3, 3), .7)
        cv2.imwrite(str(camera/'image.pgm'), image)
        storage = cv2.FileStorage(str(camera/'orb.yaml'), cv2.FILE_STORAGE_WRITE)
        for key, value in [('Camera.width', 640), ('Camera.height', 400), ('Camera1.fx', 308.16494751), ('Camera1.fy', 308.16494751), ('Camera1.cx', 317.25), ('Camera1.cy', 201.)]:
            storage.write(key.replace('.', '_'), value)
        storage.write('IMU_T_b_c1', np.eye(4))
        storage.release()
        calibration = (camera/'orb.yaml').read_text()
        for key in ['Camera.width', 'Camera.height', 'Camera1.fx', 'Camera1.fy', 'Camera1.cx', 'Camera1.cy', 'IMU.T_b_c1']:
            calibration = calibration.replace(key.replace('.', '_')+':', key+':')
        (camera/'orb.yaml').write_text(calibration)
        write_csv(camera/'frames.csv', ['timestamp_s', 'left', 'right'], [(t, 'image.pgm', 'image.pgm') for t in np.arange(10.1, 12., .05)])
        write_csv(camera/'accel.csv', ['timestamp_s', 'x', 'y', 'z'], [(t, 0, -9.81, 0) for t in np.arange(9.5, 12.1, .005)])
        write_csv(camera/'gyro.csv', ['timestamp_s', 'x', 'y', 'z'], [(t, 0, 0, 0) for t in np.arange(9.5, 12.1, .005)])
        x, y = np.meshgrid(np.linspace(-3, 3, 80), np.linspace(-2, 2, 50))
        xyz = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, 5.)])
        records = []
        for scan in range(10):
            begin = 10.+scan*.2
            points = np.zeros(len(xyz), dtype=DTYPE)
            for j, key in enumerate(['x', 'y', 'z']):
                points[key] = xyz[:, j]
            points['timestamp'] = begin+np.arange(len(xyz))*.2/len(xyz)
            points['ring'] = np.arange(len(xyz)) % 16
            points['intensity'] = 100
            # A measured no-return interval must not suppress camera updates.
            missing = (points['timestamp'] >= 10.80) & (points['timestamp'] < 10.95)
            for key in ['x', 'y', 'z']:
                points[key][missing] = 0
            name = f'scans/{scan}.bin'
            points.tofile(decoded/name)
            records.append([scan, begin, float(points['timestamp'][-1]), 250, len(points), int((~missing).sum()), name])
        write_csv(decoded/'scans.csv', ['scan', 'sensor_begin_s', 'sensor_end_s', 'packets', 'points', 'nonzero_points', 'file'], records)
        storage = cv2.FileStorage(str(directory/'config.yaml'), cv2.FILE_STORAGE_WRITE)
        storage.write('recording', str(camera))
        storage.write('decoded', str(decoded))
        storage.write('lidar_to_camera_time_offset_s', 0.)
        storage.write('R_camera_lidar', np.eye(3))
        storage.write('t_camera_lidar_m', np.zeros((3, 1)))
        storage.release()
        run = subprocess.run([str(ROOT/'bin/livo-replay'), str(directory/'config.yaml'), str(directory/'result')], capture_output=True, text=True)
        if run.returncode:
            raise RuntimeError(run.stderr[-3000:])
        with (directory/'result/trajectory.csv').open() as stream:
            poses = list(csv.DictReader(stream))
        with (directory/'result/diagnostics.csv').open() as stream:
            diagnostics = list(csv.DictReader(stream))
        position = np.array([[float(v[k]) for k in ['x', 'y', 'z']] for v in poses])
        maximum = float(np.linalg.norm(position, axis=1).max())
        pose_times = {round(float(v['timestamp_s']), 6) for v in poses}
        camera_on_empty = sum(float(v['points']) == 0 and round(float(v['timestamp_s']), 6) in pose_times for v in diagnostics)
        assert len(poses) > 20 and maximum < .001, (len(poses), maximum)
        assert max(float(v['lidar_constraints']) for v in diagnostics) > 100
        assert camera_on_empty > 0, 'Camera pose outputs stopped during empty LiDAR slices'
        result = {'passed': True, 'stationary_poses': len(poses), 'max_position_error_m': maximum,
                  'camera_outputs_with_empty_lidar_slices': camera_on_empty,
                  'scope': 'Synthetic adapter regression; not validation of real calibration or localization accuracy'}
        (ROOT/'reports/livo-replay-readiness-20261120/stationary-adapter-test.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
