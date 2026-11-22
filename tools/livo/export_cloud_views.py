#!/usr/bin/env python3
"""Visualize recovered scans and failed-replay pose reconstructions honestly."""
import csv
import json
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR', '/tmp/bhu-livo-matplotlib')
import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
REPORT = ROOT/'reports/livo-replay-readiness-20261120'
OUTPUT = REPORT/'point-cloud-views'
DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('time', '<f8'), ('ring', '<u2'), ('intensity', 'u1'), ('confidence', 'u1')])


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def voxelize(points, size=.2):
    keys = np.floor(points/size).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    count = np.bincount(inverse)
    result = np.zeros((len(count), 3))
    np.add.at(result, inverse, points)
    return result/count[:, None]


def export(name, title, points, trajectory, summary):
    cloud = voxelize(points)
    assert np.isfinite(cloud).all() and len(cloud) > 0
    summary = dict(summary, map_voxels=len(cloud), voxel_m=.2)
    data = {'points': np.round(cloud, 5).tolist(), 'trajectory': trajectory, 'summary': summary}
    (OUTPUT/(name+'.json')).write_text(json.dumps(data, separators=(',', ':'))+'\n')
    with (OUTPUT/(name+'.ply')).open('w') as stream:
        stream.write(f'ply\nformat ascii 1.0\nelement vertex {len(cloud)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n')
        np.savetxt(stream, cloud, fmt='%.5f')
    template = (ROOT/'web/jt16-map-viewer.html').read_text()
    template = template.replace('<title>JT16 local map</title>', '<title>'+title+'</title>')
    template = template.replace('JT16-only local map', title)
    navigation = '<p><a style="color:#9edcff" href="short-map.html">First 5 seconds</a> · <a style="color:#9edcff" href="raw-scan.html">Single raw scan</a> · <a style="color:#9edcff" href="failed-full-map.html">Full failed reconstruction</a></p>'
    template = template.replace('<p id="stats"></p>', '<p id="stats"></p>'+navigation+'<p style="color:#ffd48a">'+summary['warning']+'</p>')
    template = template.replace('FAST-LIO2 + JT16 built-in IMU. Local coordinates in metres; calibration is provisional. This is a saved local map without loop closure.',
        'JT16 measurements. Reconstruction uses interpolated exploratory FAST-LIVO2 poses; it is not the upstream registered map. Metres; approximate up orientation; no North alignment or loop closure.')
    template = template.replace('native-frame height', 'display Z')
    if not trajectory:
        template = template.replace('id="path" checked', 'id="path"')
    template = template.replace('/*JT16_DATA*/null', json.dumps(data, separators=(',', ':')).replace('<', '\\u003c'))
    (OUTPUT/(name+'.html')).write_text(template)
    return cloud


def main():
    OUTPUT.mkdir(exist_ok=True)
    config = cv2.FileStorage(str(REPORT/'flight213-initial.yaml'), cv2.FILE_STORAGE_READ)
    rotation = config.getNode('R_camera_lidar').mat()
    translation = config.getNode('t_camera_lidar_m').mat().reshape(3)
    offset = config.getNode('lidar_to_camera_time_offset_s').real()
    config.release()
    poses = rows(REPORT/'flight213-full-01/trajectory.csv')
    times = np.array([float(v['timestamp_s']) for v in poses])
    position = np.array([[float(v[k]) for k in ['x', 'y', 'z']] for v in poses])
    orientations = Rotation.from_quat([[float(v[k]) for k in ['qx', 'qy', 'qz', 'qw']] for v in poses])
    interpolation = Slerp(times-times[0], orientations)
    # Display-only orientation: aircraft forward/left/up at initialization.
    mount = cv2.FileStorage(str(ROOT/'runs/vins-fusion-boot/20261118T141316.161559760Z/navigation-mount.yaml'), cv2.FILE_STORAGE_READ)
    display = np.diag([1., -1., -1.]) @ mount.getNode('Mount.R_body_optical').mat()
    mount.release()
    collected = []
    scans = rows(REPORT/'flight213-decoded/scans.csv')
    for scan in scans:
        points = np.fromfile(REPORT/'flight213-decoded'/scan['file'], dtype=DTYPE)
        xyz = np.column_stack([points[k] for k in ['x', 'y', 'z']])
        stamp = points['time']+offset
        valid = (np.linalg.norm(xyz, axis=1) >= .3) & (np.linalg.norm(xyz, axis=1) <= 60) & (stamp >= times[0]) & (stamp <= times[-1])
        if not valid.any():
            continue
        xyz, stamp = xyz[valid], stamp[valid]
        camera = xyz@rotation.T+translation
        world = interpolation(stamp-times[0]).apply(camera)
        world += np.column_stack([np.interp(stamp, times, position[:, i]) for i in range(3)])
        collected.append(np.column_stack([stamp, world@display.T]))
    all_points = np.concatenate(collected)
    trajectory_xyz = position@display.T
    trajectory = np.column_stack([times, trajectory_xyz]).tolist()
    short = all_points[:, 0] <= times[0]+5
    short_trajectory = [p for p in trajectory if p[0] <= times[0]+5]
    first = export('short-map', 'Flight 213 · first 5 seconds · unvalidated reconstruction', all_points[short, 1:], short_trajectory,
        {'clouds_published': int(sum(times[0] <= float(s['sensor_end_s'])+offset <= times[0]+5 for s in scans)),
         'source': 'Interpolated failed-trial camera poses + initial LiDAR/camera transform',
         'warning': 'UNVALIDATED: first five seconds only. This replay later diverged; this is a diagnostic reconstruction, not a verified map.'})
    export('failed-full-map', 'Flight 213 · full failed reconstruction', all_points[:, 1:], trajectory,
        {'clouds_published': int(sum(times[0] <= float(s['sensor_end_s'])+offset <= times[-1] for s in scans)),
         'source': 'Interpolated failed-trial camera poses + initial LiDAR/camera transform',
         'warning': 'FAILED REPLAY: runaway poses stretch this cloud across hundreds of metres. Do not interpret this as actual terrain or aircraft travel.'})
    raw = np.fromfile(REPORT/'flight213-decoded'/scans[150]['file'], dtype=DTYPE)
    xyz = np.column_stack([raw[k] for k in ['x', 'y', 'z']])
    xyz = xyz[(np.linalg.norm(xyz, axis=1) >= .3) & (np.linalg.norm(xyz, axis=1) <= 60)]
    export('raw-scan', 'Flight 213 · one recovered JT16 scan', (xyz@rotation.T+translation)@display.T, [],
        {'clouds_published': 1, 'source': 'Decoded scan 150, no replay poses applied',
         'warning': 'SINGLE SCAN: actual range returns, shown using initial mounting orientation. This 200 ms scan is not motion-compensated.'})
    fig = plt.figure(figsize=(10, 7), facecolor='#101722')
    ax = fig.add_subplot(111, projection='3d', facecolor='#101722')
    ax.scatter(first[:, 0], first[:, 1], first[:, 2], c=first[:, 2], cmap='viridis', s=1, alpha=.8, rasterized=True)
    tr = np.array(short_trajectory)
    ax.plot(tr[:, 1], tr[:, 2], tr[:, 3], color='#ffb74d', lw=1.5, label='Estimated trajectory')
    lo, hi = first.min(0), first.max(0)
    center, half = (lo+hi)/2, max((hi-lo).max()/2, 1)
    ax.set(xlim=(center[0]-half, center[0]+half), ylim=(center[1]-half, center[1]+half), zlim=(center[2]-half, center[2]+half), xlabel='Forward (m)', ylabel='Left (m)', zlabel='Approx. up (m)')
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=24, azim=-60)
    for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
        axis.label.set_color('#dce6f0')
        axis.set_pane_color((.06, .09, .14, 1))
    ax.tick_params(colors='#b8c8dc')
    fig.suptitle('Flight 213 — recovered JT16 point cloud', color='white', fontsize=16)
    ax.set_title(f'First 5 s · {len(first):,} display voxels · UNVALIDATED reconstruction', color='#ffd48a', fontsize=10)
    fig.savefig(OUTPUT/'preview.png', dpi=160, bbox_inches='tight', facecolor=fig.get_facecolor())
    (OUTPUT/'README.txt').write_text('Open short-map.html. Includes raw-scan and failed-full-map links.\nAll reconstructions are diagnostic and unvalidated; no reliable LIVO map was established.\n')
    print(json.dumps({'short_map_voxels': len(first), 'short_map_poses': len(short_trajectory), 'output': str(OUTPUT)}, indent=2))


if __name__ == '__main__':
    main()
