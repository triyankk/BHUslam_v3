#!/usr/bin/env python3
"""Check capture timing, IMU units, feature availability and stereo alignment."""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np


def timing(t, hz):
    t = np.asarray(t, dtype=float)
    if len(t) < 2:
        raise ValueError('Need at least two samples')
    d = np.diff(t)
    return dict(count=len(t), duration_s=float(t[-1]-t[0]),
                rate_hz=float((len(t)-1)/(t[-1]-t[0])) if t[-1] > t[0] else 0,
                nonmonotonic=int(np.sum(d <= 0)), interval_median_ms=float(np.median(d)*1000),
                interval_p99_ms=float(np.percentile(d, 99)*1000),
                interval_max_ms=float(np.max(d)*1000),
                estimated_missing_samples=int(np.maximum(np.rint(d*hz)-1, 0).sum()))


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument('recording', type=Path)
    args = p.parse_args()
    root = args.recording
    with (root/'frames.csv').open() as f:
        frames = list(csv.DictReader(f))
    report = {'frames': timing([float(f['timestamp_s']) for f in frames], 30)}
    skew = [abs(int(f['left_us'])-int(f['right_us'])) for f in frames]
    report['stereo_skew_max_us'] = max(skew)
    for side in ['left', 'right']:
        index = np.array([int(f[side+'_index']) for f in frames])
        report[side+'_index_gaps'] = int(np.maximum(np.diff(index)-1, 0).sum())
    imu = {}
    for kind in ['accel', 'gyro']:
        x = np.loadtxt(root/(kind+'.csv'), delimiter=',', skiprows=1, ndmin=2)
        report[kind] = timing(x[:, 0], 200)
        report[kind].update(mean_xyz=np.mean(x[:, 1:], axis=0).tolist(),
                            std_xyz=np.std(x[:, 1:], axis=0).tolist(),
                            median_norm=float(np.median(np.linalg.norm(x[:, 1:], axis=1))),
                            finite=bool(np.all(np.isfinite(x))))
        imu[kind] = x
    report['imu_covers_images'] = all(x[0, 0] <= float(frames[0]['timestamp_s']) and
                                     x[-1, 0] >= float(frames[-1]['timestamp_s']) for x in imu.values())
    report['acceleration_scale_plausible_SI'] = 8 < report['accel']['median_norm'] < 12
    orb = cv2.ORB_create(1500)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    measurements = []
    for i in np.linspace(0, len(frames)-1, min(12, len(frames)), dtype=int):
        row = frames[i]
        left = cv2.imread(str(root/row['left']), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(root/row['right']), cv2.IMREAD_GRAYSCALE)
        kl, dl = orb.detectAndCompute(left, None)
        kr, dr = orb.detectAndCompute(right, None)
        good = []
        if dl is not None and dr is not None and len(dr) >= 2:
            good = [m for pair in matcher.knnMatch(dl, dr, k=2) if len(pair) == 2
                    for m, n in [pair] if m.distance < .7*n.distance
                    and kl[m.queryIdx].pt[0] > kr[m.trainIdx].pt[0]]
        error = [abs(kl[m.queryIdx].pt[1]-kr[m.trainIdx].pt[1]) for m in good]
        measurements.append(dict(frame=int(i), left_features=len(kl), right_features=len(kr),
                                 positive_disparity_matches=len(good),
                                 vertical_error_median_px=float(np.median(error)) if error else None))
    report['stereo_feature_samples'] = measurements
    # This is a capture check; feature matching diagnostics are scene-dependent.
    report['capture_timing_pass'] = all(
        report[k]['nonmonotonic'] == 0 and abs(report[k]['rate_hz']-hz) < hz*.05
        and report[k]['estimated_missing_samples'] == 0
        and report[k]['interval_max_ms'] < 2.5*1000/hz
        for k, hz in [('frames', 30), ('accel', 200), ('gyro', 200)])
    passed = (report['capture_timing_pass'] and report['acceleration_scale_plausible_SI']
              and report['imu_covers_images'] and all(report[k]['finite'] for k in imu)
              and report['left_index_gaps'] == 0 and report['right_index_gaps'] == 0
              and report['stereo_skew_max_us'] <= 500)
    report['quality_pass'] = bool(passed)
    (root/'quality.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
