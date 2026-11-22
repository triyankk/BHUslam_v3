#!/usr/bin/env python3
"""Compare exploratory LIVO replays with recorded VINS; neither is ground truth."""
import csv
import json
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR', '/tmp/bhu-livo-matplotlib')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'reports/livo-replay-readiness-20261120'
BASE = ROOT/'runs/vins-fusion-boot/20261118T141316.161559760Z/live/flights/flight-518567821/pose_history.csv'


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def positions(data):
    return np.array([[float(v[k]) for k in ['x', 'y', 'z']] for v in data])


def describe(t, p):
    delta = np.diff(p, axis=0)
    dt = np.diff(t)
    speed = np.linalg.norm(delta, axis=1)/dt
    return {'samples': len(t), 'duration_s': float(t[-1]-t[0]),
            'path_length_m': float(np.linalg.norm(delta, axis=1).sum()),
            'endpoint_displacement_m': float(np.linalg.norm(p[-1]-p[0])),
            'max_excursion_m': float(np.linalg.norm(p-p[0], axis=1).max()),
            'speed_p99_m_s': float(np.quantile(speed, .99)),
            'speed_max_m_s': float(speed.max()), 'max_output_gap_s': float(dt.max())}


def main():
    baseline = rows(BASE)
    lookup = {round(float(v['timestamp_s']), 5): v for v in baseline}
    report = {'reference': 'Recorded live VINS, not independent ground truth',
              'accuracy_improvement_established': False,
              'calibration_validated': False, 'experiments': []}
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for name, label in [('flight213-full-01', '0.5 m voxels'), ('flight213-coarse-01', '2.0 m voxels')]:
        directory = OUT/name
        if not (directory/'trajectory.csv').exists():
            continue
        trajectory, diagnostics = rows(directory/'trajectory.csv'), rows(directory/'diagnostics.csv')
        if len(trajectory) < 2:
            continue
        matched = [(v, lookup[round(float(v['timestamp_s']), 5)]) for v in trajectory if round(float(v['timestamp_s']), 5) in lookup]
        livo = [v[0] for v in matched]
        vins = [v[1] for v in matched]
        t = np.array([float(v['timestamp_s']) for v in livo])
        p, ref = positions(livo), positions(vins)
        # Rigid alignment preserves metric scale. This measures disagreement,
        # not accuracy: VINS is itself an estimator with known limitations.
        u, singular, vt = np.linalg.svd((p-p.mean(0)).T@(ref-ref.mean(0)))
        correction = np.diag([1., 1., np.linalg.det(vt.T@u.T)])
        rotation = vt.T@correction@u.T
        aligned = (rotation@(p-p.mean(0)).T).T+ref.mean(0)
        differences = np.linalg.norm(aligned-ref, axis=1)
        stderr = (OUT/(name+'.stderr')).read_text() if (OUT/(name+'.stderr')).exists() else ''
        fields = {k: np.array([float(v[k]) for v in diagnostics]) for k in ['points', 'lidar_constraints', 'visual_points', 'processing_ms']}
        experiment = {'name': name, 'completed': (directory/'summary.json').exists(),
                      'error': next((v for v in stderr.splitlines() if 'LIVO_REPLAY_ERROR' in v), None),
                      'processed_images': len(diagnostics), 'matched_samples': len(matched),
                      'livo': describe(t, p), 'vins_same_timestamps': describe(t, ref),
                      'rigidly_aligned_disagreement_rmse_m': float(np.sqrt(np.mean(differences**2))),
                      'rigidly_aligned_disagreement_p95_m': float(np.quantile(differences, .95)),
                      'frames_with_lidar_constraints': int(np.count_nonzero(fields['lidar_constraints'])),
                      'frames_with_visual_candidates': int(np.count_nonzero(fields['visual_points'])),
                      'processing_ms_p95': float(np.quantile(fields['processing_ms'], .95)),
                      'processing_ms_max': float(fields['processing_ms'].max()),
                      'alignment_singular_values': singular.tolist(),
                      'note': 'Visual candidates are not a validated tracking-health metric. Endpoint displacement is not loop-closure error.'}
        report['experiments'].append(experiment)
        axes[0].plot(t-t[0], np.linalg.norm(p-p[0], axis=1), label='LIVO '+label)
        axes[0].plot(t-t[0], np.linalg.norm(ref-ref[0], axis=1), '--', label='VINS on '+label+' interval')
        dtime = np.array([float(v['timestamp_s']) for v in diagnostics])
        axes[1].plot(dtime-dtime[0], fields['lidar_constraints'], label='LiDAR '+label, alpha=.75)
        axes[1].plot(dtime-dtime[0], fields['visual_points'], '--', label='Visual candidates '+label, alpha=.75)
    axes[0].set(xlabel='Seconds since first matched pose', ylabel='Displacement from first pose (m)', title='Estimator displacement; no ground truth')
    axes[1].set(xlabel='Seconds since replay start', ylabel='Constraints / candidates', title='Measurement support during replay')
    for ax in axes:
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    fig.suptitle('Flight 213 — exploratory LIVO, initial mount and single-anchor timing')
    fig.tight_layout()
    fig.savefig(OUT/'comparison.png', dpi=160)
    (OUT/'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
