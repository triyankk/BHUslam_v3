#!/usr/bin/env python3
"""Plot a captured ToF bridge run and independent Cube USB proximity reports."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    directory = args.directory
    rows = [json.loads(line) for line in (directory/'bridge/measurements.jsonl').read_text().splitlines()]
    reports = json.loads((directory/'cube-reports.json').read_text())['reports']
    cube = [row for row in reports if row['component'] == 1 and row['message']['orientation'] == 4]
    forwarded = [row for row in reports if row['component'] == 191]
    by_stamp = {int(row['monotonic_s']*1000) & 0xffffffff: row for row in rows}
    offsets = [r['elapsed_s']-by_stamp[r['message']['time_boot_ms']]['elapsed_s']
               for r in forwarded if r['message']['time_boot_ms'] in by_stamp]
    if not offsets or not cube:
        raise ValueError('Need matching forwarded packets and Cube rear proximity reports')
    offset = float(np.median(offsets))
    sensor_t = np.array([r['elapsed_s'] for r in rows])
    sensor_m = np.array([r.get('selected_mm',r['nearest_mm'])/1000 if r.get('valid',r['transmitted']) else np.nan for r in rows])
    cube_t = np.array([r['elapsed_s']-offset for r in cube])
    cube_m = np.array([r['message']['current_distance']/100 for r in cube])
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.labelcolor': '#334155', 'text.color': '#0f172a',
                         'xtick.color': '#475569', 'ytick.color': '#475569'})
    fig, axes = plt.subplots(2, 1, figsize=(12, 7.8), sharex=True,
                             gridspec_kw={'height_ratios': [2.4, 1]})
    fig.subplots_adjust(left=.09, right=.97, top=.83, bottom=.13, hspace=.23)
    fig.suptitle('TMF8820 → Jetson UART → Cube Orange+', x=.09, ha='left',
                 y=.98, fontsize=20, fontweight='bold')
    fig.text(.09, .918, 'PIO UART · TELEM1 · 57,600 baud · rear-facing',
             fontsize=11, color='#475569')
    fig.text(.09, .885, f'{len(rows)} sensor samples at 15 Hz  |  {len(cube)} Cube proximity reports',
             fontsize=11, color='#475569')
    axes[0].plot(sensor_t, sensor_m, color='#2563eb', linewidth=1.25,
                 alpha=.85, label='Jetson: nearest confident ToF return')
    axes[0].scatter(cube_t, cube_m, color='#ea580c', s=18, alpha=.85,
                    linewidths=0, zorder=3, label='Cube: processed rear proximity')
    axes[0].set_ylabel('Distance (m)')
    axes[0].set_ylim(0, max(np.nanmax(sensor_m), cube_m.max())+.25)
    axes[0].legend(loc='lower right', frameon=True, framealpha=.95, fontsize=10)
    axes[1].plot(sensor_t[1:], np.diff(sensor_t)*1000, color='#2563eb',
                 linewidth=1.0, label='Sensor frame interval')
    axes[1].scatter(cube_t[1:], np.diff(cube_t)*1000, color='#ea580c', s=9,
                    alpha=.55, label='Cube report receipt interval')
    axes[1].axhline(1000/15, color='#94a3b8', linestyle='--', linewidth=.8)
    axes[1].set_ylabel('Interval (ms)')
    axes[1].set_xlabel('Time from sensor capture start (s)')
    axes[1].set_ylim(bottom=0)
    axes[1].legend(loc='upper right', fontsize=9, framealpha=.95)
    for ax in axes:
        ax.grid(True, color='#e2e8f0', linewidth=.7)
        ax.set_axisbelow(True)
    axes[1].set_xlim(0, max(sensor_t.max(), cube_t.max())+.2)
    fig.text(.09, .037,
             'Cube reports were observed independently over USB. Its receive-time axis is aligned using matching forwarded packets.\n'
             'This alignment compares trends; it does not measure end-to-end latency. No known-distance ground truth was recorded.',
             fontsize=8.5, color='#64748b', linespacing=1.5)
    for suffix in ('png', 'svg'):
        path = directory / f'bridge-distance.{suffix}'
        fig.savefig(path, dpi=180, facecolor='white')
        print(path)
    plt.close(fig)


if __name__ == '__main__':
    main()
