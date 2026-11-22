#!/usr/bin/env python3
"""Plot a TUM camera trajectory with gaps determined by tracking.csv."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("slam_directory", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="Output filename without extension")
    args = parser.parse_args()
    poses = np.atleast_2d(np.loadtxt(args.slam_directory / "trajectory.txt"))
    with (args.slam_directory / "tracking.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    states = {round(float(row["timestamp_s"]), 6): int(row["state"]) for row in rows}
    valid = np.array([states.get(round(t, 6)) == 2 for t in poses[:, 0]])
    if not valid.any() or not np.isfinite(poses).all():
        raise ValueError("Need finite poses and at least one tracked frame")
    times = poses[:, 0] - poses[0, 0]
    xyz = poses[:, 1:4]
    keep = valid[:-1] & valid[1:]
    segments = np.stack([xyz[:-1], xyz[1:]], axis=1)[keep]
    color_times = ((times[:-1] + times[1:]) / 2)[keep]
    norm = plt.Normalize(0, times[-1])
    cmap = "viridis"
    fig = plt.figure(figsize=(13, 7.5), facecolor="white")
    grid = fig.add_gridspec(2, 2, height_ratios=[5, 0.65], hspace=0.28, wspace=0.25)
    ax3 = fig.add_subplot(grid[0, 0], projection="3d")
    ax2 = fig.add_subplot(grid[0, 1])
    line3 = Line3DCollection(segments, cmap=cmap, norm=norm, linewidth=1.8)
    line3.set_array(color_times)
    ax3.add_collection3d(line3)
    line2 = LineCollection(segments[:, :, [0, 2]], cmap=cmap, norm=norm, linewidth=1.8)
    line2.set_array(color_times)
    ax2.add_collection(line2)
    first, last = np.flatnonzero(valid)[[0, -1]]
    # Mark boundaries rather than drawing an invented path through lost frames.
    recovered = np.flatnonzero(valid[1:] & ~valid[:-1]) + 1
    for index, marker, color, label in [(first, "o", "#138447", "Start"), (last, "s", "#d44632", "Last tracked pose")]:
        ax3.scatter(*xyz[index], marker=marker, c=color, s=55, depthshade=False)
        ax2.scatter(xyz[index, 0], xyz[index, 2], marker=marker, c=color, s=55, label=label, zorder=5)
    if len(recovered):
        ax3.scatter(*xyz[recovered].T, marker="x", c="#d87900", s=35, depthshade=False)
        ax2.scatter(xyz[recovered, 0], xyz[recovered, 2], marker="x", c="#d87900", s=35, label="Tracking recovered", zorder=5)
    good_xyz = xyz[valid]
    center = (good_xyz.min(axis=0) + good_xyz.max(axis=0)) / 2
    radius = max(np.ptp(good_xyz, axis=0).max() * 0.56, 0.01)
    ax3.set(xlim=(center[0]-radius, center[0]+radius), ylim=(center[1]-radius, center[1]+radius), zlim=(center[2]-radius, center[2]+radius), xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)", title="3D camera trajectory")
    ax3.set_box_aspect((1, 1, 1))
    ax2.autoscale()
    ax2.margins(0.12)
    ax2.set_aspect("equal", adjustable="datalim")
    ax2.set(xlabel="X (m)", ylabel="Z (m)", title="X–Z projection (initial camera frame)")
    ax2.grid(alpha=0.25)
    ax2.legend(loc="best", fontsize=8)
    fig.colorbar(line2, ax=[ax3, ax2], label="Time since recording start (s)", shrink=0.75, pad=0.06)
    timeline = fig.add_subplot(grid[1, :])
    row_times = np.array([float(row["timestamp_s"]) for row in rows]) - poses[0, 0]
    row_valid = np.array([int(row["state"]) == 2 for row in rows])
    timeline.scatter(row_times[row_valid], np.zeros(sum(row_valid)), color="#138447", marker="|", s=120, label="Tracked")
    timeline.scatter(row_times[~row_valid], np.zeros(sum(~row_valid)), color="#d44632", marker="|", s=120, label="Tracking unavailable")
    timeline.set(xlim=(0, times[-1]), yticks=[], xlabel="Recording time (s)", title="Tracking availability — path lines stop across tracking gaps", ylim=(-0.5, 0.5))
    timeline.legend(loc="upper right", fontsize=8)
    name = args.slam_directory.parent.name
    fig.suptitle(f"Gemini 336L · {name} · stereo SLAM\n{sum(row_valid):,}/{len(rows):,} frames tracked · {times[-1]:.2f} seconds", fontsize=14)
    fig.text(0.5, 0.015, "Recorded estimate, not ground truth. Coordinates are relative to the first keyframe; axes are not gravity aligned. VIO reset repeatedly and is not shown as a continuous route.", ha="center", fontsize=8)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for extension in ("png", "svg"):
        path = args.output.with_suffix("." + extension)
        fig.savefig(path, dpi=180, bbox_inches="tight")
        print(path)
    plt.close(fig)


if __name__ == "__main__":
    main()
