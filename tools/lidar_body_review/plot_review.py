"""Plot C++ review output; no geometry classification or ROS processing here."""
import csv
import os
from pathlib import Path
import sys

os.environ.setdefault("MPLCONFIGDIR", "/tmp/lidar-body-review-matplotlib")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

if len(sys.argv) != 3:
    raise SystemExit("Usage: plot_review.py POINTS_CSV OUTPUT_PNG")
groups = {(side, status): [] for side in ("front", "rear")
          for status in ("body", "margin", "kept")}
with Path(sys.argv[1]).with_name("geometry.csv").open() as stream:
    geometry = {k: float(v) for k, v in next(csv.DictReader(stream)).items()}
half_x = geometry["body_length"] / 2
half_y = geometry["body_width"] / 2
margin = geometry["body_margin"]
with Path(sys.argv[1]).open() as stream:
    for row in csv.DictReader(stream):
        groups[row["source"], row["classification"]].append(
            (float(row["plate_y_m"]), float(row["plate_x_m"])))

plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                     "axes.spines.right": False})
fig, axes = plt.subplots(2, 2, figsize=(11, 11))
for column, side in enumerate(("front", "rear")):
    for row, limit in enumerate((3.0, .34)):
        ax = axes[row, column]
        for status, color, label in (("kept", "#087e8b", "Retained"),
                                     ("body", "#dc6b22", "Inside tight CAD envelope"),
                                     ("margin", "#9d3b7d", "Inside 10 mm margin")):
            data = np.asarray(groups[side, status])
            if not len(data):
                continue
            visible = np.max(np.abs(data), axis=1) <= limit
            ax.scatter(data[visible, 0], data[visible, 1], s=1.5 if row == 0 else 3,
                       color=color, alpha=.28, linewidths=0, rasterized=True, label=label)
        ax.add_patch(Rectangle((-half_y, -half_x), 2*half_y, 2*half_x, fill=False,
                               edgecolor="#555555", lw=1))
        ax.add_patch(Rectangle((-half_y-margin, -half_x-margin),
                               2*(half_y+margin), 2*(half_x+margin), fill=False,
                               edgecolor="#b22222", lw=1.3, linestyle="--"))
        for sensor, x in (("F", .110), ("R", -.110)):
            ax.plot(.005, x, "ko", ms=4)
            if row == 1:
                ax.annotate(sensor, (.005, x), xytext=(6, 0), textcoords="offset points")
        ax.set(xlim=(limit, -limit), ylim=(-limit, limit), aspect="equal",
               xlabel="Plate Y (m); robot left is left",
               ylabel="Plate X (m); robot forward is up")
        ax.grid(alpha=.16)
        ax.set_title(f"{side.capitalize()} LiDAR | " + ("scene view" if row == 0 else "body boundary detail"))
        if row == 1:
            legend = ax.legend(loc="upper left", fontsize=8, markerscale=3, framealpha=.95)
            for handle in (getattr(legend, "legend_handles", None) or legend.legendHandles):
                handle.set_alpha(1)

fig.suptitle("Revised FL0 steering envelope: offline body-box comparison", fontsize=16, y=.97)
fig.text(.5, .927, "27 Oct 2026 | 550 front + 557 rear scans | reported steering 90 degrees",
         ha="center", fontsize=11)
fig.text(.5, .048,
         f"Dashed box: {2*(half_x+margin)*1000:g} x {2*(half_y+margin)*1000:g} mm exclusion. "
         f"Solid box: {2*half_x*1000:g} x {2*half_y*1000:g} mm tight CAD envelope.\n"
         "Points are accumulated endpoints from separate sensors; blank areas do not establish free space.\n"
         "Plate-relative coordinates only. No base/odometry transform, live pairing or acquisition timing is validated.",
         ha="center", fontsize=9, linespacing=1.5)
fig.subplots_adjust(left=.09, right=.98, bottom=.17, top=.88, hspace=.30, wspace=.25)
fig.savefig(sys.argv[2], dpi=160)
