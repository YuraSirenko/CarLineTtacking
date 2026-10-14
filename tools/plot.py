"""Plot a simulator run: python3 tools/plot.py build/run.csv docs/run.png"""
import csv
import math
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main(csv_path, png_path):
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    t = [float(r["t"]) for r in rows]
    x = [float(r["x"]) for r in rows]
    y = [float(r["y"]) for r in rows]
    err = [1000 * float(r["cross_track_m"]) for r in rows]

    ts = [2 * math.pi * i / 1000 for i in range(1001)]
    tx = [1.2 * math.cos(s) for s in ts]
    ty = [0.7 * math.sin(s) + 0.25 * math.sin(2 * s) for s in ts]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
    ax1.plot(tx, ty, color="black", linewidth=4, alpha=0.25, label="line")
    ax1.plot(x, y, color="tab:blue", linewidth=1, label="car")
    ax1.plot(x[0], y[0], "o", color="tab:red", label="start")
    ax1.set_aspect("equal")
    ax1.set_title("Path")
    ax1.legend(loc="upper right")

    ax2.plot(t, err, color="tab:blue")
    ax2.set_xlabel("time, s")
    ax2.set_ylabel("cross-track error, mm")
    ax2.set_title("Tracking error")
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(png_path, dpi=120)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
