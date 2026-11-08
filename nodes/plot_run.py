#!/usr/bin/env python3
"""Plots of one run of your node, from the CSV the monitor wrote (given).

    ros2 run ohm_icp plot_run.py                                  the newest run in runs/
    ros2 run ohm_icp plot_run.py runs/task1_every10_dmax0.3_141502.csv
    ros2 run ohm_icp plot_run.py --save                           also save it as .png next to the .csv

Three plots: the paths (truth, odometry, ICP, all from the start of the run), the position error and the
heading error of odometry and ICP over the driven distance.
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from ohm_icp import paths                                      # noqa: E402


def load(path: str):
    """(settings, rows) of one CSV; every value a float."""
    with open(path, encoding="utf-8") as handle:
        first = handle.readline()
        settings = json.loads(first[1:]) if first.startswith("#") else {}
        if not settings:
            handle.seek(0)
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(handle)]
    return settings, rows


def driven(rows) -> list:
    """The length of the true path up to every row, m."""
    out, total = [], 0.0
    for a, b in zip([rows[0]] + rows[:-1], rows):
        total += math.hypot(b["x_true"] - a["x_true"], b["y_true"] - a["y_true"])
        out.append(total)
    return out


def newest() -> str:
    found = sorted(glob.glob(os.path.join(paths.lab_file("runs"), "*.csv")), key=os.path.getmtime)
    if not found:
        raise SystemExit("no run in runs/ yet: start the launch file and your node first")
    return found[-1]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("csv", nargs="?", help="a run of runs/ (default: the newest)")
    ap.add_argument("--save", action="store_true", help="save the figure as .png next to the .csv")
    args = ap.parse_args()
    path = args.csv or newest()
    settings, rows = load(path)
    if len(rows) < 2:
        raise SystemExit(f"{path}: fewer than two matches")
    import matplotlib.pyplot as plt
    s = driven(rows)
    fig = plt.figure(figsize=(12, 4.8))
    title = ", ".join(f"{k} {v}" for k, v in settings.items())
    fig.suptitle(f"{os.path.basename(path)}: {title}", fontsize=10)
    ax = fig.add_subplot(1, 3, 1)
    for name, label, color in (("true", "truth", "0.3"), ("odom", "odometry", "tab:blue"), ("icp", "ICP", "tab:red")):
        ax.plot([r["x_" + name] for r in rows], [r["y_" + name] for r in rows], color=color, label=label, lw=1.2)
    ax.set_aspect("equal")
    ax.set_xlabel("x in m")
    ax.set_ylabel("y in m")
    ax.legend(fontsize=8)
    ax.set_title("paths from the start of the run", fontsize=10)
    for k, (key, unit, scale, text) in enumerate((("err", "m", 1.0, "position error"),
                                                   ("head", "deg", 180.0 / math.pi, "heading error"))):
        ax = fig.add_subplot(1, 3, 2 + k)
        ax.plot(s, [r[key + "_odom"] * scale for r in rows], color="tab:blue", label="odometry")
        ax.plot(s, [r[key + "_icp"] * scale for r in rows], color="tab:red", label="ICP")
        ax.set_xlabel("driven distance in m")
        ax.set_ylabel(f"{text} in {unit}")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_title(text, fontsize=10)
    fig.tight_layout()
    if args.save:
        out = os.path.splitext(path)[0] + ".png"
        fig.savefig(out, dpi=110)
        print(f"saved {out}")
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
