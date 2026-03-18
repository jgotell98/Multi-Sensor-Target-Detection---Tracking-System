import argparse
import csv
import os
from dataclasses import dataclass

import matplotlib

if os.environ.get("MST_HEADLESS") == "1":
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation


@dataclass
class Series:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    id: np.ndarray


def read_truth(path: str) -> Series:
    t, x, y, oid = [], [], [], []
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            t.append(float(row["t_s"]))
            oid.append(int(row["object_id"]))
            x.append(float(row["x"]))
            y.append(float(row["y"]))
    return Series(np.array(t), np.array(x), np.array(y), np.array(oid))


def read_tracks(path: str) -> Series:
    t, x, y, tid = [], [], [], []
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            t.append(float(row["t_s"]))
            tid.append(int(row["track_id"]))
            x.append(float(row["x"]))
            y.append(float(row["y"]))
    return Series(np.array(t), np.array(x), np.array(y), np.array(tid))


def read_radar_dets(path: str) -> Series:
    t, x, y, did = [], [], [], []
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            if row["sensor"] != "radar":
                continue
            t.append(float(row["t_s"]))
            x.append(float(row["x"]))
            y.append(float(row["y"]))
            did.append(0)
    return Series(np.array(t), np.array(x), np.array(y), np.array(did))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="indir", default="output")
    ap.add_argument("--extent", type=float, default=120.0)
    ap.add_argument("--save", default=None, help="Save a PNG snapshot (headless-friendly).")
    ap.add_argument("--dpi", type=int, default=140, help="PNG DPI for --save.")
    ap.add_argument("--no-show", action="store_true", help="Do not open a GUI window.")
    args = ap.parse_args()

    backend = matplotlib.get_backend().lower()
    if backend == "agg" and not (args.save or args.no_show):
        print(
            "Matplotlib is using the non-GUI 'Agg' backend, so no window will appear.\n"
            "Fix: unset MST_HEADLESS/MPLBACKEND (or install a GUI backend like Tk/Qt) and rerun.\n"
            "PowerShell: `Remove-Item Env:\\MST_HEADLESS -ErrorAction SilentlyContinue; "
            "Remove-Item Env:\\MPLBACKEND -ErrorAction SilentlyContinue`",
            flush=True,
        )

    truth = read_truth(os.path.join(args.indir, "truth.csv"))
    tracks = read_tracks(os.path.join(args.indir, "tracks.csv"))
    dets = read_radar_dets(os.path.join(args.indir, "detections.csv"))

    all_t = np.unique(truth.t)
    if all_t.size == 0:
        raise SystemExit("No frames found in truth.csv")

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_title("Multi-Sensor Tracking (Truth / Detections / Tracks)")
    ax.set_xlim(-args.extent, args.extent)
    ax.set_ylim(-args.extent, args.extent)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)

    truth_sc = ax.scatter([], [], s=60, c="tab:green", label="Truth")
    det_sc = ax.scatter([], [], s=20, c="tab:orange", alpha=0.6, label="Detections")
    track_sc = ax.scatter([], [], s=50, c="tab:blue", marker="x", label="Tracks")
    ax.legend(loc="upper right")

    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")

    def frame_points(series: Series, t0: float):
        m = series.t == t0
        return series.x[m], series.y[m]

    def update(i: int):
        t0 = float(all_t[i])
        tx, ty = frame_points(truth, t0)
        dx, dy = frame_points(dets, t0)
        kx, ky = frame_points(tracks, t0)

        truth_sc.set_offsets(np.c_[tx, ty] if tx.size else np.empty((0, 2)))
        det_sc.set_offsets(np.c_[dx, dy] if dx.size else np.empty((0, 2)))
        track_sc.set_offsets(np.c_[kx, ky] if kx.size else np.empty((0, 2)))
        time_text.set_text(f"t = {t0:0.2f} s")
        return truth_sc, det_sc, track_sc, time_text

    if args.save or args.no_show:
        update(len(all_t) - 1)
        if args.save:
            fig.savefig(args.save, dpi=args.dpi)
            print(f"Saved: {args.save}")
        return

    # Keep a reference to the animation; otherwise it can be garbage-collected
    # before rendering on some backends.
    anim = FuncAnimation(fig, update, frames=len(all_t), interval=50, blit=True)
    plt.show(block=True)
    _ = anim


if __name__ == "__main__":
    main()
