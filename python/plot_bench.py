import argparse
import json
import os

import matplotlib

if os.environ.get("MST_HEADLESS") == "1":
    matplotlib.use("Agg")

import matplotlib.pyplot as plt


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="indir", default="output")
    ap.add_argument("--save", default=None, help="Save a PNG (headless-friendly).")
    ap.add_argument("--dpi", type=int, default=140, help="PNG DPI for --save.")
    ap.add_argument("--no-show", action="store_true", help="Do not open a GUI window.")
    args = ap.parse_args()

    path = os.path.join(args.indir, "bench.json")
    with open(path) as f:
        bench = json.load(f)

    frames = bench.get("frames", 0)
    avg_ms = bench.get("avg_frame_ms", 0.0)
    p95_ms = bench.get("p95_frame_ms", 0.0)
    rmse = bench.get("rmse_pos_m", 0.0)

    print(f"frames={frames} avg_frame_ms={avg_ms:.3f} p95_frame_ms={p95_ms:.3f} rmse_pos_m={rmse:.3f}")

    fig, ax = plt.subplots(figsize=(6, 3))
    ax.bar(["avg_ms", "p95_ms", "rmse_m"], [avg_ms, p95_ms, rmse], color=["tab:blue", "tab:orange", "tab:green"])
    ax.set_title("Benchmark summary")
    ax.grid(True, axis="y", alpha=0.3)
    plt.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=args.dpi)
        print(f"Saved: {args.save}")
        return
    if args.no_show:
        return
    plt.show()


if __name__ == "__main__":
    main()
