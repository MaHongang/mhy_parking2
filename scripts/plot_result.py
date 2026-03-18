import argparse
import csv
import os
from pathlib import Path

import matplotlib.pyplot as plt


def read_csv_xy(path: Path):
    if not path.exists():
        return []
    rows = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows


def to_float(v, default=0.0):
    try:
        return float(v)
    except Exception:
        return default


def plot_hybrid(ax, rows):
    xs = [to_float(r.get("x")) for r in rows]
    ys = [to_float(r.get("y")) for r in rows]
    if len(xs) == 0:
        ax.text(0.5, 0.5, "hybrid_astar.csv 为空", ha="center", va="center")
        ax.set_title("Hybrid A*")
        ax.grid(True)
        return
    ax.plot(xs, ys, "r-", linewidth=2, label="hybrid_a*")
    ax.set_title(f"Hybrid A* (N={len(xs)})")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True)
    ax.axis("equal")


def plot_traj(ax, rows, title):
    xs = [to_float(r.get("x")) for r in rows]
    ys = [to_float(r.get("y")) for r in rows]
    if len(xs) == 0:
        ax.text(0.5, 0.5, f"{title} 为空", ha="center", va="center")
        ax.set_title(title)
        ax.grid(True)
        return
    ax.plot(xs, ys, "-", linewidth=2)
    ax.set_title(f"{title} (N={len(xs)})")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True)
    ax.axis("equal")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "result_dir",
        help="例如: result/2026_01_27_17_48_41",
    )
    args = ap.parse_args()

    result_dir = Path(args.result_dir)
    if not result_dir.exists():
        raise SystemExit(f"目录不存在: {result_dir}")

    hybrid = read_csv_xy(result_dir / "hybrid_astar.csv")
    coarse = read_csv_xy(result_dir / "coarse_trajectory.csv")
    opt = read_csv_xy(result_dir / "optimized_trajectory.csv")

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    plot_hybrid(axes[0], hybrid)
    plot_traj(axes[1], coarse, "Coarse trajectory")
    plot_traj(axes[2], opt, "Optimized trajectory")
    fig.suptitle(str(result_dir))
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # 允许在无 DISPLAY 环境下仍可保存图片（用户可自行加 --save）
    main()

