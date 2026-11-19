"""Summarize the inertia switch and the second-pose gain checks."""

import csv
from pathlib import Path
import sys

import numpy as np
from analyze import font_manager, interval, plt


METRICS = ("rms_transfer_pct", "worst_tone_pct", "tip_rms_mm")


def paired(path, group_key, groups, amount, metric):
    with path.open(newline="") as stream:
        rows = [row for row in csv.DictReader(stream) if float(row["peak_mm"]) == amount]
    samples = []
    for group in groups:
        matching = sorted((r for r in rows if r[group_key] == group), key=lambda r: int(r["block"]))
        assert [int(r["block"]) for r in matching] == list(range(1, 7))
        samples.append(np.array([float(row[metric]) for row in matching]))
    return samples


def main(data, output):
    font = "Noto Sans CJK SC"
    windows_font = Path("C:/Windows/Fonts/msyh.ttc")
    if windows_font.is_file():
        font_manager.fontManager.addfont(windows_font)
        font = "Microsoft YaHei"
    plt.rcParams.update({
        "font.family": ["STIXGeneral", font], "font.size": 9, "axes.unicode_minus": False,
        "axes.linewidth": .65, "xtick.direction": "in", "ytick.direction": "in",
        "axes.spines.top": False, "axes.spines.right": False, "savefig.dpi": 300,
        "figure.facecolor": "white",
    })
    fig, axes = plt.subplots(1, 2, figsize=(7.45, 3.2), sharey=True)
    fig.subplots_adjust(left=.095, right=.984, top=.8, bottom=.19, wspace=.34)
    configs = [
        ("tuned-inertia-trials.csv", "mode", ("off", "dynamic"), "(a) init 姿态的惯性前馈"),
        ("second-pose-gain-trials.csv", "kp_y", ("7.7", "12.0"), "(b) 取料起始姿态的 PID"),
    ]
    symbols = [
        ("o", "white", -.13, "RMS 传递率"),
        ("s", "black", .13, "最差频点传递率"),
    ]
    for ax, (file, key, groups, title) in zip(axes, configs):
        print(file)
        ax.axhline(0, linewidth=.7, color=".58", linestyle="--")
        for index, amount in enumerate((20, 30)):
            for n, metric in enumerate(METRICS):
                a, b = paired(data/file, key, groups, amount, metric)
                delta = a-b
                mean, width = delta.mean(), interval(delta)
                print(f"{amount} mm {metric}: {a.mean():.5f} to {b.mean():.5f}, "
                      f"delta {mean:+.5f} [{mean-width:+.5f}, {mean+width:+.5f}], "
                      f"{np.count_nonzero(delta>0)}/6 lower")
                if n < len(symbols):
                    marker, fill, offset, legend = symbols[n]
                    x = index+offset
                    ax.scatter(x+np.linspace(-.036, .036, len(delta)), delta, s=8, color=".62")
                    ax.errorbar(x, mean, yerr=width, fmt=marker, markersize=4, color="black",
                                markerfacecolor=fill, elinewidth=.85, capsize=3,
                                label=legend if index == 0 else None)
        ax.set_xticks([0, 1])
        ax.set_xticklabels(["20", "30"])
        ax.set_xlim(-.43, 1.43)
        ax.set_xlabel("合成底座位移指令峰值 (mm)")
        ax.set_title(title, loc="left", fontsize=10, pad=9)
    axes[0].set_ylabel("配对改善量 (百分点)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", frameon=False, ncol=2, fontsize=8)
    fig.savefig(output)


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
