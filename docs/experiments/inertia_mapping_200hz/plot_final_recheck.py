"""Plot the paired position RMS results from the archived final-config rerun."""

from pathlib import Path
import sys

from analyze import font_manager, interval, plt, read_columns
from matplotlib.lines import Line2D
from matplotlib.ticker import FixedLocator
import numpy as np


POSES = (("init", "init 姿态"), ("1st_reco_start", "取料起始姿态"))
SOURCE = Path(__file__).with_name("final-per-axis-recheck-trials.csv")


def plot(output):
    trials = read_columns(SOURCE, ("pose", "axis", "peak_mm", "block", "mode"))
    font = "Noto Sans CJK SC"
    windows_font = Path("C:/Windows/Fonts/msyh.ttc")
    if windows_font.is_file():
        font_manager.fontManager.addfont(windows_font)
        font = "Microsoft YaHei"
    style = {
        "font.family": ["STIXGeneral", font],
        "font.size": 11,
        "figure.facecolor": "white",
        "axes.linewidth": 0.65,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.spines.left": False,
        "xtick.direction": "in",
    }
    with plt.rc_context(style):
        fig, (rms, effect) = plt.subplots(
            1, 2, figsize=(7.45, 4.25), sharey=True, width_ratios=(1.45, 1)
        )
        fig.subplots_adjust(left=0.111, right=0.882, top=0.827, bottom=0.135, wspace=0.18)
        rms.set_title("(a) 末端三维位移 RMS", loc="left", fontsize=12, pad=8)
        effect.set_title("(b) 开启前馈的改善量", loc="left", fontsize=12, pad=8)
        rms.set(xlim=(0, 2.9), ylim=(14.15, -0.5), xlabel="RMS 均值 (mm)")
        effect.set(xlim=(-0.17, 1.62), xlabel="配对差 (mm)，向右为改善")
        rms.xaxis.set_major_locator(FixedLocator([0, 0.5, 1, 1.5, 2, 2.5]))
        effect.xaxis.set_major_locator(FixedLocator([0, 0.5, 1, 1.5]))
        effect.text(1.33, -0.05, "相对改善", transform=effect.get_yaxis_transform(),
                    ha="right", va="center", fontsize=9.5)

        effect.vlines(0, 0.56, 13.7, colors="0.35", linestyles="--", linewidth=0.8)
        for panel in (rms, effect):
            panel.axhline(6.64, color="0.65", linewidth=0.55)
        rows, labels = [], []
        for pose_index, (pose, label) in enumerate(POSES):
            start = 1 if pose_index == 0 else 8.15
            rms.text(0.02, start - 0.9, label, va="center", fontsize=10.5, weight="bold")
            for index, (axis, peak) in enumerate(
                (axis, peak) for axis in ("x", "y", "z") for peak in (20, 30)
            ):
                samples = {
                    mode: np.array([
                        float(trials[pose, axis, f"{peak:.1f}", str(block), mode]["tip_xyz_rms_mm"])
                        for block in range(1, 6)
                    ])
                    for mode in ("off", "dynamic")
                }
                baseline = samples["off"].mean()
                enabled = samples["dynamic"].mean()
                diffs = samples["off"] - samples["dynamic"]
                delta, half_width = diffs.mean(), interval(diffs)
                crosses_zero = abs(delta) <= half_width
                color = "0.40" if crosses_zero else "black"
                y = start + index
                rows.append(y)
                labels.append(f"{axis.upper()}  {peak}")
                rms.plot([baseline, enabled], [y, y], color="0.53", lw=1.05)
                rms.plot(baseline, y, "o", color=color, mfc="white", ms=5.1, mew=0.85)
                rms.plot(enabled, y, "s", color=color, mfc=color, ms=3.8, mew=0.8)
                effect.errorbar(delta, y, xerr=half_width, fmt="o", color=color,
                                mfc="white" if crosses_zero else color, ms=3.8,
                                capsize=2.3, elinewidth=0.9)
                effect.text(1.33, y, f"{100 * delta / baseline:+.1f}%",
                            transform=effect.get_yaxis_transform(),
                            va="center", ha="right", color=color)

        rms.set_yticks(rows, labels)
        rms.tick_params(axis="y", which="both", length=0)
        effect.tick_params(axis="y", which="both", left=False, labelleft=False)
        fig.legend(
            handles=[
                Line2D([], [], marker="o", mfc="white", color="black",
                       linestyle="none", label="关闭动力学前馈"),
                Line2D([], [], marker="s", color="black", linestyle="none",
                       label="开启动力学前馈"),
            ],
            loc="upper center", bbox_to_anchor=(0.5, 0.991), ncol=2, frameon=False,
            handletextpad=0.25,
        )
        fig.savefig(output, dpi=360)
        plt.close(fig)


if __name__ == "__main__":
    plot(Path(sys.argv[1]))
