"""Plot the X-axis confirmation and the Z-axis gain screening from archived CSVs."""

import csv
from pathlib import Path
import sys

import numpy as np
from analyze import font_manager, plt


def read_rows(path, stage):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return [row for row in csv.DictReader(stream) if row["stage"] == stage]


def main(data, image):
    x_rows = read_rows(data/"pid-gain-x-responses.csv", "validation")
    z_rows = read_rows(data/"pid-gain-z-trials.csv", "screen")
    windows_font = Path("C:/Windows/Fonts/msyh.ttc")
    font = "Noto Sans CJK SC"
    if windows_font.is_file():
        font_manager.fontManager.addfont(windows_font)
        font = "Microsoft YaHei"
    plt.rcParams.update({
        "font.family": ["STIXGeneral", font], "font.size": 9,
        "axes.unicode_minus": False, "axes.linewidth": .65, "xtick.direction": "in",
        "ytick.direction": "in", "axes.spines.top": False, "axes.spines.right": False,
        "savefig.dpi": 300, "figure.facecolor": "white",
    })
    fig, (x_axis, z_axis) = plt.subplots(1, 2, figsize=(7.45, 3.2))
    fig.subplots_adjust(left=.095, right=.98, top=.84, bottom=.19, wspace=.37)
    frequencies = sorted({float(r["frequency_hz"]) for r in x_rows})
    for kp, kd, label, line, marker, fill in (
            (17.7, .10, "原增益", "--", "o", "white"), (12.0, .35, "调后增益", "-", "s", "black")):
        means = []
        for f in frequencies:
            selected = [r for r in x_rows if (float(r["kp_x"]), float(r["kd_x"])) == (kp, kd)
                        and float(r["frequency_hz"]) == f]
            assert sorted(int(r["block"]) for r in selected) == list(range(1, 7))
            means.append(np.mean([float(r["transfer_pct"]) for r in selected]))
        x_axis.plot(frequencies, means, line, color="black", marker=marker,
                    markerfacecolor=fill, markersize=3.4, markeredgewidth=.8,
                    linewidth=.85, label=label)
    x_axis.set(xlabel="基座扰动频率 (Hz)", ylabel="X 向末端位移传递率 (%)",
               xlim=(.25, 8.3), ylim=(0, None), xticks=[.5, 2, 4, 6, 8])
    x_axis.set_xticklabels(["0.5", "2", "4", "6", "8"])
    x_axis.legend(loc="upper left", frameon=False)

    settings = sorted({(float(r["kp_z"]), float(r["kd_z"])) for r in z_rows})
    for gains in settings:
        selected = [r for r in z_rows if (float(r["kp_z"]), float(r["kd_z"])) == gains]
        assert sorted(int(r["block"]) for r in selected) == [1, 2]
        rms, worst = [np.mean([float(r[key]) for r in selected])
                      for key in ("rms_transfer_pct", "worst_tone_pct")]
        baseline = gains == (7.7, .10)
        z_axis.scatter(rms, worst, color="black" if baseline else ".5",
                       marker="s" if baseline else "o", s=20 if baseline else 14)
        z_axis.annotate(f"({gains[0]:g}, {gains[1]:.2f})", (rms, worst), xytext=(5, 0),
                        textcoords="offset points", va="center", fontsize=8)
    z_axis.set(xlabel="Z 向 RMS 位移传递率 (%)",
               ylabel="最差频点传递率 (%)", xlim=(9.25, 14.55), ylim=(14, 39))
    for ax, title in ((x_axis, "(a) X 向独立验证"), (z_axis, "(b) Z 向增益筛选")):
        ax.set_title(title, loc="left", fontsize=10, pad=9)
    fig.savefig(image)


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
