"""Summarize six paired blocks and regenerate the report's multisine figure."""

import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager, pyplot as plt
import numpy as np
from scipy.stats import t


MODES = ("off", "static", "dynamic")


def read_columns(path, keys):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    index = {tuple(row[key] for key in keys): row for row in rows}
    assert len(index) == len(rows), f"Duplicate rows in {path}"
    return index


def series(index, label, mode, column):
    return np.array([float(index[(*label, str(block), mode)][column]) for block in range(1, 7)])


def interval(values):
    return t.ppf(.975, len(values)-1) * values.std(ddof=1) / np.sqrt(len(values))


def describe(samples):
    means = " / ".join(f"{x.mean():.2f}" for x in samples)
    differences = [samples[0]-samples[1], samples[0]-samples[2], samples[1]-samples[2]]
    paired = " / ".join(f"{d.mean():+.2f} [{d.mean()-interval(d):+.2f}, {d.mean()+interval(d):+.2f}]"
                        for d in differences)
    return f"{means}   differences: {paired}"


def main(directory, figure):
    trials = read_columns(directory/"trials.csv", ("profile", "block", "mode"))
    responses = read_columns(directory/"responses.csv", ("profile", "frequency_hz", "block", "mode"))
    assert len(trials) == 54 and len(responses) == 270
    print("Modes: off/static/dynamic. Differences: off-static/off-dynamic/static-dynamic.")
    for profile, hz in (("single_3hz", "3.0"), ("single_5hz", "5.0")):
        values = [series(responses, (profile, hz), mode, "transfer_pct") for mode in MODES]
        print(profile, describe(values))
    for name, col in (("multisine RMS ratio", "rms_transfer_pct"), ("multisine tip RMS (mm)", "tip_rms_mm")):
        print(name, describe([series(trials, ("multisine",), mode, col) for mode in MODES]))
    for profile in ("single_3hz", "single_5hz", "multisine"):
        print(profile, "offset peak (mrad)",
              [series(trials, (profile,), mode, "offset_peak_mrad").mean() for mode in MODES])
    print("clips", sum(int(row["clip_updates"]) for row in trials.values()))
    print("minimum active % when enabled",
          min(float(row["active_pct"]) for row in trials.values() if row["mode"] != "off"))

    frequencies = sorted({label[1] for label in responses if label[0] == "multisine"}, key=float)
    values = np.array([[series(responses, ("multisine", hz), mode, "transfer_pct")
                        for hz in frequencies] for mode in MODES])
    for hz, samples in zip(frequencies, values.transpose(1, 0, 2)):
        print("multisine", hz, describe(samples))

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
    f = np.array(frequencies, dtype=float)
    fig, (transfer, contrast) = plt.subplots(1, 2, figsize=(7.45, 3.2))
    fig.subplots_adjust(left=.085, right=.985, top=.86, bottom=.18, wspace=.34)
    for samples, label, line, marker, color, fill in zip(
            values, ("关闭", "静态", "动态"), ("--", ":", "-"), ("o", "^", "s"),
            (".43", ".22", "black"), ("white", "white", "black")):
        transfer.plot(f, samples.mean(axis=1), linestyle=line, marker=marker,
                      markersize=3.3, markerfacecolor=fill, color=color,
                      markeredgewidth=.75, linewidth=.85, label=label)
    transfer.set_ylabel("末端位移传递率 (%)")
    transfer.set_ylim(bottom=0)
    transfer.legend(frameon=False, loc="upper left", ncol=3,
                    columnspacing=.85, handlelength=1.6, fontsize=8)

    contrast.axhline(0, color=".62", linewidth=.7, linestyle="--")
    for delta, label, marker, color, fill, dx in (
            (values[0]-values[2], "关闭 - 动态", "o", ".43", "white", -.075),
            (values[1]-values[2], "静态 - 动态", "s", "black", "black", +.075)):
        contrast.errorbar(f+dx, delta.mean(axis=1), yerr=[interval(d) for d in delta],
                          fmt=marker, markersize=3.6, color=color, markerfacecolor=fill,
                          markeredgewidth=.8, elinewidth=.8, capsize=2, label=label)
    contrast.set_ylabel("动态映射的改善量 (百分点)")
    contrast.legend(frameon=False, loc="best", fontsize=8, ncol=2, columnspacing=.8)
    for ax, tag in ((transfer, "(a)"), (contrast, "(b)")):
        ax.set_xlabel("基座扰动频率 (Hz)")
        ax.set_xticks([.5, 2, 4, 6, 8])
        ax.set_xticklabels(["0.5", "2", "4", "6", "8"])
        ax.set_xlim(.23, 8.27)
        ax.text(0, 1.045, tag, transform=ax.transAxes, fontsize=10, fontweight="bold")

    fig.savefig(figure)


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
