"""Plot the independent 200 Hz PID validation at two disturbance amplitudes."""

import csv
from pathlib import Path
import sys

import numpy as np
from analyze import font_manager, interval, plt


def read(path):
    with path.open(newline="") as stream:
        return [r for r in csv.DictReader(stream) if r["stage"] == "validation"]


def main(data, output):
    trials = read(data/"pid-gain-trials.csv")
    responses = read(data/"pid-gain-responses.csv")
    assert len(trials) == 24 and len(responses) == 312
    baseline = ("7.7", "0.1")
    gains = {tuple(r[k] for k in ("kp_y", "kd_y")) for r in trials}
    assert len(gains) == 2 and baseline in gains
    tuned, = gains-{baseline}

    def series(rows, gain, peak, key):
        selected = [r for r in rows if (r["kp_y"], r["kd_y"]) == gain and float(r["peak_mm"]) == peak]
        selected.sort(key=lambda r: int(r["block"]))
        assert [int(r["block"]) for r in selected] == list(range(1, 7))
        return np.array([float(r[key]) for r in selected])

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
    fig, (rate, pairs) = plt.subplots(1, 2, figsize=(7.45, 3.2))
    fig.subplots_adjust(left=.09, right=.98, bottom=.18, top=.86, wspace=.32)
    frequencies = sorted({float(r["frequency_hz"]) for r in responses})
    for gain, label, line, marker, fill in (
            (baseline, "原增益", "--", "o", "white"), (tuned, "调后增益", "-", "s", "black")):
        bands = [[r for r in responses if float(r["frequency_hz"]) == f] for f in frequencies]
        means = [series(rows, gain, 20, "transfer_pct").mean() for rows in bands]
        rate.plot(frequencies, means, line, marker=marker, color="black", markerfacecolor=fill,
                  markersize=3.4, markeredgewidth=.8, linewidth=.9, label=label)

    rate.legend(frameon=False, loc="upper left")
    rate.set(xlabel="基座扰动频率 (Hz)", ylabel="末端位移传递率 (%)",
             xlim=(.23, 8.27), ylim=(0, None), xticks=[.5, 2, 4, 6, 8])
    rate.set_xticklabels(["0.5", "2", "4", "6", "8"])

    pairs.axhline(0, color=".55", linestyle="--", linewidth=.75)
    for key, label, marker, fill, offset in (
            ("rms_transfer_pct", "RMS 比值", "o", "white", -.12),
            ("worst_tone_pct", "最差频点", "s", "black", .12)):
        for index, peak in enumerate((20, 30)):
            delta = series(trials, baseline, peak, key) - series(trials, tuned, peak, key)
            xpos = index+offset
            pairs.scatter(xpos+np.linspace(-.035, .035, len(delta)), delta, s=8, color=".65")
            pairs.errorbar(xpos, delta.mean(), yerr=interval(delta), fmt=marker,
                           markersize=4.1, color="black", markerfacecolor=fill, capsize=3,
                           elinewidth=.85, label=label if index == 0 else None)
    pairs.set(xlabel="合成底座位移指令峰值 (mm)", ylabel="配对改善量 (百分点)",
              xlim=(-.42, 1.42), xticks=(0, 1))
    pairs.set_xticklabels(["20", "30"])
    pairs.legend(frameon=False, fontsize=8, loc="upper center", bbox_to_anchor=(.53, 1))
    for ax, label in ((rate, "(a)"), (pairs, "(b)")):
        ax.text(0, 1.045, label, transform=ax.transAxes, fontsize=10, fontweight="bold")
    fig.savefig(output)


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
