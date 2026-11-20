"""Recompute peak and RMS from timed ROS position samples and summarize by pair."""

import csv
from pathlib import Path
import sys

import numpy as np
from scipy.stats import t


MULTISINE = np.array([.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 6, 7, 8])


def motion_metrics(samples, start, duration, frequencies):
    data = np.asarray(samples)
    onset = start-1.5 if len(frequencies) else start
    before, guard = (.50, .02) if len(frequencies) else (.55, .05)
    baseline = data[(data[:, 0] >= onset-before) & (data[:, 0] < onset-guard), 1]
    assert len(baseline) >= 20
    segment = data[(data[:, 0] >= start) & (data[:, 0] < start+duration)]
    ts, y = segment[:, 0]-start, segment[:, 1]
    if len(frequencies):
        trend = np.column_stack((np.ones(len(ts)), ts))
        phase = 2*np.pi*ts[:, None]*frequencies
        design = np.column_stack((trend, np.cos(phase), np.sin(phase)))
        intercept, drift = np.linalg.lstsq(design, y, rcond=None)[0][:2]
        error = y - (intercept + drift*ts)
    else:
        error = y-np.mean(baseline)
    rms = np.sqrt(np.trapz(error**2, ts)/(ts[-1]-ts[0]))
    return 1000*rms, 1000*np.max(np.abs(error)), 1000*np.std(baseline)


def read_trace(path):
    with path.open(newline="") as f:
        headers = next(csv.reader(f))
    indexes = [headers.index(key) for key in ("field.header.stamp", "field.pose.pose.position.y")]
    data = np.loadtxt(path, delimiter=",", skiprows=1, usecols=indexes)
    data[:, 0] = data[:, 0]//1e9+(data[:, 0] % 1e9)/1e9
    return data


def main(raw, output):
    with (raw/"acceleration-source-trials.csv").open(newline="") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 36
    base, tip = [read_trace(raw/(name+".csv")) for name in ("base", "ee")]
    for row in rows:
        start, duration = (float(row[key]) for key in ("fit_start_s", "fit_duration_s"))
        frequencies = MULTISINE if row["profile"] == "multisine" else np.empty(0)
        b_rms, b_peak, _ = motion_metrics(base, start, duration, frequencies)
        e_rms, e_peak, idle = motion_metrics(tip, start, duration, frequencies)
        assert abs(e_rms-float(row["tip_rms_mm"])) < .0001 and abs(b_rms-float(row["base_rms_mm"])) < .0001
        row.update(base_rms_mm=b_rms, tip_rms_mm=e_rms, rms_transfer_pct=100*e_rms/b_rms,
                   base_peak_mm=b_peak, tip_peak_mm=e_peak, idle_rms_mm=idle)
    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    for profile in ("multisine", "pulse"):
        print(profile)
        for column in ("tip_rms_mm", "tip_peak_mm", "idle_rms_mm"):
            grouped = []
            for source in ("command", "model", "measured"):
                selected = sorted((r for r in rows if r["profile"] == profile and r["source"] == source),
                                  key=lambda r: int(r["block"]))
                assert [int(r["block"]) for r in selected] == list(range(1, 7))
                grouped.append(np.array([float(r[column]) for r in selected]))
            print(column, " / ".join(f"{x.mean():.5f}" for x in grouped))
            for label, diff in (("command-model", grouped[0]-grouped[1]),
                                ("command-measured", grouped[0]-grouped[2]),
                                ("model-measured", grouped[1]-grouped[2])):
                ci = t.ppf(.975, 5)*diff.std(ddof=1)/np.sqrt(6)
                print(f"  {label}: {diff.mean():+.5f} [{diff.mean()-ci:+.5f}, {diff.mean()+ci:+.5f}] "
                      f"lower in {sum(diff > 0)}/6")


if __name__ == "__main__":
    main(*(Path(arg) for arg in sys.argv[1:]))
