"""Summarize the gain-sign and projection experiment by matched block."""

import argparse
import csv
from pathlib import Path

import numpy as np
from scipy.stats import t


def read_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def interval(samples):
    count = len(samples)
    average = np.mean(samples)
    radius = t.ppf(.975, count-1)*np.std(samples, ddof=1)/np.sqrt(count)
    return f"{average:+.4f} [{average-radius:+.4f}, {average+radius:+.4f}]"


def summarize(directory):
    trials = read_rows(directory/"inertia-factorial-trials.csv")
    responses = read_rows(directory/"inertia-factorial-responses.csv")
    # A directory can hold more than one run, one per pose and disturbance axis,
    # and the trial ids restart with each of them. Summarize them separately.
    for run in dict.fromkeys((row["pose"], row["axis"]) for row in trials):
        report([row for row in trials if (row["pose"], row["axis"]) == run],
               [row for row in responses if (row["pose"], row["axis"]) == run])


def report(trials, responses):
    magnitude = max(float(row["gain"]) for row in trials)
    blocks = sorted({int(row["block"]) for row in trials})
    cells = [(0.0, 0), (-magnitude, 0), (magnitude, 0), (-magnitude, 1), (magnitude, 1)]

    def key(row):
        return int(row["block"]), float(row["gain"]), int(row["project_translation"])

    by_trial = {key(row): row for row in trials}
    assert len(trials) == len(by_trial) == len(blocks)*len(cells)
    header = trials[0]
    print(f'{header["pose"]} {header["axis"].upper()}, {header["peak_mm"]} mm, {len(blocks)} blocks')
    for name in ("tip_rms_mm", "tip_xyz_rms_mm", "tip_orientation_rms_mrad"):
        values = {cell: np.array([float(by_trial[(block, *cell)][name]) for block in blocks]) for cell in cells}
        baseline = values[(0.0, 0)]
        for cell in cells:
            gain, projection = cell
            print(f"  {name} {gain:+g}, projection={projection}: {values[cell].mean():.4f}; "
                  f"improvement over zero gain {interval(baseline-values[cell])}")
        for gain in (-magnitude, magnitude):
            print(f"  {name} projection improvement at {gain:+g}: "
                  f"{interval(values[(gain, 0)] - values[(gain, 1)])}")
        interaction = (values[(magnitude, 1)] - values[(-magnitude, 1)] -
                       values[(magnitude, 0)] + values[(-magnitude, 0)])
        print(f"  {name} sign × projection contrast: {interval(interaction)}")

    response = {(float(row["frequency_hz"]), *key(row)):
                .01*float(row["transfer_pct"])*np.exp(1j*np.deg2rad(float(row["phase_deg"])))
                for row in responses}
    for frequency in (5, 8):
        for projection in (0, 1):
            # Normalize by the zero-gain phasor before averaging blocks.
            sensitivity = np.array([
                (response[(frequency, block, magnitude, projection)] -
                 response[(frequency, block, -magnitude, projection)]) /
                (2*magnitude*response[(frequency, block, 0.0, 0)]) for block in blocks])
            print(f"  {frequency:g} Hz, projection={projection}, normalized sensitivity: "
                  f"{sensitivity.mean():+.4f}; in-phase part {interval(sensitivity.real)}")

    # The per-sample telemetry stays with the raw recording, which is far larger
    # than the rest of this directory. These are the per-trial aggregates of the
    # same quantities, written alongside each trial by compare_inertia_factors.
    projected = [row for row in trials if int(row["project_translation"])]
    print(f'  Projected axis: {sum(int(row["axis_samples"]) for row in projected)} viable samples, '
          f'{sum(int(row["axis_sign_reversals"]) for row in projected)} sign reversals, '
          f'max |mapped/raw - 1| {max(float(row["max_axis_gain_error"]) for row in projected):.3g}')
    print("  Max |requested torque - gain × mapped torque|: "
          f'{max(float(row["max_request_error_nm"]) for row in trials):.3g} N·m')
    print(f'  Model valid on {sum(int(row["model_valid_samples"]) for row in trials)} of '
          f'{sum(int(row["diag_samples"]) for row in trials)} diagnostic samples')
    print(f"  Minimum sigma {min(float(row['sigma_min']) for row in trials):.4f}; "
          f"clip updates {sum(int(row['clip_updates']) for row in trials)}; "
          f"joint speed warnings {sum(int(row['joint_speed_warnings']) for row in trials)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directories", type=Path, nargs="+")
    for folder in parser.parse_args().directories:
        summarize(folder)
