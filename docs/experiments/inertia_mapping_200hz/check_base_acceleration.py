"""Compare commanded acceleration with the measured motion at each excited tone."""

import csv
from pathlib import Path
import sys

import numpy as np


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def phasors(samples, begin, duration, omega):
    segment = samples[(samples[:, 0] >= begin) & (samples[:, 0] < begin+duration)]
    assert len(segment) >= 190*duration
    t = segment[:, 0]-begin
    angle = t[:, None]*omega
    design = np.column_stack((np.ones(len(t)), t, np.cos(angle), np.sin(angle)))
    coeff = np.linalg.lstsq(design, segment[:, 1:3], rcond=None)[0]
    count = len(omega)
    return coeff[2:2+count] - 1j*coeff[2+count:]


def main(raw, data, output):
    trials = read_csv(data/"tuned-inertia-trials.csv")
    responses = read_csv(data/"tuned-inertia-responses.csv")
    assert len(trials) == 24 and len(responses) == 312
    with raw.open(newline="") as stream:
        names = next(csv.reader(stream))
    columns = ["field.header.stamp", "field.pose.pose.position.y", "field.twist.twist.linear.y"]
    samples = np.loadtxt(raw, delimiter=",", skiprows=1, usecols=[names.index(x) for x in columns])
    samples[:, 0] = samples[:, 0]//1e9 + (samples[:, 0] % 1e9)/1e9
    frequencies = np.array(sorted({float(r["frequency_hz"]) for r in responses}))
    omega = 2*np.pi*frequencies
    rng = np.random.default_rng(412195)
    # The logged trial manifest predates storing the phase. Reconstruct the
    # two waveforms per block from their seed, then check all amplitudes.
    phases = {(block, peak): rng.uniform(-np.pi, np.pi, len(frequencies))
              for block in range(1, 7) for peak in (20., 30.)}
    rows = []
    for trial in trials:
        block, peak = int(trial["block"]), float(trial["peak_mm"])
        phase = phases[block, peak]
        start, duration = (float(trial[k]) for k in ("fit_start_s", "fit_duration_s"))
        matching = sorted(
            (r for r in responses if all(r[k] == trial[k] for k in ("block", "peak_mm", "mode"))),
            key=lambda r: float(r["frequency_hz"]))
        amplitude = np.array([float(r["command_mm"])*.001 for r in matching])
        weight = 1/(1+(frequencies/4)**2)
        t = np.linspace(0, duration, 8000)
        normalized = .001*peak*weight/np.max(np.abs(np.sin(t[:, None]*omega+phase) @ weight))
        np.testing.assert_allclose(amplitude, normalized, rtol=1e-12)
        # cos/sin phasors use the fit origin, 1.5 s after waveform onset.
        cmd_position = -1j*amplitude*np.exp(1j*(phase+1.5*omega))
        cmd_accel = -(omega**2)*cmd_position
        position, velocity = phasors(samples, start, duration, omega).T
        np.testing.assert_allclose(np.abs(position)*1000, [float(r["base_mm"]) for r in matching], atol=1e-5)
        actual_accel = 1j*omega*velocity
        pose_accel = -(omega**2)*position
        for i, f in enumerate(frequencies):
            ratio = actual_accel[i]/cmd_accel[i]
            estimate_ratio = pose_accel[i]/actual_accel[i]
            rows.append(dict(block=block, peak_mm=peak, mode=trial["mode"], frequency_hz=f,
                             command_m_s2=abs(cmd_accel[i]), measured_m_s2=abs(actual_accel[i]),
                             gain=abs(ratio), lag_deg=-np.angle(ratio, deg=True),
                             pose_to_velocity_gain=abs(estimate_ratio),
                             pose_to_velocity_phase_deg=np.angle(estimate_ratio, deg=True)))
        error = np.linalg.norm(actual_accel-cmd_accel)/np.linalg.norm(cmd_accel)
        print(f"{peak:g} mm block {block} {trial['mode']}: normalized complex mismatch {100*error:.2f}%")
    with output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    for peak in (20, 30):
        for f in (1, 3, 5, 8):
            chosen = [r for r in rows if r["peak_mm"] == peak and r["frequency_hz"] == f and r["mode"] == "dynamic"]
            gain = [r["gain"] for r in chosen]
            lag = [r["lag_deg"] for r in chosen]
            print(f"{peak} mm, {f} Hz: gain {min(gain):.4f} to {max(gain):.4f}, "
                  f"lag {min(lag):.2f} to {max(lag):.2f} degrees, "
                  f"block 6 lag {lag[-1]:.2f} degrees")
    discrepancy = max(abs(r["pose_to_velocity_gain"]-1) for r in rows)
    phase_difference = max(abs(r["pose_to_velocity_phase_deg"]) for r in rows)
    print(f"Cross-check of velocity- and position-derived acceleration: max "
          f"gain deviation {100*discrepancy:.2f}%, phase {phase_difference:.2f} degrees")


if __name__ == "__main__":
    main(*(Path(arg) for arg in sys.argv[1:]))
