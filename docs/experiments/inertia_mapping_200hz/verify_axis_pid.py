"""Recompute PID tuning metrics and check the complete end-effector pose."""

import csv
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import t

from analyze_acceleration_sources import MULTISINE


def read_columns(path, columns):
    with path.open(newline="") as f:
        names = next(csv.reader(f))
    indexes = [names.index(name) for name in ("field.header.stamp", *columns)]
    trace = np.loadtxt(path, delimiter=",", skiprows=1, usecols=indexes)
    trace[:, 0] = trace[:, 0]//1e9 + (trace[:, 0] % 1e9)/1e9
    assert np.all(np.diff(trace[:, 0]) > 0)
    return trace


def metrics(trace, start):
    stamps, values = trace[:, 0]-start, trace[:, 1:]
    angle = 2*np.pi*stamps[:, None]*MULTISINE
    trend = np.column_stack((np.ones(len(stamps)), stamps))
    design = np.column_stack((trend, np.cos(angle), np.sin(angle)))
    coefficients = np.linalg.lstsq(design, values, rcond=None)[0]
    clean = values - trend @ coefficients[:2]
    rms = np.sqrt(np.trapz(clean*clean, stamps, axis=0)/(stamps[-1]-stamps[0]))
    n = len(MULTISINE)
    return rms, coefficients[2:2+n]-1j*coefficients[2+n:]


def main(axis, raw, output):
    index = "xyz".index(axis)
    xyz = [f"field.pose.pose.position.{a}" for a in "xyz"]
    quaternion = [f"field.pose.pose.orientation.{a}" for a in "xyzw"]
    base = read_columns(raw/"base.csv", [xyz[index]])
    tip = read_columns(raw/"ee.csv", xyz+quaternion)
    with (raw/"pid-gain-trials.csv").open(newline="") as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        start, duration = (float(row[key]) for key in ("fit_start_s", "fit_duration_s"))
        b, p = [data[(data[:, 0] >= start) & (data[:, 0] < start+duration)] for data in (base, tip)]
        q = Rotation.from_quat(p[:, 4:])
        tip_pose = np.column_stack((p[:, :4], (q[0].inv()*q).as_rotvec()))
        base_rms, base_phasors = metrics(b, start)
        tip_rms, tip_phasors = metrics(tip_pose, start)
        worst = 100*max(abs(tip_phasors[:, index]/base_phasors[:, 0]))
        assert abs(1000*tip_rms[index]-float(row["tip_rms_mm"])) < .002
        assert abs(100*tip_rms[index]/base_rms[0]-float(row["rms_transfer_pct"])) < .10
        assert abs(worst-float(row["worst_tone_pct"])) < .10
        row.update(tip_xyz_rms_mm=1000*np.linalg.norm(tip_rms[:3]),
                   tip_orientation_rms_mrad=1000*np.linalg.norm(tip_rms[3:]))
    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    print(f"{axis.upper()}: reproduced directional RMS and worst tone for all {len(rows)} runs")
    validation = [r for r in rows if r["stage"] == "validation"]
    if not validation:
        return
    baseline = (17.7, .10) if axis == "x" else (7.7, .10)
    gains = {(float(r["kp_"+axis]), float(r["kd_"+axis])) for r in validation}
    candidate, = gains-{baseline}
    print(f"{baseline} -> {candidate}")
    for metric in ("tip_rms_mm", "rms_transfer_pct", "worst_tone_pct", "tip_xyz_rms_mm",
                   "tip_orientation_rms_mrad", "idle_rms_mm", "joint_speed_warnings", "torque_clip_updates"):
        series = []
        for gain in (baseline, candidate):
            chosen = sorted((r for r in validation if (float(r["kp_"+axis]), float(r["kd_"+axis])) == gain),
                            key=lambda r: int(r["block"]))
            assert [int(r["block"]) for r in chosen] == list(range(1, 7))
            series.append(np.array([float(r[metric]) for r in chosen]))
        a, b = series
        paired = a-b
        width = t.ppf(.975, len(paired)-1)*paired.std(ddof=1)/np.sqrt(len(paired))
        print(f"{metric}: {a.mean():.7f} -> {b.mean():.7f}, delta {paired.mean():+.7f} "
              f"[{paired.mean()-width:+.7f}, {paired.mean()+width:+.7f}], {sum(paired>0)}/6 lower")


if __name__ == "__main__":
    main(sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3]))
