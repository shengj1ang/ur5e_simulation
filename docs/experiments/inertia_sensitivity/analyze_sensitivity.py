#!/usr/bin/env python3
"""Turn the parameter sweep into sensitivities and sign-flip margins.

The inertial feedforward predicts the difference of two torque terms an order
of magnitude larger than it, so the question a sweep has to answer is not how
far the difference moves when the model is wrong, but how far a parameter has
to be wrong for the difference to change sign. A weight calibrated against a
term whose sign the model is not sure of is calibrated against noise.

For every pose, base axis and parameter family in parameter-sensitivity.csv:

  net_mm        the net along-axis offset of the unperturbed model, in mm for a
                base acceleration of 1 m/s^2
  share_pct     that net as a percentage of the larger of the two terms it is
                the difference of
  slope         d(net)/d(parameter), per 1 % for the multiplicative families and
                per mm for a centre-of-mass shift
  slope_pct     the same slope as a percentage of net_mm, i.e. how much of the
                prediction one step of this parameter is worth
  flip_at       the parameter value at which the net crosses zero, interpolated
                between the swept points either side of the crossing
  margin        how far that is from the nominal model, in steps of the family's
                own unit; blank when the net does not cross zero in the range
  extrapolated  net / slope, the margin the local slope implies when the sweep
                does not reach the crossing. It is what a plausible parameter
                error has to be multiplied by before the sign is in question,
                so it is the number to read when nothing flips in range.

A family whose slope is exactly zero does not reach this feedforward at all.
That is not a degenerate case here: the reference joint acceleration is solved
for the twist (-a_base, 0), so the tip body is in pure translation, and for a
body in pure translation the inertial load and the apparent-gravity load are
the same wrench with opposite signs. Everything bolted past the tip therefore
cancels out of the net exactly, whatever its mass or centre of mass.

Writes parameter-sensitivity-summary.csv next to the input and prints the
tightest margins per condition.
"""
import csv
import sys
from pathlib import Path

# The four conditions the deployed calibration actually uses, from
# inertia_feedforward_gain in sim_pid_control's sim_vel_controller.yaml.
DEPLOYED = {("init", "x"), ("init", "y"), ("init", "z"), ("1st_reco_start", "y")}
NOMINAL = {"scale": 1.0, "mm": 0.0}
STEP = {"scale": 0.01, "mm": 1.0}          # one step, in the units of 'value'
STEP_NAME = {"scale": "1%", "mm": "1mm"}


def crossings(points):
    """Values where the net crosses zero, linearly interpolated."""
    found = []
    for (v0, y0), (v1, y1) in zip(points, points[1:]):
        if y0 == 0.0:
            found.append(v0)
        elif (y0 < 0.0) != (y1 < 0.0):
            found.append(v0 + (v1 - v0) * y0 / (y0 - y1))
    if points and points[-1][1] == 0.0:
        found.append(points[-1][0])
    return found


def slope_at(points, nominal):
    """d(net)/d(value) at the nominal model, centred where there is room."""
    index = next(i for i, (v, _) in enumerate(points) if abs(v - nominal) < 1e-9)
    low = max(index - 1, 0)
    high = min(index + 1, len(points) - 1)
    if high == low:
        return 0.0
    return (points[high][1] - points[low][1]) / (points[high][0] - points[low][0])


def analyse(rows):
    groups = {}
    for row in rows:
        key = (row["pose"], row["axis"], row["family"], row["unit"])
        groups.setdefault(key, []).append(
            (float(row["value"]), float(row["total_mm"]), row))

    results = []
    for (pose, axis, family, unit), points in groups.items():
        assert unit in NOMINAL, f"unknown unit {unit!r} in family {family}"
        points.sort()
        nominal = NOMINAL[unit]
        at_nominal = [row for value, _, row in points if abs(value - nominal) < 1e-9]
        assert at_nominal, f"{family} has no unperturbed point at {nominal}"
        reference = at_nominal[0]
        net = float(reference["total_mm"])
        terms = (abs(float(reference["inertia_term_mm"])),
                 abs(float(reference["gravity_term_mm"])))

        curve = [(value, total) for value, total, _ in points]
        step = STEP[unit]
        slope = slope_at(curve, nominal) * step
        zeros = crossings(curve)
        nearest = min(zeros, key=lambda v: abs(v - nominal)) if zeros else None

        results.append(dict(
            pose=pose, axis=axis, family=family, unit=unit,
            deployed=int((pose, axis) in DEPLOYED),
            net_mm=net,
            inertia_term_mm=float(reference["inertia_term_mm"]),
            gravity_term_mm=float(reference["gravity_term_mm"]),
            share_pct=100.0 * abs(net) / max(terms) if max(terms) else float("nan"),
            step=STEP_NAME[unit],
            slope_mm_per_step=slope,
            slope_pct_per_step=100.0 * slope / net if net else float("nan"),
            flip_at="" if nearest is None else round(nearest, 6),
            margin_steps="" if nearest is None else round(abs(nearest - nominal) / step, 3),
            extrapolated_steps="" if not slope else round(abs(net / slope), 3),
            swept_from=curve[0][0], swept_to=curve[-1][0]))
    return results


def report(results):
    conditions = sorted({(r["pose"], r["axis"]) for r in results},
                        key=lambda c: (c[0] != "init", c[0], c[1]))
    for pose, axis in conditions:
        here = [r for r in results if (r["pose"], r["axis"]) == (pose, axis)]
        first = here[0]
        mark = " (deployed)" if first["deployed"] else ""
        print(f"\n{pose} {axis.upper()}{mark}: net {first['net_mm']:+.4f} mm from "
              f"inertia {first['inertia_term_mm']:+.4f} and gravity "
              f"{first['gravity_term_mm']:+.4f} mm, {first['share_pct']:.1f} % of "
              f"the larger term")
        def margin_of(row):
            """The flip margin to rank on: measured where there is one."""
            if row["margin_steps"] != "":
                return row["margin_steps"], False
            if row["extrapolated_steps"] != "":
                return row["extrapolated_steps"], True
            return None, False

        ranked = sorted(((r,) + margin_of(r) for r in here if margin_of(r)[0] is not None),
                        key=lambda item: item[1])
        print(f"  {'family':<26} {'step':>5} {'mm/step':>10} {'%/step':>9} "
              f"{'margin':>13}")
        for r, steps, extrapolated in ranked[:6]:
            margin = f"{steps:.3f} {r['step'][1:]}" + ("*" if extrapolated else "")
            print(f"  {r['family']:<26} {r['step']:>5} {r['slope_mm_per_step']:>+10.5f} "
                  f"{r['slope_pct_per_step']:>+9.2f} {margin:>13}")
        if any(extrapolated for _, _, extrapolated in ranked[:6]):
            print("  * extrapolated from the local slope: the net does not cross "
                  "zero inside the swept range")
        # The per-link centre-of-mass families say what six independent errors
        # of the same size do: their sensitivities add in quadrature, not in
        # sum, so this margin is the one to read rather than com_arm_*'s.
        per_link = [float(r["slope_mm_per_step"]) for r in here
                    if r["family"].startswith("com_") and r["unit"] == "mm"
                    and not r["family"].startswith(("com_arm_", "com_payload_"))]
        if per_link and first["net_mm"]:
            quadrature = sum(value*value for value in per_link) ** 0.5
            if quadrature:
                print(f"  six links' centres of mass, independent errors of equal "
                      f"size: {abs(first['net_mm'])/quadrature:.2f} mm each would "
                      f"reach zero")
        untouched = [r["family"] for r in here if margin_of(r)[0] is None]
        if untouched:
            print(f"  {len(untouched)} families leave the net untouched: "
                  f"{', '.join(sorted(untouched))}")


def main(directory):
    rows = list(csv.DictReader((directory/"parameter-sensitivity.csv").open()))
    assert rows, "parameter-sensitivity.csv is empty"
    results = analyse(rows)
    out = directory/"parameter-sensitivity-summary.csv"
    with out.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    report(results)
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main(Path(sys.argv[1] if len(sys.argv) > 1 else "."))
