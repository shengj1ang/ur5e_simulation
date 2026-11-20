#!/usr/bin/env python3
"""Does /gantry/base_linear_acceleration carry what the base actually does?

Drives one gantry tone at a time and locks in on two signals over the same
window: the acceleration the node published, and the acceleration the base
reached, taken as the base velocity phasor times j*omega. Reports their ratio.
With the correction on the ratio should be 1; with it off it should reproduce
the bias recorded in base-acceleration-comparison.csv, which is the control
that says this method agrees with the original measurement.

Nothing else in the repository starts the gantry's velocity controller, so load
it here the way gantry_disturbance.launch would.

Usage: verify_base_response.py <output dir>
"""
import csv
import subprocess
import sys
from pathlib import Path

import numpy as np
import rosparam
import rospkg
import rospy
from controller_manager_msgs.srv import LoadController, SwitchController
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry

GANTRY = "gantry_velocity_controller"
# Keep the commanded acceleration inside the band the model was identified over
# (0.04 to 2.65 m/s^2), and the travel small enough not to risk the gantry's own
# limits: amplitude = target/omega^2, capped at 5 mm of excursion.
FREQUENCIES = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0)
TARGET_ACCELERATION = 1.5
AMPLITUDE_CAP = 0.005
RAMP = 1.0
DURATION = 8.0
SETTLE = 2.5          # skip the ramp and the first second of steady state
FIT = 5.0


def phasor(samples, begin, duration, omega):
    """Least squares offset + drift + one tone, returned as a - jb."""
    data = np.asarray(samples)
    window = data[(data[:, 0] >= begin) & (data[:, 0] < begin + duration)]
    assert len(window) >= 100 * duration, len(window)
    t = window[:, 0] - begin
    design = np.column_stack((np.ones(len(t)), t, np.cos(omega * t), np.sin(omega * t)))
    coefficients = np.linalg.lstsq(design, window[:, 1], rcond=None)[0]
    return complex(coefficients[2], -coefficients[3]), len(window)


def start_gantry():
    config = Path(rospkg.RosPack().get_path("sim_bringup"))/"config"/f"{GANTRY}.yaml"
    for content, namespace in rosparam.load_file(str(config)):
        rosparam.upload_params(namespace, content)
    rospy.wait_for_service("/controller_manager/load_controller", timeout=90.0)
    load = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)
    switch = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
    assert load(GANTRY).ok, "could not load the gantry controller"
    assert switch([GANTRY], [], 2, False, 5.0).ok, "could not start the gantry controller"
    rospy.loginfo("gantry velocity controller is running")


def main():
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    rospy.init_node("verify_base_response", anonymous=True)

    published, measured = [], []
    rospy.Subscriber("/gantry/base_linear_acceleration", Vector3Stamped,
                     lambda m: published.append((m.header.stamp.to_sec(), m.vector.y,
                                                 m.vector.x, m.vector.z)),
                     queue_size=200, tcp_nodelay=True)
    rospy.Subscriber("/base_pose_ground_truth", Odometry,
                     lambda m: measured.append((m.header.stamp.to_sec(),
                                                m.twist.twist.linear.y)),
                     queue_size=200, tcp_nodelay=True)

    deadline = rospy.Time.now() + rospy.Duration(90)
    while not measured and rospy.Time.now() < deadline:
        rospy.sleep(0.2)
    assert measured, "no /base_pose_ground_truth"
    start_gantry()

    rows = []
    for correction in ("on", "off"):
        for frequency in FREQUENCIES:
            omega = 2.0 * np.pi * frequency
            amplitude = min(TARGET_ACCELERATION / omega**2, AMPLITUDE_CAP)
            first = len(published)
            command = ["rosrun", "sim_bringup", "gantry_disturbance.py",
                       "_axis:=y", "_amplitude:=%.6f" % amplitude,
                       "_frequency:=%g" % frequency,
                       "_duration:=%g" % DURATION, "_ramp:=%g" % RAMP]
            if correction == "off":
                command.append("_base_response_frequency:=0")
            log = out / ("drive-%s-%gHz.log" % (correction, frequency))
            with log.open("w") as stream:
                subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                               check=True, timeout=180)
            rospy.sleep(0.5)
            assert len(published) - first > 100 * DURATION * 0.8, len(published) - first
            begin = published[first][0] + SETTLE

            commanded, count = phasor(published[first:], begin, FIT, omega)
            base_velocity, _ = phasor(measured, begin, FIT, omega)
            base_acceleration = 1j * omega * base_velocity
            # The rig is only meaningful if the gantry actually moved.
            assert abs(base_acceleration) > 0.3 * abs(commanded), (
                "the base barely moved: published %.4f, base %.4f - is the gantry "
                "controller running?" % (abs(commanded), abs(base_acceleration)))
            ratio = commanded / base_acceleration
            leak = max(max(abs(r[2]), abs(r[3])) for r in published[first:])
            rows.append(dict(correction=correction, frequency_hz=frequency,
                             amplitude_mm=1000 * amplitude,
                             published_m_s2=abs(commanded),
                             base_m_s2=abs(base_acceleration),
                             ratio=abs(ratio),
                             phase_deg=np.degrees(np.angle(ratio)),
                             samples=count, cross_axis_max=leak))
            rospy.loginfo("%s %.0f Hz: published %.4f, base %.4f, ratio %.4f, phase %+.2f deg",
                          correction, frequency, abs(commanded), abs(base_acceleration),
                          abs(ratio), np.degrees(np.angle(ratio)))

    path = out / "base-response-verification.csv"
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print()
    print("correction  f     published  base      ratio    phase")
    for row in rows:
        print("%-10s %4.0f  %8.4f  %8.4f  %.4f  %+7.3f"
              % (row["correction"], row["frequency_hz"], row["published_m_s2"],
                 row["base_m_s2"], row["ratio"], row["phase_deg"]))
    for correction in ("on", "off"):
        subset = [r for r in rows if r["correction"] == correction]
        print("%s: worst |ratio-1| %.4f, worst |phase| %.3f deg"
              % (correction, max(abs(r["ratio"] - 1) for r in subset),
                 max(abs(r["phase_deg"]) for r in subset)))
    print("ALL DONE")


if __name__ == "__main__":
    main()
