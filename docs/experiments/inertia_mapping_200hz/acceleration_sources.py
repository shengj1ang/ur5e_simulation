"""Compare three inertial feedforward inputs with the same base motion."""

import math
from itertools import permutations

import numpy as np
import rospy
from rosgraph_msgs.msg import Log

from bench import PROFILES, append_row
from pid_tuning import wave
from analyze_acceleration_sources import motion_metrics


class VelocityAcceleration:
    """Causal derivative of the three newest velocity samples, with a 35 Hz low pass."""

    def __init__(self):
        self.previous = None
        self.slope = None
        self.output = 0.

    def update(self, stamp, velocity):
        previous = self.previous
        if previous is None:
            self.previous = (stamp, velocity)
            return None
        dt = stamp-previous[0]
        if dt <= 0:
            return None
        self.previous = (stamp, velocity)
        if not .002 <= dt <= .020:
            self.slope = None
            self.output = 0.
            return None
        slope = (velocity-previous[1])/dt
        if self.slope is None:
            self.slope = (slope, dt)
            return None
        old_slope, old_dt = self.slope
        self.slope = (slope, dt)
        derivative = slope + dt/(dt+old_dt)*(slope-old_slope)
        self.output += -math.expm1(-2*math.pi*35*dt)*(derivative-self.output)
        return self.output


def displacement_pulse(amplitude, rise, dwell):
    """One smooth excursion from rest followed by a smooth return."""
    def evaluate(t):
        if 0 <= t < rise:
            u, sign = t/rise, 1.
        elif rise+dwell <= t < 2*rise+dwell:
            u, sign = (t-rise-dwell)/rise, -1.
        else:
            return 0., 0.
        return (sign*amplitude*30*u*u*(1-u)**2/rise,
                sign*amplitude*60*u*(1-u)*(1-2*u)/rise**2)
    return evaluate


def compare_sources(out, run_leg, base, tip):
    assert tuple(rospy.get_param(k)[7] for k in ("/K_p", "/K_d")) == (12.0, .35)
    rng = np.random.default_rng(307220)
    logs = []
    log_subscription = rospy.Subscriber("/rosout_agg", Log, logs.append, queue_size=100)
    frequencies = PROFILES["multisine"][0]
    with (out/"acceleration-source-trials.csv").open("w", newline="") as results:
        for block, order in enumerate(permutations(("command", "model", "measured")), 1):
            multisine = wave(rng, frequencies, 4., .030)
            rise = .17 + .017*(block-1)
            dwell = .24 if block % 2 else .32
            pulse = displacement_pulse(.030 * (-1)**(block-1), rise, dwell)
            conditions = [("multisine", multisine, None),
                          ("pulse", (np.empty(0), np.empty(0), np.empty(0), 2.5), pulse)]
            for profile, settings, event in (conditions if block % 2 else conditions[::-1]):
                for source in order:
                    log_index = len(logs)
                    start, b, e, b_rms, e_rms, hz, active, diag, rtf = run_leg(
                        profile, block, "dynamic", *settings, source=source, pulse=event)
                    duration = settings[3]
                    base_event_rms, base_peak, _ = motion_metrics(base, start, duration, settings[0]/(2*np.pi))
                    tip_event_rms, tip_peak, noise = motion_metrics(tip, start, duration, settings[0]/(2*np.pi))
                    assert active >= .9 and 24 <= base_peak <= 39, (profile, block, source, active, base_peak)
                    if profile == "pulse":
                        b_rms, e_rms = base_event_rms/1000, tip_event_rms/1000
                    warnings = sum("joint velocity" in row.msg and "over the" in row.msg for row in logs[log_index:])
                    row = dict(profile=profile, block=block, source=source, fit_start_s=start,
                               fit_duration_s=duration, base_rate_hz=hz, base_rms_mm=1000*b_rms,
                               tip_rms_mm=1000*e_rms, rms_transfer_pct=100*e_rms/b_rms,
                               base_peak_mm=base_peak, tip_peak_mm=tip_peak, idle_rms_mm=noise,
                               active_pct=100*active, offset_peak_mrad=1000*np.max(diag[:,4]),
                               clip_updates=int(diag[-1,3]-diag[0,3]), joint_speed_warnings=warnings, rtf=rtf)
                    append_row(results, row)
                    print(f"{profile} {block}/6 {source}: RMS {row['tip_rms_mm']:.3f} mm, "
                          f"peak {tip_peak:.3f} mm, active {100*active:.1f}%, caps {warnings}", flush=True)
    print("COMPLETE: 36 paired acceleration source trials", flush=True)
