"""Isolate the effect of inertial feedforward under the tuned Y gains."""

import numpy as np
import rospy
from rosgraph_msgs.msg import Log

from bench import ARM, PROFILES, append_row
from pid_tuning import FULL_CONDITIONS, VALIDATION_SEED, wave


def compare_tuned_inertia(out, run_leg):
    kp, kd = (rospy.get_param(name)[7] for name in ("/K_p", "/K_d"))
    assert (kp, kd) == (12.0, .35), (kp, kd)
    assert rospy.get_param("/mapping_bench_base_rate") == 200.0
    assert rospy.get_param(f"/{ARM}/ff_gravity") == 1.0
    assert rospy.get_param(f"/{ARM}/ff_coriolis") == 1.0

    logs = []
    log_subscription = rospy.Subscriber("/rosout_agg", Log, logs.append, queue_size=100)
    rng = np.random.default_rng(VALIDATION_SEED)
    frequencies = PROFILES["multisine"][0]
    with (out/"tuned-inertia-trials.csv").open("w", newline="") as trials, \
         (out/"tuned-inertia-responses.csv").open("w", newline="") as responses:
        for block in range(1, 7):
            settings = {peak: wave(rng, frequencies, duration, peak) for peak, duration in FULL_CONDITIONS}
            modes = ("off", "dynamic") if block % 2 else ("dynamic", "off")
            for peak in (list(settings) if block % 2 else list(settings)[::-1]):
                omega, _, amplitude, duration = settings[peak]
                for leg, mode in enumerate(modes, 1):
                    log_index = len(logs)
                    start, b, y, b_rms, y_rms, hz, active, diag, rtf = run_leg(
                        "tuned_inertia", block, mode, *settings[peak])
                    h = y / b
                    key = dict(block=block, leg=leg, mode=mode, kp_y=kp, kd_y=kd, peak_mm=peak*1000)
                    warnings = sum("joint velocity" in r.msg and "over the" in r.msg for r in logs[log_index:])
                    append_row(trials, dict(**key, fit_start_s=start, fit_duration_s=duration,
                                            base_rms_mm=1000*b_rms, tip_rms_mm=1000*y_rms,
                                            rms_transfer_pct=100*y_rms/b_rms, worst_tone_pct=100*max(abs(h)),
                                            joint_speed_warnings=warnings, base_rate_hz=hz, active_pct=100*active,
                                            torque_clip_updates=int(diag[-1,3]-diag[0,3]),
                                            offset_peak_mrad=1000*np.max(diag[:,4]), rtf=rtf))
                    for f, a, bp, yp, response in zip(frequencies, amplitude, b, y, h):
                        append_row(responses, dict(**key, frequency_hz=f, command_mm=1000*a,
                                                   base_mm=1000*abs(bp), tip_mm=1000*abs(yp),
                                                   transfer_pct=100*abs(response), phase_deg=np.angle(response, deg=True)))
                    print(f"pair {block} {peak*1000:g} mm {mode}: RMS ratio {100*y_rms/b_rms:.2f}%, "
                          f"worst {100*max(abs(h)):.2f}%, active {100*active:.1f}%, speed warnings {warnings}",
                          flush=True)
    print("COMPLETE: 24 tuned-PID inertia trials", flush=True)
