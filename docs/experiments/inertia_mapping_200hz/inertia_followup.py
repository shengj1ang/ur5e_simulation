"""Isolate the effect of inertial feedforward under the tuned Y gains."""

import csv

import numpy as np
import rospy
from rosgraph_msgs.msg import Log
from sim_cartesian_velocity_control.msg import InertiaExperimentSetting, InertiaExperimentState

from bench import ARM, JOINTS, PROFILES, append_row
from pid_tuning import FULL_CONDITIONS, VALIDATION_SEED, wave


def compare_tuned_inertia(out, run_leg, *, null, phase_balanced, pose_metrics, seed, peak_mm, axis,
                          cross_axis=""):
    index = 7 * "xyz".index(axis)
    kp, kd = (rospy.get_param(name)[index] for name in ("/K_p", "/K_d"))
    if not phase_balanced:
        assert (kp, kd) == (12.0, .35), (kp, kd)
    assert rospy.get_param("/mapping_bench_base_rate") == 200.0
    assert rospy.get_param(f"/{ARM}/ff_gravity") == 1.0
    assert rospy.get_param(f"/{ARM}/ff_coriolis") == 1.0

    logs = []
    log_subscription = rospy.Subscriber("/rosout_agg", Log, logs.append, queue_size=100)
    rng = np.random.default_rng(VALIDATION_SEED if seed is None else seed)
    frequencies = PROFILES["multisine"][0]
    name = "inertia-null" if null else "tuned-inertia"
    conditions = [(peak, duration) for peak, duration in FULL_CONDITIONS
                  if peak_mm is None or int(round(1000*peak)) == peak_mm]
    blocks = 3 if null else 6
    if phase_balanced:
        blocks = 5  # Cover all five disturbance sample phases at a 1 ms physics step.
    with (out/f"{name}-trials.csv").open("w", newline="") as trials, \
         (out/f"{name}-responses.csv").open("w", newline="") as responses:
        for block in range(1, blocks+1):
            slot = block-1 if phase_balanced else None
            settings = {peak: wave(rng, frequencies, duration, peak) for peak, duration in conditions}
            modes = ("off", "dynamic") if block % 2 else ("dynamic", "off")
            if null:
                modes = ("off", "off")
            for peak in (list(settings) if block % 2 else list(settings)[::-1]):
                omega, _, amplitude, duration = settings[peak]
                for leg, mode in enumerate(modes, 1):
                    log_index = len(logs)
                    start, b, y, b_rms, y_rms, hz, active, diag, rtf = run_leg(
                        "tuned_inertia", block, mode, *settings[peak], phase_slot=slot)
                    h = y / b
                    key = dict(block=block, leg=leg, mode=mode, peak_mm=peak*1000)
                    key.update({f"kp_{axis}": kp, f"kd_{axis}": kd})
                    if cross_axis:
                        key["cross_axis"] = cross_axis
                    if phase_balanced:
                        key["phase_slot_ms"] = slot
                    pose = pose_metrics(start, duration) if phase_balanced else {}
                    warnings = sum("joint velocity" in r.msg and "over the" in r.msg for r in logs[log_index:])
                    append_row(trials, dict(**key, fit_start_s=start, fit_duration_s=duration,
                                            base_rms_mm=1000*b_rms, tip_rms_mm=1000*y_rms,
                                            rms_transfer_pct=100*y_rms/b_rms, worst_tone_pct=100*max(abs(h)),
                                            joint_speed_warnings=warnings, base_rate_hz=hz, active_pct=100*active,
                                            torque_clip_updates=int(diag[-1,3]-diag[0,3]),
                                            offset_peak_mrad=1000*np.max(diag[:,4]), rtf=rtf, **pose))
                    for f, a, bp, yp, response in zip(frequencies, amplitude, b, y, h):
                        append_row(responses, dict(**key, frequency_hz=f, command_mm=1000*a,
                                                   base_mm=1000*abs(bp), tip_mm=1000*abs(yp),
                                                   transfer_pct=100*abs(response), phase_deg=np.angle(response, deg=True)))
                    print(f"pair {block} {peak*1000:g} mm {mode}: RMS ratio {100*y_rms/b_rms:.2f}%, "
                          f"worst {100*max(abs(h)):.2f}%, active {100*active:.1f}%, speed warnings {warnings}",
                          flush=True)
    print(f"COMPLETE: {2*len(conditions)*blocks} {name} trials", flush=True)


def compare_inertia_factors(out, run_leg, pose_metrics, pose, axis, magnitude, seed, peak_mm,
                            cross_axis=""):
    assert rospy.get_param("/mapping_bench_base_rate") == 200.0
    seed = VALIDATION_SEED if seed is None else seed
    peak_mm = 20 if peak_mm is None else peak_mm
    duration = next(duration for peak, duration in FULL_CONDITIONS if round(1000*peak) == peak_mm)
    frequencies = PROFILES["multisine"][0]
    rng = np.random.default_rng(seed)
    cells = [(0.0, False), (-magnitude, False), (magnitude, False),
             (-magnitude, True), (magnitude, True)]
    first_order = rng.permutation(len(cells))
    traces, logs = [], []
    state_subscription = rospy.Subscriber(f"/{ARM}/inertia_experiment_state", InertiaExperimentState,
                                          traces.append, queue_size=40, tcp_nodelay=True)
    log_subscription = rospy.Subscriber("/rosout_agg", Log, logs.append, queue_size=100)
    settings = rospy.Publisher(f"/{ARM}/inertia_experiment_setting", InertiaExperimentSetting,
                               latch=True, queue_size=1, tcp_nodelay=True)

    with (out/"inertia-factorial-trials.csv").open("w", newline="") as trials, \
         (out/"inertia-factorial-responses.csv").open("w", newline="") as responses, \
         (out/"inertia-factorial-telemetry.csv").open("w", newline="") as telemetry:
        trace_writer = csv.writer(telemetry)
        trace_writer.writerow(["trial_id", "stamp_s", "model_valid", "sigma_min", "raw_axis_offset_m",
                               "mapped_axis_offset_m", *[f"{stage}_{joint}" for stage in
                               ("raw_torque_nm", "mapped_torque_nm", "requested_torque_nm") for joint in JOINTS]])
        for block in range(1, 6):
            signal = wave(rng, frequencies, duration, peak_mm/1000)
            # Each cell uses each position in the order once across the five phase slots.
            order = np.roll(first_order, block-1)
            for leg, cell in enumerate(order, 1):
                trial_id = (block-1)*len(cells) + leg
                gain, project = cells[cell]
                settings.publish(InertiaExperimentSetting(trial_id=trial_id, gain=gain, project_translation=project))
                for _ in range(80):
                    if traces and traces[-1].setting.trial_id == trial_id:
                        break
                    rospy.sleep(.025)
                else:
                    raise RuntimeError(f"No stationary acknowledgement for trial {trial_id}")

                trace_start, log_start = len(traces), len(logs)
                mode = "dynamic" if gain else "off"
                start, base, tip, base_rms, tip_rms, rate, active, diag, rtf = run_leg(
                    "inertia_factorial", block, mode, *signal, phase_slot=block-1, keep_acceleration=True)
                samples = [m for m in traces[trace_start:] if start <= m.header.stamp.to_sec() < start+duration]
                assert len(samples) >= 90*duration, (trial_id, len(samples))
                assert all(m.setting.trial_id == trial_id and m.setting.gain == gain and
                           m.setting.project_translation == project for m in samples), trial_id
                trace_writer.writerows((trial_id, m.header.stamp.to_sec(), int(m.model_valid), m.sigma_min,
                                        m.raw_axis_offset, m.mapped_axis_offset, *m.raw_torque,
                                        *m.mapped_torque, *m.requested_torque) for m in samples)
                telemetry.flush()
                # Summarize the telemetry per trial as well. The per-sample file is
                # the raw record and stays with the recording; these columns are what
                # the analysis and the report quote, so the trials file carries them.
                viable = [m for m in samples if m.model_valid and abs(m.raw_axis_offset) > 1e-8]
                verification = dict(
                    diag_samples=len(samples),
                    model_valid_samples=sum(m.model_valid for m in samples),
                    axis_samples=len(viable),
                    axis_sign_reversals=sum(m.raw_axis_offset*m.mapped_axis_offset < 0 for m in viable),
                    max_axis_gain_error=max((abs(m.mapped_axis_offset/m.raw_axis_offset - 1)
                                             for m in viable), default=0.0),
                    max_request_error_nm=max((abs(requested - gain*mapped) for m in samples for mapped, requested
                                              in zip(m.mapped_torque, m.requested_torque)), default=0.0))
                response = tip/base
                key = dict(pose=pose, axis=axis, cross_axis=cross_axis, seed=seed,
                           peak_mm=peak_mm, block=block, leg=leg,
                           trial_id=trial_id, gain=gain, project_translation=int(project), phase_slot_ms=block-1)
                warnings = sum("joint velocity" in r.msg and "over the" in r.msg for r in logs[log_start:])
                append_row(trials, dict(**key, fit_start_s=start, fit_duration_s=duration,
                                        base_rms_mm=1000*base_rms, tip_rms_mm=1000*tip_rms,
                                        rms_transfer_pct=100*tip_rms/base_rms, worst_tone_pct=100*max(abs(response)),
                                        base_rate_hz=rate, active_pct=100*active,
                                        clip_updates=int(diag[-1,3]-diag[0,3]), joint_speed_warnings=warnings,
                                        sigma_min=min(m.sigma_min for m in samples), rtf=rtf,
                                        **pose_metrics(start, duration), **verification))
                for f, b, y, h in zip(frequencies, base, tip, response):
                    append_row(responses, dict(**key, frequency_hz=f, base_mm=1000*abs(b), tip_mm=1000*abs(y),
                                               transfer_pct=100*abs(h), phase_deg=np.angle(h, deg=True)))
                print(f"block {block}/5 gain {gain:+g}, projection {project}: "
                      f"RMS transfer {100*tip_rms/base_rms:.2f}%, active {100*active:.1f}%", flush=True)
    print("COMPLETE: 25 factorial trials", flush=True)
