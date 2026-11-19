"""Screen and validate Y-axis PID gains without restarting Gazebo or the joint controller."""

from itertools import product
import signal
import subprocess
import time

import numpy as np
import rosnode
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from rosgraph_msgs.msg import Log
from scipy.stats import t
from tf.transformations import quaternion_inverse, quaternion_matrix, quaternion_multiply

from bench import ARM, PROFILES, append_row, fit


def pose_components(pose):
    p, q = pose.position, pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])


def wave(rng, frequencies, duration, peak):
    omega = 2*np.pi*frequencies
    phase = rng.uniform(-np.pi, np.pi, len(omega))
    weights = 1/(1+(frequencies/4)**2)
    grid = np.linspace(0, duration, 8000)
    maximum = np.max(np.abs(np.sin(grid[:, None]*omega+phase) @ weights))
    return omega, phase, peak*weights/maximum, duration


def tune_gains(out, run_leg, hold, initial_target, current_base, tip, validate_gains=None):
    p, q = pose_components(current_base())
    goal_p, goal_q = pose_components(initial_target.pose)
    world_p = p + quaternion_matrix(q)[:3, :3] @ goal_p
    world_q = quaternion_multiply(q, goal_q)
    kp, kd = [rospy.get_param(name) for name in ("/K_p", "/K_d")]
    baseline = (7.7, .10)
    candidates = [baseline, *product((7.7, 12.0, 17.7), (.05, .20, .35))]
    pid_node = "/sim_pid_controller"
    command = rospy.Publisher(f"/{ARM}/command_cart_vel", Twist, queue_size=1, tcp_nodelay=True)
    logs = []
    log_subscription = rospy.Subscriber("/rosout_agg", Log, logs.append, queue_size=100)
    process = None

    def stop_pid():
        nonlocal process
        if process is None:
            rosnode.kill_nodes([pid_node])
        else:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=8)
            process = None
        for _ in range(3):
            command.publish(Twist())
            rospy.sleep(.015)

    def set_gains(gains, log):
        nonlocal process
        stop_pid()
        kp[7], kd[7] = gains
        rospy.set_param("/K_p", kp)
        rospy.set_param("/K_d", kd)
        first = len(logs)
        process = subprocess.Popen(
            ["rosrun", "sim_pid_control", "pid_controller_node", "__name:="+pid_node[1:]],
            stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            if any(r.name == pid_node and "Running the pid control loop" in r.msg for r in logs[first:]):
                break
            assert process.poll() is None, "PID process exited during initialization"
            rospy.sleep(.035)
        else:
            raise RuntimeError("PID did not enter its control loop")
        assert hold.get_num_connections() > 0
        bp, bq = pose_components(current_base())
        goal = PoseStamped()
        goal.header.stamp, goal.header.frame_id = rospy.Time.now(), "pid"
        target_translation = quaternion_matrix(bq)[:3, :3].T @ (world_p - bp)
        target_orientation = quaternion_multiply(quaternion_inverse(bq), world_q)
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = target_translation
        (goal.pose.orientation.x, goal.pose.orientation.y,
         goal.pose.orientation.z, goal.pose.orientation.w) = target_orientation
        hold.publish(goal)
        rospy.sleep(1.5)
        idle_begin = rospy.Time.now().to_sec()
        idle_index = len(tip)
        rospy.sleep(2.55)
        _, idle_rms, _ = fit(tip[idle_index:], idle_begin+.03, 2.5, np.empty(0), 42)
        return 1000*idle_rms

    def measure(stage, block, gains, peak, signal_settings, idle):
        omega, _, amplitude, duration = signal_settings
        log_index = len(logs)
        start, b, y, b_rms, y_rms, hz, active, diag, rtf = run_leg(
            stage, block, "dynamic", *signal_settings)
        assert process.poll() is None
        h = y/b
        speed_warnings = sum("joint velocity" in r.msg and "over the" in r.msg for r in logs[log_index:])
        result = dict(stage=stage, block=block, kp_y=gains[0], kd_y=gains[1], peak_mm=1000*peak,
                      fit_start_s=start, fit_duration_s=duration, base_rms_mm=1000*b_rms,
                      tip_rms_mm=1000*y_rms, rms_transfer_pct=100*y_rms/b_rms,
                      worst_tone_pct=100*max(abs(h)), worst_tone_hz=omega[np.argmax(abs(h))]/(2*np.pi),
                      idle_rms_mm=idle, joint_speed_warnings=speed_warnings, base_rate_hz=hz,
                      active_pct=100*active, torque_clip_updates=int(diag[-1,3]-diag[0,3]), rtf=rtf)
        append_row(trials, result)
        for f, a, base_phasor, tip_phasor, response in zip(omega/(2*np.pi), amplitude, b, y, h):
            append_row(responses, dict(stage=stage, block=block, kp_y=gains[0], kd_y=gains[1],
                                       peak_mm=1000*peak, frequency_hz=f, command_mm=1000*a,
                                       base_mm=1000*abs(base_phasor), tip_mm=1000*abs(tip_phasor),
                                       transfer_pct=100*abs(response), phase_deg=np.angle(response, deg=True)))
        print(f"{stage} {block}: {gains}, input {1000*peak:g} mm, "
              f"RMS {result['rms_transfer_pct']:.2f}%, worst {result['worst_tone_pct']:.2f}%, "
              f"idle {idle:.5f} mm, speed warnings {speed_warnings}", flush=True)
        return result

    def average(rows, key):
        return np.mean([r[key] for r in rows])

    freqs = PROFILES["multisine"][0]
    rng = np.random.default_rng(611754)
    results = {g: [] for g in candidates}
    with (out/"pid-gain-trials.csv").open("w", newline="") as trials, \
         (out/"pid-gain-responses.csv").open("w", newline="") as responses, \
         (out/"pid-process.log").open("w") as log:
        try:
            if validate_gains:
                winner = tuple(validate_gains)
                assert winner != baseline
            else:
                # Reverse the order on a second waveform to balance slow drift.
                order = [candidates[i] for i in rng.permutation(len(candidates))]
                for block, ordered in enumerate((order, order[::-1]), 1):
                    settings = wave(rng, freqs, 6.0, .020)
                    for gains in ordered:
                        idle = set_gains(gains, log)
                        results[gains].append(measure("screen", block, gains, .020, settings, idle))
                base_rows = results[baseline]
                eligible = [g for g, rows in results.items()
                            if average(rows, "rms_transfer_pct") < average(base_rows, "rms_transfer_pct")
                            and average(rows, "worst_tone_pct") < average(base_rows, "worst_tone_pct")
                            and average(rows, "idle_rms_mm") <= max(.010, 2*average(base_rows, "idle_rms_mm"))
                            and sum(r["joint_speed_warnings"] for r in rows) <=
                            sum(r["joint_speed_warnings"] for r in base_rows)
                            and not any(r["torque_clip_updates"] for r in rows)]
                if not eligible:
                    print("COMPLETE: no screened gain improved both disturbance metrics", flush=True)
                    return
                winner = min(eligible, key=lambda g: average(results[g], "rms_transfer_pct"))
                print(f"SELECTED for independent validation: {winner}, baseline {baseline}", flush=True)
            conditions = [(0.020, 6.0)] if validate_gains else [(0.020, 8.0), (0.030, 4.0)]
            validation_rng = np.random.default_rng(451195 if validate_gains else 412195)
            paired = {peak: {g: [] for g in (baseline, winner)} for peak, _ in conditions}
            blocks = 4 if validate_gains else 6
            for block in range(1, blocks+1):
                settings = {peak: wave(validation_rng, freqs, duration, peak)
                            for peak, duration in conditions}
                for gains in ((baseline, winner) if block % 2 else (winner, baseline)):
                    idle = set_gains(gains, log)
                    order = list(settings)
                    for peak in (order if block % 2 else order[::-1]):
                        paired[peak][gains].append(measure("validation", block, gains, peak, settings[peak], idle))
            for peak, group in paired.items():
                for key in ("rms_transfer_pct", "worst_tone_pct", "idle_rms_mm"):
                    diffs = np.array([x[key]-y[key] for x, y in zip(group[baseline], group[winner])])
                    width = t.ppf(.975, len(diffs)-1)*diffs.std(ddof=1)/np.sqrt(len(diffs))
                    print(f"{peak*1000:g} mm {key}: {average(group[baseline], key):.6f} to "
                          f"{average(group[winner], key):.6f}, delta {diffs.mean():+.6f} "
                          f"[{diffs.mean()-width:+.6f}, {diffs.mean()+width:+.6f}]", flush=True)
            print(f"COMPLETE: baseline {baseline}, candidate {winner}", flush=True)
        finally:
            stop_pid()
