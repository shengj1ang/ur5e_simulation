#!/usr/bin/env python3
"""Run paired world-hold disturbance trials in one isolated Gazebo bringup."""

import argparse
import csv
from itertools import permutations
from pathlib import Path
import time

import actionlib
import numpy as np
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import LoadController, SwitchController
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sim_cartesian_velocity_control.msg import InertialFeedforwardState
from std_msgs.msg import Bool, Float64MultiArray
from tf.transformations import quaternion_matrix
from trajectory_msgs.msg import JointTrajectoryPoint


ARM = "sim_cartesian_velocity_controller_sim"
GANTRY = "gantry_velocity_controller"
TRAJECTORY = "arm_joint_controller"
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
MODES = ("off", "static", "dynamic")
RAMP = 1.0
FIT_DELAY = 1.5
PROFILES = {
    "single_3hz": (np.array([3.0]), 4.0),
    "single_5hz": (np.array([5.0]), 4.0),
    "multisine": (np.array([0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 6, 7, 8]), 8.0),
}


def window(t, steady_end):
    """Return a quintic position envelope and its two time derivatives."""
    if RAMP <= t <= steady_end:
        return 1.0, 0.0, 0.0
    u = min(t, steady_end + RAMP - t) / RAMP
    direction = 1 if t < RAMP else -1
    envelope = u**3 * (10 - 15*u + 6*u*u)
    velocity = direction * 30*u*u*(1-u)**2 / RAMP
    acceleration = 60*u*(1-u)*(1-2*u) / RAMP**2
    return envelope, velocity, acceleration


def fit(buffer, begin, duration, omega, minimum_samples):
    samples = np.asarray(buffer)
    segment = samples[(samples[:, 0] >= begin) & (samples[:, 0] < begin+duration)]
    assert len(segment) >= minimum_samples * duration, len(segment)
    t, y = segment[:, 0]-begin, segment[:, 1]
    angles = t[:, None] * omega
    design = np.column_stack((np.ones(len(t)), t, np.cos(angles), np.sin(angles)))
    coefficients = np.linalg.lstsq(design, y, rcond=None)[0]
    n = len(omega)
    phasors = coefficients[2:2+n] - 1j*coefficients[2+n:]
    detrended = y - design[:, :2] @ coefficients[:2]
    rms = np.sqrt(np.trapz(detrended**2, t) / (t[-1]-t[0]))
    return phasors, rms, 1/np.median(np.diff(t))


def append_row(stream, row):
    writer = csv.DictWriter(stream, fieldnames=list(row))
    if stream.tell() == 0:
        writer.writeheader()
    writer.writerow(row)
    stream.flush()


def main(out, tune, validate_gains):
    rospy.init_node("ur5e_mapping_200hz")
    assert rospy.get_param("/regulate_in_world") is True
    assert rospy.get_param(f"/{ARM}/inertia_feedforward") is True
    assert rospy.get_param(f"/{ARM}/inertia_mapping_experiment") is True
    configured_rate = rospy.get_param("/mapping_bench_base_rate")
    base, tip, inertia = [], [], []
    base_pose = None

    def read_base(msg):
        nonlocal base_pose
        base_pose = msg.pose.pose
        base.append((msg.header.stamp.to_sec(), msg.pose.pose.position.y))

    def read_tip(msg):
        tip.append((msg.header.stamp.to_sec(), msg.pose.pose.position.y))

    def read_inertia(msg):
        inertia.append((msg.header.stamp.to_sec(), msg.active, msg.static_mapping,
                        sum(msg.torque_clip_count), max(abs(x) for x in msg.inertial_offset)))

    subscribers = [
        rospy.Subscriber("/base_pose_ground_truth", Odometry, read_base, queue_size=20, tcp_nodelay=True),
        rospy.Subscriber("/ee_pose_ground_truth", Odometry, read_tip, queue_size=20, tcp_nodelay=True),
        rospy.Subscriber(f"/{ARM}/inertial_feedforward_state", InertialFeedforwardState,
                         read_inertia, queue_size=20, tcp_nodelay=True),
    ]
    command = rospy.Publisher(f"/{GANTRY}/command", Float64MultiArray, queue_size=1, tcp_nodelay=True)
    acceleration = rospy.Publisher("/gantry/base_linear_acceleration", Vector3Stamped,
                                   queue_size=1, tcp_nodelay=True)
    mapping = rospy.Publisher(f"/{ARM}/use_static_inertia_mapping", Bool,
                              latch=True, queue_size=1, tcp_nodelay=True)
    hold = rospy.Publisher("/now_equilibrium", PoseStamped, queue_size=1)
    rospy.on_shutdown(lambda: command.publish(Float64MultiArray(data=[0.0]*4)))
    switch_name = "/controller_manager/switch_controller"
    rospy.wait_for_service(switch_name, timeout=40.0)
    switch = rospy.ServiceProxy(switch_name, SwitchController)
    # Start the gantry here once the launch's spawners have loaded both arm controllers.
    client = actionlib.SimpleActionClient(f"/{TRAJECTORY}/follow_joint_trajectory", FollowJointTrajectoryAction)
    assert client.wait_for_server(rospy.Duration(30))
    rospy.wait_for_service("/controller_manager/load_controller", timeout=10.0)
    load = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)
    assert load(GANTRY).ok
    assert switch([GANTRY], [], 2, False, 5.0).ok
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = JOINTS
    goal.trajectory.points = [JointTrajectoryPoint(positions=[0., -1.57, 1.57, 0., 1.57, 1.57],
                                                   time_from_start=rospy.Duration(3.0))]
    goal.goal_time_tolerance = rospy.Duration(2.0)
    client.send_goal(goal)
    assert client.wait_for_result(rospy.Duration(8.0)) and client.get_state() == 3, client.get_result()
    assert switch([ARM], [TRAJECTORY], 2, False, 5.0).ok
    for _ in range(100):
        if all(pub.get_num_connections() for pub in (command, acceleration, mapping, hold)) and base:
            break
        rospy.sleep(.05)
    assert all(pub.get_num_connections() for pub in (command, acceleration, mapping, hold)) and base
    rospy.sleep(1.0)
    state = rospy.wait_for_message(f"/{ARM}/ee_state", Odometry, timeout=10)
    target = PoseStamped(header=state.header, pose=state.pose.pose)
    target.header.frame_id = "pid"
    hold.publish(target)
    rospy.sleep(1.5)
    print(f"READY: native {configured_rate:g} Hz p3d, one world-frame target", flush=True)

    def select(mode):
        use_static = mode == "static"
        mapping.publish(Bool(data=use_static))
        for _ in range(80):
            if inertia and not inertia[-1][1] and inertia[-1][2] == use_static and inertia[-1][4] < 1e-5:
                return
            rospy.sleep(.025)
        raise RuntimeError(f"Stationary mapping switch to {mode} did not complete")

    def run_leg(profile, block, mode, omega, phase, amplitude, duration):
        select(mode)
        indices = tuple(map(len, (base, tip, inertia)))
        steady_end = FIT_DELAY + duration + .1
        start = rospy.Time.now()
        start_wall = time.monotonic()
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            t = (now-start).to_sec()
            if t >= steady_end+RAMP:
                break
            env, d_env, dd_env = window(t, steady_end)
            angles = omega*t + phase
            x = amplitude * np.sin(angles)
            v = amplitude*omega * np.cos(angles)
            a = -(omega**2)*x
            command.publish(Float64MultiArray(data=[0.0, env*v.sum()+d_env*x.sum(), 0.0, 0.0]))
            if mode != "off":
                msg = Vector3Stamped()
                msg.header.stamp, msg.header.frame_id = now, "base_link"
                acc = env*a.sum() + 2*d_env*v.sum() + dd_env*x.sum()
                q = base_pose.orientation
                world_y_in_base = quaternion_matrix([q.x, q.y, q.z, q.w])[1, :3]
                msg.vector.x, msg.vector.y, msg.vector.z = acc * world_y_in_base
                acceleration.publish(msg)
            rate.sleep()
        command.publish(Float64MultiArray(data=[0.0]*4))
        rtf = (rospy.Time.now()-start).to_sec() / (time.monotonic()-start_wall)
        rospy.sleep(.50)
        begin = start.to_sec()+FIT_DELAY
        p_base, base_rms, base_rate = fit(base[indices[0]:], begin, duration, omega, .875*configured_rate)
        p_tip, tip_rms, _ = fit(tip[indices[1]:], begin, duration, omega, 42)
        diag = np.asarray(inertia[indices[2]:])
        diag = diag[(diag[:, 0] >= begin) & (diag[:, 0] < begin+duration)]
        assert len(diag) >= 90*duration and abs(base_rate-configured_rate) < 1, (profile, mode, base_rate)
        assert np.all(diag[:, 2] == (mode == "static")), (profile, mode)
        active = np.mean(diag[:, 1])
        assert active == 0 if mode == "off" else active > .4, (profile, mode, active)
        assert np.all((.65*amplitude <= abs(p_base)) & (abs(p_base) <= 1.35*amplitude)), (
            profile, mode, abs(p_base)/amplitude)
        return begin, p_base, p_tip, base_rms, tip_rms, base_rate, active, diag, rtf

    if tune or validate_gains:
        from pid_tuning import tune_gains
        tune_gains(out, run_leg, hold, target, lambda: base_pose, tip, validate_gains)
        return

    rng = np.random.default_rng(440200)
    center_start = np.median([p[1] for p in base[-60:]])
    with (out/"trials.csv").open("w", newline="") as trials, (out/"responses.csv").open("w", newline="") as responses:
        for block, order in enumerate(permutations(MODES), 1):
            profiles = list(PROFILES)
            shift = (block-1) % len(profiles)
            for profile in profiles[shift:] + profiles[:shift]:
                frequency, duration = PROFILES[profile]
                omega = 2*np.pi*frequency
                phase = rng.uniform(-np.pi, np.pi, len(omega))
                amplitude = 1/(1+(frequency/4)**2)
                grid = np.linspace(0, duration, 8000)
                target_peak = np.max(np.abs(np.sin(grid[:, None]*omega+phase) @ amplitude))
                amplitude = .020*amplitude / target_peak
                for leg, mode in enumerate(order, 1):
                    begin, b, y, b_rms, y_rms, hz, active, diag, rtf = run_leg(
                        profile, block, mode, omega, phase, amplitude, duration)
                    h = y/b
                    key = dict(profile=profile, block=block, leg=leg, mode=mode)
                    append_row(trials, dict(**key, fit_start_s=begin, fit_duration_s=duration,
                                            base_rms_mm=1000*b_rms, tip_rms_mm=1000*y_rms,
                                            rms_transfer_pct=100*y_rms/b_rms, base_rate_hz=hz,
                                            active_pct=100*active, clip_updates=int(diag[-1,3]-diag[0,3]),
                                            offset_peak_mrad=1000*np.max(diag[:,4]), rtf=rtf))
                    for f, a, bp, yp, response in zip(frequency, amplitude, b, y, h):
                        append_row(responses, dict(**key, frequency_hz=f, command_mm=1000*a,
                                                   base_mm=1000*abs(bp), tip_mm=1000*abs(yp),
                                                   transfer_pct=100*abs(response),
                                                   phase_deg=np.angle(response, deg=True)))
                    print(f"{profile} {block}/6 {mode}: RMS ratio {100*y_rms/b_rms:.2f}%, "
                          f"model active {100*active:.0f}%, real-time factor {rtf:.2f}", flush=True)
    center_end = np.median([p[1] for p in base[-60:]])
    print(f"COMPLETE: 54 trials, net base travel {1000*(center_end-center_start):.2f} mm", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    suite = parser.add_mutually_exclusive_group()
    suite.add_argument("--tune", action="store_true")
    suite.add_argument("--validate-gains", type=float, nargs=2, metavar=("KP_Y", "KD_Y"))
    args = parser.parse_args()
    main(args.output, args.tune, args.validate_gains)
