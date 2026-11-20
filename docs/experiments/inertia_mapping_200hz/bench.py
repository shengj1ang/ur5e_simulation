#!/usr/bin/env python3
"""Run paired world-hold disturbance trials in one isolated Gazebo bringup."""

import argparse
import csv
import importlib.util
from itertools import permutations
import os
from pathlib import Path
import time
from xml.etree import ElementTree

import actionlib
import numpy as np
import rospkg
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


def load_base_response():
    """The gantry velocity-loop correction, as the bringup node applies it.

    Imported from gantry_disturbance.py rather than copied, so the model has
    one home. UR5E_BENCH_BASE_RESPONSE=off publishes the commanded
    acceleration instead, which is what the runs recorded here were measured
    with and what reproduces them.
    """
    if os.environ.get("UR5E_BENCH_BASE_RESPONSE", "on") == "off":
        return None
    path = Path(rospkg.RosPack().get_path("sim_bringup"))/"scripts"/"gantry_disturbance.py"
    spec = importlib.util.spec_from_file_location("gantry_disturbance", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return lambda: module.BaseResponse(module.BASE_RESPONSE_FREQUENCY,
                                       module.BASE_RESPONSE_DAMPING)


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


def pose_positions(name):
    package = Path(rospkg.RosPack().get_path("sim_moveit_config"))
    srdf = ElementTree.parse(package/"config"/"sim.srdf")
    pose = srdf.find(f"./group_state[@name='{name}'][@group='manipulator']")
    if pose is None:
        raise ValueError(f"Unknown manipulator pose: {name}")
    joints = {joint.attrib["name"]: float(joint.attrib["value"]) for joint in pose.findall("joint")}
    return [joints[name] for name in JOINTS]


def main(out, args):
    rospy.init_node("ur5e_mapping_200hz")
    assert rospy.get_param("/regulate_in_world") is True
    assert rospy.get_param(f"/{ARM}/inertia_feedforward") is True
    assert rospy.get_param(f"/{ARM}/inertia_mapping_experiment") is True
    configured_rate = rospy.get_param("/mapping_bench_base_rate")
    phase_balanced = (args.inertia_phase_balanced or args.inertia_factorial_gain is not None
                      or args.compare_sensor_inputs)
    base, tip, inertia = [], [], []
    tip_world = []
    base_pose = None
    axis = "xyz".index(args.axis)
    # The disturbance direction, in gantry axes. A cross axis drives a second
    # one with the same waveform, which takes the base acceleration out of the
    # single-axis domain the inertia weights were identified in and so out of
    # the controller's alignment gate. unit is that direction normalised, for
    # projecting the base and tip traces onto it; scale is how much longer the
    # combined stroke is than one axis', which the amplitude check carries.
    weights = np.zeros(3)
    weights[axis] = 1.0
    if args.cross_axis:
        name, colon, fraction = args.cross_axis.partition(":")
        assert colon and len(name) == 1 and name in "xyz" and name != args.axis, args.cross_axis
        value = float(fraction)
        assert np.isfinite(value) and value != 0.0, args.cross_axis
        weights["xyz".index(name)] = value
    scale = float(np.linalg.norm(weights))
    unit = weights / scale
    measured_acceleration = None
    imu_acceleration = None
    if args.compare_acceleration_sources or args.compare_sensor_inputs:
        from acceleration_sources import VelocityAcceleration
        estimator = VelocityAcceleration()

    def read_base(msg):
        nonlocal base_pose, measured_acceleration
        base_pose = msg.pose.pose
        place = base_pose.position
        base.append((msg.header.stamp.to_sec(), float(unit @ (place.x, place.y, place.z))))
        if args.compare_acceleration_sources or args.compare_sensor_inputs:
            velocity = msg.twist.twist.linear
            value = estimator.update(msg.header.stamp.to_sec(), float(unit @ (velocity.x, velocity.y, velocity.z)))
            measured_acceleration = (msg.header.stamp, value, base_pose.orientation) if value is not None else None

    def read_tip(msg):
        stamp = msg.header.stamp.to_sec()
        position, orientation = msg.pose.pose.position, msg.pose.pose.orientation
        tip.append((stamp, float(unit @ (position.x, position.y, position.z))))
        if phase_balanced:
            tip_world.append((stamp, position.x, position.y, position.z, orientation.x,
                              orientation.y, orientation.z, orientation.w))

    def read_inertia(msg):
        inertia.append((msg.header.stamp.to_sec(), msg.active, msg.static_mapping,
                        sum(msg.torque_clip_count), max(abs(x) for x in msg.inertial_offset)))

    subscribers = [
        rospy.Subscriber("/base_pose_ground_truth", Odometry, read_base, queue_size=20, tcp_nodelay=True),
        rospy.Subscriber("/ee_pose_ground_truth", Odometry, read_tip, queue_size=20, tcp_nodelay=True),
        rospy.Subscriber(f"/{ARM}/inertial_feedforward_state", InertialFeedforwardState,
                         read_inertia, queue_size=20, tcp_nodelay=True),
    ]
    if args.compare_sensor_inputs:
        from sensor_msgs.msg import Imu
        from acceleration_sources import BaseImuAcceleration
        imu_estimator = BaseImuAcceleration(unit)

        def read_base_imu(msg):
            nonlocal imu_acceleration
            imu_acceleration = imu_estimator.update(msg)

        subscribers.append(rospy.Subscriber("/mapping_bench/base_imu", Imu, read_base_imu,
                                            queue_size=20, tcp_nodelay=True))
    command = rospy.Publisher(f"/{GANTRY}/command", Float64MultiArray, queue_size=1, tcp_nodelay=True)
    acceleration = rospy.Publisher("/gantry/base_linear_acceleration", Vector3Stamped,
                                   queue_size=1, tcp_nodelay=True)
    base_response = load_base_response()
    rospy.loginfo("bench: base response correction %s",
                  "on" if base_response else "off")
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
    positions = pose_positions(args.pose)
    if args.pose_fraction is not None:
        endpoint = pose_positions("1st_reco_start")
        positions = [q + args.pose_fraction * (p-q) for q, p in zip(positions, endpoint)]
    goal.trajectory.points = [JointTrajectoryPoint(positions=positions,
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
    if phase_balanced:
        assert tip_world, "No world-frame tool pose received before starting the trials"
        tip_reference = np.array(tip_world[-1][1:])
    disturbance = args.axis.upper() + (f"+{args.cross_axis.upper()}" if args.cross_axis else "")
    print(f"READY: native {configured_rate:g} Hz p3d, SRDF pose {args.pose}, {disturbance} disturbance", flush=True)

    def pose_metrics(begin, duration):
        trace = np.asarray(tip_world)
        trace = trace[(trace[:, 0] >= begin) & (trace[:, 0] < begin + duration)]
        time = trace[:, 0] - begin
        displacement = np.linalg.norm(trace[:, 1:4] - tip_reference[:3], axis=1)
        rotations = trace[:, 4:]
        reference = tip_reference[3:]
        cosine = abs(rotations @ reference) / (np.linalg.norm(rotations, axis=1) *
                                                np.linalg.norm(reference))
        angle = 2 * np.arccos(np.clip(cosine, 0, 1))
        errors = np.column_stack((displacement, angle))
        rms = 1000 * np.sqrt(np.trapz(errors**2, time, axis=0) / (time[-1] - time[0]))
        return dict(tip_xyz_rms_mm=rms[0], tip_xyz_peak_mm=1000*max(displacement),
                    tip_orientation_rms_mrad=rms[1], tip_orientation_peak_mrad=1000*max(angle))

    def select(mode):
        use_static = mode == "static"
        mapping.publish(Bool(data=use_static))
        for _ in range(80):
            if inertia and not inertia[-1][1] and inertia[-1][2] == use_static and inertia[-1][4] < 1e-5:
                return
            rospy.sleep(.025)
        raise RuntimeError(f"Stationary mapping switch to {mode} did not complete")

    def run_leg(profile, block, mode, omega, phase, amplitude, duration,
                source=None, pulse=None, phase_slot=None, keep_acceleration=False):
        select(mode)
        indices = tuple(map(len, (base, tip, inertia)))
        steady_end = FIT_DELAY + duration + .1
        start = rospy.Time.now()
        if phase_slot is not None:
            # Repeat the phase of the 200 Hz disturbance relative to the 1 ms
            # physics step, 200 Hz base publisher, 125 Hz arm publisher and 250 Hz PID.
            frame_ns = 40_000_000
            first_tick = (start.to_nsec() // frame_ns + 1) * frame_ns + phase_slot * 1_000_000
            start = rospy.Time(0, first_tick)
            rospy.sleep(start - rospy.Time.now())
        start_wall = time.monotonic()
        rate = rospy.Rate(200)
        if phase_slot is not None:
            rate.last_time = start
        response = base_response() if base_response and source in (None, "model") else None
        last_t = None
        last_sample = None
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            t = (now-start).to_sec()
            if t >= steady_end+RAMP:
                break
            if pulse is None:
                env, d_env, dd_env = window(t, steady_end)
                angles = omega*t + phase
                x = amplitude * np.sin(angles)
                v = amplitude*omega * np.cos(angles)
                a = -(omega**2)*x
                velocity = env*v.sum()+d_env*x.sum()
                acc = env*a.sum() + 2*d_env*v.sum() + dd_env*x.sum()
            else:
                velocity, acc = pulse(t-FIT_DELAY)
            gantry_velocity = [0.0]*4
            gantry_velocity[:3] = (weights * velocity).tolist()
            command.publish(Float64MultiArray(data=gantry_velocity))
            if mode != "off" or keep_acceleration:
                stamp, q = now, base_pose.orientation
                if source in ("measured", "imu"):
                    sample = measured_acceleration if source == "measured" else imu_acceleration
                    if sample is None or sample[0] == last_sample or not 0 <= (now-sample[0]).to_sec() <= .025:
                        rate.sleep()
                        continue
                    stamp, acc, q = sample
                    last_sample = stamp
                elif response is not None:
                    acc = response(acc, 0. if last_t is None else t-last_t)
                    last_t = t
                msg = Vector3Stamped()
                msg.header.stamp, msg.header.frame_id = stamp, "base_link"
                direction_in_base = weights @ quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
                msg.vector.x, msg.vector.y, msg.vector.z = acc * direction_in_base
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
        min_active = 0.0 if pulse is not None else .4
        assert active == 0 if mode == "off" else active > min_active, (profile, mode, active)
        low, high = (.65, 1.35) if axis == 1 else (.35, 1.80)
        expected = scale * amplitude          # the projected stroke, not one axis'
        assert np.all((low*expected <= abs(p_base)) & (abs(p_base) <= high*expected)), (
            profile, mode, abs(p_base)/expected)
        return begin, p_base, p_tip, base_rms, tip_rms, base_rate, active, diag, rtf

    if args.compare_sensor_inputs:
        from acceleration_sources import compare_sensor_inputs
        assert base_response is not None and configured_rate == 200
        assert imu_acceleration is not None and abs(imu_acceleration[1]) < 0.2, imu_acceleration
        compare_sensor_inputs(out, run_leg, pose_metrics, base, args.inertia_seed, args.inertia_peak)
        return

    if args.compare_acceleration_sources:
        from acceleration_sources import compare_sources
        assert base_response is not None and configured_rate == 200
        compare_sources(out, run_leg, base, tip)
        return

    if args.compare_tuned_inertia or args.inertia_null:
        from inertia_followup import compare_tuned_inertia
        compare_tuned_inertia(out, run_leg, null=args.inertia_null,
                              phase_balanced=args.inertia_phase_balanced,
                              pose_metrics=pose_metrics, seed=args.inertia_seed,
                              peak_mm=args.inertia_peak, axis=args.axis,
                              cross_axis=args.cross_axis)
        return

    if args.inertia_factorial_gain is not None:
        from inertia_followup import compare_inertia_factors
        compare_inertia_factors(out, run_leg, pose_metrics, args.pose, args.axis,
                                args.inertia_factorial_gain, args.inertia_seed, args.inertia_peak,
                                cross_axis=args.cross_axis, pose_fraction=args.pose_fraction)
        return

    if args.tune or args.validate_gains:
        from pid_tuning import tune_gains
        tune_gains(out, run_leg, hold, target, lambda: base_pose, tip, args.validate_gains,
                   args.full_validation, args.axis)
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
    parser.add_argument("--pose", default="init", help="SRDF manipulator group state")
    parser.add_argument("--pose-fraction", type=float,
                        help="interpolate from init towards 1st_reco_start for Y gain tests")
    parser.add_argument("--axis", choices=("x", "y", "z"), default="y", help="Base translation axis")
    parser.add_argument("--cross-axis", default="", metavar="AXIS:FRACTION",
                        help="drive a second base axis with the same waveform at this "
                             "fraction of the primary amplitude, e.g. y:0.5. The recorded "
                             "gains are the primary axis'")
    parser.add_argument("--full-validation", action="store_true", help="Check both 20 mm and 30 mm in six paired blocks")
    parser.add_argument("--inertia-phase-balanced", action="store_true",
                        help="Match the disturbance sample phase within each inertia block")
    parser.add_argument("--inertia-seed", type=int, help="Seed for independent inertia comparison waveforms")
    parser.add_argument("--inertia-peak", type=int, choices=(20, 30),
                        help="Repeat only one disturbance peak in an inertia comparison (mm)")
    suite = parser.add_mutually_exclusive_group()
    suite.add_argument("--tune", action="store_true")
    suite.add_argument("--validate-gains", type=float, nargs=2, metavar=("KP", "KD"))
    suite.add_argument("--compare-tuned-inertia", action="store_true")
    suite.add_argument("--inertia-null", action="store_true",
                       help="Repeat disturbances with inertial feedforward off to measure repeatability")
    suite.add_argument("--inertia-factorial-gain", type=float, metavar="MAGNITUDE",
                       help="Cross both gain signs with projection on/off and a shared zero-gain baseline")
    suite.add_argument("--compare-acceleration-sources", action="store_true")
    suite.add_argument("--compare-sensor-inputs", action="store_true",
                       help="compare command-model, odometry and IMU acceleration with feedforward off")
    args = parser.parse_args()
    factorial = args.inertia_factorial_gain is not None
    if factorial and (not np.isfinite(args.inertia_factorial_gain) or args.inertia_factorial_gain <= 0):
        parser.error("Inertia gain magnitude must be finite and positive")
    inertia_comparison = (args.inertia_null or args.compare_tuned_inertia or factorial
                          or args.compare_sensor_inputs)
    if args.compare_sensor_inputs and (args.axis != "y" or args.cross_axis):
        parser.error("Sensor input follow-up uses pure Y translation")
    if args.pose_fraction is not None and (not factorial or args.pose != "init"
                                           or args.axis != "y" or args.cross_axis
                                           or not 0 < args.pose_fraction < 1):
        parser.error("--pose-fraction requires a pure Y gain factorial starting at init and a fraction in (0,1)")
    if (args.inertia_phase_balanced or args.inertia_seed is not None or args.inertia_peak) and not inertia_comparison:
        parser.error("Inertia phase, seed and peak options require an inertia comparison")
    if args.axis != "y" and not (args.tune or args.validate_gains or args.inertia_phase_balanced or factorial):
        parser.error("--axis x/z requires PID gain testing or a phase-balanced inertia comparison")
    if args.cross_axis and not (args.inertia_phase_balanced or factorial):
        parser.error("--cross-axis requires a phase-balanced inertia comparison")
    main(args.output, args)
