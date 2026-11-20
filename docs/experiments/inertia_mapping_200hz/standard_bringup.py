"""Check the normal MoveIt bringup, PID handoff, and return to MoveIt."""

from pathlib import Path
import subprocess
import sys
import time

import moveit_commander
import numpy as np
import rosnode
import rospkg
import rosparam
import rospy
from controller_manager_msgs.srv import ListControllers, LoadController, SwitchController
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sim_cartesian_velocity_control.msg import InertialFeedforwardState

from bench import ARM, GANTRY, TRAJECTORY, append_row, fit


def main(out, expected_rate):
    moveit_commander.roscpp_initialize([])
    rospy.init_node("check_standard_bringup")
    rospy.wait_for_service("/get_planning_scene", timeout=55)
    group = moveit_commander.MoveGroupCommander("manipulator", wait_for_servers=40)
    group.set_max_velocity_scaling_factor(.35)
    group.set_max_acceleration_scaling_factor(.35)
    endpoint = "/controller_manager/"
    rospy.wait_for_service(endpoint+"switch_controller", timeout=30)
    listing = rospy.ServiceProxy(endpoint+"list_controllers", ListControllers)
    switch = rospy.ServiceProxy(endpoint+"switch_controller", SwitchController)
    load = rospy.ServiceProxy(endpoint+"load_controller", LoadController)

    def state(name):
        return next((c.state for c in listing().controller if c.name == name), "missing")

    def move_to(pose):
        group.set_start_state_to_current_state()
        group.set_named_target(pose)
        success, trajectory, _, error = group.plan()
        assert success and trajectory.joint_trajectory.points, (pose, error.val)
        assert group.execute(trajectory, wait=True), pose
        group.stop()
        joints = group.get_active_joints()
        desired = [group.get_named_target_values(pose)[joint] for joint in joints]
        largest_error = max(abs(q-r) for q, r in zip(group.get_current_joint_values(), desired))
        assert largest_error < .06, (pose, largest_error)
        print(f"MoveIt executed {pose}. Maximum joint error {largest_error*1000:.1f} mrad", flush=True)

    for _ in range(100):
        arm_state = state(ARM)
        trajectory_state = state(TRAJECTORY)
        if trajectory_state == "running" and arm_state in ("initialized", "stopped"):
            break
        rospy.sleep(.1)
    else:
        raise RuntimeError(f"Trajectory controller: {trajectory_state}, Cartesian controller: {arm_state}")
    assert tuple(rospy.get_param(name)[7] for name in ("/K_p", "/K_d")) == (12.0, .35)
    move_to("init")

    config = Path(rospkg.RosPack().get_path("sim_bringup"))/"config"/"gantry_velocity_controller.yaml"
    for content, namespace in rosparam.load_file(str(config)):
        rosparam.upload_params(namespace, content)
    assert load(GANTRY).ok and switch([GANTRY], [], 2, False, 5.0).ok
    assert switch([ARM], [TRAJECTORY], 2, False, 5.0).ok
    assert state(ARM) == "running" and state(TRAJECTORY) == "stopped"
    hold = rospy.Publisher("/now_equilibrium", PoseStamped, queue_size=1)
    for _ in range(100):
        if hold.get_num_connections():
            break
        rospy.sleep(.05)
    assert hold.get_num_connections()
    relative_pose = rospy.wait_for_message(f"/{ARM}/ee_state", Odometry, timeout=10)
    target = PoseStamped(header=relative_pose.header, pose=relative_pose.pose.pose)
    target.header.frame_id = "pid"
    hold.publish(target)
    rospy.sleep(1.5)

    base, tip, model, acceleration = [], [], [], []

    def read_position(msg, samples):
        samples.append((msg.header.stamp.to_sec(), msg.pose.pose.position.y, time.monotonic()))

    def read_model(msg):
        model.append((msg.header.stamp.to_sec(), msg.active, sum(msg.torque_clip_count)))

    subscriptions = [
        rospy.Subscriber("/base_pose_ground_truth", Odometry, read_position, callback_args=base, queue_size=20),
        rospy.Subscriber("/ee_pose_ground_truth", Odometry, read_position, callback_args=tip, queue_size=20),
        rospy.Subscriber(f"/{ARM}/inertial_feedforward_state", InertialFeedforwardState, read_model, queue_size=20),
        rospy.Subscriber("/gantry/base_linear_acceleration", Vector3Stamped,
                         lambda msg: acceleration.append(msg.header.stamp.to_sec()), queue_size=20),
    ]
    with (out/"smoke.csv").open("w", newline="") as results:
        for frequency in (3., 5.):
            accel_index = len(acceleration)
            with (out/f"disturbance-{frequency:g}.log").open("w") as log:
                subprocess.run(
                    ["rosrun", "sim_bringup", "gantry_disturbance.py", "_axis:=y",
                     "_amplitude:=0.02", f"_frequency:={frequency:g}", "_duration:=7.0", "_ramp:=1.0"],
                    stdout=log, stderr=subprocess.STDOUT, check=True, timeout=40)
            assert len(acceleration)-accel_index >= 1000
            start = acceleration[accel_index]+1.6
            duration = 4.
            omega = 2*np.pi*np.array([frequency])
            b, _, rate = fit(base, start, duration, omega, .85*expected_rate)
            e, _, _ = fit(tip, start, duration, omega, 42)
            diag = np.array([r for r in model if start <= r[0] < start+duration])
            gated = diag[:, 1].mean()
            clips = int(diag[-1, 2]-diag[0, 2])
            assert abs(rate-expected_rate) < 1.0 and gated >= .8 and clips == 0, (rate, gated, clips)
            assert .018 <= abs(b[0]) <= .022, abs(b[0])
            stamps = np.array([r for r in base if start <= r[0] < start+duration])
            rtf = (stamps[-1, 0]-stamps[0, 0])/(stamps[-1, 2]-stamps[0, 2])
            transfer = 100*abs(e[0]/b[0])
            append_row(results, dict(base_rate_hz=rate, frequency_hz=frequency, base_amplitude_mm=1000*abs(b[0]),
                                     tip_amplitude_mm=1000*abs(e[0]), transfer_pct=transfer,
                                     inertia_active_pct=100*gated, torque_clip_updates=clips, rtf=rtf))
            print(f"{rate:.1f} Hz base, {frequency:g} Hz disturbance, "
                  f"transfer {transfer:.2f}%, inertia active {100*gated:.1f}%, RTF {rtf:.3f}", flush=True)

    rosnode.kill_nodes(["/sim_pid_controller"])
    handoff = subprocess.run(
        ["roslaunch", "sim_bringup", "init_pose_start_controllers.launch", "mode:=moveit"],
        check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=18)
    (out/"moveit-handoff.log").write_text(handoff.stdout)
    assert "MoveIt handoff ready" in handoff.stdout, handoff.stdout
    assert state(TRAJECTORY) == "running" and state(ARM) == "stopped"
    move_to("1st_reco_start")
    print("PASS: normal Gazebo and MoveIt bringup, MoveIt -> PID -> MoveIt", flush=True)


if __name__ == "__main__":
    main(Path(sys.argv[1]), float(sys.argv[2]))
