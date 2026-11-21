#!/usr/bin/env python3
"""Bring the arm to a pose, hand it to the Cartesian controller, hold a point.

This is the part of docs/experiments/inertia_mapping_200hz/bench.py that sets a
run up, with the measurement stripped out: it exists so that the old baseline
workspace and the current one can be staged identically before a disturbance is
injected for the camera.

The baseline has no inertial feedforward and no InertialFeedforwardState
message, so nothing here imports it.

  --probe   print the staged world poses and exit (used to aim the cameras)
"""

import argparse
from pathlib import Path
import sys
from xml.etree import ElementTree

import actionlib
import rospkg
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import LoadController, SwitchController
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = "sim_cartesian_velocity_controller_sim"
GANTRY = "gantry_velocity_controller"
TRAJECTORY = "arm_joint_controller"
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def pose_positions(name):
    package = Path(rospkg.RosPack().get_path("sim_moveit_config"))
    srdf = ElementTree.parse(package / "config" / "sim.srdf")
    pose = srdf.find(f"./group_state[@name='{name}'][@group='manipulator']")
    if pose is None:
        raise ValueError(f"Unknown manipulator pose: {name}")
    values = {j.attrib["name"]: float(j.attrib["value"]) for j in pose.findall("joint")}
    return [values[j] for j in JOINTS]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose", default="init")
    parser.add_argument("--probe", action="store_true")
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--frames", help="spawn the camera rig and save frames here")
    parser.add_argument("--rod-axis", default="z", choices=("y", "z"),
                        help="which way the world-fixed datum rod lies; it has "
                             "to be perpendicular to the disturbance")
    parser.add_argument("--max-update-rate", type=float, default=0.0,
                        help="physics steps per second; below 1/time_step this "
                             "slows the simulation down so the camera sensors "
                             "can render every frame they owe. 0 leaves it alone.")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("stage", anonymous=True)
    hold = rospy.Publisher("/now_equilibrium", PoseStamped, queue_size=1, latch=True)

    rospy.wait_for_service("/controller_manager/switch_controller", timeout=90.0)
    switch = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
    rospy.wait_for_service("/controller_manager/load_controller", timeout=30.0)
    load = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)

    assert load(GANTRY).ok, "could not load the gantry controller"
    assert switch([GANTRY], [], 2, False, 5.0).ok, "could not start the gantry controller"

    client = actionlib.SimpleActionClient(f"/{TRAJECTORY}/follow_joint_trajectory",
                                          FollowJointTrajectoryAction)
    assert client.wait_for_server(rospy.Duration(60)), "no trajectory controller"
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = JOINTS
    goal.trajectory.points = [JointTrajectoryPoint(positions=pose_positions(args.pose),
                                                   time_from_start=rospy.Duration(3.0))]
    goal.goal_time_tolerance = rospy.Duration(2.0)
    client.send_goal(goal)
    assert client.wait_for_result(rospy.Duration(15.0)), "trajectory did not finish"

    assert switch([ARM], [TRAJECTORY], 2, False, 5.0).ok, "could not start the arm controller"
    rospy.sleep(1.0)

    state = rospy.wait_for_message(f"/{ARM}/ee_state", Odometry, timeout=20)
    target = PoseStamped(header=state.header, pose=state.pose.pose)
    target.header.frame_id = "pid"
    hold.publish(target)
    rospy.sleep(args.settle)

    tip = rospy.wait_for_message("/ee_pose_ground_truth", Odometry, timeout=20)
    base = rospy.wait_for_message("/base_pose_ground_truth", Odometry, timeout=20)
    t, b = tip.pose.pose.position, base.pose.pose.position
    print(f"TIP_WORLD {t.x:.6f} {t.y:.6f} {t.z:.6f}", flush=True)
    print(f"BASE_WORLD {b.x:.6f} {b.y:.6f} {b.z:.6f}", flush=True)
    print(f"STAGED sim_time={rospy.Time.now().to_sec():.3f} pose={args.pose}", flush=True)
    if args.probe:
        return 0

    if args.frames:
        # Spawned last and timed, because Gazebo numbers saved frames from the
        # moment the sensor starts: frame N is at SPAWN + N/update_rate in
        # simulated time, which is what lines the two recordings up.
        import scene
        from gazebo_msgs.srv import SpawnModel
        spawn_name = "/gazebo/spawn_sdf_model"
        rospy.wait_for_service(spawn_name, timeout=30.0)
        spawn = rospy.ServiceProxy(spawn_name, SpawnModel)
        # The wide camera frames the tool and the arm below it, so it is aimed
        # a little under the tool rather than at the midpoint of the whole arm.
        sdf = scene.build((t.x, t.y, t.z), args.frames,
                          wide_centre=(t.x, t.y, t.z - 0.12),
                          rod_axis=args.rod_axis)
        Path(args.frames).mkdir(parents=True, exist_ok=True)
        Path(args.frames, "rig.sdf").write_text(sdf)
        result = spawn("video_rig", sdf, "", PoseStamped().pose, "world")
        assert result.success, result.status_message
        print(f"RIG_SPAWNED sim_time={rospy.Time.now().to_sec():.3f}", flush=True)
        for name, mm in scene.scales().items():
            print(f"SCALE {name} {mm:.4f} mm_per_px", flush=True)

    if args.max_update_rate > 0:
        from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties
        rospy.wait_for_service("/gazebo/get_physics_properties", timeout=30.0)
        current = rospy.ServiceProxy("/gazebo/get_physics_properties",
                                     GetPhysicsProperties)()
        rospy.wait_for_service("/gazebo/set_physics_properties", timeout=30.0)
        applied = rospy.ServiceProxy("/gazebo/set_physics_properties",
                                     SetPhysicsProperties)(
            time_step=current.time_step, max_update_rate=args.max_update_rate,
            gravity=current.gravity, ode_config=current.ode_config)
        assert applied.success, applied.status_message
        print(f"PHYSICS time_step={current.time_step} "
              f"max_update_rate={args.max_update_rate} "
              f"target_rtf={args.max_update_rate*current.time_step:.3f}", flush=True)

    # Hold the node open so the latched equilibrium survives for the recording.
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
