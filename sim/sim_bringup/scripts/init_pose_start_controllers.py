#!/usr/bin/env python3
#
# Put the arm at the pose from config/initial_pose.yaml, then hand the joints over
# to the Cartesian velocity controller.
#
# Run it on top of a bringup that is already up, e.g.:
#   roslaunch sim_bringup with_moveit_new.launch
#   roslaunch sim_bringup init_pose_start_controllers.launch
#
# The two controllers claim the same arm joints. To hand them back to MoveIt
# and wait for the measured positions to settle before planning:
#   roslaunch sim_bringup init_pose_start_controllers.launch mode:=moveit
#

import rospy
import subprocess
import math
import threading
import time
from collections import deque

from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest
from sensor_msgs.msg import JointState

SET_MODEL_CONFIGURATION = "/gazebo/set_model_configuration"
SWITCH_CONTROLLER = "/controller_manager/switch_controller"

# The name urdf_spawner gives the model in with_moveit_new.launch / with_admittance.launch.
# Override with _model_name:=... if you spawn it under a different name.
DEFAULT_MODEL_NAME = "sim"
# The Gazebo variant of the Cartesian velocity controller: Gazebo offers no
# VelocityJointInterface, so the "...Sim" controller integrates the twist and
# commands positions instead.
ARM_CONTROLLER = "sim_cartesian_velocity_controller_sim"
# Holds the same joints, so it has to go before ARM_CONTROLLER can start.
TRAJECTORY_CONTROLLER = "arm_joint_controller"


def set_initial_pose_gazebo(model_name):
    set_model_config_client = rospy.ServiceProxy(SET_MODEL_CONFIGURATION, SetModelConfiguration)
    req = SetModelConfigurationRequest()
    req.model_name = model_name
    req.urdf_param_name = "robot_description"
    req.joint_names = rospy.get_param("/initial_pose/joint_names")
    req.joint_positions = rospy.get_param("/initial_pose/joint_values")
    rospy.wait_for_service(SET_MODEL_CONFIGURATION)
    res = set_model_config_client(req)
    if not res.success:
        rospy.logwarn("set_model_configuration failed: %s", res.status_message)
    return res.success


def pause_gazebo():
    subprocess.check_call("rosservice call /gazebo/pause_physics", shell=True)


def unpause_gazebo():
    subprocess.check_call("rosservice call /gazebo/unpause_physics", shell=True)


def reset_world():
    subprocess.check_call("rosservice call /gazebo/reset_world", shell=True)


# There used to be a clear_wrenches() here calling /gazebo/clear_body_wrenches on
# a body named "raigor". That body does not exist in this model, and nothing in
# this project applies Gazebo body wrenches anyway: the admittance controller's
# input is the /ft_sensor/data topic (see scripts/fake_wrench.py). Dropped.


def start_ros_controllers():
    start_controllers_client = rospy.ServiceProxy(SWITCH_CONTROLLER, SwitchController)
    req = SwitchControllerRequest()
    req.start_controllers.append(ARM_CONTROLLER)
    req.start_controllers.append("joint_state_controller")
    req.stop_controllers.append(TRAJECTORY_CONTROLLER)
    req.strictness = req.BEST_EFFORT
    rospy.wait_for_service(SWITCH_CONTROLLER)
    res = start_controllers_client(req)
    if not res.ok:
        rospy.logwarn("switch_controller reported failure; check "
                      "'rosrun controller_manager controller_manager list'")
    return res.ok


def wait_for_arm_settle(joint_names):
    """Verify a continuous 350 ms of measured arm positions within 1 mrad."""
    poses = deque()
    changed = threading.Condition()
    window = .35

    def sample(msg):
        joints = dict(zip(msg.name, msg.position))
        if any(name not in joints or not math.isfinite(joints[name]) for name in joint_names):
            return
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0:
            return
        with changed:
            if poses and stamp == poses[-1][0]:
                return
            if poses and not 0 < stamp-poses[-1][0] <= .12:
                poses.clear()
            poses.append((stamp, [joints[name] for name in joint_names]))
            while len(poses) >= 2 and stamp-poses[1][0] >= window:
                poses.popleft()
            changed.notify()

    subscriber = rospy.Subscriber("/joint_states", JointState, sample, queue_size=10)
    last_ros_time = rospy.get_time()
    settle_deadline = last_ros_time + 5.0 if last_ros_time > 0.0 else None
    clock_deadline = time.monotonic() + 60.0
    try:
        with changed:
            while not rospy.is_shutdown():
                ros_now = rospy.get_time()
                wall_now = time.monotonic()
                if ros_now < last_ros_time:
                    poses.clear()
                    settle_deadline = None
                if ros_now != last_ros_time:
                    # Slow simulation is fine as long as its clock keeps moving.
                    last_ros_time = ros_now
                    clock_deadline = wall_now + 60.0
                if settle_deadline is None and ros_now > 0.0:
                    settle_deadline = ros_now + 5.0
                if settle_deadline is not None and ros_now >= settle_deadline:
                    rospy.logerr("MoveIt controllers are active, but joint stability was not confirmed "
                                 "within 5 seconds of ROS time. Check arm motion and /joint_states before planning.")
                    return False
                if wall_now >= clock_deadline:
                    rospy.logerr("MoveIt controllers are active, but ROS time has not advanced for "
                                 "60 seconds of wall time. Check /clock and Gazebo before planning.")
                    return False
                if len(poses) >= 4 and poses[-1][0]-poses[0][0] >= window:
                    axes = zip(*(joints for _, joints in poses))
                    if all(max(axis)-min(axis) <= .001 for axis in axes):
                        return True
                changed.wait(timeout=.10)
    finally:
        subscriber.unregister()
    return False


def return_to_moveit():
    list_topic = "/controller_manager/list_controllers"
    rospy.wait_for_service(list_topic, timeout=5)
    rospy.wait_for_service(SWITCH_CONTROLLER, timeout=5)
    listing = rospy.ServiceProxy(list_topic, ListControllers)
    states = {c.name: c.state for c in listing().controller}
    if states.get(TRAJECTORY_CONTROLLER) != "running" or states.get(ARM_CONTROLLER) == "running":
        req = SwitchControllerRequest()
        if states.get(TRAJECTORY_CONTROLLER) != "running":
            req.start_controllers = [TRAJECTORY_CONTROLLER]
        if states.get(ARM_CONTROLLER) == "running":
            req.stop_controllers = [ARM_CONTROLLER]
        req.strictness = req.STRICT
        req.timeout = 3.0
        if not rospy.ServiceProxy(SWITCH_CONTROLLER, SwitchController)(req).ok:
            rospy.logerr("Could not hand the joints back to MoveIt")
            return False
    states = {c.name: c.state for c in listing().controller}
    if states.get(TRAJECTORY_CONTROLLER) != "running" or states.get(ARM_CONTROLLER) == "running":
        rospy.logerr("Unexpected controller state after switching to MoveIt")
        return False
    joint_names = rospy.get_param("/"+TRAJECTORY_CONTROLLER+"/joints", [])
    if not joint_names:
        rospy.logerr("MoveIt controllers are active, but the trajectory controller has no configured joints.")
        return False
    if not wait_for_arm_settle(joint_names):
        return False
    rospy.loginfo("MoveIt handoff ready: measured arm joints have settled")
    return True


if __name__ == "__main__":
    # Service calls need an initialised node, so this has to come first.
    rospy.init_node("set_initial_pose_node")
    mode = rospy.get_param("~mode", "cartesian")
    if mode == "moveit":
        try:
            raise SystemExit(0 if return_to_moveit() else 1)
        except (KeyError, rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("MoveIt handoff failed: %s", exc)
            raise SystemExit(1)
    if mode != "cartesian":
        rospy.logerr("Expected mode cartesian or moveit, got %s", mode)
        raise SystemExit(1)
    model_name = rospy.get_param("~model_name", DEFAULT_MODEL_NAME)

    pause_gazebo()
    set_initial_pose_gazebo(model_name)
    unpause_gazebo()
    start_ros_controllers()
