#!/usr/bin/env python3
#
# Put the arm at the pose from config/initial_pose.yaml, then hand the joints over
# to the Cartesian velocity controller.
#
# Run it on top of a bringup that is already up, e.g.:
#   roslaunch sim_bringup with_moveit_new.launch
#   roslaunch sim_bringup init_pose_start_controllers.launch
#
# Note that this takes the arm away from MoveIt: arm_joint_controller and
# sim_cartesian_velocity_controller_sim claim the same six joints, so the switch
# below stops the trajectory controller. Start it again to plan with MoveIt:
#   rosrun controller_manager controller_manager stop sim_cartesian_velocity_controller_sim
#   rosrun controller_manager controller_manager start arm_joint_controller
#

import rospy
import subprocess

from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

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


if __name__ == "__main__":
    # Service calls need an initialised node, so this has to come first.
    rospy.init_node("set_initial_pose_node")
    model_name = rospy.get_param("~model_name", DEFAULT_MODEL_NAME)

    pause_gazebo()
    set_initial_pose_gazebo(model_name)
    unpause_gazebo()
    start_ros_controllers()
