#!/usr/bin/env bash
#
# Phase 1 of 2 - install ROS Noetic and every system dependency this project needs.
# This is the only step that requires root.
#
#   sudo bash setup/01_install_ros_and_deps.sh
#
# Safe to re-run: apt skips whatever is already installed. A full log is kept in
# /var/log/ur5e_apt_install.log.
#
# Target platform is Ubuntu 20.04 (focal) + ROS Noetic, which is what the
# MoveIt configuration in this repository was generated with. See README.md.
#
set -euo pipefail

LOG=/var/log/ur5e_apt_install.log
exec > >(tee -a "$LOG") 2>&1
echo "=== phase 1 started $(date -Is) ==="

if [ "$(id -u)" -ne 0 ]; then
  echo "ERROR: this phase needs root. Run: sudo bash setup/01_install_ros_and_deps.sh" >&2
  exit 1
fi

codename="$(lsb_release -cs)"
if [ "$codename" != "focal" ]; then
  echo "ERROR: expected Ubuntu 20.04 (focal), found '$codename'." >&2
  echo "       ROS Noetic has no binary packages for this release." >&2
  exit 1
fi

# Some VM DNS setups hand out a bogus AAAA record for the Ubuntu/ROS mirrors,
# which makes apt stall on IPv6 for a long time before falling back. Pinning apt
# to IPv4 avoids minutes of dead waiting per invocation. Remove this file if your
# network is IPv6-native.
echo 'Acquire::ForceIPv4 "true";' > /etc/apt/apt.conf.d/99force-ipv4

echo "--- [1/5] base build tools ---"
apt-get update
apt-get install -y --no-install-recommends \
  curl wget gnupg2 ca-certificates lsb-release git \
  build-essential cmake libeigen3-dev \
  mesa-utils net-tools

echo "--- [2/5] ROS Noetic apt source ---"
# apt-key is deprecated on focal, so the key goes into its own keyring and is
# referenced by signed-by instead of the global trust store.
install -d -m 0755 /usr/share/keyrings
wget -4 -qO- https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | gpg --dearmor > /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
  > /etc/apt/sources.list.d/ros-noetic.list
apt-get update

echo "--- [3/5] ros-noetic-desktop-full (Gazebo 11 + RViz, ~2.5 GB download) ---"
DEBIAN_FRONTEND=noninteractive apt-get install -y ros-noetic-desktop-full

echo "--- [4/5] project dependencies ---"
# Mapped from the package.xml files in this repository:
#   sim_general_planning   -> moveit, moveit_visual_tools, pcl_ros, pcl_conversions
#   sim_moveit_config      -> moveit planners/kinematics, joint_state_publisher_gui,
#                             pilz_industrial_motion_planner (MoveIt 1.1 / Noetic only)
#   sim_cartesian_velocity_control -> control_toolbox, kdl_parser, kdl_conversions,
#                             controller_interface, forward_command_controller
#   sim_general_control    -> controller_manager, effort_controllers, joint_state_controller,
#                             diff_drive_controller, rqt_gui
#   sim_bringup            -> gazebo_ros, controller_manager_msgs, soem
DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-noetic-moveit \
  ros-noetic-moveit-visual-tools \
  ros-noetic-pilz-industrial-motion-planner \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-gazebo-ros-control \
  ros-noetic-control-toolbox \
  ros-noetic-kdl-parser \
  ros-noetic-kdl-conversions \
  ros-noetic-joint-state-publisher-gui \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  ros-noetic-soem \
  ros-noetic-rqt-gui \
  ros-noetic-rqt-joint-trajectory-controller \
  python3-catkin-tools python3-rosdep python3-wstool python3-osrf-pycommon

echo "--- [5/5] rosdep init ---"
# rosdep init needs root; "rosdep update" must run as the normal user and is
# therefore done by phase 2.
rosdep init 2>/dev/null || echo "(rosdep already initialised)"

echo "=== phase 1 finished OK $(date -Is) ==="
echo "Next:  bash setup/02_build_workspace.sh   (as your normal user, no sudo)"
