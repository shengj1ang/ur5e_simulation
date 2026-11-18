#!/usr/bin/env bash
#
# Phase 2 of 2 - create the catkin workspace, fetch the Gazebo mimic-joint
# plugin, and build. No root required.
#
#   bash setup/02_build_workspace.sh
#
# Re-run it after editing the sources: it re-syncs this checkout into the build
# workspace and rebuilds incrementally.
#
# Why the sources are copied instead of built in place:
#   catkin_make has to create symlinks inside <workspace>/src, and VMware /
#   VirtualBox shared folders (vmhgfs/vboxsf) do not support symlinks at all.
#   Editing the repository on the host and building in the guest therefore needs
#   one copy step. If this checkout already lives inside the workspace, nothing
#   is copied.
#
set -euo pipefail

WS="${WS:-$HOME/ur5e_sim_ws}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG_NAME="ur5e_simulation"
# Maintained fork of the original roboticsgroup plugin; the upstream one does not
# build against Gazebo 11.
MIMIC_URL="https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git"

echo "=== phase 2 started $(date -Is) ==="
echo "    repo:      $REPO_ROOT"
echo "    workspace: $WS"

if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo "ERROR: ROS Noetic not found. Run phase 1 first:" >&2
  echo "       sudo bash setup/01_install_ros_and_deps.sh" >&2
  exit 1
fi
# ROS's profile scripts reference unset variables, so nounset has to be lifted
# across the source call or it aborts with "ROS_DISTRO: unbound variable".
set +u
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
set -u

echo "--- [1/6] rosdep update ---"
rosdep update --rosdistro noetic >/dev/null 2>&1 \
  && echo "    ok" \
  || echo "    WARN: rosdep update failed (not fatal; catkin_make does not need it)"

echo "--- [2/6] workspace skeleton ---"
mkdir -p "$WS/src"
[ -f "$WS/src/CMakeLists.txt" ] || ( cd "$WS/src" && catkin_init_workspace >/dev/null )

echo "--- [3/6] project sources ---"
DEST="$WS/src/$PKG_NAME"
if [ "$REPO_ROOT" = "$DEST" ]; then
  echo "    building this checkout in place"
else
  mkdir -p "$DEST"
  # .git is excluded on purpose: the copy is a build snapshot, never a place to
  # commit from. Commit in the original checkout.
  if command -v rsync >/dev/null; then
    rsync -a --delete --exclude '.git' --exclude 'build' --exclude 'devel' \
      "$REPO_ROOT"/ "$DEST"/
  else
    rm -rf "$DEST"; mkdir -p "$DEST"
    ( cd "$REPO_ROOT" && tar --exclude=.git --exclude=build --exclude=devel -cf - . ) \
      | ( cd "$DEST" && tar -xf - )
  fi
  # A shared folder can still deliver CRLF files if the host checkout predates
  # .gitattributes; strip the CRs so shebangs and xacro parsing work.
  find "$DEST" -type f \( -name '*.sh' -o -name '*.py' \) -exec sed -i 's/\r$//' {} +
  chmod +x "$DEST"/sim/sim_bringup/scripts/*.py 2>/dev/null || true
  echo "    synced $(du -sh "$DEST" | cut -f1)"
fi

echo "--- [4/6] Gazebo mimic-joint plugin ---"
# robotiq_85_gripper.transmission.xacro references the plugin by its historical
# file name, so after the build a symlink bridges old name -> new name.
if [ -d "$WS/src/roboticsgroup_upatras_gazebo_plugins/.git" ]; then
  echo "    already present"
else
  git clone --quiet "$MIMIC_URL" "$WS/src/roboticsgroup_upatras_gazebo_plugins"
  echo "    cloned"
fi

echo "--- [5/6] unresolved dependencies according to rosdep ---"
cd "$WS"
rosdep check --from-paths src --ignore-src --rosdistro noetic 2>&1 | head -20 || true

echo "--- [6/6] catkin_make ---"
catkin_make -j"$(nproc)"

NEW_SO="$WS/devel/lib/libroboticsgroup_upatras_gazebo_mimic_joint_plugin.so"
OLD_SO="$WS/devel/lib/libroboticsgroup_gazebo_mimic_joint_plugin.so"
if [ -f "$NEW_SO" ]; then
  ln -sfn "$NEW_SO" "$OLD_SO"
  echo "mimic plugin alias: $(basename "$OLD_SO") -> $(basename "$NEW_SO")"
else
  echo "WARN: $NEW_SO was not built; the gripper's mimic joints will not follow."
fi

# Idempotent shell setup.
if ! grep -q "ur5e_simulation environment" "$HOME/.bashrc" 2>/dev/null; then
  cat >> "$HOME/.bashrc" <<RC

# --- ur5e_simulation environment ---
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash
# VMware's GL driver crashes Gazebo 11 unless GPU shader support is disabled.
export SVGA_VGPU10=0
RC
  echo "appended environment block to ~/.bashrc"
fi

echo "=== phase 2 finished OK $(date -Is) ==="
echo "Open a new shell (or: source $WS/devel/setup.bash) and try:"
echo "  roslaunch sim_bringup with_moveit_new.launch"
