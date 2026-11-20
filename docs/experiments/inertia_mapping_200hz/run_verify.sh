#!/usr/bin/env bash
# Verify the published base acceleration against the base's actual motion.
# Own process group per child, killed by group, so nothing is orphaned.
set -eo pipefail
source /opt/ros/noetic/setup.bash
source ~/ur5e_sim_ws/devel/setup.bash

out=${1:?output dir}
mkdir -p "$out"
out=$(cd "$out" && pwd)
export ROS_MASTER_URI=http://127.0.0.1:11391
export GAZEBO_MASTER_URI=http://127.0.0.1:11392
export ROS_LOG_DIR="$out/ros-logs"

groups=()
cleanup() {
    for pgid in "${groups[@]}"; do
        kill -INT -- "-$pgid" 2>/dev/null || true
    done
    sleep 6
    for pgid in "${groups[@]}"; do
        kill -9 -- "-$pgid" 2>/dev/null || true
    done
}
trap cleanup EXIT
trap 'exit 130' INT TERM

setsid roslaunch sim_bringup with_moveit_new.launch gui:=false headless:=true \
    >"$out/launch.log" 2>&1 &
groups+=("$!")

for _ in $(seq 60); do
    if ! kill -0 "${groups[0]}" 2>/dev/null; then
        echo "bringup died" >&2; tail -30 "$out/launch.log" >&2; exit 1
    fi
    if rosparam get /robot_description >/dev/null 2>&1; then break; fi
    sleep 1
done
rosparam get /robot_description >/dev/null
# The spawn must have succeeded, or the whole run measures a leftover model.
if grep -q "model name sim already exist" "$out/launch.log"; then
    echo "a previous model is still in gazebo - aborting" >&2; exit 1
fi

python3 "$(dirname "$0")/verify_base_response.py" "$out" 2>&1 | tee "$out/verify.log"
