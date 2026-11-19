#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source ~/ur5e_sim_ws/devel/setup.bash

bench_dir=$(cd "$(dirname "$0")" && pwd)
output_dir=$(mkdir -p "$1" && cd "$1" && pwd)
expected_rate=$2
shift 2
export ROS_MASTER_URI=http://127.0.0.1:11386
export GAZEBO_MASTER_URI=http://127.0.0.1:11387
export ROS_LOG_DIR="$output_dir/ros-logs"

launch_args=(gui:=false headless:=true "$@")
roslaunch sim_bringup with_moveit_new.launch "${launch_args[@]}" >"$output_dir/launch.log" 2>&1 &
processes=($!)
trap 'kill -INT "${processes[@]}" 2>/dev/null || true; wait || true' EXIT
trap 'exit 130' INT TERM
for _ in {1..40}; do
    if ! kill -0 "${processes[0]}" 2>/dev/null; then
        tail -20 "$output_dir/launch.log" >&2
        exit 1
    fi
    if rosparam get /robot_description >/dev/null 2>&1; then
        break
    fi
    sleep 1
done
rosparam get /robot_description >/dev/null
roslaunch sim_pid_control pid_controller.launch >"$output_dir/pid.log" 2>&1 &
processes+=($!)
python3 "$bench_dir/standard_bringup.py" "$output_dir" "$expected_rate" 2>&1 | tee "$output_dir/smoke.log"
