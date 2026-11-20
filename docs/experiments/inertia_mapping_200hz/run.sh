#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
source ~/ur5e_sim_ws/devel/setup.bash

bench_dir=$(cd "$(dirname "$0")" && pwd)
output_dir=$(mkdir -p "$1" && cd "$1" && pwd)
shift
export ROS_MASTER_URI=http://127.0.0.1:11376
export GAZEBO_MASTER_URI=http://127.0.0.1:11377
export ROS_LOG_DIR="$output_dir/ros-logs"

launch_args=("base_pose_update_rate:=${UR5E_BENCH_BASE_RATE:-200.0}")
if [[ ${UR5E_BENCH_BASE_IMU:-0} == 1 ]]; then
    launch_args+=("model:=$bench_dir/base_imu.xacro")
fi
if [[ -n ${UR5E_BENCH_PID_PARAMS:-} ]]; then
    launch_args+=("pid_params_file:=$UR5E_BENCH_PID_PARAMS")
fi
if [[ -n ${UR5E_BENCH_INERTIA_GAIN:-} ]]; then
    launch_args+=("inertia_feedforward_gain:=$UR5E_BENCH_INERTIA_GAIN")
    launch_args+=("inertia_project_translation:=${UR5E_BENCH_INERTIA_PROJECT:-false}")
fi
if [[ -n ${UR5E_BENCH_INERTIA_THRESHOLD:-} ]]; then
    launch_args+=("inertia_alignment_threshold:=$UR5E_BENCH_INERTIA_THRESHOLD")
fi
roslaunch "$bench_dir/bench.launch" "${launch_args[@]}" >"$output_dir/launch.log" 2>&1 &
processes=($!)
trap 'kill -INT "${processes[@]}" 2>/dev/null || true; wait || true' EXIT
trap 'exit 130' INT TERM
for _ in {1..40}; do
    if rosparam get /sim_cartesian_velocity_controller_sim/inertia_mapping_experiment >/dev/null 2>&1; then
        break
    fi
    sleep 1
done
topics=(/base_pose_ground_truth /ee_pose_ground_truth
        /sim_cartesian_velocity_controller_sim/inertial_feedforward_state)
names=(base ee inertia)
if [[ ${UR5E_BENCH_BASE_IMU:-0} == 1 ]]; then
    topics+=(/gantry/base_linear_acceleration)
    names+=(acceleration)
fi
if [[ ${UR5E_BENCH_RECORD_TIMING:-0} == 1 ]]; then
    topics+=(/pid_control/sample_age)
    names+=(pid-sample-age)
fi
for i in "${!topics[@]}"; do
    rostopic echo -p "${topics[$i]}" >"$output_dir/${names[$i]}.csv" 2>"$output_dir/${names[$i]}.log" &
    processes+=($!)
done
python3 "$bench_dir/bench.py" "$output_dir" "$@" | tee "$output_dir/bench.log"
