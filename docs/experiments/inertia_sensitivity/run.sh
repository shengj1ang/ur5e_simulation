#!/usr/bin/env bash
# How far the robot description's rotational inertia tensors move the inertial
# feedforward's static equivalent along-axis offset.
#
# No simulation and no ROS master: this reads the same robot description the
# bench spawns, builds the chain the Cartesian velocity controller builds -
# base_link to ee_link with everything past the tip folded in - and walks the
# same path computeInertialTorque() walks at zero joint velocity, with the same
# damped IK parameters and the same Gazebo joint P gains. The model figures it
# prints first are the check that it is the controller's model: they are the
# ones README section 4.4 publishes for this URDF.
set -eo pipefail

source /opt/ros/noetic/setup.bash
source ~/ur5e_sim_ws/devel/setup.bash

tool_dir=$(cd "$(dirname "$0")" && pwd)
output_dir=$(mkdir -p "$1" && cd "$1" && pwd)

# The header comes from the package, so this cannot drift from the controller.
g++ -O2 -std=c++14 \
    -I"$(rospack find sim_cartesian_velocity_control)/include" \
    -I/opt/ros/noetic/include -I/usr/include/eigen3 \
    "$tool_dir/inertia_sensitivity.cpp" -o "$output_dir/inertia_sensitivity" \
    -L/opt/ros/noetic/lib -lkdl_parser -lorocos-kdl -lurdf -lurdfdom_model

rosrun xacro xacro "$(rospack find sim_description)/urdf/xacro/main.xacro" \
    base_pose_update_rate:=200.0 >"$output_dir/robot.urdf"

# The six arm links. The gantry and the end-effector hardware are left alone:
# the first does not move with the joints, the second is the measured payload.
arm_links=shoulder_link,upper_arm_link,forearm_link,wrist_1_link,wrist_2_link,wrist_3_link

echo "scale,pose,axis,total_mm,inertia_term_mm,gravity_term_mm,tau_inertia_norm_nm,tau_gravity_norm_nm" \
    >"$output_dir/sensitivity.csv"
: >"$output_dir/model-check.txt"
for scale in 0.90 1.00 1.10; do
    python3 "$tool_dir/scale_inertia.py" "$output_dir/robot.urdf" --scale "$scale" \
        --links "$arm_links" --out "$output_dir/robot-$scale.urdf" >/dev/null
    echo "# inertia tensors scaled by $scale" >>"$output_dir/model-check.txt"
    # The commented lines are the model figures, the rest is the data.
    "$output_dir/inertia_sensitivity" "$output_dir/robot-$scale.urdf" "$scale" \
        | awk -v model="$output_dir/model-check.txt" -v data="$output_dir/sensitivity.csv" \
            '/^#/ { print >> model; next } { print >> data }'
done
cat "$output_dir/model-check.txt"
echo "wrote $output_dir/sensitivity.csv"
