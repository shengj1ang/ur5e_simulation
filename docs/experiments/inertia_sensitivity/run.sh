#!/usr/bin/env bash
# How far the robot description's own parameters move the inertial
# feedforward's static equivalent along-axis offset.
#
# No simulation and no ROS master: this reads the same robot description the
# bench spawns, builds the chain the Cartesian velocity controller builds -
# base_link to ee_link with everything past the tip folded in - and walks the
# same path computeInertialTorque() walks at zero joint velocity, with the same
# damped IK parameters and the same Gazebo joint P gains. The model figures it
# prints first are the check that it is the controller's model: they are the
# ones README section 4.4 publishes for this URDF.
#
# Two phases, two sets of files.
#
# sensitivity.csv / model-check.txt are the published record: the rotational
# inertia tensors of the six arm links at 0.90, 1.00 and 1.10. Unchanged.
#
# parameter-sensitivity.csv / parameter-model-check.txt sweep the parameters
# that reach the gravity term as well, which the inertia tensors cannot. The
# feedforward predicts the difference of two terms an order of magnitude larger
# than it, so what matters is not how far the difference moves but how far a
# parameter would have to move to flip its sign - and with it the sign of the
# weight calibrated against it. analyze_sensitivity.py turns the sweep into
# those margins.
set -eo pipefail
export LC_ALL=C

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

# ---------------------------------------------------------------------------
# Phase 2: the parameter sweep.
echo "family,value,unit,pose,axis,total_mm,inertia_term_mm,gravity_term_mm,tau_inertia_norm_nm,tau_gravity_norm_nm" \
    >"$output_dir/parameter-sensitivity.csv"
: >"$output_dir/parameter-model-check.txt"

# One perturbed model, one row per pose and axis. 'unit' says what 'value' is
# in: 'scale' for the multiplicative families, 'mm' for a centre-of-mass shift.
perturb() {   # family unit value <scale_inertia.py arguments...>
    local family=$1 unit=$2 value=$3
    shift 3
    python3 "$tool_dir/scale_inertia.py" "$output_dir/robot.urdf" "$@" \
        --out "$output_dir/perturbed.urdf" >/dev/null
    echo "# $family $value $unit" >>"$output_dir/parameter-model-check.txt"
    "$output_dir/inertia_sensitivity" "$output_dir/perturbed.urdf" "$family,$value,$unit" \
        | awk -v model="$output_dir/parameter-model-check.txt" \
              -v data="$output_dir/parameter-sensitivity.csv" \
            '/^#/ { print >> model; next } { print >> data }'
}

offset_vector() {   # axis mm -> "dx,dy,dz" in metres
    local metres
    metres=$(awk -v mm="$2" 'BEGIN { printf "%.6f", mm/1000 }')
    case $1 in
        x) echo "$metres,0,0" ;;
        y) echo "0,$metres,0" ;;
        z) echo "0,0,$metres" ;;
    esac
}

# The inertia tensors, wider than the published three points and then one link
# at a time. Reaches the inertia term only.
for scale in $(seq 0.80 0.05 1.20); do
    perturb inertia_all scale "$scale" --scale "$scale" --links "$arm_links"
done
for link in ${arm_links//,/ }; do
    for scale in 0.90 0.95 1.00 1.05 1.10; do
        perturb "inertia_$link" scale "$scale" --scale "$scale" --links "$link"
    done
done

# Mass reaches both terms. Uniformly it should scale the difference and leave
# its sign alone; one link at a time is the differential error that need not.
for scale in $(seq 0.95 0.01 1.05); do
    perturb mass_all scale "$scale" --mass-scale "$scale" --links "$arm_links"
done
for link in ${arm_links//,/ }; do
    for scale in 0.950 0.975 1.000 1.025 1.050; do
        perturb "mass_$link" scale "$scale" --mass-scale "$scale" --links "$link"
    done
done

# The payload is a measured aggregate of 0.767 kg, so +-0.05 kg is +-6.5 %.
for scale in 0.80 0.90 0.935 1.00 1.065 1.10 1.20; do
    perturb mass_payload scale "$scale" --mass-scale "$scale" --subtree-of ee_link
done

# Centres of mass. com_arm_* moves all six arm links the same way at once,
# which is the correlated worst case and not the likely error. The per-link
# families further down are what a description whose centres of mass are
# approximations actually risks: six independent errors, which add in
# quadrature rather than in sum. analyze_sensitivity.py reports both.
# Offsets apply in each selected link's own frame, so for the payload
# cluster the aggregate centre of mass moves by less than asked for; the
# realized one is in parameter-model-check.txt.
# Per link and in that link's own frame, so for the payload
# cluster the aggregate centre of mass moves by less than the offset asked for;
# the realized one is in parameter-model-check.txt.
for axis in x y z; do
    for mm in -10 -5 0 5 10; do
        perturb "com_payload_$axis" mm "$mm" \
            --com-offset="$(offset_vector "$axis" "$mm")" --subtree-of ee_link
    done
    for mm in -5.0 -2.5 0.0 2.5 5.0; do
        perturb "com_arm_$axis" mm "$mm" \
            --com-offset="$(offset_vector "$axis" "$mm")" --links "$arm_links"
    done
done

for link in ${arm_links//,/ }; do
    for axis in x y z; do
        for mm in -5.0 -2.5 0.0 2.5 5.0; do
            perturb "com_${link}_${axis}" mm "$mm" --com-offset="$(offset_vector "$axis" "$mm")" --links "$link"
        done
    done
done

echo "wrote $output_dir/parameter-sensitivity.csv"
python3 "$tool_dir/analyze_sensitivity.py" "$output_dir"
