#!/usr/bin/env bash
# Record one run: stage the arm, spawn the camera rig, shake the base.
#
#   bash record.sh <label> <new|old> <axis> <amplitude_m> <frequency_hz> <seconds> [pose] [ff_gain]
#
# Frames land in ~/ur5e_video/out/<label>/frames/{wide,macro}, the world-frame
# tool and base traces in ~/ur5e_video/out/<label>/{ee,base}.csv.
#
# Runs under xvfb-run: gzserver has to render the camera sensors, and the
# baseline and the current workspace must both render into the same kind of
# context for the two recordings to look alike.
set -eo pipefail

label=${1:?usage: record.sh label new|old axis amplitude frequency seconds [pose] [ff_gain]}
side=${2:?}
axis=${3:-y}
amplitude=${4:-0.02}
frequency=${5:-1.0}
seconds=${6:-10}
pose=${7:-init}
ff_gain=${8:-}

here=$(cd "$(dirname "$0")" && pwd)
out="$here/out/$label"
rm -rf "$out"
mkdir -p "$out/frames"

source /opt/ros/noetic/setup.bash
case "$side" in
  new) source ~/ur5e_sim_ws/devel/setup.bash; launch="$here/video_new.launch" ;;
  old) source ~/ur5e_old_ws/devel/setup.bash; launch="$here/video_old.launch" ;;
  *)   echo "side must be new or old" >&2; exit 2 ;;
esac

export ROS_MASTER_URI=http://127.0.0.1:11386
export GAZEBO_MASTER_URI=http://127.0.0.1:11387
export ROS_LOG_DIR="$out/ros-logs"
export PYTHONPATH="$here:$PYTHONPATH"
# Camera sensors need a GL context. :0 is the VM's VMware SVGA3D device with
# direct rendering; Xvfb falls back to llvmpipe, which renders a handful of
# frames in the time this run takes. gui:=false means nothing is drawn on the
# desktop - the sensors render offscreen - but it does use that GPU.
export DISPLAY="${UR5E_VIDEO_DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
# Physics steps per second. The default slows the simulation to roughly a
# quarter of real time so that every camera frame is actually rendered; the
# frames are stamped in simulated time, so the finished video is unaffected.
rate=${UR5E_VIDEO_UPDATE_RATE:-250}

launch_args=()
[[ -n $ff_gain ]] && launch_args+=("inertia_feedforward_gain:=$ff_gain")

roslaunch "$launch" "${launch_args[@]}" >"$out/launch.log" 2>&1 &
launch_pid=$!
pids=($launch_pid)
trap 'kill -INT "${pids[@]}" 2>/dev/null || true; sleep 2; kill -9 "${pids[@]}" 2>/dev/null || true' EXIT

for _ in {1..120}; do
  rosservice list 2>/dev/null | grep -q /controller_manager/switch_controller && break
  sleep 1
done

# The datum rod has to lie across the motion, not along it: a vertical
# disturbance read against a vertical rod shows nothing.
case "$axis" in z) rod=y ;; *) rod=z ;; esac

python3 "$here/stage.py" --pose "$pose" --frames "$out/frames" \
        --rod-axis "$rod" --max-update-rate "$rate" >"$out/stage.log" 2>&1 &
stage_pid=$!
pids+=($stage_pid)
for _ in {1..120}; do
  grep -q RIG_SPAWNED "$out/stage.log" 2>/dev/null && break
  kill -0 $stage_pid 2>/dev/null || { echo "staging died:" >&2; cat "$out/stage.log" >&2; exit 1; }
  sleep 1
done
grep -q RIG_SPAWNED "$out/stage.log" || { echo "camera rig never spawned:" >&2; cat "$out/stage.log" >&2; exit 1; }
cat "$out/stage.log"

rostopic echo -p /ee_pose_ground_truth   >"$out/ee.csv"   2>/dev/null & pids+=($!)
rostopic echo -p /base_pose_ground_truth >"$out/base.csv" 2>/dev/null & pids+=($!)
sleep 2

roslaunch sim_bringup gantry_disturbance.launch \
    axis:="$axis" amplitude:="$amplitude" frequency:="$frequency" \
    duration:="$seconds" ramp:=1.0 >"$out/disturbance.log" 2>&1 &
pids+=($!)

# Simulated time can run slower than wall time; wait on the clock that matters.
# rospy.Time.now() reads zero until the first /clock message arrives, so take
# the start only once the clock is live - otherwise the first message looks
# like the whole wait has already elapsed.
python3 - "$seconds" <<'PY'
import sys, rospy
rospy.init_node("video_timer", anonymous=True)
target = float(sys.argv[1]) + 4.0
while not rospy.is_shutdown() and rospy.Time.now().to_sec() == 0.0:
    rospy.sleep(0.1)
start = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < target:
    rospy.sleep(0.2)
print(f"waited {(rospy.Time.now()-start).to_sec():.1f} simulated seconds", flush=True)
PY

sleep 1
echo "RECORDED $label: $(ls "$out/frames/wide" 2>/dev/null | wc -l) wide frames, $(ls "$out/frames/macro" 2>/dev/null | wc -l) macro frames"
