# Working on this project

**English** · [简体中文](WORKFLOW.zh-CN.md)

[README.md](../README.md) says how to install the project and start it. This file
is the other half: how to get from a blank Ubuntu VM to a robot that moves, how to
change something and see your change, and how to find out what went wrong when it
does not work. Every command here was run on Ubuntu 20.04 / ROS Noetic / Gazebo 11
in a VMware VM.

---

## 1. Where everything lives

Working from Windows with the simulation in a VM means the same code exists in
three places. Mixing them up is the most common way to lose an afternoon.

| Place | What it is | What you do there |
|---|---|---|
| `C:\...\ur5e_simulation` (Windows) | the git checkout | edit code, `git commit` |
| `/mnt/hgfs/ur5e_simulation` (VM) | the same files, through the VMware shared folder | nothing, just read |
| `~/ur5e_sim_ws/src/ur5e_simulation` (VM) | a **copy**, made by phase 2 | this is what actually builds and runs |

```
Windows checkout  ──shared folder──>  /mnt/hgfs/...  ──rsync (phase 2)──>  ~/ur5e_sim_ws/src/...
     ^                                                                              |
     └────────────── git commit here, never in the copy ────────────────┘      catkin_make
```

Two consequences, and they are the whole reason this section exists:

* **Editing the Windows checkout does not change what runs.** The build workspace
  is a copy. Nothing happens until you re-run phase 2, which re-syncs it.
* **Editing the copy is temporary.** Phase 2 syncs with `rsync --delete`, so the
  next run overwrites whatever you changed there. It is fine for a quick
  experiment, as long as you know it will vanish.

Why a copy at all: `catkin_make` has to create symlinks inside `src/`, and VMware
and VirtualBox shared folders (`vmhgfs`, `vboxsf`) do not support symlinks, so
building a shared-folder checkout in place cannot work.

If `/mnt/hgfs` is empty, the shared folder is not mounted: enable it in the VM's
settings and make sure `open-vm-tools-desktop` is installed. When it is mounted,
`mount | grep hgfs` shows `vmhgfs-fuse on /mnt/hgfs`.

---

## 2. From a blank VM to a robot that moves

The manual path, with what success looks like at each step.

**Step 1 — Ubuntu 20.04 in the VM.** It has to be 20.04 (focal). ROS Noetic has
binary packages for focal only, and phase 1 refuses to run on anything else.
Give the VM 4 CPUs, 8 GB RAM and 40 GB disk; Gazebo is heavy and
`ros-noetic-desktop-full` alone is about 2.5 GB of download.

**Step 2 — get the code into the VM.** Either a shared folder pointing at your
Windows checkout, or `git clone` straight into the VM. With a shared folder,
check it is there:

```bash
ls /mnt/hgfs/ur5e_simulation
```

**Step 3 — install ROS and the dependencies.** Twenty to forty minutes, mostly
download:

```bash
sudo bash /mnt/hgfs/ur5e_simulation/setup/01_install_ros_and_deps.sh
```

Success looks like `=== phase 1 finished OK ===`. The full log is in
`/var/log/ur5e_apt_install.log`. Re-running it is safe.

**Step 4 — build.** No sudo:

```bash
bash /mnt/hgfs/ur5e_simulation/setup/02_build_workspace.sh
```

Success looks like `[100%] Built target ...` for each node,
`mimic plugin alias: ...` near the end, and `=== phase 2 finished OK ===`. Put the
workspace somewhere else with `WS=~/other_ws bash .../02_build_workspace.sh`.

**Step 5 — open a new shell.** Phase 2 appended the environment to `~/.bashrc`
(ROS, the workspace, and `SVGA_VGPU10=0`, which VMware's GL driver needs or
Gazebo 11 crashes on startup). A shell opened before that step knows nothing about
any of it; `source ~/ur5e_sim_ws/devel/setup.bash` fixes it by hand.

**Step 6 — start the simulation.**

```bash
roslaunch sim_bringup with_moveit_new.launch
```

Gazebo and RViz open, the robot appears next to a cinder block, and after a few
seconds:

```bash
rosrun controller_manager controller_manager list
```

should show `arm_joint_controller`, `gripper_controller` and
`joint_state_controller` as `running`. The stream of
`No p gain specified for pid` errors is normal and harmless — see
[README §6](../README.md#6-known-issues).

**Step 7 — make it move.** In RViz, the MotionPlanning panel: drag the ball on the
end effector somewhere reachable, then **Plan & Execute**. The arm moves in Gazebo
too. That is the whole stack working: MoveIt planned it, `arm_joint_controller`
executed it, Gazebo simulated it.

---

## 3. The edit → build → run loop

What you have to do after a change depends only on what kind of file it was.

| You changed | Rebuild? | What to do |
|---|---|---|
| `*.cpp`, `*.h` | **yes** | re-sync, then `catkin_make` |
| `*.srv` (`mission_mode`) | **yes** | re-sync, then `catkin_make` |
| `CMakeLists.txt`, `package.xml` | **yes** | re-sync, then `catkin_make` |
| `*.yaml` (gains, controllers, limits) | no | re-sync, restart the launch file |
| `*.launch` | no | re-sync, restart the launch file |
| `*.xacro`, `*.urdf`, `*.world` | no | re-sync, restart the launch file |
| `*.srdf` (named poses, groups) | no | re-sync, restart the launch file |
| `*.py` in `scripts/` | no | re-sync, re-run the script |
| `*.rviz` | no | re-sync, restart RViz |

"Re-sync" means getting the file from your Windows checkout into the build
workspace. The blunt way, which also rebuilds:

```bash
bash /mnt/hgfs/ur5e_simulation/setup/02_build_workspace.sh
```

The fast way when you touched one package:

```bash
cd ~/ur5e_sim_ws
catkin_make --pkg sim_general_planning
```

(The full `catkin_make` with nothing changed still takes a while, because it walks
every package. `--pkg` builds just that one.)

Nothing takes effect in a shell that was open before the build unless you
`source ~/ur5e_sim_ws/devel/setup.bash` again — and nothing takes effect in a
*running* node at all. Stop the launch file and start it again.

---

## 4. Recipes for the changes you will actually make

### Change a control gain

`sim_control/sim_pid_control/config/pid_params.yaml` (PID) or
`sim_control/sim_admittance_control/config/admittance_params.yaml` (admittance).
The 6×6 matrices are written row-major as a flat list, so position 0 is x, 7 is y,
14 is z, and so on down the diagonal. No rebuild; re-sync and restart the
controller launch. Confirm it arrived:

```bash
rosparam get /K_p
rosparam get /mass_arm
```

### Change how the Cartesian IK behaves near a singularity

`sim_control/sim_pid_control/config/sim_vel_controller.yaml` (and its identical
copies under `sim_admittance_control` and `sim_cartesian_velocity_control`), in both
`sim_cartesian_velocity_controller` blocks:

| Parameter | Default | What it does |
|---|---|---|
| `ik_sigma_min` | 0.05 | The singular value at which damping starts. Higher damps earlier and more often, at the cost of tracking everywhere; 0.3 already costs 10 % of the commanded speed at a well-conditioned pose. 0 disables damping and gives you the old undamped pseudo-inverse. |
| `ik_lambda` | 0.2 | How hard it damps once below that threshold. |
| `max_joint_velocity` | 1.0 | Hard cap on the norm of the joint velocity, applied after the solver. `<=0` disables it. |
| `velocity_filter_cutoff` | 30.0 | Low pass on the joint velocity, which these controllers differentiate from the measured position because Gazebo's reported velocity is unusable (README §6). `<=0` turns the filter off. Also read by `sim_cartesian_state_controller`. |

No rebuild; re-sync and restart. The controller prints the three values when it
initialises, and warns (throttled) whenever it is solving near a singularity.

### Disturb the arm's base on purpose

```bash
roslaunch sim_bringup gantry_disturbance.launch axis:=y amplitude:=0.02 frequency:=1.0 duration:=20
```

On top of a running bringup, with whichever arm controller you are testing
already started. It loads a velocity controller for the four gantry axes and
drives one of them sinusoidally, ramped in and out so the run does not open with
a step. `axis` is x, y, z or r; `amplitude` is the excursion, not the velocity.

Measure the end effector on `/ee_pose_ground_truth`, which is in world.
`ee_state` is in `base_link` and shows almost nothing, because that is exactly
the blind spot this disturbance exploits - see README section 4.4 for the
numbers and what they mean.

### Turn the model's position feedforward up or down

`ff_gravity` and `ff_coriolis` in the `sim_cartesian_velocity_controller*` blocks
of `sim_vel_controller.yaml`, both 1.0. They weight the `(G + C qdot)/p` offset
the Cartesian controller adds to its position command to cancel the joint
servos' deflection; 0 disables a term. `joint_stiffness` is the `p` and is read
from `/gazebo_ros_control/pid_gains/<joint>/p` unless you set it, so if you
retune the joint gains the compensation follows automatically. No rebuild;
re-sync and restart.

The controller says at startup whether it is on, and says so too when it finds
no stiffness - which is correct and not a fault when the joints are positioned
kinematically. README section 4.4 has what it buys and what it does not.

### Change how stiff the arm is

`sim/sim_bringup/config/gazebo_joint_pid_gains.yaml`, one `{p, i, d, i_clamp}`
per joint. This is the plant, not a controller: it sets how hard the joint pushes
to reach its commanded position, so it decides the gravity sag, the compliance
under an external force, and whether a dynamics model has anything to feed
forward against. The file explains how the numbers were picked and why `i` is
zero; read it before changing them. No rebuild, but these have to be on the
parameter server before the model spawns, so restart the whole bringup, not just
a controller.

Two failure modes to recognise. Gains too high: the PID output saturates against
the joint's effort limit, the derivative term can no longer damp anything, and
the arm and the gantry slam against each other at full torque - you will see
+/-150 Nm in `/joint_states` efforts from the first sample. Gains too low: the
arm sags visibly and the trajectory controller aborts on its 0.1 rad tolerance.
Size `p` from the sag you want, `p = (gravity torque)/(sag)`, and `d = 2 J w`
with `w = sqrt(p/J)`.

Going back to the old kinematic behaviour is `joint_pid_gains:=false` on the
bringup.

### Change which frame the target is fixed in

`regulate_in_world` in `sim_control/sim_pid_control/config/pid_params.yaml`. True
(the default) carries the equilibrium into world when it arrives and regulates
there, so the gantry moving the arm's base is an error the feedback can see.
False regulates in `base_link`, where it cannot. No rebuild; re-sync and restart
the controller launch. The node says which one it is doing when it starts.

It needs `/base_pose_ground_truth`. If that never arrives the controller warns,
throttled, and regulates in `base_link` instead - so a setup without it degrades
rather than misbehaves.

README section 4.4 has the measured rejection at three frequencies, and why the
two settings are good at opposite ends of the band.

### Switch the model feedforward on

`A_inertia`, `A_coriolis` and `A_gravity` in
`sim_control/sim_pid_control/config/pid_params.yaml`, 6x6 admittances that turn
each model wrench into a velocity contribution. All zero means the law is exactly
what it was before the model existed, which is the default; set one to a small
diagonal to bring that term in. No rebuild; re-sync and restart the controller
launch. The node prints which terms are on when it starts, and warns (throttled)
if they are on while the dynamics are unusable.

Before spending time tuning them, read README section 6 on why this plant has no
dynamics for them to correct. A quick way to confirm the path is live at all:
set `A_gravity` to 1e-4 on the diagonal and check that the arm settles about
0.58 mm high in z, which is the published gravity wrench through that admittance
divided by `K_p`.

### Change what the dynamics controller models

`sim_cartesian_dynamics_controller` in the same yaml files. `sigma_min` sets when
it marks its messages invalid, `pinv_rcond` the relative tolerance of the
pseudo-inverse of `J M^-1 J^T`, `gravity` the gravity vector in `root_name`, and
`include_tip_payload` whether to fold everything bolted past `tip_name` into the
model — leave that on, it is worth 29 % of the gravity torque here (README §4.4).
Changing `tip_name` moves where the dynamics are evaluated and changes what counts
as payload. No rebuild; re-sync and restart.

### Change the scene

The world is chosen in the bringup launch file. `with_moveit_new.launch` loads
`sim_gazebo/world/one_brick.world`; `gazebo_test.launch` takes it as an argument:

```bash
roslaunch sim_bringup gazebo_test.launch world:=$(rospack find sim_gazebo)/world/example.world
```

To add your own, drop a `.world` next to those and point `world_name` at it. Note
that a world referencing `model://something` pulls that model from Gazebo's online
database — `puzzle.world` fails exactly this way.

### Change where or how the robot is spawned

In `with_moveit_new.launch`, the `urdf_spawner` node's `args`: `-x -y -z` place the
model. `-J <joint> <value>` is supposed to set a starting joint angle, but it has no
effect here — `gazebo_ros_control` overwrites it on the first physics step, so the
arm always comes up at the URDF zero pose (README §6). To start somewhere else, send
a trajectory to `arm_joint_controller` after bringup (§5 below). `-model sim` is the name
Gazebo knows the robot by — if you change it, change it in
`sim_bringup/scripts/init_pose_start_controllers.py` too (or pass
`_model_name:=...`), because that script addresses the model by name.

### Add a named pose

`sim_planning/sim_moveit_config/config/sim.srdf`, next to the existing
`<group_state>` entries (`init`, `home1`, `back`, `finale`, `1st_reco_start`,
`2nd_reco_start`, `open`, `close`):

```xml
<group_state name="my_pose" group="manipulator">
    <joint name="shoulder_pan_joint" value="0.5" />
    <!-- ... all six joints ... -->
</group_state>
```

No rebuild. Restart the launch file, then `arm.setNamedTarget("my_pose")` in C++,
`arm.set_named_target("my_pose")` in Python, or pick it from the RViz dropdown.
Editing the SRDF by hand is fine; `roslaunch sim_moveit_config setup_assistant.launch`
does it with a GUI.

### Change the robot model

`sim/sim_description/urdf/xacro/main.xacro` is the top level; it pulls in
`ur5e.urdf.xacro`, the gripper, the transmissions and `common.gazebo.xacro`. Check
your edit parses before launching anything:

```bash
xacro ~/ur5e_sim_ws/src/ur5e_simulation/sim/sim_description/urdf/xacro/main.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

Expect `robot name is: sim` and `Successfully Parsed XML`. The
`material 'flat_black' undefined` warning is pre-existing and harmless.

### Add a planning or control node

1. Write `src/my_node.cpp` in the package.
2. Add it to that package's `CMakeLists.txt`:

   ```cmake
   add_executable(my_node src/my_node.cpp)
   target_link_libraries(my_node ${catkin_LIBRARIES})
   add_dependencies(my_node ${catkin_EXPORTED_TARGETS})
   ```

   The `add_dependencies` line is not optional if your node includes a generated
   header such as `mission_mode/mode_switch.h`. Without it, `make -j4` races
   against message generation and fails with
   `mission_mode/mode_switch.h: No such file or directory`. This bit the four
   existing nodes in `sim_general_planning`.
3. Write a launch file for it in `launch/`.
4. Re-sync and `catkin_make --pkg <that package>`.
5. Check the binary exists:
   `ls ~/ur5e_sim_ws/devel/lib/<package>/my_node`.

### Make a node parameterisable instead of hard-coding

The pattern used in `general_arm_planning.cpp`:

```cpp
std::string arm_state_topic;
n_.param<std::string>("arm_state_topic", arm_state_topic,
                      "/sim_cartesian_velocity_controller_sim/ee_state");
```

Then override it from a launch file with
`<param name="arm_state_topic" value="..."/>`, and no rebuild is needed to change
the value.

---

## 5. Driving the robot by hand

### MoveIt, from RViz

MotionPlanning panel → drag the interactive marker → **Plan & Execute**. The
*Planning* tab's *Goal State* dropdown also offers the SRDF named poses.

### One joint at a time

```bash
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

Pick `arm_joint_controller`, press the power button, drag the sliders.

### Named poses in a loop

Useful when you just want the robot moving while you watch. Save as
`/tmp/cycle.py` and run it with `python3 /tmp/cycle.py` on top of a running
bringup:

```python
#!/usr/bin/env python3
import sys, rospy, moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("named_target_cycle", anonymous=True)
arm = moveit_commander.MoveGroupCommander("manipulator")
ee = moveit_commander.MoveGroupCommander("endeffector")
arm.set_max_velocity_scaling_factor(0.5)

poses = ["init", "home1", "1st_reco_start", "2nd_reco_start", "back", "finale"]
grip = ["close", "open"]
i = 0
while not rospy.is_shutdown():
    p, g = poses[i % len(poses)], grip[i % len(grip)]
    rospy.loginfo("arm -> %s", p)
    arm.set_named_target(p); ok = arm.go(wait=True); arm.stop()
    rospy.loginfo("arm %s: %s", p, "ok" if ok else "FAILED")
    ee.set_named_target(g); ee.go(wait=True); ee.stop()
    i += 1
```

Roughly 20 s per pose, most of it planning. Note that `rospy.loginfo` goes to
stdout, which Python block-buffers when you redirect it to a file — if the log
file looks empty, the node is probably fine and the output is just stuck in the
buffer. `~/.ros/log/latest/rosout.log` shows it immediately.

### The Cartesian controllers

The Cartesian launch files only *load* their controller; they do not start it, and
`arm_joint_controller` holds the same six joints:

```bash
roslaunch sim_pid_control pid_controller.launch          # loads the controller
rosrun controller_manager controller_manager list        # -> "initialized", not running
rosrun controller_manager controller_manager stop arm_joint_controller
rosrun controller_manager controller_manager start sim_cartesian_velocity_controller_sim
```

`roslaunch sim_bringup init_pose_start_controllers.launch` is meant to do the pose
and the switch in one go, but only the switch works (README §6) — place the arm with
a trajectory first if you need a particular pose. Afterwards MoveIt cannot execute
until you switch back:

```bash
rosrun controller_manager controller_manager stop sim_cartesian_velocity_controller_sim
rosrun controller_manager controller_manager start arm_joint_controller
```

With the Cartesian controller running, commanding the end effector directly:

```bash
rostopic pub -r 10 /sim_cartesian_velocity_controller_sim/command_cart_vel \
  geometry_msgs/Twist '{linear: {x: 0.02, y: 0.0, z: 0.0}}'
```

### Feeding the pick-and-place demo

`x_arm_planning` waits for an object pose that no node in this repository
publishes. You can stand in for the missing vision node — the message is a
`PoseStamped` in the camera frame, and `frame_id` is the trigger (`1st` to pick,
`2nd` to place):

```bash
rostopic pub -1 /object_visual_odometry geometry_msgs/PoseStamped \
  '{header: {frame_id: "1st"}, pose: {position: {x: 0.0, y: 0.0, z: 0.30}, orientation: {w: 1.0}}}'
```

The node then prints `Caution: object pose received, now picking!` and tries a
Cartesian path to it. An unreachable guess fails with
`Path planning failed with only 0.0625 success after 100 attempts` and the state
machine moves on, which is a useful way to see the sequence without a camera.

---

## 6. Checking your work

```bash
rostopic list | grep ee_state            # does the topic I expect exist?
rostopic echo -n1 /joint_states          # one sample of every joint
rostopic hz /sim_cartesian_velocity_controller_sim/ee_state
rosnode info /x_arm_planning             # what is it really subscribed to?
rosparam get /K_p                        # did my yaml reach the parameter server?
rosrun controller_manager controller_manager list
```

Ask Gazebo directly, which is how you tell a wrong name from a wrong value:

```bash
rosservice call /gazebo/get_world_properties                    # which models are loaded
rosservice call /gazebo/get_joint_properties "joint_name: 'shoulder_pan_joint'"
rosservice call /gazebo/set_model_configuration \
  "{model_name: sim, urdf_param_name: robot_description,
    joint_names: [shoulder_pan_joint], joint_positions: [1.0]}"
```

A wrong model name answers `success: False` with
`SetModelConfiguration: model does not exist`, which is exactly how the stale
`raigor` name in the init-pose script was found.

Check a launch file without running it:

```bash
roslaunch --files sim_bringup gazebo_test.launch     # every file it pulls in
roslaunch --nodes sim_bringup with_moveit_new.launch # every node it would start
```

Logs: `~/.ros/log/latest/` — `rosout.log` is every node's output together, which
is where to look when a node dies before you can read its terminal. To find the
real error in the `pid_gains` noise:

```bash
grep -iE "error|fail|abort" ~/.ros/log/latest/rosout.log | grep -v "p gain"
```

---

## 7. Stopping and restarting cleanly

Ctrl-C in the terminal running `roslaunch` is the normal way. Over SSH, send the
interrupt to the `roslaunch` process:

```bash
pgrep -f "bin/roslaunch sim_bringup"      # get the pid
kill -INT <pid>
```

Then **wait until everything is actually gone** before launching again:

```bash
pgrep -a -f "gzserve[r]|gzclien[t]|rosmaste[r]"
```

Starting a new `roslaunch` while the old `rosmaster` is still shutting down gives
you a launch with no master and an endless
`Error in XmlRpcClient::writeRequest: write error (Connection refused)`. Shutdown
takes a good ten to twenty seconds.

Two traps worth knowing:

* `pkill -f gzserver` run from a shell whose own command line contains the word
  `gzserver` kills that shell too. Hence the `gzserve[r]` bracket trick above.
* A `gzserver` that survives holds port 11345, and the next Gazebo will not start.
  `kill` it by pid.

---

## 8. Symptom → cause

| Symptom | Cause |
|---|---|
| My code change does nothing | You edited the Windows checkout and did not re-sync, or the node was already running. |
| My change disappeared | You edited `~/ur5e_sim_ws/src/ur5e_simulation` and phase 2 re-synced over it with `rsync --delete`. |
| `bad interpreter: ...^M` | CRLF line endings. `.gitattributes` forces LF; refresh an old checkout with `git rm --cached -r .` then `git reset --hard`. |
| `catkin_make` fails on symlinks | You are building inside the shared folder. Build the copy in `$HOME`. |
| `mission_mode/mode_switch.h: No such file` | A node includes a generated header without `add_dependencies(... ${catkin_EXPORTED_TARGETS})`. |
| Gazebo dies at startup | `SVGA_VGPU10=0` is missing in this shell. |
| `Unable to find uri[model://...]` | The world wants a model from Gazebo's online database. |
| MoveIt plans but nothing moves | `arm_joint_controller` is stopped, or something else holds the joints. |
| The arm thrashes and the gantry flies off on a Cartesian command | The arm is at or near a singularity and `ik_sigma_min` is too low for it. Check the controller's throttled singularity warning. |
| `-J` in the spawner args changes nothing | It cannot; `gazebo_ros_control` overwrites it. Drive the arm with `arm_joint_controller` instead (README §6). |
| `/joint_states` shows a velocity while the arm is still | Gazebo's reported joint velocities are wrong under a position interface (README §6). `ee_state`'s twist no longer uses them; `/joint_states` still does. |
| One arm joint winds to its limit and sits there at full torque | Its URDF limits span more than 2*pi, which gazebo_ros_control's error calculation cannot represent. Keep the span under 2*pi (README section 6). |
| `/joint_states` efforts are pinned at the effort limit from the first sample | The joint PID gains saturate immediately; see "Change how stiff the arm is". |
| The gantry will not move when commanded | `gantry_velocity_controller` is not started, or its effort limit is below what it has to push. Nothing in bringup starts it; `gantry_disturbance.launch` does. |
| The gantry drifts or spins when the arm moves | Its effort limit is below the reaction the arm can produce, so Gazebo's velocity motor loses (README section 6). |
| The arm holds a perfect `ee_state` pose while the tool wanders the room | `ee_state` is in `base_link`. Either set `regulate_in_world: true` or measure on `/ee_pose_ground_truth` (README section 4.4). |
| The arm drops a centimetre the moment the Cartesian controller is started | Its reference is latched from the already-sagged measurement and the servo sags again from there. `ff_gravity: 1` cancels it (README section 4.4). |
| The model feedforward does nothing although the admittances are set | The dynamics are unusable - the arm is at a singularity (it starts at one) or `sim_cartesian_dynamics_controller` is not running. The node says so, throttled to every 5 s. |
| The whole plugin library fails to load, `undefined symbol: ..._6HYBRIDE` | KDL declares some `static const int` members (`ChainJntToJacDotSolver::HYBRID` and friends) without defining them. Binding a reference to one — which `setRepresentation(const int&)` and `setHybridRepresentation()` do — leaves the library undefined. Copy the value into a local first. |
| The PID node never starts, no error | Its constructor blocks until TF `base_link` -> `ee_link` exists. Something has to publish it — `robot_state_publisher`, which full bringup starts and a cut-down rig may not. |
| `Connection refused` spam from XmlRpc | Relaunched before the previous `rosmaster` finished dying. |
| `ROS_DISTRO: unbound variable` | `set -u` in a script that sources ROS. Lift `nounset` around the `source`. |
| The node's log file is empty but it works | Python buffers stdout when redirected. Read `~/.ros/log/latest/rosout.log`. |

---

## 9. Committing

Commit in the Windows checkout, which is the git repository. The build workspace
copy deliberately has no `.git` — phase 2 excludes it — so there is nothing to
commit there and no risk of committing a half-synced snapshot.
