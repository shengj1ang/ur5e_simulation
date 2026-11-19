# ur5e_simulation

**English** · [简体中文](README.zh-CN.md)

Gazebo simulation of a UR5e arm with a Robotiq 85 gripper on a 4-DOF gantry base,
a MoveIt configuration for it, and three different ways of driving the arm:

| Mode | Package | Idea |
|---|---|---|
| Joint trajectory | `sim_moveit_config` | Standard `JointTrajectoryController`, commanded by MoveIt. |
| Cartesian admittance | `sim_admittance_control` | Force-driven compliant motion: a mass-damper-spring model turns an external wrench into a Cartesian velocity. |
| Cartesian PID | `sim_pid_control` | Feedforward + feedback pose tracking (`K_p`/`K_i`/`K_d` plus the `N_p`/`N_d` feedforward terms). |

Both Cartesian controllers are *nodes*. Neither talks to Gazebo directly — they
publish a Cartesian velocity twist, and a custom `ros_control` plugin from
`sim_cartesian_velocity_control` resolves it to joint motion through KDL:

```
     admittance / PID node  --command_cart_vel-->  CartesianVelocityControllerSim  -->  Gazebo
            ^                                       (KDL, ros_control plugin)
            |                                                    |
            +----------------------  ee_state  ------------------+
```

On real hardware the plugin claims a `VelocityJointInterface`
(`CartesianVelocityController`); under Gazebo there is no velocity interface, so
the `...Sim` variant integrates the twist and commands a `PositionJointInterface`
instead. That is why every simulation config refers to
`sim_cartesian_velocity_controller_sim`.

---

## 1. Verified environment

Everything below was installed and run end-to-end in a VMware virtual machine:

| | |
|---|---|
| OS | Ubuntu 20.04 LTS (focal) |
| ROS | Noetic |
| Simulator | Gazebo 11 |
| Planning | MoveIt 1.1 (`ros-noetic-moveit`) |
| Graphics | VMware SVGA, software GL (`SVGA_VGPU10=0`) |

**Do you have to use 20.04?** For the scripted path, yes. ROS Noetic ships binary
packages for focal only, so `setup/01_install_ros_and_deps.sh` refuses to run on
any other release. The original upstream README also mentioned Ubuntu 18.04 +
Melodic; that combination is not covered by these scripts and was not tested
here. Some leftovers in the tree still point at the Melodic era — the root
`CMakeLists.txt` is the usual catkin toplevel symlink and it still points into
`/opt/ros/melodic` — but nothing in the build uses it, since catkin discovers
packages through their `package.xml`.

---

## 2. What is in the repository

```
sim/
  sim_bringup        launch files that start a whole scene (+ helper scripts)
  sim_description    URDF/xacro of the robot, meshes, RViz configs
  sim_gazebo         Gazebo worlds (one_brick, example, puzzle)
sim_control/
  sim_cartesian_velocity_control  ros_control plugin: Cartesian twist -> joints (KDL)
  sim_admittance_control          admittance controller node + parameters
  sim_pid_control                 Cartesian PID controller node + gains
  sim_general_control             servo/teleop configs (rqt, spacenav)
sim_planning/
  sim_moveit_config     MoveIt config: SRDF, kinematics, controller lists, move_group
  sim_general_planning  MoveIt demo nodes (general_arm_planning, x_arm_planning,
                        twelve_monkeys, hex_love)
  mission_mode          mode_switch.srv, shared by the planning nodes
setup/
  01_install_ros_and_deps.sh  phase 1: apt (needs root)
  02_build_workspace.sh       phase 2: workspace + build (no root)
docs/                         images used by this README
```

The robot model is a UR5e with a Robotiq 85 gripper mounted on a gantry with
`x_joint`, `y_joint`, `z_joint` and `r_joint`. The gantry joints use a velocity
interface and were originally driven by a `machine_tool_control` package which is
**not** part of this repository — see [§6](#6-known-issues).

---

## 3. Quick start

Two scripts, in this order. They are idempotent: re-running them is safe and only
does the work that is still missing.

```bash
git clone https://github.com/shengj1ang/ur5e_simulation.git
cd ur5e_simulation
sudo bash setup/01_install_ros_and_deps.sh     # ~2.5 GB download, 15-40 min
bash setup/02_build_workspace.sh               # no sudo
```

Then open a **new** shell (phase 2 appends the environment to `~/.bashrc`) and:

```bash
roslaunch sim_bringup with_moveit_new.launch
```

Gazebo and RViz come up, the robot is spawned as model `sim` next to a cinder
block, and `arm_joint_controller` + `gripper_controller` are running. You can drag
the interactive marker in RViz and hit **Plan & Execute**.

A full log of phase 1 is kept in `/var/log/ur5e_apt_install.log`.

### Phase 1 — `sudo bash setup/01_install_ros_and_deps.sh`

1. Refuses to continue unless `lsb_release -cs` is `focal`.
2. Pins apt to IPv4 (`/etc/apt/apt.conf.d/99force-ipv4`). Many VM DNS setups hand
   out a bogus AAAA record for the Ubuntu/ROS mirrors, which makes every apt call
   stall for minutes before falling back. Delete that file if your network is
   IPv6-native.
3. Installs build tools, adds the ROS apt source with a dedicated keyring
   (`signed-by=`, since `apt-key` is deprecated on focal), and installs
   `ros-noetic-desktop-full` — this is the 2.5 GB part (Gazebo 11 + RViz).
4. Installs the project's own dependencies, mapped one by one from the
   `package.xml` files: `moveit`, `moveit_visual_tools`,
   `pilz_industrial_motion_planner`, `ros-control` / `ros-controllers`,
   `gazebo-ros-control`, `control_toolbox`, `kdl_parser`, `kdl_conversions`,
   `joint_state_publisher_gui`, `pcl_ros`, `pcl_conversions`, `soem`, `rqt_gui`,
   `rqt_joint_trajectory_controller`, plus `catkin_tools` / `rosdep` / `wstool`.
5. `rosdep init`.

### Phase 2 — `bash setup/02_build_workspace.sh`

1. `rosdep update`, as your normal user. A failure here is not fatal —
   `catkin_make` does not need it.
2. Creates `~/ur5e_sim_ws/src` and runs `catkin_init_workspace`. Override the
   location with `WS=~/somewhere_else bash setup/02_build_workspace.sh`.
3. **Copies** this checkout to `$WS/src/ur5e_simulation` (rsync, with `.git`,
   `build` and `devel` excluded) and strips any CR characters from `*.sh`/`*.py`.
   Why a copy: `catkin_make` has to create symlinks inside `src/`, and VMware /
   VirtualBox shared folders (`vmhgfs`, `vboxsf`) do not support symlinks at all,
   so building a shared-folder checkout in place cannot work. If the checkout
   already lives inside the workspace, nothing is copied.
   **Commit in the original checkout, never in the copy** — the copy has no `.git`
   on purpose.
4. Clones the Gazebo mimic-joint plugin into the workspace:
   [`roboticsgroup_upatras_gazebo_plugins`](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins).
   This is the maintained fork; the plugin named in the old README does not build
   against Gazebo 11. Nothing is copied into `/usr/lib` any more — it is built as
   a normal workspace package.
5. `rosdep check` (informational), then `catkin_make -j$(nproc)`.
6. Symlinks `libroboticsgroup_gazebo_mimic_joint_plugin.so` →
   `libroboticsgroup_upatras_gazebo_mimic_joint_plugin.so` in `devel/lib`, because
   `robotiq_85_gripper.transmission.xacro` still refers to the historical file
   name. Without the alias the gripper's mimic joints do not follow.
7. Appends an environment block to `~/.bashrc`, once:

   ```bash
   source /opt/ros/noetic/setup.bash
   source ~/ur5e_sim_ws/devel/setup.bash
   export SVGA_VGPU10=0     # VMware's GL driver crashes Gazebo 11 without this
   ```

Re-run phase 2 after editing the sources: it re-syncs and rebuilds incrementally.

**Working on the code from here on:** [docs/WORKFLOW.md](docs/WORKFLOW.md) covers
the host/VM split, which changes need a rebuild and which do not, recipes for the
usual edits (gains, worlds, named poses, a new node), how to drive the robot by
hand, and how to find out what went wrong.

---

## 4. Running things

All commands assume `source ~/ur5e_sim_ws/devel/setup.bash` (the `.bashrc` block
does it for you).

### 4.1 Bringup

```bash
# Gazebo + MoveIt + RViz + trajectory controllers (the main entry point)
roslaunch sim_bringup with_moveit_new.launch

# same, but spawns into example.world and loads the Cartesian velocity controller
roslaunch sim_bringup with_admittance.launch

# just a world, no robot and no controllers - the quickest GL/Gazebo smoke test
roslaunch sim_bringup gazebo_test.launch
roslaunch sim_bringup gazebo_test.launch world:=$(rospack find sim_gazebo)/world/example.world
```

Controllers after `with_moveit_new.launch`:

| Controller | State | Type |
|---|---|---|
| `joint_state_controller` | running | `joint_state_controller/JointStateController` |
| `arm_joint_controller` | running | `position_controllers/JointTrajectoryController`, 6 arm joints |
| `gripper_controller` | running | `position_controllers/JointTrajectoryController` |
| `joint_group_position_controller` | loaded, stopped | `position_controllers/JointGroupPositionController` |

`rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller` is the
quickest way to poke individual joints.

### 4.2 MoveIt planning demos

Start these on top of a running `with_moveit_new.launch`:

```bash
roslaunch sim_general_planning x_arm_planning.launch       # pick-and-place state machine
roslaunch sim_general_planning general_arm_planning.launch # equilibrium follower
roslaunch sim_general_planning twelve_monkeys.launch       # path through many object poses
roslaunch sim_general_planning hex_love.launch             # k-means variant of the above
```

The SRDF defines the groups `manipulator` and `endeffector`, with the named states
`init`, `home1`, `back`, `finale`, `1st_reco_start`, `2nd_reco_start`, and
`open`/`close` for the gripper.

What each one actually needs:

* `x_arm_planning` drives the full pre-pick → pick → pre-place → place sequence
  with MoveIt named targets, but the pick pose comes from
  `/object_visual_odometry` and a `mission_mode/mode_switch` service call. Nothing
  in this repository publishes that topic (it came from a vision node), so on its
  own the node moves to `1st_reco_start`, opens the gripper and then waits.
* `general_arm_planning` follows an equilibrium pose and switches between
  `arm_joint_controller` and `sim_cartesian_velocity_controller_sim` itself. It
  reads the arm state from `/sim_cartesian_velocity_controller_sim/ee_state`;
  point it elsewhere (the real robot's controller has no `_sim` suffix) with the
  `arm_state_topic` parameter. It also reads `/base_pose_ground_truth`, which a
  p3d plugin on `base_link` does publish - the gantry is unactuated, so that
  pose drifts rather than staying put (section 6).
* `twelve_monkeys` and `hex_love` are self-contained: they only publish a
  `nav_msgs/Path` and markers for RViz, so they need no controllers at all.

### 4.3 Cartesian control

Each of these loads `sim_cartesian_velocity_controller_sim` and starts its
controller node:

```bash
roslaunch sim_general_planning unity.launch   # PID control + planning node
roslaunch sim_general_planning adm.launch     # admittance control + planning node

# or just the controllers, on top of a running bringup:
roslaunch sim_pid_control pid_controller.launch
roslaunch sim_admittance_control admittance_controller.launch
```

Two things to know:

* These launch files use `controller_manager load`, which **loads but does not
  start** the controller. Start it explicitly:

  ```bash
  rosrun controller_manager controller_manager list
  rosrun controller_manager controller_manager start sim_cartesian_velocity_controller_sim
  ```

* `arm_joint_controller` and `sim_cartesian_velocity_controller_sim` claim the
  same six joints, so only one of them can run. If the start above is rejected,
  stop the trajectory controller first — MoveIt execution stops working until you
  switch back:

  ```bash
  rosrun controller_manager controller_manager stop arm_joint_controller
  ```

  `roslaunch sim_bringup init_pose_start_controllers.launch` is meant to do both
  steps for you — set the arm to the pose in
  `sim_bringup/config/initial_pose.yaml`, then switch from
  `arm_joint_controller` to the Cartesian controller. Only the switch actually
  happens; the pose part is a no-op (§6).

Topics:

| Topic | Direction | Meaning |
|---|---|---|
| `/sim_cartesian_velocity_controller_sim/command_cart_vel` | in | commanded end-effector twist |
| `/sim_cartesian_velocity_controller_sim/ee_state` | out | current end-effector state |
| `/sim_cartesian_dynamics_controller/ee_dynamics` | out | operational-space inertia, Coriolis and gravity (§4.4) |
| `/ft_sensor/data` | in | external wrench for the admittance controller |

The kinematic chain used by the plugin is `base_link` → `ee_link` at 125 Hz
(`sim_vel_controller.yaml` in each control package).

### 4.4 Operational-space dynamics

`sim_cartesian_dynamics_controller` publishes the arm's task-space dynamics so
that a controller can feed the model forward instead of only reacting to error.
It reads the joint states and nothing else, so it claims no joint and runs
alongside whichever controller is driving the arm:

```bash
rosrun controller_manager controller_manager spawn sim_cartesian_dynamics_controller
rostopic echo /sim_cartesian_dynamics_controller/ee_dynamics
```

Each message carries, at the configuration it is stamped for,

| Field | Meaning |
|---|---|
| `inertia` | Λ = (J M⁻¹ Jᵀ)⁻¹, the inertia the arm presents at the end effector |
| `inertia_inv` | J M⁻¹ Jᵀ, which stays finite at a singularity where Λ does not |
| `coriolis` | μ = Λ (J M⁻¹ C q̇ − J̇ q̇) |
| `gravity` | p = Λ J M⁻¹ G, the wrench that holds the arm up |
| `jacobian_sigma_min`, `inertia_rank`, `valid` | whether the numbers mean anything here |

so that `F = inertia · ẍ_desired + coriolis + gravity`.

Three things are worth knowing before using it.

**It is a model of the simulation, not of a UR5e.** The quantities come from
the same URDF Gazebo integrates, so they match the simulated plant exactly.
That description's link masses are the manufacturer's, but its inertia tensors
are cylinder approximations.

**It includes what is bolted to the end effector.** KDL's chain stops at
`ee_link` and carries none of the 0.767 kg of coupler, gripper and camera that
hang off it. Measured at the SRDF `init` pose, folding that in changes the
gravity torque at `shoulder_lift` from 14.645 Nm to 18.848 Nm (+29 %), the
apparent mass at the end effector from 3.40/4.14/7.76 kg to 4.16/4.91/8.52 kg,
and the gravity wrench from 36.9 N to 44.4 N. The controller walks the URDF
subtree past the tip and folds it in; `include_tip_payload: false` turns that
off.

**Λ does not exist at a singularity, and the arm starts at one.** At the spawn
pose J M⁻¹ Jᵀ is rank 5 and the message comes out with `valid: false`, a
`jacobian_sigma_min` of 0.00000 and an `inertia` that is the rank-5
pseudo-inverse — finite, plausible-looking and not the operational-space
inertia. Check `valid` and fall back to whatever you did without a model.
At a well-conditioned pose `inertia · inertia_inv` differs from the identity by
3e-14 and Λ is symmetric to 6e-16.

**Using it as a feedforward.** `sim_pid_control` subscribes to this topic and
forms, in the operational-space way,

```
F = Λ · a_ff + μ + p
```

with `a_ff` the acceleration the arm has to produce at the end effector to leave
it where it is while the gantry moves its base — minus the base acceleration
carried to the end effector, the same signal `N_d` acts on. The arm is commanded
in velocity, so each wrench enters the command as the motion it would produce
through a chosen compliance: `A_inertia`, `A_coriolis` and `A_gravity` in
`pid_params.yaml`, 6×6 admittances in (m/s)/N and (rad/s)/(N·m), one per term so
each can be switched on by itself.

All three are zero by default, which reproduces the law exactly as it was. The
plant does have dynamics now (§6), so that is no longer because there is nothing
to correct; it is because `a_ff` is the base acceleration and the gantry is held
still, so the inertia term has no signal to act on until something drives those
axes — which their raised effort limits now make possible. The gravity term is a
constant that the outer loop already nulls to 2 um. The path is verified end to
end — with `A_gravity` set to
1e-4·I, the measured `p` of 44.442 N in z predicts a steady offset of
44.442 × 1e-4 / `K_p`(z) 7.70 = 0.577 mm, and the arm settles 0.577 mm high in z
on every sample. With the dynamics forced invalid the term is dropped and the
offset goes back to 0.006 mm, with a throttled warning.

**What it does against a real base disturbance.** The arm is mounted on the
gantry, so moving the gantry moves the arm's base while the arm holds its end
effector somewhere. That is the disturbance `N_p` and `N_d` are written against,
and `gantry_disturbance.launch` produces it on purpose:

```bash
roslaunch sim_bringup gantry_disturbance.launch axis:=y amplitude:=0.02 frequency:=1.0
```

It loads a velocity controller for the four gantry axes — nothing else in this
repository drives them — and drives one sinusoidally, ramped in and out. The
quantity to watch is the end effector in **world**, on `/ee_pose_ground_truth`:
`ee_state` is in `base_link`, so the feedback loop cannot see any of this.

Measured at the SRDF `init` pose with the joint PID gains on, y axis, 20 mm
amplitude, peak-to-peak end-effector motion in world as a fraction of the base's
(40.7 mm):

| controller | 1 Hz | rejection |
|---|---|---|
| feedback only (`N_p` = `N_d` = 0) | 101.5 % | -1.5 % |
| + kinematic feedforward (shipped default) | 72.5 % | 27.5 % |
| + model inertia term (`A_inertia` = 0.01·I) | 104.3 % | -4.3 % |

Three things to take from that.

**The feedback alone does nothing at all**, and this is structural rather than a
tuning problem. Its setpoint is a pose in `base_link`; the base moving does not
change that pose, so there is no error to act on and the end effector rides the
gantry one for one.

**The kinematic feedforward works, and its rejection rises with frequency** —
3.6 % at 0.3 Hz, 27.5 % at 1 Hz, 70.1 % at 3 Hz. That shape has the same cause:
holding the end effector still in world means *moving* in `base_link`, which the
`K_p` term opposes. With the plant an integrator, the arm's `base_link`
displacement is `N_p·jw/(jw + K_p)` times the base's, which with `K_p` 7.7 gives
0.23, 0.61 and 0.90 at those three frequencies — the same monotonic rise. The
measured rejection is lower than that throughout, so there is further lag in the
loop this estimate does not capture. The fix is not a larger `N_p`: it is to
regulate against a target fixed in world rather than in `base_link`, so that the
feedback helps instead of fighting.

**The model inertia term, as written, makes it worse**, and the reason is worth
being precise about rather than tuning around:

* The node estimates `a_ff` by backward-differencing the base twist carried to
  the end effector, sampled at whatever `base_pose_update_rate` is - 50 Hz when
  this was measured. Differentiation amplifies noise already present in the
  sampled velocity. If the velocity itself comes from position
  differences, this chain takes a second difference. The installed Noetic
  p3d 2.9.3 publishes Gazebo's link velocity directly. Multiplying the
  acceleration estimate by Λ (5 to 9 kg) feeds its noise into the velocity
  command. With no injected disturbance, enabling the term raised world-frame
  end-effector jitter from 0.006 mm to **6.850 mm** peak-to-peak. [Noetic p3d 2.9.3 source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_p3d.cpp#L250-L328)
* Even without the noise the shape is wrong. The arm is commanded in velocity
  and the command is integrated into a position, so a wrench admitted as a
  velocity arrives as a position 90 degrees out of phase with the deflection it
  was meant to cancel. If the model is to compensate the joint servos' finite
  compliance, it belongs as a *position* feedforward where the integration
  happens — in `cartesian_velocity_controller.cpp`, next to `q_cmd_` — not as a
  velocity admittance one loop out.

So the admittances stay at zero, now for a measured reason rather than an
argued one.

**Fixing the target in world is what moves the number.** `regulate_in_world` in
`pid_params.yaml`, now true by default, carries the equilibrium into the world
frame at the moment it is received and forms the error there, rotating it back
into `base_link` to command. The interface does not change: callers still
publish a `base_link` pose on `/now_equilibrium`, but the request becomes "hold
this point in the room", and the base moving is an error the feedback acts on
instead of being invisible to it. It needs `/base_pose_ground_truth`; without
it the controller falls back to `base_link` and says so.

Same disturbance, same `N_p`/`N_d`, fraction of the base's excursion reaching
the end effector in world — lower is better:

| disturbance | `base_link` | world |
|---|---|---|
| 0.3 Hz | 96.4 % | **1.9 %** |
| 1 Hz | 71.6 % | **7.8 %** |
| 3 Hz | **29.9 %** | 52.1 % |

The two mechanisms reject opposite ends of the band. In `base_link` the
feedforward cancels the disturbance's velocity, so what gets through is
`((1-N_p)jw + K_p)/(jw + K_p)`: at low frequency the `K_p` term dominates and
nothing is rejected, at high frequency only the 3 % that `N_p` = 0.97 leaves. In
world the feedback sees the disturbance directly, so what gets through is
`jw/(jw + K_p)`: excellent low, nothing high. With both, the ideal is the
product `(1-N_p)jw/(jw + K_p)`, and 1.9 % and 7.8 % are close to it. Dropping
the feedforward while keeping world regulation gives 32.9 % at 1 Hz, so it is
genuinely the combination and not either alone.

3 Hz is the exception, and it is worth not papering over: the product predicts
about 3 % and the measurement is 52.1 %, worse than `base_link` alone. At
18.8 rad/s the loop's own lag has turned the extra feedback into a liability -
the sensitivity function peaks above one and the correction arrives out of
phase. The arm's joint servos sit at 28 to 44 rad/s, so 3 Hz is not far below
where they stop following either.

The default is true because the base disturbance this machine actually has is
near DC, where world regulation is at its best and `base_link` regulation
rejects nothing at all. Measured with nothing driving the gantry: the base sinks
at **18.3 um/s**, and with world regulation the arm rises in `base_link` at
14 um/s to hold its world point, so the `base_link` error grows while the tool
stays where it was put. In `base_link` regulation that error would read zero and
the tool would sink with the base. If what you care about is above a couple of
hertz, set it false and rely on the feedforward.

**The model does work, as a position feedforward one loop in.** Not as the
velocity admittance above: as an offset on the position command, inside
`cartesian_velocity_controller.cpp` where that command is formed. The joint loop
is a spring - its torque is `p*(q_cmd - q)` with no integral term - so whatever
torque a joint has to hold shows up as a position error of that torque over `p`.
The dynamics say what the torque is, so adding `(G + C·q̇)/p` to the command
cancels the part of the deflection the model knows about. `ff_gravity` and
`ff_coriolis` in `sim_vel_controller.yaml` weight the two terms, both 1.0 by
default; `joint_stiffness` is the `p`, read from
`/gazebo_ros_control/pid_gains/<joint>/p` unless set explicitly, so the
compensation uses the same number the plant does. With no stiffness anywhere the
feedforward switches itself off, which is the right answer for a kinematically
positioned joint: there is no deflection to compensate.

The clean way to see it is the handover. The trajectory controller leaves the arm
sagging below its command; the Cartesian controller then latches that sagged
position as its own reference, and without compensation the servo sags again
from there. Joint error against the pose that was commanded, mrad:

| | shoulder_lift | elbow | wrist_1 | wrist_3 | ee z |
|---|---|---|---|---|---|
| before handover | 6.49 | 19.89 | 18.51 | -20.14 | |
| after, `ff_gravity: 0` | 13.04 | 39.65 | 35.64 | -40.22 | 0.45888 |
| after, `ff_gravity: 1` | **6.48** | **19.87** | **18.49** | **-20.12** | **0.47328** |

Every joint doubles without it and holds exactly with it, which is the
clearest confirmation available that the offset is the deflection: the second
sag is the same size as the first. 14.4 mm at the end effector. The sign and
size were also predicted before any of this was written - at this pose the
model gives `G` of -18.848 Nm at shoulder_lift and -18.815 Nm at elbow, which
over `p` of 2925 and 940 is -6.4 and -20.0 mrad, against the 6.5 and 20.0 mrad
the arm was measured sitting past its command.

What it does not do, and should not be expected to: it carries no information
about an external force, so it does not stiffen the arm against being pushed.
Under 100 N the end effector still drops 37.5 mm with it on against 35.6 mm
with it off. And with the PID loop closed on top, steady state was already
micrometres, so nothing changes there - a 10 mm step still settles inside 0.5 s.
Where it earns its place is one loop in: the Cartesian controller on its own,
which is also what `sim_admittance_control` and anything commanding twists
directly are using.

**PID simulation enables position feedforward for translational base inertia.**
The PID launch sets `inertia_feedforward` in `sim_vel_controller.yaml`. The
standalone Cartesian and admittance configurations use the plugin default of
false. Start the Cartesian controller and PID as in §4.3 and hold a world-fixed
target with the three outer velocity admittance matrices at zero.

For X, Y and Z disturbances, the gantry node publishes the analytic derivative
of its velocity command, including the ramps, on
`/gantry/base_linear_acceleration`. It is a simulation-time stamped
`geometry_msgs/Vector3Stamped` in m/s² and expressed in `base_link`.
Other sources can publish a translational acceleration using the same interface
or set `base_acceleration_topic`. This model assumes a translating base and a
world-fixed end-effector target.

The payload-aware KDL chain supplies both the joint inertia needed to hold the
tool in world and the inertial coupling from the accelerating base. The
controller converts their sum to a position offset by solving
`D·Δq̇ + P·Δq = τ_inertia` with the Gazebo PD gains read at load time. The
acceleration reference assumes that the arm can track it. Damped IK or a joint
velocity scale below 0.999 suspends the inertial request. The exact velocity cap
still applies at the boundary. An individual joint at its command-lead limit
also gets zero requested torque. Offsets decay through the inverse PD when
requests are suspended or the input expires.

Record `/sim_cartesian_velocity_controller_sim/inertial_feedforward_state`
to see whether the model is active, the raw model torque, the torque target
after gating and clipping, the applied offsets, and per-joint clip counts.
The controller also emits a throttled warning when the inertial torque clips.

For a paired A/B test, start the gantry velocity controller once and keep
`inertia_feedforward: true` in the same bringup. Alternate OFF and ON without
reloading the Cartesian controller or resetting its reference. For OFF,
remap the acceleration publisher as shown below. For ON, omit the remap.

```bash
rosrun sim_bringup gantry_disturbance.py \
  /gantry/base_linear_acceleration:=/suppressed_accel \
  _axis:=y _amplitude:=0.02 _frequency:=3.0 _duration:=6.5 _ramp:=1.0
```

Record both world-frame positions and the acceleration and controller-state
topics. Confirm that OFF receives zero acceleration samples and that the
model is active in ON. Fit each position to a single tone plus an offset and
linear drift over a whole number of periods. Report each pair's OFF minus ON
transmission in percentage points, with the sample standard deviation across
at least five pairs per frequency. Include 1 Hz as the low-acceleration control.

In a headless test at the SRDF `init` pose with a 20 mm Y-axis disturbance,
every state sample in the OFF fit windows reported inactive, and every state
sample in the ON windows reported active. The clip counters stayed at zero
during the frequency trials.

| Frequency | OFF mean | ON mean | Paired OFF − ON, mean ± sample SD (percentage points) |
|---|---:|---:|---:|
| 1 Hz | 8.05% | 7.95% | +0.10 ± 0.14 |
| 3 Hz | 34.09% | 33.45% | +0.64 ± 2.25 |
| 5 Hz | 63.03% | 62.28% | +0.75 ± 1.01 |

An independent run of six pairs across two further bringups found +0.13 points
at 3 Hz and +2.96 at 5 Hz. A paired difference is internally controlled, so
pairs pool across bringups even though legs do not. Pooling every pair from all
three bringups:

| Frequency | Pairs | Pooled OFF − ON, mean ± SE | 95 % interval | Resolved |
|---|---:|---:|---:|---|
| 1 Hz | 8 | −0.04 ± 0.09 | [−0.26, +0.18] | no, and that is the control working |
| 3 Hz | 11 | +0.36 ± 0.57 | [−0.90, +1.62] | no |
| 5 Hz | 11 | +1.96 ± 0.70 | [+0.41, +3.51] | yes, at 95 % |

Every pair above ran OFF then ON, so a baseline drifting across the pair enters
each difference at first order. Measured again in ABBA order - OFF ON ON OFF per
block, where a linear drift cancels exactly between the block's two differences
- in one further bringup:

| Frequency | Blocks | ABBA OFF − ON, mean ± SE | 95 % interval | Resolved |
|---|---:|---:|---:|---|
| 1 Hz | 3 | −0.61 ± 0.16 | [−1.28, +0.07] | no |
| 3 Hz | 7 | +0.05 ± 0.62 | [−1.47, +1.57] | no |
| 5 Hz | 6 | +1.01 ± 1.10 | [−1.81, +3.82] | no |

The order was not the problem. The drift ABBA removed averaged +0.21 points per
segment over the 5 Hz blocks - individual blocks reached ±4 with random sign -
so the fixed-order estimates were not materially biased, and the two designs
differ by sampling rather than by order. ABBA did not reduce the scatter either:
the block-to-block standard deviation at 5 Hz is 2.68 points, the same as the
pair-to-pair standard deviation before it. What varies is not a drift that any
ordering can cancel. It is the arm's own state, at the two-to-three point level,
and it is the floor of this rig.

Combining the two designs by inverse variance, the ABBA estimates being unbiased
and the fixed-order ones measured to be nearly so:

| Frequency | Combined OFF − ON | 95 % interval |
|---|---:|---:|
| 1 Hz | −0.18 ± 0.08 | [−0.33, −0.02] |
| 3 Hz | +0.22 ± 0.42 | [−0.60, +1.04] |
| 5 Hz | +1.69 ± 0.59 | [+0.53, +2.84] |

So the term is worth of order one point at 5 Hz, nothing at 3 Hz, and about two
tenths of a point against you at 1 Hz, where there is no inertial load to
compensate and only the cost of injecting an offset is left. That last row is a
reason to gate the term on the magnitude of the base acceleration rather than
leave it on at all frequencies. The 5 Hz figure excludes zero in the combination
but not in ABBA alone, so it is not settled: narrowing its interval to half a
point would take about 127 ABBA blocks, some two and a half hours of simulation,
against about 47 blocks at 3 Hz.

Two things were confirmed rather than assumed. The controller's own state topic
showed the model active for 100 % of the fit window in 33 of 34 ON segments, the
one exception being a segment the base-amplitude screen had already rejected.
And the clip counters stayed at zero across all 68 ABBA segments, so whatever
the term is worth, that is the whole of the model's effect here rather than a
clipped part of it.

The test pose is a well-conditioned one, which is worth stating because the
ceiling on rejection would otherwise be suspect. At the SRDF `init` pose the
geometric Jacobian, computed from the UR5e DH parameters, has a smallest
singular value of 0.214 and a condition number of 8.6 - the best of the six
named poses in the SRDF, and well clear of the 0.05 at which `ik_sigma_min`
begins to damp.

The analytic acceleration bypasses p3d for this feedforward term. World-frame
feedback still forms its pose error from `/base_pose_ground_truth`, among its
other inputs. That stream ran at 50 Hz for everything above, and its half-sample
delay is about 10.8° at 3 Hz - the largest single term in the loop's phase lag.
That turned out to matter more than anything else in this section.

**Then the rate, the gains, and the question again.** Everything above was
measured at the then-default 50 Hz base-state rate with the original Y-axis
gains. Both have since changed, which re-opens the result. The
[Chinese report](docs/inertial_feedforward_report.html) carries the full
measurement and the per-segment data; the headline is that the model term was
never the binding constraint.

Raising `base_pose_update_rate` from 50 to 200 Hz, with nothing else touched,
moved single-tone transmission at the `init` pose from 33.08 % to 15.75 % at
3 Hz and from 62.73 % to 33.07 % at 5 Hz - paired improvements of 17.33
[15.09, 19.57] and 29.67 [24.42, 34.91] points. Against the one-point-ish the
dynamics model is worth, that is an order of magnitude more, bought by sampling
the base faster rather than by modelling it better. 200 Hz is now the default in
`with_moveit_new.launch`.

At 200 Hz and the original gains the model term still could not be resolved:
OFF − ON came out −0.33 [−1.57, 0.92] points at 3 Hz, −0.52 [−3.45, 2.41] at
5 Hz, and +0.32 [−0.90, 1.53] on a 13-tone multisine. Retuning Y to `K_p` 12.0
and `K_d` 0.35, from 7.7 and 0.10, then cut end-effector RMS by about a third at
both test poses - 1.24 to 0.85 mm against a 20 mm command, a paired
RMS-transmission improvement of 4.76 [2.99, 6.53] points at `init` and 5.51
[3.64, 7.38] at the SRDF pickup pose.

Only with those gains in place does the inertial branch separate from zero in a
single design: at `init` with a 20 mm multisine, turning it on took RMS from
0.82 to 0.76 mm, 0.70 [0.33, 1.06] points of RMS transmission. At 30 mm it is
+0.55 [−1.37, 2.48], still unresolved, and one of the six blocks got worse with
it on. So the model earns its place behind a fast base signal and a tuned loop,
not instead of them.

Two side results are worth keeping. Admitting the wrench through the dynamic
operational-space mapping rather than a static inertia makes no measurable
difference at the tip - all three paired intervals span zero - but it does
roughly halve the joint-level compensation asked for: 4.03 to 2.50 mrad peak at
5 Hz with the base at 50 Hz, 4.34 to 2.65 at 200 Hz. And the feedforward is fed
the *commanded* base acceleration, while the measured one runs about 15 % high
with 11.8-12.7° of lag at 8 Hz, so at the top of the band the term
under-compensates by a known amount. That is the next thing to fix if the band
is to be widened.

### 4.5 Tuning

| File | What |
|---|---|
| `sim_control/sim_pid_control/config/pid_params.yaml` | `K_p`/`K_i`/`K_d`, kinematic feedforward `N_p`/`N_d`, model feedforward `A_inertia`/`A_coriolis`/`A_gravity` (§4.4), `workspace_limits`, `arm_max_vel`, loop rate |
| `sim_control/sim_admittance_control/config/admittance_params.yaml` | `mass_arm`, `damping_arm`, `stiffness_arm`, `equilibrium_point_spring`, `workspace_limits` |
| `sim_planning/sim_moveit_config/config/sim_trajectory_control.yaml` | trajectory controller tolerances and publish rates |
| `sim/sim_bringup/config/initial_pose.yaml` | joint values for the initial-pose helper |
| `sim/sim_bringup/config/gazebo_joint_pid_gains.yaml` | per-joint PID gains for gazebo_ros_control, i.e. how stiff the arm is (§6) |
| `sim/sim_bringup/config/gantry_velocity_controller.yaml` | the only controller for the four gantry axes; used to inject a base disturbance (§4.4) |
| `sim_control/*/config/sim_vel_controller.yaml` | `ik_sigma_min`/`ik_lambda`/`max_joint_velocity`, `velocity_filter_cutoff`, and the dynamics controller's `sigma_min`/`pinv_rcond`/`include_tip_payload`/`gravity` |

All gain matrices are 6×6, written out row-major as a flat list.

Two knobs live in a launch file rather than a YAML. `base_pose_update_rate` on
`with_moveit_new.launch` (default 200.0) sets how fast the `base_link` p3d
plugin publishes `/base_pose_ground_truth`, which feeds both the world-frame
pose error and the kinematic feedforward; it is a xacro argument, so it takes
effect at spawn and changing it means restarting the bringup. Separately,
`inertia_mapping_experiment` on `pid_controller.launch` (default false) exposes
`/use_static_inertia_mapping`, a bool that switches the inertial feedforward
between the dynamic mapping and a static inertia for paired comparisons.

---

## 5. Other demos

### twelve_monkeys

Find a reasonable manipulation path through a large set of object poses:

1. Assume an ellipse with the start and end poses as its foci.
2. Take the *k* nearest object poses.
3. Rank them by distance to the start and end poses.
4. Done.

Blue points: object poses · pink arrows: manipulation poses · green: the path.

![result](docs/img.png)

### hex_love

The same problem solved with k-means instead.

![result](docs/hex_love.png)

---

## 6. Known issues

* **`machine_tool_control` is not in this repository.** `with_moveit_new.launch`
  used to include it unconditionally, which made bringup fail with
  `resource not found` in a plain checkout. The include is now opt-in:

  ```bash
  roslaunch sim_bringup with_moveit_new.launch use_machine_tool:=true
  ```

  Pass that only once the package is in your workspace. Without it the gantry
  joints (`x_joint`, `y_joint`, `z_joint`, `r_joint`) have no controller, and an
  unactuated joint does not stay put: over a long session they creep (measured
  `x_joint` +1.46 m, `y_joint` +4.95 m, `r_joint` -9.35 rad after a few hours),
  which slowly carries the arm base with them. Anything that feeds the base
  motion forward — `sim_pid_control`'s `N_p`/`N_d` terms — reacts to that creep.

* **`No p gain specified for pid` spam on startup.** `gazebo_ros_control` prints
  this for every joint whose `/gazebo_ros_control/pid_gains/...` parameter is
  missing. It is not fatal — with a `PositionJointInterface` Gazebo sets the joint
  position directly. The dynamics parameters in this repository were deliberately
  left alone; add a `pid_gains` block if you want the warnings gone.

* **Gazebo dies immediately under VMware/VirtualBox.** You need
  `export SVGA_VGPU10=0` (phase 2 puts it in `~/.bashrc`). Check the GL stack with
  `glxinfo | grep "OpenGL renderer"`.

* **`bad interpreter: ...^M` or strange xacro parse errors.** A CRLF working tree.
  `.gitattributes` forces `eol=lf` for everything but binary assets; if your
  checkout predates it, refresh it with `git rm --cached -r .` followed by
  `git reset --hard`. Phase 2 also strips CRs from the copy it builds.

* **`puzzle.world` needs a model this machine may not have.** It references
  `model://puzzle_ball_joints`, which is not in the repository and comes from
  Gazebo's online model database; without it the world opens with nothing but the
  ground plane and `Error Code 12 Msg: Unable to find uri[model://puzzle_ball_joints]`
  in the log. `one_brick.world` (cinder block) and `example.world` (ground plane
  only) both load as they are.

* **`/base_pose_ground_truth` moves, and the controllers' feedback does not see
  it.** `main.xacro` carries a second `libgazebo_ros_p3d.so` plugin on
  `base_link`, so the topic is published at `base_pose_update_rate` - 200 Hz by
  default now, 50 Hz when the numbers below were measured - and carries the arm
  base's pose and twist in `world`. Measured: the PID node's callback fires 1410 times
  in one run, with the base motion carried to the end effector coming out at
  about (0.22, 0.06, -0.07) mm/s - the unactuated gantry creeping, `z_joint`
  sinking from -0.0009 m to -0.0025 m over 38 s.

  That matters because `ee_state` reports the end-effector pose in `base_link`,
  so the feedback loop is blind to the base moving: the arm holds its pose
  relative to a base that is itself drifting. The `N_p` and `N_d` feedforward
  terms exist for exactly this, and they are live.

  (Both p3d plugins are also named `p3d_base_controller`. Both load and both
  topics publish, so this is cosmetic, but a duplicate plugin name is not
  something to copy.)

* **The arm's joints are now servos, not teleports, and that change pulled
  three other things with it.** `sim_bringup/config/gazebo_joint_pid_gains.yaml`
  gives the six arm joints `/gazebo_ros_control/pid_gains`, so a position
  command runs through a per-joint PID whose output is a torque clamped to the
  URDF effort limit instead of being satisfied by Gazebo kinematically. Pass
  `joint_pid_gains:=false` to the bringup for the old behaviour.

  What it buys, measured at the SRDF `init` pose with the Cartesian controller
  holding a zero command and a force applied at the end effector:

  | | kinematic (before) | servo (now) |
  |---|---|---|
  | 10 N | 1 um | 3.9 mm |
  | 50 N | ~10 um | 18.7 mm |
  | 100 N | ~10 um | 35.6 mm |
  | effective stiffness | ~1e7 N/m | 2810 N/m |

  Linear in the force, where before it was not a spring at all. The arm also
  droops 29 mm at that pose under its own weight, with each joint sagging by
  its gravity torque over its p gain - 20 mrad by construction, measured 20
  mrad at the elbow and 6.5 mrad at shoulder_lift. The closed loop still holds
  2 um of steady-state error and settles a 10 mm step inside 0.5 s, because the
  velocity command integrates into a position offset that takes the sag out.

  Three things had to change with it, none of them optional:

  **The gantry had to be able to push back.** Its axes were limited to an effort
  of 30, less than the 174 N the arm weighs (z had been raised to 3e7 to paper
  over that). Nothing drives them, so gazebo_ros_control leaves them in VELOCITY
  mode, where Gazebo's joint motor holds zero velocity up to that effort - a
  brake weaker than its own load. It did not matter while the arm exerted no
  reaction on its base; with the gains it does, and shoulder_pan can push 150 Nm
  against r_joint's 30. Measured with the limits still at 30: r_joint span past
  1.2 rad in the first seconds and shoulder_pan sat saturated chasing it. x, y
  and z are now 2000 and r is 500, and the gantry holds to 0.003 rad and 0.03 mm.

  **The arm joint limits had to come inside 2pi.** `main.xacro` passed
  `joint_limited="false"`, which makes the macro ignore the limits given and
  hard-code every arm joint to ±2pi, a span of 4pi. gazebo_ros_control computes
  a revolute joint's error with
  `angles::shortest_angular_distance_with_limits`, which normalises the span and
  so cannot represent a range wider than 2pi: with ±2pi it reads a small error
  as a nearly-full-turn one. Harmless while the error was never used as a
  control signal. With the gains, shoulder_pan - the one arm joint gravity puts
  no torque on, so nothing masked it - wound a full turn into its lower limit
  and sat there saturated at -150 Nm. The limits are ±3.0 rad now, 172 degrees,
  wider than anything in this project asks for.

  **The Cartesian controller's setpoint needed anti-windup.** It integrates its
  own position command, which could not drift from the measurement while the
  plant was kinematic. A servo with a torque limit can fall behind, and then the
  integral climbs while the joint stands still. `max_command_lead` (0.1 rad)
  clamps the command to within that of the measurement and warns, throttled. It
  did not fire in any of the runs above, which is what a backstop should do.

  The gains are sized from saturation, not bandwidth: p is the gravity torque
  over the sag allowed (20 mrad), d is 2*J*sqrt(p/J). A first attempt sized for
  a 60 rad/s loop saturated at 11 mrad of error, and the arm and the gantry
  slammed against each other at full torque from the first physics step. The
  file has the table.

* **Nothing in the bringup can set the arm's startup pose.** Two mechanisms try
  to and neither works, so the arm always comes up at the URDF's own zero pose,
  fully extended, which is the elbow singularity.

  `init_pose_start_controllers.launch` calls `/gazebo/set_model_configuration`,
  the service reports `success: True`, and the arm does not move. Measured: arm
  at the SRDF `init` pose, `initial_pose.yaml` asking for `elbow_joint: 0.0`,
  arm still at `elbow_joint 1.5700` afterwards. Only the controller switch in
  that script has an effect.

  The `-J <joint> <value>` arguments the spawner passes in
  `with_moveit_new.launch` are a no-op as well. That is easy to miss, because
  the file asks for 0 on all six arm joints, which is where the arm would be
  anyway. Measured by asking for something else: spawning with
  `-J shoulder_lift_joint 1.57 -J elbow_joint 1.57 -J wrist_2_joint 1.57
  -J wrist_3_joint 1.57` leaves every one of those joints at 0.000.

  Both have the same cause as the behaviour documented in
  `cartesian_velocity_controller.cpp`: `gazebo_ros_control`'s
  `DefaultRobotHWSim` keeps a position command for every joint of a
  `PositionJointInterface` and writes it to Gazebo every cycle, whether or not
  any controller is running (this model sets no
  `/gazebo_ros_control/pid_gains`, so that write is kinematic). Whatever that
  command holds wins over any teleport.

  To place the arm, drive it there with a controller: send a trajectory to
  `arm_joint_controller` and switch afterwards (docs/WORKFLOW.md §5).

  `init_pose_start_controllers.launch` also takes the arm away from MoveIt,
  which is what it is for — it switches from `arm_joint_controller` to the
  Cartesian velocity controller — but MoveIt execution stops working until you
  switch back (`controller_manager start arm_joint_controller`).

* **The Cartesian IK is damped now; it used not to be, and the arm still starts
  at a singularity.** `cartesian_velocity_controller` solves with
  `KDL::ChainIkSolverVel_wdls` (damped least squares). It used to use
  `KDL::ChainIkSolverVel_pinv_givens`, a plain pseudo-inverse, and combined with
  the startup pose above that was destructive: commanding 0.02 m/s along x from
  the fully extended arm asked for a joint velocity of norm 1.84 rad/s — a gain
  of 92 per second — which flung the unactuated gantry (`y_joint` 1.82 m,
  `r_joint` 15.6 rad, `ee_state` reporting up to 105 m/s) and dragged the end
  effector 0.76 m in the wrong direction.

  With damping, the same command from the same pose tracks x at 0.0200 m/s
  exactly while the gantry joints stay under 1 mm and 0.001 rad, until the arm
  runs out of reach and the solution degrades smoothly instead of exploding. At
  a well-conditioned pose (SRDF `init`) damped and undamped are the same to the
  measurement: both track at 100.0 % with dy and dz of 0.0000 m.

  Three parameters control this, in `sim_vel_controller.yaml` and
  `controllers.yaml`. `ik_sigma_min` (0.05) is the singular value at which
  damping starts; KDL's own default for it is 1e-5, far below anything this
  chain reaches, so damping would never engage and the value has to be chosen
  for the robot. `ik_lambda` (0.2) is the damping factor. `max_joint_velocity`
  (3.14 rad/s, the arm's own limit from the robot description) caps the norm of
  the solution unconditionally. It was 1.0, which never fired at a singularity
  — the damping is what holds those — but did fire tracking a base disturbance,
  where it cost 14.6 points of rejection at 3 Hz. The controller now also warns,
  throttled, when it is solving near a singularity, which it used to do
  silently. The measured singular values behind these defaults are in the
  comment on `init()` in `cartesian_velocity_controller.cpp`.

* **The joint velocities Gazebo reports are not the velocities of the joints.**
  The controllers in `sim_cartesian_velocity_control` no longer use them — they
  differentiate the measured position instead — but `/joint_states` still
  carries the raw value, so anything else that reads it is still reading this.

  `gazebo_ros_control` fills the velocity from Gazebo's `GetVelocity()`. This
  model drives its joints through a `PositionJointInterface` with no
  `/gazebo_ros_control/pid_gains`, so `SetPosition()` moves them kinematically
  and never sets a velocity; what `GetVelocity()` returns is the velocity the
  links would pick up from gravity if the position were not being re-asserted.
  Measured in one run: while `arm_joint_controller` swept `elbow_joint` through
  1.565 rad in 11 s — a true 0.1424 rad/s — the reported velocity averaged
  **0.00022 rad/s**, three orders of magnitude out. At rest it is not zero
  either, and it depends on the pose: `shoulder_lift_joint` reads 0.0416 rad/s
  fully extended and 0.0114 rad/s at the SRDF `init` pose, both while its
  position does not move at all to five decimals. It is not a biased velocity,
  it is unrelated to the velocity.

  `ee_state` used to compute its twist from this. Commanding 0.02 m/s along x
  it reported (+0.0045, 0, -0.0075) m/s — 23 % of the actual speed, with a z
  component while z was not moving. It now reports (+0.0199, +0.0003, -0.0000)
  against a true (+0.0200, 0, 0), and 0.0000 at rest.

  This was not harmless. `sim_pid_control` feeds `ee_state`'s twist into its
  derivative term, so `K_d` was multiplying a constant artefact. Measured on
  the same rig, holding a setpoint: **0.090 mm of steady-state error before,
  0.01–0.04 mm after**, and before it was a fixed offset where after it is
  noise. 0.090 mm is also what the term predicts — `K_d` 0.1 against the
  -0.0075 m/s artefact, divided by `K_p` 7.7, is 0.098 mm. Those are the z
  gains, which the later Y-axis retune left alone.

  One caveat on the replacement. The position interface tracks its setpoint
  exactly, so in simulation the plant between command and position is close to
  a pure integrator, and a velocity differentiated from that position is close
  to the velocity that was commanded. It is the velocity of the simulated
  robot, which is what the controllers need, but it is not independent evidence
  that the robot did what it was told. `velocity_filter_cutoff` (30 Hz) sets
  the low pass on it; the residual ripple at that setting is 0.6 mm/s on a
  20 mm/s command.

* **Parallel builds used to fail** with
  `mission_mode/mode_switch.h: No such file or directory`. The four executables in
  `sim_general_planning` now declare
  `add_dependencies(... ${catkin_EXPORTED_TARGETS})`, so message generation runs
  first. If you add a node that includes a generated header, do the same.

* **`warehouse_ros_mongo` is disabled.** The warehouse launch files in
  `sim_moveit_config` reference it, but the dependency is commented out in
  `package.xml` because it does not build out of the box.

---

## 7. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ERROR: expected Ubuntu 20.04 (focal)` | ROS Noetic has no packages for your release. Use 20.04. |
| apt hangs for minutes per command | IPv6 mirror resolution. Phase 1 writes the IPv4 pin for you. |
| `ROS_DISTRO: unbound variable` while sourcing | `set -u` plus ROS's own profile scripts. Phase 2 lifts `nounset` around the `source` call. |
| `catkin_make` fails creating symlinks | You are building inside a VMware/VirtualBox shared folder. Let phase 2 copy the sources to `$HOME` instead. |
| Gripper fingers do not follow the driven knuckle | The mimic plugin alias is missing. Re-run phase 2 and look for `mimic plugin alias:` in its output. |
| `Could not find the planner configuration 'pilz...'` | `ros-noetic-pilz-industrial-motion-planner` is not installed (phase 1, step 4). |
| MoveIt plans but the robot does not move | Something else is holding the arm joints, or `arm_joint_controller` was stopped. Check `rosrun controller_manager controller_manager list`. |

---

## 8. Repository conventions

* **LF line endings everywhere.** The checkout is routinely used from Windows and
  Linux at the same time (shared folder, WSL, network mount), so `.gitattributes`
  forces `eol=lf` regardless of `core.autocrlf`. `*.stl`, `*.png` and `*.jpg` are
  binary; `*.dae` stays text.
* **Commit in the original checkout**, not in `$WS/src/ur5e_simulation` — that copy
  is a build snapshot without a `.git`.

---

## 9. Credits

Upstream project by Archer Wang (archer7wang@outlook.com) —
<https://github.com/wangarcher/ur5e_simulation>. This fork adds the scripted
Ubuntu 20.04 / ROS Noetic setup, the Python 3 and parallel-build fixes, and this
documentation.
