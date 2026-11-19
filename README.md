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
  `arm_state_topic` parameter. It also expects `/base_pose_ground_truth` from a
  mobile base that is not part of this model, so the base pose stays at zero.
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
| `/ft_sensor/data` | in | external wrench for the admittance controller |

The kinematic chain used by the plugin is `base_link` → `ee_link` at 125 Hz
(`sim_vel_controller.yaml` in each control package).

### 4.4 Tuning

| File | What |
|---|---|
| `sim_control/sim_pid_control/config/pid_params.yaml` | `K_p`/`K_i`/`K_d`, feedforward `N_p`/`N_d`, `workspace_limits`, `arm_max_vel`, 1 kHz loop |
| `sim_control/sim_admittance_control/config/admittance_params.yaml` | `mass_arm`, `damping_arm`, `stiffness_arm`, `equilibrium_point_spring`, `workspace_limits` |
| `sim_planning/sim_moveit_config/config/sim_trajectory_control.yaml` | trajectory controller tolerances and publish rates |
| `sim/sim_bringup/config/initial_pose.yaml` | joint values for the initial-pose helper |

All gain matrices are 6×6, written out row-major as a flat list.

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

* **`/base_pose_ground_truth` is never published.** `general_arm_planning`
  subscribes to it for the mobile base of the robot it was originally written for.
  This model has a fixed gantry instead, so the base pose it uses stays at zero.

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
  (1.0 rad/s) caps the norm of the solution and is an unconditional backstop
  that the other two keep it from reaching. The controller now also warns,
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
  -0.0075 m/s artefact, divided by `K_p` 7.7, is 0.098 mm.

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
