# ur5e_simulation

[English](README.md) · **简体中文**

UR5e 机械臂 + Robotiq 85 夹爪 + 四自由度龙门底座的 Gazebo 仿真，配套 MoveIt
配置，并提供三种驱动机械臂的方式：

| 方式 | 所在包 | 原理 |
|---|---|---|
| 关节轨迹 | `sim_moveit_config` | 标准 `JointTrajectoryController`，由 MoveIt 下发轨迹。 |
| 笛卡尔导纳控制 | `sim_admittance_control` | 力驱动的柔顺运动：用质量-阻尼-弹簧模型把外力/力矩转换成笛卡尔速度。 |
| 笛卡尔 PID 控制 | `sim_pid_control` | 前馈 + 反馈的位姿跟踪（`K_p`/`K_i`/`K_d` 加上 `N_p`/`N_d` 前馈项）。 |

两个笛卡尔控制器都是**普通节点**，它们并不直接和 Gazebo 打交道：节点只发布笛卡尔
速度（twist），由 `sim_cartesian_velocity_control` 里的自定义 `ros_control`
插件用 KDL 解算成关节运动。

```
     导纳 / PID 节点  --command_cart_vel-->  CartesianVelocityControllerSim  -->  Gazebo
          ^                                   (KDL，ros_control 插件)
          |                                                |
          +----------------------  ee_state  --------------+
```

在真实硬件上，这个插件申请的是 `VelocityJointInterface`
（`CartesianVelocityController`）；而 Gazebo 这边没有速度接口，所以 `...Sim`
变体把速度积分后用 `PositionJointInterface` 下发。这就是为什么所有仿真配置里用的
都是 `sim_cartesian_velocity_controller_sim`。

---

## 1. 已验证的环境

以下环境在 VMware 虚拟机里完整装过、跑通过：

| | |
|---|---|
| 操作系统 | Ubuntu 20.04 LTS (focal) |
| ROS | Noetic |
| 仿真器 | Gazebo 11 |
| 规划 | MoveIt 1.1（`ros-noetic-moveit`） |
| 图形 | VMware SVGA，软件 GL（`SVGA_VGPU10=0`） |

**是否必须用 20.04？** 走脚本这条路的话，是的。ROS Noetic 只为 focal 提供二进制
包，所以 `setup/01_install_ros_and_deps.sh` 在别的发行版上会直接拒绝执行。原始
README 里还提到过 Ubuntu 18.04 + Melodic，但这套脚本不覆盖那个组合，这边也没有
测过。仓库里还留着一些 Melodic 时代的痕迹——根目录的 `CMakeLists.txt` 是 catkin
的 toplevel 软链，指向的还是 `/opt/ros/melodic`——不过编译并不会用到它，catkin
是通过各个包的 `package.xml` 来发现包的。

---

## 2. 仓库结构

```
sim/
  sim_bringup        启动整个场景的 launch 文件（以及辅助脚本）
  sim_description    机器人的 URDF/xacro、网格模型、RViz 配置
  sim_gazebo         Gazebo 世界文件（one_brick、example、puzzle）
sim_control/
  sim_cartesian_velocity_control  ros_control 插件：笛卡尔速度 -> 关节（KDL）
  sim_admittance_control          导纳控制节点 + 参数
  sim_pid_control                 笛卡尔 PID 控制节点 + 增益
  sim_general_control             伺服/遥操作配置（rqt、spacenav）
sim_planning/
  sim_moveit_config     MoveIt 配置：SRDF、运动学、控制器列表、move_group
  sim_general_planning  MoveIt 演示节点（general_arm_planning、x_arm_planning、
                        twelve_monkeys、hex_love）
  mission_mode          mode_switch.srv，被上面几个规划节点共用
setup/
  01_install_ros_and_deps.sh  第一阶段：apt 安装（需要 root）
  02_build_workspace.sh       第二阶段：建工作空间并编译（不需要 root）
docs/                         README 里用到的图片
```

机器人模型是一台 UR5e 配 Robotiq 85 夹爪，装在带 `x_joint`、`y_joint`、`z_joint`、
`r_joint` 的龙门上。龙门这几个关节用的是速度接口，原本由 `machine_tool_control`
包驱动，而这个包**不在**本仓库里——见 [§6](#6-已知问题)。

---

## 3. 快速开始

两个脚本，按顺序跑。脚本是幂等的：重复执行是安全的，只会补做还缺的那部分。

```bash
git clone https://github.com/shengj1ang/ur5e_simulation.git
cd ur5e_simulation
sudo bash setup/01_install_ros_and_deps.sh     # 约 2.5 GB 下载，15-40 分钟
bash setup/02_build_workspace.sh               # 不要加 sudo
```

然后**新开一个终端**（第二阶段会把环境写进 `~/.bashrc`），执行：

```bash
roslaunch sim_bringup with_moveit_new.launch
```

Gazebo 和 RViz 会起来，机器人以模型名 `sim` 生成在一块水泥砖旁边，
`arm_joint_controller` 和 `gripper_controller` 处于 running 状态。在 RViz 里拖动
交互标记，点 **Plan & Execute** 就能规划执行。

第一阶段的完整日志留在 `/var/log/ur5e_apt_install.log`。

### 第一阶段 — `sudo bash setup/01_install_ros_and_deps.sh`

1. 先检查 `lsb_release -cs`，不是 `focal` 就直接退出。
2. 把 apt 钉在 IPv4（`/etc/apt/apt.conf.d/99force-ipv4`）。很多虚拟机的 DNS 会给
   Ubuntu/ROS 镜像返回一个无效的 AAAA 记录，导致每次 apt 都要卡几分钟才回落到
   IPv4。如果你的网络原生支持 IPv6，删掉这个文件即可。
3. 安装基础编译工具，用独立 keyring 添加 ROS 软件源（focal 上 `apt-key` 已废弃，
   所以改用 `signed-by=`），然后安装 `ros-noetic-desktop-full`——2.5 GB 主要就是
   这一步（含 Gazebo 11 和 RViz）。
4. 安装本项目自己的依赖，这些是逐条从各个 `package.xml` 里对出来的：`moveit`、
   `moveit_visual_tools`、`pilz_industrial_motion_planner`、
   `ros-control` / `ros-controllers`、`gazebo-ros-control`、`control_toolbox`、
   `kdl_parser`、`kdl_conversions`、`joint_state_publisher_gui`、`pcl_ros`、
   `pcl_conversions`、`soem`、`rqt_gui`、`rqt_joint_trajectory_controller`，以及
   `catkin_tools` / `rosdep` / `wstool`。
5. `rosdep init`。

### 第二阶段 — `bash setup/02_build_workspace.sh`

1. 以普通用户身份执行 `rosdep update`。这一步失败不致命，`catkin_make` 并不需要它。
2. 创建 `~/ur5e_sim_ws/src` 并执行 `catkin_init_workspace`。想换位置就用
   `WS=~/别的路径 bash setup/02_build_workspace.sh`。
3. 把当前这份代码**复制**到 `$WS/src/ur5e_simulation`（用 rsync，排除 `.git`、
   `build`、`devel`），并清掉 `*.sh`/`*.py` 里的 CR 字符。
   为什么要复制：`catkin_make` 必须在 `src/` 里建软链，而 VMware / VirtualBox 的
   共享文件夹（`vmhgfs`、`vboxsf`）完全不支持软链，所以直接在共享目录里编译是不
   可能成功的。如果这份代码本来就在工作空间里，就不会复制。
   **提交代码请在原始 checkout 里做，不要在副本里做**——副本故意不带 `.git`。
4. 把 Gazebo 的 mimic joint 插件克隆进工作空间：
   [`roboticsgroup_upatras_gazebo_plugins`](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins)。
   这是还在维护的那个 fork；旧 README 里提到的那个插件在 Gazebo 11 上编不过。现在
   也不再往 `/usr/lib` 里拷 .so 了——它就是工作空间里一个普通包。
5. `rosdep check`（只是提示性输出），然后 `catkin_make -j$(nproc)`。
6. 在 `devel/lib` 里把 `libroboticsgroup_gazebo_mimic_joint_plugin.so` 软链到
   `libroboticsgroup_upatras_gazebo_mimic_joint_plugin.so`，因为
   `robotiq_85_gripper.transmission.xacro` 里引用的还是旧文件名。没有这个软链，
   夹爪的 mimic 关节不会跟动。
7. 往 `~/.bashrc` 里追加一段环境配置（只追加一次）：

   ```bash
   source /opt/ros/noetic/setup.bash
   source ~/ur5e_sim_ws/devel/setup.bash
   export SVGA_VGPU10=0     # 不加这句，VMware 的 GL 驱动会让 Gazebo 11 崩溃
   ```

改完源码后重跑第二阶段即可：它会重新同步并增量编译。

**接下来怎么改代码**：见 [docs/WORKFLOW.zh-CN.md](docs/WORKFLOW.zh-CN.md)——讲清楚了
Windows 与虚拟机这两边的关系、哪些改动要重新编译哪些不用、常见改动（增益、场景、预设
姿态、新增节点）的具体做法、怎么手动指挥机械臂，以及出问题时怎么自己查。

---

## 4. 运行

以下命令都假设已经 `source ~/ur5e_sim_ws/devel/setup.bash`（`.bashrc` 里那段已经
帮你做了）。

### 4.1 启动仿真

```bash
# Gazebo + MoveIt + RViz + 轨迹控制器（主入口）
roslaunch sim_bringup with_moveit_new.launch

# 同上，但场景是 example.world，并且会加载笛卡尔速度控制器
roslaunch sim_bringup with_admittance.launch

# 只开一个世界，不加载机器人也不加载控制器——验证 GL / Gazebo 最快的办法
roslaunch sim_bringup gazebo_test.launch
roslaunch sim_bringup gazebo_test.launch world:=$(rospack find sim_gazebo)/world/example.world
```

`with_moveit_new.launch` 起来之后的控制器：

| 控制器 | 状态 | 类型 |
|---|---|---|
| `joint_state_controller` | running | `joint_state_controller/JointStateController` |
| `arm_joint_controller` | running | `position_controllers/JointTrajectoryController`，6 个臂关节 |
| `gripper_controller` | running | `position_controllers/JointTrajectoryController` |
| `joint_group_position_controller` | 已加载，未启动 | `position_controllers/JointGroupPositionController` |

想单独试某个关节，最快的办法是
`rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller`。

### 4.2 MoveIt 规划演示

这些都是在 `with_moveit_new.launch` 已经跑起来的基础上再启动：

```bash
roslaunch sim_general_planning x_arm_planning.launch       # 抓取-放置状态机
roslaunch sim_general_planning general_arm_planning.launch # 平衡点跟随节点
roslaunch sim_general_planning twelve_monkeys.launch       # 在大量物体位姿中找操作路径
roslaunch sim_general_planning hex_love.launch             # 上面那件事的 k-means 版本
```

SRDF 里定义了 `manipulator` 和 `endeffector` 两个组，预设姿态有 `init`、`home1`、
`back`、`finale`、`1st_reco_start`、`2nd_reco_start`，夹爪有 `open`/`close`。

这几个节点各自真正需要什么：

* `x_arm_planning` 会用 MoveIt 的预设姿态走完 预抓取 → 抓取 → 预放置 → 放置 整套
  流程，但抓取位姿来自 `/object_visual_odometry` 话题和一次
  `mission_mode/mode_switch` 服务调用。本仓库里没有任何东西发布那个话题（它原本来自
  一个视觉节点），所以单独跑它只会走到 `1st_reco_start`、把夹爪打开，然后一直等。
* `general_arm_planning` 跟随一个平衡点位姿，并且会自己在 `arm_joint_controller`
  和 `sim_cartesian_velocity_controller_sim` 之间切换。它从
  `/sim_cartesian_velocity_controller_sim/ee_state` 读臂的状态；要换成别的话题
  （真机上的控制器没有 `_sim` 后缀），用 `arm_state_topic` 参数指定。它还需要
  `/base_pose_ground_truth`，这个话题由挂在 `base_link` 上的 p3d 插件发布，确实有数据——
  龙门没有控制器，所以那个位姿是在漂，而不是不动（见第 6 节）。
* `twelve_monkeys` 和 `hex_love` 是自包含的：它们只往 RViz 发
  `nav_msgs/Path` 和 marker，完全不需要控制器。

### 4.3 笛卡尔控制

下面这些都会加载 `sim_cartesian_velocity_controller_sim` 并启动对应的控制节点：

```bash
roslaunch sim_general_planning unity.launch   # PID 控制 + 规划节点
roslaunch sim_general_planning adm.launch     # 导纳控制 + 规划节点

# 或者在已经起好的仿真之上，只跑控制器：
roslaunch sim_pid_control pid_controller.launch
roslaunch sim_admittance_control admittance_controller.launch
```

有两点需要注意：

* 这些 launch 用的是 `controller_manager load`，也就是**只加载、不启动**。要手动
  启动：

  ```bash
  rosrun controller_manager controller_manager list
  rosrun controller_manager controller_manager start sim_cartesian_velocity_controller_sim
  ```

* `arm_joint_controller` 和 `sim_cartesian_velocity_controller_sim` 申请的是同样
  那 6 个关节，所以同一时间只能有一个在跑。如果上面的 start 被拒绝，先把轨迹控制器
  停掉——注意这期间 MoveIt 的执行功能会失效，等你切回来才恢复：

  ```bash
  rosrun controller_manager controller_manager stop arm_joint_controller
  ```

  `roslaunch sim_bringup init_pose_start_controllers.launch` 会把这两步一起做掉：
  本意是先把臂摆到 `sim_bringup/config/initial_pose.yaml` 里的姿态，再从
  `arm_joint_controller` 切到笛卡尔控制器。

相关话题：

| 话题 | 方向 | 含义 |
|---|---|---|
| `/sim_cartesian_velocity_controller_sim/command_cart_vel` | 输入 | 末端速度指令 |
| `/sim_cartesian_velocity_controller_sim/ee_state` | 输出 | 末端当前状态 |
| `/sim_cartesian_dynamics_controller/ee_dynamics` | 输出 | 操作空间惯量、科氏力和重力（见 4.4） |
| `/ft_sensor/data` | 输入 | 导纳控制用的外部力/力矩 |

插件使用的运动链是 `base_link` → `ee_link`，频率 125 Hz（见各控制包里的
`sim_vel_controller.yaml`）。

### 4.4 操作空间动力学

`sim_cartesian_dynamics_controller` 把机械臂的任务空间动力学发出来，好让控制器能把模型
前馈出去，而不是只对误差作反应。它只读关节状态，不占用任何关节，因此可以和正在驱动机械臂
的那个控制器同时运行：

```bash
rosrun controller_manager controller_manager spawn sim_cartesian_dynamics_controller
rostopic echo /sim_cartesian_dynamics_controller/ee_dynamics
```

每条消息在它对应的位形下携带：

| 字段 | 含义 |
|---|---|
| `inertia` | Λ = (J M⁻¹ Jᵀ)⁻¹，机械臂在末端呈现出来的惯量 |
| `inertia_inv` | J M⁻¹ Jᵀ，在奇异点上它仍然有限，而 Λ 不存在 |
| `coriolis` | μ = Λ (J M⁻¹ C q̇ − J̇ q̇) |
| `gravity` | p = Λ J M⁻¹ G，把机械臂撑住所需的力旋量 |
| `jacobian_sigma_min`、`inertia_rank`、`valid` | 这些数在这里是否有意义 |

于是 `F = inertia · ẍ_desired + coriolis + gravity`。

用之前有三件事要知道。

**它是仿真的模型，不是真实 UR5e 的模型。** 这些量来自 Gazebo 正在积分的同一份 URDF，所以
它和被仿真的对象严格一致。那份描述里连杆质量是厂家数据，但惯量张量是圆柱体近似。

**它把末端上栓着的东西算进去了。** KDL 的链到 `ee_link` 就停了，耦合器、夹爪和相机那
0.767 kg 全挂在它后面、一点也没算进去。在 SRDF 的 `init` 位姿上实测，把这段折进去之后：
`shoulder_lift` 的重力力矩从 14.645 Nm 变成 18.848 Nm（+29%），末端等效质量从
3.40/4.14/7.76 kg 变成 4.16/4.91/8.52 kg，重力力旋量从 36.9 N 变成 44.4 N。控制器会自己
沿 URDF 走完末端之后的子树把它累加进来；`include_tip_payload: false` 可以关掉。

**奇异点上 Λ 不存在，而机械臂恰好就从奇异点启动。** 在启动位姿上 J M⁻¹ Jᵀ 的秩是 5，消息
带着 `valid: false`、`jacobian_sigma_min` 为 0.00000，`inertia` 是秩 5 的伪逆——有限、看着
像那么回事、但并不是操作空间惯量。请先看 `valid`，不成立时退回到没有模型时的做法。在条件
良好的位姿上，`inertia · inertia_inv` 与单位阵相差 3e-14，Λ 对称到 6e-16。

**怎么把它当前馈用。** `sim_pid_control` 订阅这个话题，按操作空间的写法构造

```
F = Λ · a_ff + μ + p
```

其中 `a_ff` 是"龙门在动的时候，机械臂为了把末端留在原地所必须产生的加速度"，也就是基座
加速度搬到末端之后取负——和 `N_d` 作用的是同一个信号。机械臂是用速度指令驱动的，所以每个
力旋量只能以"它通过某个选定的柔顺性会产生多少运动"的形式进入指令：这个柔顺性就是
`pid_params.yaml` 里的 `A_inertia`、`A_coriolis`、`A_gravity`，6×6 的导纳矩阵，单位是
(m/s)/N 和 (rad/s)/(N·m)，一项一个，所以每项都能单独打开。

三个默认都是零，也就是完全复现原来的控制律。现在这个 plant 已经有动力学了（见第 6 节），
所以默认为零不再是因为"没有东西可纠正"，而是因为 `a_ff` 是基座加速度、而龙门被稳稳按住，
于是惯量项没有信号可作用——直到有东西去驱动那几个轴，而它们被提高的 effort 上限现在让这
件事成为可能。重力项是一个常量，外环已经把它压到 2 um。整条路径是端到端验证过的——把
`A_gravity` 设成 1e-4·I，实测的
`p` 在 z 上是 44.442 N，于是预测稳态偏移 44.442 × 1e-4 ÷ `K_p`(z) 7.70 = 0.577 mm，而机械臂
在每一个采样上都稳定停在 z 方向高 0.577 mm 的位置。把动力学强制标成无效之后，这一项被丢掉、
偏移回到 0.006 mm，并伴随一条限频警告。

**它对真实基座扰动的效果。** 机械臂装在龙门上，所以移动龙门就等于在机械臂保持末端位置的
同时移动它的基座。这正是 `N_p` 和 `N_d` 所针对的扰动，而 `gantry_disturbance.launch`
可以主动制造它：

```bash
roslaunch sim_bringup gantry_disturbance.launch axis:=y amplitude:=0.02 frequency:=1.0
```

它会给龙门四个轴加载一个速度控制器——本仓库里没有别的东西驱动它们——然后让其中一个轴做
带升降沿的正弦运动。要看的量是末端在**世界系**下的位置，即 `/ee_pose_ground_truth`：
`ee_state` 是 `base_link` 下的，反馈回路对这一切完全无感。

在 SRDF `init` 位姿、关节 PID 增益打开、y 轴 20 mm 幅值下实测，末端在世界系的峰峰值位移
占基座峰峰值（40.7 mm）的比例：

| 控制器 | 1 Hz | 抑制率 |
|---|---|---|
| 只有反馈（`N_p` = `N_d` = 0） | 101.5 % | -1.5 % |
| + 运动学前馈（出厂默认） | 72.5 % | 27.5 % |
| + 模型惯量项（`A_inertia` = 0.01·I） | 104.3 % | -4.3 % |

有三件事要从里面读出来。

**只有反馈时完全没有作用**，而这是结构性的，不是整定问题。它的设定点是 `base_link` 下的
一个位姿；基座移动并不改变这个位姿，于是没有误差可作用，末端就一比一地跟着龙门走。

**运动学前馈是有效的，而且抑制率随频率上升**——0.3 Hz 时 3.6 %，1 Hz 时 27.5 %，
3 Hz 时 70.1 %。这个形状是同一个原因造成的：要让末端在世界系里不动，就必须在 `base_link`
里**移动**，而 `K_p` 项会反对这件事。把 plant 当作积分器，机械臂在 `base_link` 下的位移是
基座位移的 `N_p·jw/(jw + K_p)` 倍，`K_p` 取 7.7 时在这三个频率上分别是 0.23、0.61、0.90
——同样的单调上升。实测的抑制率全程都低于这个估计，说明回路里还有这个估计没涵盖的滞后。
解决办法不是把 `N_p` 调大，而是把调节目标放在**世界系**而不是 `base_link` 里，让反馈去
帮忙而不是对抗。

**模型惯量项照目前的写法会让结果变差**，而且原因值得说清楚，而不是绕着它去整定：

* 它的输入是 `a_ff`，也就是搬到末端的基座加速度，而节点是对一个"p3d 插件已经通过对位姿
  求差得到的"基座速度再求一次差。把这个二阶导乘上 Λ（5 到 9 kg），噪声就直接进了速度指令。
  在完全没有扰动的情况下，末端在世界系的抖动峰峰值从 0.006 mm 变成了 **6.850 mm**。
* 就算没有噪声，形状也是错的。机械臂是用速度指令驱动的，而指令会被积分成位置，所以一个
  以导纳方式进入的力旋量，最终到达位置时和它本想抵消的那个形变相差 90 度。如果要让模型去
  补偿关节伺服的有限柔顺，它应该作为**位置**前馈出现在做积分的那一层——
  `cartesian_velocity_controller.cpp` 里 `q_cmd_` 旁边——而不是外面一层的速度导纳。

所以这三个导纳继续保持为零，现在是基于实测的理由，而不是推理出来的理由。

**真正改变数字的是把目标钉在世界系里。** `pid_params.yaml` 里的 `regulate_in_world`
（现在默认为 true）会在收到 equilibrium 的那一刻把它搬到世界系，在那里构造误差，再旋回
`base_link` 发出指令。接口没有变：调用方仍然在 `/now_equilibrium` 上发 `base_link` 下的
位姿，但这个请求的含义变成了"把这个点钉在房间里"，于是基座移动成了反馈能看到并作用的误差，
而不再是它的盲区。它需要 `/base_pose_ground_truth`；没有的话控制器会退回 `base_link`
并明确报出来。

同样的扰动、同样的 `N_p`/`N_d`，到达末端（世界系）的部分占基座行程的比例——越小越好：

| 扰动 | `base_link` | 世界系 |
|---|---|---|
| 0.3 Hz | 96.4 % | **1.9 %** |
| 1 Hz | 71.6 % | **7.8 %** |
| 3 Hz | **29.9 %** | 52.1 % |

这两种机制抑制的是频带的两端。在 `base_link` 下前馈抵消的是扰动的速度，于是透过来的是
`((1-N_p)jw + K_p)/(jw + K_p)`：低频时 `K_p` 项主导、什么也抑制不了，高频时只剩
`N_p` = 0.97 留下的那 3 %。在世界系下反馈直接看到扰动，透过来的是 `jw/(jw + K_p)`：
低频极好、高频无效。两者同时存在时，理想情况是它们的乘积 `(1-N_p)jw/(jw + K_p)`，
而 1.9 % 和 7.8 % 已经接近它了。保持世界系目标但去掉前馈，1 Hz 时是 32.9 %——所以起作用的
确实是这个组合，而不是其中任何一个单独。

3 Hz 是例外，而且不值得粉饰：乘积预测约 3 %，实测是 52.1 %，比只用 `base_link` 还差。
在 18.8 rad/s 上，回路自身的滞后把这个额外的反馈变成了负担——灵敏度函数峰值超过 1，
修正到达时相位已经错了。机械臂的关节伺服在 28 到 44 rad/s，所以 3 Hz 离"它们跟不动"也
不远了。

默认设为 true，是因为这台机器实际存在的基座扰动接近直流——那正是世界系目标最强、而
`base_link` 目标完全无效的地方。在没有任何东西驱动龙门的情况下实测：基座以 **18.3 um/s**
持续下沉，而在世界系目标下机械臂在 `base_link` 里以 14 um/s 上升去守住它的世界系点，
于是 `base_link` 下的误差在增长，而刀尖留在了原处。换成 `base_link` 目标，那个误差会读作零，
刀尖会跟着基座一起沉下去。如果你关心的是两赫兹以上，把它设成 false，靠前馈。

### 4.5 参数在哪里调

| 文件 | 内容 |
|---|---|
| `sim_control/sim_pid_control/config/pid_params.yaml` | `K_p`/`K_i`/`K_d`、前馈 `N_p`/`N_d`、`workspace_limits`、`arm_max_vel`，1 kHz 控制周期 |
| `sim_control/sim_admittance_control/config/admittance_params.yaml` | `mass_arm`、`damping_arm`、`stiffness_arm`、`equilibrium_point_spring`、`workspace_limits` |
| `sim_planning/sim_moveit_config/config/sim_trajectory_control.yaml` | 轨迹控制器的容差和发布频率 |
| `sim/sim_bringup/config/initial_pose.yaml` | 初始位姿辅助脚本用的关节值 |
| `sim_control/*/config/sim_vel_controller.yaml` | `ik_sigma_min`/`ik_lambda`/`max_joint_velocity`、`velocity_filter_cutoff`，以及动力学控制器的 `sigma_min`/`pinv_rcond`/`include_tip_payload`/`gravity` |
| `sim/sim_bringup/config/gazebo_joint_pid_gains.yaml` | gazebo_ros_control 的逐关节 PID 增益，也就是机械臂有多硬（见第 6 节） |
| `sim/sim_bringup/config/gantry_velocity_controller.yaml` | 龙门四轴唯一的控制器，用来注入基座扰动（见 4.4） |

所有增益矩阵都是 6×6，按行优先展平成一个一维列表写的。

---

## 5. 其他演示

### twelve_monkeys

在一大堆物体位姿中找一条还算合理的操作路径：

1. 假设有一个以起点位姿和终点位姿为焦点的椭圆。
2. 取最近的 k 个物体位姿。
3. 按到起点和终点的距离排序。
4. 完成。

蓝点：物体位姿 · 粉色箭头：操作位姿 · 绿色：操作路径。

![result](docs/img.png)

### hex_love

同一个问题，换成 k-means 来做。

![result](docs/hex_love.png)

---

## 6. 已知问题

* **`machine_tool_control` 不在本仓库里。** `with_moveit_new.launch` 原来会无条件
  include 它，结果在干净的 checkout 里启动会直接报 `resource not found`。现在这个
  include 改成了可选项：

  ```bash
  roslaunch sim_bringup with_moveit_new.launch use_machine_tool:=true
  ```

  只有当这个包确实在你的工作空间里时才需要加这个参数。不加的话，龙门那几个关节
  （`x_joint`、`y_joint`、`z_joint`、`r_joint`）就没有控制器——而没有控制器的关节
  并不会待着不动：长时间开着会慢慢漂移（实测跑几个小时后 `x_joint` +1.46 m、
  `y_joint` +4.95 m、`r_joint` -9.35 rad），机械臂基座就跟着被带走。任何把底座运动
  作为前馈输入的东西（`sim_pid_control` 的 `N_p`/`N_d` 项）都会去响应这个漂移。

* **启动时刷屏的 `No p gain specified for pid`。** 只要某个关节缺
  `/gazebo_ros_control/pid_gains/...` 参数，`gazebo_ros_control` 就会打这条。它不
  致命——用 `PositionJointInterface` 时 Gazebo 是直接设置关节位置的。本仓库里的动力
  学参数被有意保留原样；想消掉这些警告，自己加一段 `pid_gains` 配置即可。

* **在 VMware/VirtualBox 下 Gazebo 一起来就挂。** 需要
  `export SVGA_VGPU10=0`（第二阶段已经写进 `~/.bashrc`）。可以用
  `glxinfo | grep "OpenGL renderer"` 看一下 GL 栈。

* **`bad interpreter: ...^M` 或者莫名其妙的 xacro 解析错误。** 说明工作区是 CRLF。
  `.gitattributes` 已经对除二进制资源以外的所有文件强制 `eol=lf`；如果你的 checkout
  比这个配置更早，用 `git rm --cached -r .` 再 `git reset --hard` 刷新一遍。第二阶段
  也会把它复制出去的那份里的 CR 清掉。

* **`puzzle.world` 依赖一个本机可能没有的模型。** 它引用了
  `model://puzzle_ball_joints`，这个模型不在仓库里，要从 Gazebo 的在线模型库下载；
  拿不到的话，世界里除了地面什么都没有，日志里会出现
  `Error Code 12 Msg: Unable to find uri[model://puzzle_ball_joints]`。
  `one_brick.world`（水泥砖）和 `example.world`（只有地面）都是开箱能加载的。

* **`/base_pose_ground_truth` 是在动的，而控制器的反馈看不见它。** `main.xacro` 里还有
  第二个 `libgazebo_ros_p3d.so` 插件挂在 `base_link` 上，所以这个话题以 50 Hz 发布，内容是
  臂基座在 `world` 下的位姿和速度。实测：一次运行里 PID 节点的回调触发了 1410 次，底盘运动
  搬到末端之后约为 (0.22, 0.06, -0.07) mm/s——没有控制器的龙门在缓慢爬行，`z_joint` 在 38 秒
  里从 -0.0009 m 沉到 -0.0025 m。

  这件事要紧，是因为 `ee_state` 报的是末端在 `base_link` 下的位姿，于是反馈回路对"基座在动"
  完全无感：机械臂相对于一个本身在漂的基座保持住了位姿。`N_p` 和 `N_d` 这两个前馈项正是
  为此存在的，而且它们是活的。

  （两个 p3d 插件的 `name` 都叫 `p3d_base_controller`。两个都加载了、两个话题都在发，所以
  这只是写法问题，但重名插件不是值得照抄的东西。）

* **臂关节现在是伺服，不再是"传送"，而这一改动牵出了另外三件事。**
  `sim_bringup/config/gazebo_joint_pid_gains.yaml` 给六个臂关节配了
  `/gazebo_ros_control/pid_gains`，于是位置指令会经过一个逐关节的 PID，输出是被 URDF
  effort 上限夹住的力矩，而不是由 Gazebo 以运动学方式直接满足。给 bringup 传
  `joint_pid_gains:=false` 可以回到原来的行为。

  换来了什么——在 SRDF 的 `init` 位姿上、笛卡尔控制器保持零指令、在末端施加外力时实测：

  | | 运动学（之前） | 伺服（现在） |
  |---|---|---|
  | 10 N | 1 um | 3.9 mm |
  | 50 N | ~10 um | 18.7 mm |
  | 100 N | ~10 um | 35.6 mm |
  | 等效刚度 | ~1e7 N/m | 2810 N/m |

  对力是线性的，而之前它根本不是一根弹簧。机械臂在那个位姿上也会因自重下坠 29 mm，
  每个关节的稳态偏差等于它的重力力矩除以 p 增益——按构造是 20 mrad，实测 elbow 20 mrad、
  shoulder_lift 6.5 mrad。闭环仍然保持 2 um 的稳态偏差、10 mm 阶跃 0.5 秒内收敛，因为
  速度指令被积分成位置偏置，正好把下坠补掉。

  跟着必须改的三件事，没有一件是可选的：

  **龙门得撑得住。** 它的轴 effort 上限只有 30，而机械臂自重 174 N（z 轴之前被改成 3e7
  来掩盖这件事）。没有东西驱动这些轴，所以 gazebo_ros_control 把它们留在 VELOCITY 模式，
  由 Gazebo 的关节马达以不超过该 effort 的力维持零速度——也就是一个比自身负载还弱的刹车。
  在手臂对基座完全不施加反作用时这无所谓；配了增益之后就有所谓了，shoulder_pan 能推
  150 Nm 去顶 r_joint 的 30。上限仍为 30 时实测：r_joint 几秒内转过 1.2 rad，
  shoulder_pan 一直饱和着追它。现在 x、y、z 是 2000，r 是 500，龙门稳定在 0.003 rad
  和 0.03 mm。

  **臂关节限位得缩进 2pi 以内。** `main.xacro` 传的是 `joint_limited="false"`，这会让那个
  宏忽略传进去的限位、把每个臂关节硬编码成 ±2pi，跨度 4pi。而 gazebo_ros_control 用
  `angles::shortest_angular_distance_with_limits` 算旋转关节的误差，这个函数会对跨度做
  归一化，因此无法表示超过 2pi 的范围：在 ±2pi 下，一个很小的误差会被读成"差了将近一整圈"。
  误差从来不被当作控制量时这是无害的。配了增益之后，shoulder_pan——唯一一个重力不施加
  力矩的臂关节，所以没有任何东西掩盖它——整整绕了一圈顶到下限位，并在那里以 -150 Nm 饱和。
  限位现在是 ±3.0 rad，172 度，比这个项目用到的任何位姿都宽。

  **笛卡尔控制器的设定点需要抗积分饱和。** 它积分自己的位置指令，而在运动学 plant 上这个
  指令不可能和测量值分开。带力矩上限的伺服是会落后的，那时积分会一直爬而关节原地不动。
  `max_command_lead`（0.1 rad）把指令夹在测量值的这个范围内，并限频告警。上面所有运行里
  它一次都没触发——兜底就该是这样。

  增益是按"不饱和"而不是按带宽选的：p 等于重力力矩除以允许的稳态偏差（20 mrad），
  d = 2*J*sqrt(p/J)。第一次按 60 rad/s 的带宽来选，结果在 11 mrad 误差处就饱和，手臂和
  龙门从第一个物理步起就以全力矩互顶。那个文件里有表。

* **bringup 里没有任何一种办法能设定机械臂的启动位姿。** 有两处尝试去设，两处都无效，
  于是机械臂每次都停在 URDF 自己的零位——完全伸直，也就是肘部奇异位形。

  `init_pose_start_controllers.launch` 调 `/gazebo/set_model_configuration`，服务返回
  `success: True`，而机械臂并没有动。实测：机械臂停在 SRDF 的 `init` 位姿，
  `initial_pose.yaml` 要求 `elbow_joint: 0.0`，跑完之后机械臂仍然在 `elbow_joint 1.5700`。
  这个脚本里真正生效的只有控制器切换那一步。

  `with_moveit_new.launch` 里 spawner 传的 `-J <关节> <值>` 同样是空操作。这一点很容易
  看漏，因为那个文件给六个臂关节要的都是 0，而机械臂本来就在 0。换个值就看得出来了：
  用 `-J shoulder_lift_joint 1.57 -J elbow_joint 1.57 -J wrist_2_joint 1.57
  -J wrist_3_joint 1.57` 启动，这几个关节实测全是 0.000。

  两者的成因，和 `cartesian_velocity_controller.cpp` 里记下的那条是同一个：
  `gazebo_ros_control` 的 `DefaultRobotHWSim` 为 `PositionJointInterface` 的每个关节
  持有一个位置指令，并且每个周期都把它写进 Gazebo——不管有没有控制器在运行（本模型没有
  配 `/gazebo_ros_control/pid_gains`，所以这个写入是运动学式的）。那个指令里存着什么，
  什么就压过任何传送。

  要摆位，只能用控制器把它开过去：给 `arm_joint_controller` 发一条轨迹，之后再切
  （见 docs/WORKFLOW.md §5）。

  `init_pose_start_controllers.launch` 同时会把臂从 MoveIt 手里拿走——这正是它的用途，
  从 `arm_joint_controller` 切到笛卡尔速度控制器——但在你切回去之前
  （`controller_manager start arm_joint_controller`），MoveIt 的执行功能是不可用的。

* **笛卡尔 IK 现在带阻尼了；以前不带，而且机械臂启动时仍然在奇异点上。**
  `cartesian_velocity_controller` 现在用 `KDL::ChainIkSolverVel_wdls`（阻尼最小二乘）。
  它以前用的是 `KDL::ChainIkSolverVel_pinv_givens`，一个不带阻尼的纯伪逆；配上上面那条
  说的启动位姿，后果是破坏性的：从完全伸直的位形给 x 方向 0.02 m/s，解出的关节速度范数
  是 1.84 rad/s——任务空间到关节空间放大了 92 倍每秒——足以把没有控制器的龙门甩出去
  （`y_joint` 1.82 m、`r_joint` 15.6 rad，`ee_state` 报到 105 m/s），同时把末端朝反方向
  拖了 0.76 m。

  加上阻尼之后，同一个位形、同一条指令，x 方向跟踪率正好 0.0200 m/s，龙门各关节保持在
  1 mm 和 0.001 rad 以内；一直到机械臂够不着了，解也是平滑地退化而不是发散。在条件良好的
  位姿上（SRDF 的 `init`），带阻尼和不带阻尼测不出区别：两者跟踪率都是 100.0%，dy 和 dz
  都是 0.0000 m。

  有三个参数控制这件事，在 `sim_vel_controller.yaml` 和 `controllers.yaml` 里。
  `ik_sigma_min`（0.05）是阻尼开始介入的奇异值；KDL 自己的默认值是 1e-5，远低于这条链
  实际会达到的任何值，照搬的话阻尼永远不会启动，所以这个值必须按机器人来选。
  `ik_lambda`（0.2）是阻尼系数。`max_joint_velocity`（1.0 rad/s）给解的范数封顶，是一道
  无条件的兜底——有前两个参数在，正常情况下够不到它。控制器现在还会在靠近奇异点求解时
  发出（限频的）警告，这件事以前是悄无声息的。这些默认值背后实测的奇异值，记在
  `cartesian_velocity_controller.cpp` 中 `init()` 的注释里。

* **Gazebo 报出来的关节速度不是关节的速度。** `sim_cartesian_velocity_control` 里的控制器
  已经不再用它了——改成对测到的位置做差分——但 `/joint_states` 上仍然是原始值，所以别的
  地方只要读这个话题，读到的还是它。

  `gazebo_ros_control` 的速度来自 Gazebo 的 `GetVelocity()`。本模型的关节走的是
  `PositionJointInterface`，又没有配 `/gazebo_ros_control/pid_gains`，于是 `SetPosition()`
  是运动学式地搬关节，从不设置速度；`GetVelocity()` 返回的，是"如果位置没有被不断重新按住、
  这些连杆在重力下会获得的速度"。同一次运行里实测：`arm_joint_controller` 把 `elbow_joint`
  在 11 秒里扫过 1.565 rad——真实速度 0.1424 rad/s——而报出来的平均速度是
  **0.00022 rad/s**，差了三个数量级。静止时也不是零，而且和位姿有关：`shoulder_lift_joint`
  在完全伸直时报 0.0416 rad/s，在 SRDF 的 `init` 位姿上报 0.0114 rad/s，两种情况下它的位置
  到小数点后五位都纹丝不动。所以它不是"带偏置的速度"，它和速度无关。

  `ee_state` 的 twist 以前就是拿这个算的。给 x 方向 0.02 m/s 时它报
  (+0.0045, 0, -0.0075) m/s——只有真实速度的 23%，而且在 z 完全没动的情况下报出了 z 分量。
  现在它报 (+0.0199, +0.0003, -0.0000)，真值是 (+0.0200, 0, 0)；静止时是 0.0000。

  这件事并非无害。`sim_pid_control` 把 `ee_state` 的 twist 喂给微分项，于是 `K_d` 一直在乘
  一个恒定的假信号。同一套测试台上保持设定点实测：**修之前稳态偏差 0.090 mm，修之后
  0.01–0.04 mm**，而且之前是一个固定偏移，之后是噪声。0.090 mm 也正是这一项算出来的值——
  `K_d` 0.1 乘 -0.0075 m/s 的假速度，再除以 `K_p` 7.7，等于 0.098 mm。

  替代方案有一点要讲清楚。位置接口是精确跟踪设定点的，所以在仿真里"指令→位置"这一段接近
  一个纯积分器，对位置做差分得到的速度也就接近当初发出去的指令。它是**被仿真的机器人的
  速度**，正是控制器需要的东西，但它不能作为"机器人确实照做了"的独立证据。
  `velocity_filter_cutoff`（30 Hz）是它上面的低通截止频率；在这个设置下，20 mm/s 的指令上
  残余抖动是 0.6 mm/s。

* **并行编译以前会失败**，报
  `mission_mode/mode_switch.h: No such file or directory`。现在
  `sim_general_planning` 里那四个可执行目标都声明了
  `add_dependencies(... ${catkin_EXPORTED_TARGETS})`，消息生成会先跑。以后你新增的
  节点如果 include 了生成的头文件，照着加一句。

* **`warehouse_ros_mongo` 被停用了。** `sim_moveit_config` 里的 warehouse 相关 launch
  仍然引用它，但 `package.xml` 里的依赖是注释掉的，因为它开箱编不过。

---

## 7. 排错

| 现象 | 原因 / 处理 |
|---|---|
| `ERROR: expected Ubuntu 20.04 (focal)` | ROS Noetic 没有你这个发行版的包，请用 20.04。 |
| 每条 apt 命令都要卡好几分钟 | IPv6 镜像解析问题。第一阶段已经帮你写了 IPv4 钉定。 |
| source 时报 `ROS_DISTRO: unbound variable` | `set -u` 和 ROS 自己的 profile 脚本冲突。第二阶段在 `source` 前后临时关掉了 `nounset`。 |
| `catkin_make` 建软链失败 | 你在 VMware/VirtualBox 共享文件夹里编译。让第二阶段把代码复制到 `$HOME` 下再编。 |
| 夹爪手指不跟着主动关节动 | mimic 插件的软链没建上。重跑第二阶段，看输出里有没有 `mimic plugin alias:`。 |
| `Could not find the planner configuration 'pilz...'` | 没装 `ros-noetic-pilz-industrial-motion-planner`（第一阶段第 4 步）。 |
| MoveIt 规划成功但机器人不动 | 关节被别的控制器占着，或者 `arm_joint_controller` 被停掉了。用 `rosrun controller_manager controller_manager list` 看一下。 |

---

## 8. 仓库约定

* **全仓库统一 LF 换行。** 这份代码经常是 Windows 和 Linux 同时在用（共享文件夹、
  WSL、网络挂载），所以 `.gitattributes` 不管 `core.autocrlf` 怎么设都强制
  `eol=lf`。`*.stl`、`*.png`、`*.jpg` 按二进制处理；`*.dae` 仍按文本处理。
* **在原始 checkout 里提交**，不要在 `$WS/src/ur5e_simulation` 里提交——那只是一份
  用来编译的快照，没有 `.git`。

---

## 9. 致谢

上游项目作者 Archer Wang（archer7wang@outlook.com）——
<https://github.com/wangarcher/ur5e_simulation>。本 fork 增加了 Ubuntu 20.04 /
ROS Noetic 的脚本化安装、Python 3 与并行编译相关修复，以及这份文档。
