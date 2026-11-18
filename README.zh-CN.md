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
  `/base_pose_ground_truth`，而这个模型里没有移动底盘，所以底盘位姿一直是零。
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
  先把臂摆到 `sim_bringup/config/initial_pose.yaml` 里的姿态，再从
  `arm_joint_controller` 切到笛卡尔控制器。

相关话题：

| 话题 | 方向 | 含义 |
|---|---|---|
| `/sim_cartesian_velocity_controller_sim/command_cart_vel` | 输入 | 末端速度指令 |
| `/sim_cartesian_velocity_controller_sim/ee_state` | 输出 | 末端当前状态 |
| `/ft_sensor/data` | 输入 | 导纳控制用的外部力/力矩 |

插件使用的运动链是 `base_link` → `ee_link`，频率 125 Hz（见各控制包里的
`sim_vel_controller.yaml`）。

### 4.4 参数在哪里调

| 文件 | 内容 |
|---|---|
| `sim_control/sim_pid_control/config/pid_params.yaml` | `K_p`/`K_i`/`K_d`、前馈 `N_p`/`N_d`、`workspace_limits`、`arm_max_vel`，1 kHz 控制周期 |
| `sim_control/sim_admittance_control/config/admittance_params.yaml` | `mass_arm`、`damping_arm`、`stiffness_arm`、`equilibrium_point_spring`、`workspace_limits` |
| `sim_planning/sim_moveit_config/config/sim_trajectory_control.yaml` | 轨迹控制器的容差和发布频率 |
| `sim/sim_bringup/config/initial_pose.yaml` | 初始位姿辅助脚本用的关节值 |

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
  （`x_joint`、`y_joint`、`z_joint`、`r_joint`）就只是没有控制器而已。

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

* **`/base_pose_ground_truth` 没有任何节点发布。** `general_arm_planning` 订阅它，
  是因为它原本写给一台带移动底盘的机器人。本模型是固定龙门，所以它用到的底盘位姿
  一直保持为零。

* **`init_pose_start_controllers.launch` 会把臂从 MoveIt 手里拿走。** 这正是它的用途
  ——从 `arm_joint_controller` 切到笛卡尔速度控制器——但在你切回去之前
  （`controller_manager start arm_joint_controller`），MoveIt 的执行功能是不可用的。

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
