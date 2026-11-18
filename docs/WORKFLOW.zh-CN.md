# 上手与改代码指南

[English](WORKFLOW.md) · **简体中文**

[README.zh-CN.md](../README.zh-CN.md) 讲的是怎么装、怎么启动。这份文档讲另外一半：
怎么从一台空白的 Ubuntu 虚拟机一步步走到机械臂真的动起来、改了东西之后要做什么才
能看到效果、以及跑不起来的时候怎么自己查。下面每条命令都在 VMware 虚拟机里的
Ubuntu 20.04 / ROS Noetic / Gazebo 11 上实际跑过。

---

## 1. 同一份代码存在三个地方

在 Windows 上写代码、在虚拟机里跑仿真，意味着同一份代码同时存在三处。搞混这三处是
最容易白费一下午的事。

| 位置 | 它是什么 | 你在这里做什么 |
|---|---|---|
| `C:\...\ur5e_simulation`（Windows） | git 仓库本体 | 改代码、`git commit` |
| `/mnt/hgfs/ur5e_simulation`（虚拟机） | 同一批文件，通过 VMware 共享文件夹看到 | 什么都不做，只是能读到 |
| `~/ur5e_sim_ws/src/ur5e_simulation`（虚拟机） | 第二阶段脚本复制出来的**副本** | 真正参与编译和运行的就是它 |

```
Windows 仓库  ──共享文件夹──>  /mnt/hgfs/...  ──rsync（第二阶段）──>  ~/ur5e_sim_ws/src/...
     ^                                                                        |
     └──────────────── git commit 只在这里做，不要在副本里做 ──────┘     catkin_make
```

由此有两个后果，这也是本节存在的全部理由：

* **只改 Windows 里的文件，跑起来的东西不会变。** 编译工作区是一份副本，你不重跑第
  二阶段去同步，它就还是旧的。
* **直接改副本是临时的。** 第二阶段用的是 `rsync --delete`，下次同步会把你在副本里
  的修改覆盖掉。临时试一下没问题，但要知道它会消失。

为什么非要复制：`catkin_make` 必须在 `src/` 里建软链，而 VMware / VirtualBox 的共享
文件夹（`vmhgfs`、`vboxsf`）根本不支持软链，所以直接在共享目录里编译是不可能成功的。

如果 `/mnt/hgfs` 是空的，说明共享文件夹没挂上：在虚拟机设置里把共享打开，并确认装了
`open-vm-tools-desktop`。挂上了的话，`mount | grep hgfs` 会看到
`vmhgfs-fuse on /mnt/hgfs`。

---

## 2. 从空白虚拟机到机械臂动起来

手工流程，每一步都给出"成功了长什么样"。

**第 1 步 —— 虚拟机里装 Ubuntu 20.04。** 必须是 20.04（focal）。ROS Noetic 只为
focal 提供二进制包，第一阶段脚本在别的版本上会直接拒绝执行。虚拟机建议 4 核、8 GB
内存、40 GB 磁盘——Gazebo 本身挺重，光 `ros-noetic-desktop-full` 就要下 2.5 GB 左右。

**第 2 步 —— 把代码弄进虚拟机。** 要么用共享文件夹指向你 Windows 上的仓库，要么直接
在虚拟机里 `git clone`。用共享文件夹的话先确认能看到：

```bash
ls /mnt/hgfs/ur5e_simulation
```

**第 3 步 —— 装 ROS 和依赖。** 二十到四十分钟，主要是下载时间：

```bash
sudo bash /mnt/hgfs/ur5e_simulation/setup/01_install_ros_and_deps.sh
```

成功的标志是 `=== phase 1 finished OK ===`。完整日志在
`/var/log/ur5e_apt_install.log`。重复跑是安全的。

**第 4 步 —— 编译。** 不要加 sudo：

```bash
bash /mnt/hgfs/ur5e_simulation/setup/02_build_workspace.sh
```

成功的标志是每个节点都 `[100%] Built target ...`、接近结尾有
`mimic plugin alias: ...`、最后 `=== phase 2 finished OK ===`。想把工作区放到别处：
`WS=~/other_ws bash .../02_build_workspace.sh`。

**第 5 步 —— 新开一个终端。** 第二阶段往 `~/.bashrc` 里追加了环境（ROS、工作区、以及
`SVGA_VGPU10=0`——VMware 的 GL 驱动缺了这句会让 Gazebo 11 一启动就崩）。在这一步之前
就打开的终端对这些一无所知，手动 `source ~/ur5e_sim_ws/devel/setup.bash` 也行。

**第 6 步 —— 启动仿真。**

```bash
roslaunch sim_bringup with_moveit_new.launch
```

Gazebo 和 RViz 会打开，机器人出现在一块水泥砖旁边。等几秒后：

```bash
rosrun controller_manager controller_manager list
```

应该能看到 `arm_joint_controller`、`gripper_controller`、`joint_state_controller`
都是 `running`。满屏的 `No p gain specified for pid` 是正常的、无害的，见
[README §6](../README.zh-CN.md#6-已知问题)。

**第 7 步 —— 让它动。** 在 RViz 的 MotionPlanning 面板里，把末端那个球拖到一个能到
的位置，点 **Plan & Execute**。Gazebo 里的机械臂会同步动起来。这一下就说明整条链路是
通的：MoveIt 规划、`arm_joint_controller` 执行、Gazebo 仿真。

---

## 3. 改完之后要做什么（编译 / 不编译）

改完要做什么，只取决于你改的是哪类文件。

| 你改了 | 要编译吗 | 要做什么 |
|---|---|---|
| `*.cpp`、`*.h` | **要** | 同步，然后 `catkin_make` |
| `*.srv`（`mission_mode`） | **要** | 同步，然后 `catkin_make` |
| `CMakeLists.txt`、`package.xml` | **要** | 同步，然后 `catkin_make` |
| `*.yaml`（增益、控制器、限位） | 不要 | 同步，重启对应的 launch |
| `*.launch` | 不要 | 同步，重启对应的 launch |
| `*.xacro`、`*.urdf`、`*.world` | 不要 | 同步，重启对应的 launch |
| `*.srdf`（预设姿态、规划组） | 不要 | 同步，重启对应的 launch |
| `scripts/` 下的 `*.py` | 不要 | 同步，重新跑脚本 |
| `*.rviz` | 不要 | 同步，重启 RViz |

"同步"就是把文件从 Windows 仓库弄到编译工作区里。最省脑子的办法（顺带也编译了）：

```bash
bash /mnt/hgfs/ur5e_simulation/setup/02_build_workspace.sh
```

只动了一个包时更快的办法：

```bash
cd ~/ur5e_sim_ws
catkin_make --pkg sim_general_planning
```

（即使什么都没改，完整的 `catkin_make` 也要走一遍所有包，有点慢；`--pkg` 只编那一个。）

另外两点：编译之前就打开的终端，不重新 `source ~/ur5e_sim_ws/devel/setup.bash` 是不
认新东西的；而**已经在运行的节点，无论如何都不会认**——必须把 launch 停掉重启。

---

## 4. 常见改动怎么做

### 改控制增益

PID 改 `sim_control/sim_pid_control/config/pid_params.yaml`，导纳改
`sim_control/sim_admittance_control/config/admittance_params.yaml`。那些 6×6 矩阵是
按行优先展平成一维列表写的，所以对角线上是第 0、7、14、21、28、35 个元素，依次对应
x、y、z 和三个姿态方向。不用编译；同步之后重启控制器的 launch。确认参数真的进去了：

```bash
rosparam get /K_p
rosparam get /mass_arm
```

### 换场景

世界文件是在 bringup 的 launch 里选的。`with_moveit_new.launch` 加载
`sim_gazebo/world/one_brick.world`；`gazebo_test.launch` 做成了参数：

```bash
roslaunch sim_bringup gazebo_test.launch world:=$(rospack find sim_gazebo)/world/example.world
```

想加自己的场景，把 `.world` 放到同一个目录，再把 `world_name` 指过去。注意：世界文件
里如果引用了 `model://什么东西`，那个模型是要从 Gazebo 在线模型库下载的——
`puzzle.world` 就是这么失败的。

### 改机器人生成的位置或初始姿态

在 `with_moveit_new.launch` 里 `urdf_spawner` 节点的 `args`：`-x -y -z` 决定模型放在
哪，`-J <关节> <值>` 设定初始关节角。`-model sim` 是 Gazebo 认的模型名——你要是改了它，
记得把 `sim_bringup/scripts/init_pose_start_controllers.py` 一起改（或者启动时传
`_model_name:=...`），因为那个脚本是按名字找模型的。

### 加一个预设姿态

在 `sim_planning/sim_moveit_config/config/sim.srdf` 里，跟现有的 `<group_state>`
（`init`、`home1`、`back`、`finale`、`1st_reco_start`、`2nd_reco_start`、`open`、
`close`）放一起：

```xml
<group_state name="my_pose" group="manipulator">
    <joint name="shoulder_pan_joint" value="0.5" />
    <!-- ……六个关节都要写 …… -->
</group_state>
```

不用编译。重启 launch 之后，C++ 里用 `arm.setNamedTarget("my_pose")`、Python 里用
`arm.set_named_target("my_pose")`，或者直接在 RViz 的下拉框里选。手写 SRDF 完全可以；
想用图形界面就 `roslaunch sim_moveit_config setup_assistant.launch`。

### 改机器人模型

顶层是 `sim/sim_description/urdf/xacro/main.xacro`，它再引入 `ur5e.urdf.xacro`、
夹爪、传动以及 `common.gazebo.xacro`。改完先确认能解析，别直接去启动：

```bash
xacro ~/ur5e_sim_ws/src/ur5e_simulation/sim/sim_description/urdf/xacro/main.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

应该看到 `robot name is: sim` 和 `Successfully Parsed XML`。那条
`material 'flat_black' undefined` 警告是本来就有的，无害。

### 新增一个规划或控制节点

1. 在对应包里写 `src/my_node.cpp`。
2. 在这个包的 `CMakeLists.txt` 里加上：

   ```cmake
   add_executable(my_node src/my_node.cpp)
   target_link_libraries(my_node ${catkin_LIBRARIES})
   add_dependencies(my_node ${catkin_EXPORTED_TARGETS})
   ```

   如果你的节点 include 了生成的头文件（比如 `mission_mode/mode_switch.h`），
   `add_dependencies` 那行**不是可选的**。少了它，`make -j4` 会和消息生成抢跑，报
   `mission_mode/mode_switch.h: No such file or directory`。`sim_general_planning`
   里那四个节点以前就栽在这上面。
3. 在 `launch/` 下给它写一个 launch 文件。
4. 同步，然后 `catkin_make --pkg <这个包>`。
5. 确认二进制出来了：`ls ~/ur5e_sim_ws/devel/lib/<包名>/my_node`。

### 把硬编码改成可配置参数

`general_arm_planning.cpp` 里用的就是这个套路：

```cpp
std::string arm_state_topic;
n_.param<std::string>("arm_state_topic", arm_state_topic,
                      "/sim_cartesian_velocity_controller_sim/ee_state");
```

之后在 launch 里用 `<param name="arm_state_topic" value="..."/>` 覆盖，改值就不用再
重新编译了。

---

## 5. 怎么手动把机械臂指挥起来

### 用 MoveIt（在 RViz 里）

MotionPlanning 面板 → 拖那个交互标记 → **Plan & Execute**。*Planning* 标签页里的
*Goal State* 下拉框也能直接选 SRDF 里的预设姿态。

### 一个关节一个关节地动

```bash
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

选 `arm_joint_controller`，点那个电源按钮，然后拖滑块。

### 让它在预设姿态之间循环（看效果用）

想让机械臂一直动着给你看，把下面存成 `/tmp/cycle.py`，在仿真已经启动的前提下
`python3 /tmp/cycle.py`：

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

大概 20 秒换一个姿态，时间主要花在规划上。另外注意：`rospy.loginfo` 是往 stdout 打的，
而 Python 在输出被重定向到文件时会按块缓冲——所以日志文件看起来是空的，通常不代表节点
出了问题，只是输出卡在缓冲区里。`~/.ros/log/latest/rosout.log` 里是立刻能看到的。

### 笛卡尔控制器

那几个笛卡尔的 launch 只**加载**控制器、并不启动它，而且 `arm_joint_controller` 占着
同样那六个关节：

```bash
roslaunch sim_pid_control pid_controller.launch          # 只是加载
rosrun controller_manager controller_manager list        # 状态是 initialized，不是 running
rosrun controller_manager controller_manager stop arm_joint_controller
rosrun controller_manager controller_manager start sim_cartesian_velocity_controller_sim
```

`roslaunch sim_bringup init_pose_start_controllers.launch` 会把"摆初始姿态"和"切控制器"
一起做掉。切过去之后 MoveIt 就执行不了了，要用回 MoveIt 得切回来：

```bash
rosrun controller_manager controller_manager stop sim_cartesian_velocity_controller_sim
rosrun controller_manager controller_manager start arm_joint_controller
```

笛卡尔控制器在跑的时候，可以直接给末端发速度指令：

```bash
rostopic pub -r 10 /sim_cartesian_velocity_controller_sim/command_cart_vel \
  geometry_msgs/Twist '{linear: {x: 0.02, y: 0.0, z: 0.0}}'
```

### 给抓取演示喂数据

`x_arm_planning` 在等一个物体位姿，而本仓库里没有任何节点发布它。你可以自己充当那个
缺失的视觉节点——消息类型是相机坐标系下的 `PoseStamped`，而 `frame_id` 是触发开关
（`1st` 触发抓取，`2nd` 触发放置）：

```bash
rostopic pub -1 /object_visual_odometry geometry_msgs/PoseStamped \
  '{header: {frame_id: "1st"}, pose: {position: {x: 0.0, y: 0.0, z: 0.30}, orientation: {w: 1.0}}}'
```

节点随后会打印 `Caution: object pose received, now picking!` 并尝试规划一条到那里的
笛卡尔路径。随便编的位姿够不到时会报
`Path planning failed with only 0.0625 success after 100 attempts`，然后状态机继续往
下走——这恰好是一个没有相机也能把整个流程看一遍的办法。

---

## 6. 怎么确认自己改的生效了

```bash
rostopic list | grep ee_state            # 我以为存在的话题到底存不存在
rostopic echo -n1 /joint_states          # 所有关节取一帧
rostopic hz /sim_cartesian_velocity_controller_sim/ee_state
rosnode info /x_arm_planning             # 它实际订阅的是什么
rosparam get /K_p                        # 我的 yaml 有没有进参数服务器
rosrun controller_manager controller_manager list
```

直接去问 Gazebo——这是区分"名字写错了"和"值不对"的办法：

```bash
rosservice call /gazebo/get_world_properties                    # 世界里加载了哪些模型
rosservice call /gazebo/get_joint_properties "joint_name: 'shoulder_pan_joint'"
rosservice call /gazebo/set_model_configuration \
  "{model_name: sim, urdf_param_name: robot_description,
    joint_names: [shoulder_pan_joint], joint_positions: [1.0]}"
```

模型名写错时它会回 `success: False` 加一句
`SetModelConfiguration: model does not exist`——初始位姿脚本里那个过期的 `raigor`
就是这么查出来的。

不启动也能检查 launch 文件：

```bash
roslaunch --files sim_bringup gazebo_test.launch     # 它会引入哪些文件
roslaunch --nodes sim_bringup with_moveit_new.launch # 它会起哪些节点
```

日志在 `~/.ros/log/latest/`，其中 `rosout.log` 是所有节点的输出汇总——节点还没等你看清
终端就挂了的时候，就去翻它。想在 `pid_gains` 的噪音里找真正的报错：

```bash
grep -iE "error|fail|abort" ~/.ros/log/latest/rosout.log | grep -v "p gain"
```

---

## 7. 怎么干净地关掉和重启

正常情况就在跑 `roslaunch` 的那个终端里按 Ctrl-C。通过 SSH 操作时，把中断信号发给
`roslaunch` 进程：

```bash
pgrep -f "bin/roslaunch sim_bringup"      # 拿到 pid
kill -INT <pid>
```

然后**一定要等它们真的全退了**再重新启动：

```bash
pgrep -a -f "gzserve[r]|gzclien[t]|rosmaste[r]"
```

上一个 `rosmaster` 还在退出的过程中就起新的 `roslaunch`，结果是新的 launch 根本没有
master，然后无限刷
`Error in XmlRpcClient::writeRequest: write error (Connection refused)`。完整关闭要
十几到二十秒。

两个值得知道的坑：

* 在一个命令行里本身就包含 `gzserver` 这个词的 shell 里执行 `pkill -f gzserver`，会把
  这个 shell 自己也杀掉。所以上面写成 `gzserve[r]` 这种加方括号的形式。
* 残留的 `gzserver` 会占着 11345 端口，下一个 Gazebo 就起不来。按 pid `kill` 掉。

---

## 8. 症状 → 原因

| 症状 | 原因 |
|---|---|
| 改了代码没反应 | 只改了 Windows 里的文件没同步；或者节点还是旧进程在跑。 |
| 我的修改不见了 | 你改的是 `~/ur5e_sim_ws/src/ur5e_simulation`，第二阶段用 `rsync --delete` 覆盖掉了。 |
| `bad interpreter: ...^M` | CRLF 换行。`.gitattributes` 已强制 LF；老 checkout 用 `git rm --cached -r .` 再 `git reset --hard` 刷新。 |
| `catkin_make` 建软链失败 | 你在共享文件夹里编译。要编 `$HOME` 下的那份副本。 |
| `mission_mode/mode_switch.h: No such file` | 节点 include 了生成的头文件，却没写 `add_dependencies(... ${catkin_EXPORTED_TARGETS})`。 |
| Gazebo 一启动就挂 | 当前这个 shell 里缺 `SVGA_VGPU10=0`。 |
| `Unable to find uri[model://...]` | 世界文件要一个得从 Gazebo 在线模型库下载的模型。 |
| MoveIt 规划成功但机器人不动 | `arm_joint_controller` 被停了，或者关节被别的控制器占着。 |
| XmlRpc 一直刷 `Connection refused` | 上一个 `rosmaster` 还没退干净就重启了。 |
| `ROS_DISTRO: unbound variable` | 脚本里 `set -u` 和 ROS 的 setup 脚本冲突，在 `source` 前后临时关掉 `nounset`。 |
| 节点日志文件是空的，但它其实在跑 | Python 重定向时会缓冲 stdout。去看 `~/.ros/log/latest/rosout.log`。 |

---

## 9. 提交

在 Windows 那份 checkout 里提交，它才是 git 仓库。编译工作区里的副本是故意不带 `.git`
的（第二阶段把它排除了），所以那边没东西可提交，也不会出现"提交了一份同步到一半的快照"
这种事。
