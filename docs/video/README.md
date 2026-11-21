# 对比视频：四年前的代码 vs 现在，以及动力学前馈关/开

`clips/comparison.mp4`（16 秒，1920×724）分两段：

| | 第一段 | 第二段 |
|---|---|---|
| 比的是 | 四年前的代码 vs 现在的代码 | 都是现在的代码，只切换动力学前馈 |
| 扰动 | 基座 Y 向 20 mm、1 Hz | 基座 Z 向 30 mm、3 Hz |
| 姿态 | SRDF `init` | SRDF `1st_reco_start` |
| 机位 | 广角，0.78 mm/像素 | 特写，0.195 mm/像素 |
| 实测末端 RMS | 10.54 → 0.30 mm | 5.83 → 3.51 mm |

## 先说清楚这段片子不是什么

**第一段的巨大差距，主要不是动力学前馈带来的。** 它来自世界系调节（原来末端误差是在 `base_link` 里算的，基座一动末端就跟着走）、PID 重新标定、基座状态 50→200 Hz 这三项。动力学前馈是在这个已经好了几十倍的基础上，再降一截 —— 那是第二段的内容，也是 [惯性前馈报告](../inertial_feedforward_report.html) 的内容。

**第一段比较的是两套系统，不只是两个控制器。** 四年前那版模型没有配 `gazebo_joint_pid_gains.yaml`，`gazebo_ros_control` 是运动学地满足关节位置指令，等于无限力矩的完美伺服；现在的关节是有力矩上限的真实伺服。被控对象本身变了。

不过老基线并没有跑坏：实测基座峰峰 40 mm、末端 Y 向峰峰 30.3 mm，传递率 75.6%，与 [`pid_params.yaml`](../../sim_control/sim_pid_control/config/pid_params.yaml) 中记录的 `base_link` 调节在 1 Hz 的 71.6% 一致。

**第二段用的是单频 3 Hz，不是报告里的 0.5–8 Hz 多正弦。** 惯性量随 ω² 增长，单频 3 Hz 比多正弦更能凸显前馈的作用，所以画面上的绝对 RMS 比报告里的大。

**第二段是一对试次，不是统计结果。** 报告里该工况做了五对配对试验并给出 95% 置信区间；这里录了三对，改善分别为 22.8% / 39.9% / 46.5%，片中用的是居中的一对（`seg2_*_a`）。散布这么大是因为本流程没有做扰动起始相位配对 —— 报告专门用它把配对差绝对值的中位数从 0.98 压到 0.17 个百分点。三对的原始轨迹都在 `trials/` 下。

## 老代码是哪一版

基线取 **`3888830`（2026-11-18，"Add a workflow guide"）** —— 环境已经移植到 Ubuntu 20.04 / ROS Noetic / Python 3、能跑起来，但控制工作的第一个提交 `0eb0d70` 还没进去。没有回到 2022 年的首次提交：那一版是 Melodic / Python 2，在现在的机器上起不来。

基线的文件内容 git 历史里已经有了，所以这里不再提交一份副本（那是 36 MB 的重复内容，且进了历史就删不掉）。仓库里存的是**重建配方**：下面这段加上 `baseline.patch`（73 行），经逐文件比对，能一字不差地重建出实际用于录制的那棵树。

```bash
mkdir -p /tmp/baseline
git archive 3888830 | tar -x -C /tmp/baseline --anchored --exclude=CMakeLists.txt
for f in sim/sim_bringup/config/gantry_velocity_controller.yaml \
         sim/sim_bringup/scripts/gantry_disturbance.py \
         sim/sim_bringup/launch/gantry_disturbance.launch; do
    git show "HEAD:$f" > "/tmp/baseline/$f"
done
cd /tmp/baseline && patch -p1 < <仓库>/docs/video/baseline.patch
```

### 基线里动过的三处，以及为什么

都是为了"能施加同一个扰动、能无头录制"，没有一处碰控制：

1. **移植了试验台**：`gantry_velocity_controller.yaml`、`gantry_disturbance.py`、`gantry_disturbance.launch`（上面 `git show HEAD` 取的三个文件）。基线里**没有任何东西能驱动龙门**，不补就没法摇基座。
2. **龙门 effort 上限对齐到现在的值**（x/y/z 30→2000、r 30→500；z 原为 3e7）。基线的 30 N 连手臂自重 174 N 都扛不住，摇不动。
3. **`with_moveit_new.launch` 加了 `use_rviz` 开关**，和现在的 bringup 一样，这样能 headless 录制。

第 2、3 项就是 `baseline.patch` 的全部内容。

保持原样的：手臂控制代码、PID 增益（X 17.7/0.1、Y 7.7/0.1）、`base_pose_update_rate` 50 Hz（基线的 xacro 里写死）、关节限位 ±2π、无关节 PID 增益。

## 录制是怎么做的

用 **Gazebo 相机传感器把帧直接存盘**，不是抓屏：

- 机位是数字给定的，两侧完全一致，手摆的 GUI 视角做不到；
- Gazebo 按**仿真时间**给帧编号，所以实时率掉到多少都不影响成片速度 —— 抓屏则必须事后重新计时；
- 画面里没有桌面、没有窗口边框、没有鼠标。

相机在 `:0` 上渲染（VMware SVGA3D 直通；Xvfb 只有 llvmpipe 软件渲染，14 秒仿真只渲得出 5 帧）。`gui:=false`，传感器离屏渲染，不会在屏幕上弹窗。物理按 `max_update_rate=250`（约 0.25 实时率）节流，让相机每一帧都渲得出来。

**红色细杆固定于世界系**，是画面里唯一不动的基准 —— 没有它，透视图里一条摆动的手臂是读不出来的。杆的方向与扰动方向垂直：Y 向扰动配竖杆，Z 向扰动配横杆。

两侧的对齐靠帧号：第 N 帧对应 `RIG_SPAWNED + N/30` 仿真秒，扰动起点从录下的基座轨迹里反读出来，不是假定的。字幕上的 RMS 由 `assemble.py` 从各自的轨迹现算，不是从报告抄的。

## 目录

| 路径 | 内容 |
|---|---|
| `clips/comparison.mp4` | 成片，两段连在一起 |
| `clips/part1_old_vs_new.mp4`、`clips/part2_ff_off_vs_on.mp4` | 两段分开的版本 |
| `stills/` | 两段各自摆动极值时刻的静帧 |
| `trials/<运行>/{ee,base}.csv` | 世界系末端与基座轨迹，只保留时间和位置三列 |
| `record.sh` | 起 bringup、摆位、生成相机、施加扰动、记录轨迹 |
| `stage.py` | 摆位 + 发布保持点 + 生成相机装置（取自 `../experiments/inertia_mapping_200hz/bench.py` 的准备部分） |
| `scene.py` | 相机与参照杆的 SDF，按中心/距离/方位角/仰角/视场计算位姿 |
| `assemble.py` | 对齐、裁剪、加字幕、左右拼接、合并两段 |
| `video_new.launch` / `video_old.launch` | 两侧各自的无头 bringup |
| `baseline.patch` | 重建老基线所需的两处改动 |

原始帧没有入库：10540 张 JPG、397 MB。要重新生成就按下面重跑。

## 用法

两个工作区：`~/ur5e_sim_ws`（现在的代码）和 `~/ur5e_old_ws`（按上面的配方重建的基线）。按本项目的惯例，在宿主机改、同步到虚拟机再跑。

```bash
tar --exclude=out -czf - . | ssh ur5e-vm "mkdir -p ~/ur5e_video && tar -xzf - -C ~/ur5e_video"
```

```bash
ssh ur5e-vm "cd ~/ur5e_video && bash record.sh <标签> <new|old> <轴> <幅值m> <频率Hz> <秒> [姿态] [前馈权重]"
```

```bash
ssh ur5e-vm "cd ~/ur5e_video && python3 assemble.py"
```

需要虚拟机上有 `ffmpeg`。
