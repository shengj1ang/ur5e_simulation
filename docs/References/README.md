# UR5e 末端抗扰参考文献

最值得先读的是 Keemink 等的导纳控制论文、From 等的移动基座动力学论文，以及 Dirkx 等的多变量频响辨识论文。三者分别说明现有力矩到运动映射为什么受内环影响、如何把平移模型推广到旋转基座，以及怎样判断某个姿态和频带是否值得补偿。

[仿真改进待办](TODO.md) · [现有惯性前馈报告](../inertial_feedforward_report.html)

PDF 放在本目录供本地阅读。索引保留了期刊页、机构库和作者全文入口，便于在其他工作区查阅。

## 导纳控制和移动基座

| 文献 | 对 UR5e 的用途 | 全文与出处 |
| --- | --- | --- |
| Keemink、van der Kooij、Stienen，2018。*Admittance control for physical human–robot interaction* | 第 5 节讨论前馈、滤波、相位和内环带宽如何影响导纳控制。用于解释为什么现有位置补偿的评估要同时看末端误差、关节补偿峰值和频响。论文所分析的接触被动性有其系统假设。 | [本地 PDF](Keemink-2018-Admittance-Control.pdf) · [期刊](https://doi.org/10.1177/0278364918768950) · [大学全文](https://ris.utwente.nl/ws/files/29087621/admittance.pdf) |
| From、Gravdahl、Lillehagen、Abbeel，2011。*Motion planning and control of robotic manipulators on seaborne platforms* | 第 2 节把基座位姿、速度和加速度对关节动力学的耦合写成显式项。用来推导龙门偏航和连杆惯性补偿，并检查零角速度时能否退回现有平移模型。 | [本地 PDF](From-2011-Seaborne-Manipulators.pdf) · [期刊](https://doi.org/10.1016/j.conengprac.2011.04.007) · [作者全文](https://people.eecs.berkeley.edu/~pabbeel/papers/FromGravdahlLillehagenAbbeel_CEP2011.pdf) |
| Woolfrey，2020。*Control of Manipulators on Moving Platforms Under Disturbance* | 博士论文第 4 章给出在移动基座上保持世界系末端位姿的预测控制，包括关节约束。适合对照可预测、重复扰动下的预见补偿。 | [本地 PDF](Woolfrey-2020-Moving-Platform-Manipulators-Thesis.pdf) · [UTS 论文库](https://opus.lib.uts.edu.au/handle/10453/140570) |
| Meng 等，2026。*Impedance Control of Ship-Borne Manipulators via Optimization-based Task-Space Inverse Dynamics* | 第 II 节在扭矩层补偿基座速度和加速度，第 III 节估计基座状态，第 IV 节分别检验约束、负载变化及跟踪。适合作为扭矩版对照和仿真试验结构的参考。该版本是 arXiv 预印本。 | [本地 PDF](Meng-2026-Ship-Borne-Manipulator-TSID-Preprint.pdf) · [在线全文](https://arxiv.org/html/2607.22030v1) |
| Khatib，1987。*A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation* | 任务空间动力学和任务与关节力矩之间的映射。用来设计直接以末端平移与姿态误差为目标的扭矩对照。 | [本地 PDF](Khatib-1987-Operational-Space-Formulation.pdf) · [斯坦福全文](https://khatib.stanford.edu/publications/pdfs/Khatib_1987_RA.pdf) |

## 频响辨识和多轴补偿

| 文献 | 对 UR5e 的用途 | 全文与出处 |
| --- | --- | --- |
| Pintelon、Schoukens、Lataire，2020。*Best Linear Approximation of Nonlinear Continuous-Time Systems Subject to Process Noise and Operating in Feedback* | 用随机相位多正弦估计闭环频响，并区分局部线性近似、噪声及非线性失真。用于把当前的多正弦开关对照扩展成频响测量。 | [本地 PDF](Pintelon-2020-Best-Linear-Approximation-in-Feedback.pdf) · [公开版本](https://arxiv.org/abs/2004.02579) |
| Dirkx、van de Wijdeven、Oomen，2020。*Frequency Response Function identification for multivariable motion control: Optimal experiment design with element-wise constraints* | 多输入、多输出频响的激励设计。用于辨识各基座轴和各补偿输入对末端平移、转角的交叉影响。 | [本地 PDF](Dirkx-2020-Multivariable-FRF-Identification.pdf) · [期刊](https://doi.org/10.1016/j.mechatronics.2020.102440) · [作者全文](https://gitlab.tue.nl/oomenpublic/pdfs/-/raw/main/pdf/DirkxWijOom2020d.pdf) |
| Mae 等，2025。*Fixed-structure sampled-data feedforward control for multivariable motion systems* | 用有物理解释的固定结构处理多轴交互和采样保持。可借鉴其参数化方法，针对龙门扰动改用因果、低阶补偿，并用独立扰动波形检验。 | [本地 PDF](Mae-2025-Sampled-Data-MIMO-Feedforward.pdf) · [期刊](https://doi.org/10.1016/j.mechatronics.2024.103288) · [作者全文](https://masahiromae.github.io/pdf/journal/202504_Mechatronics_MaeHarenClassensOhnishiOomenFujimoto.pdf) |

## 传感器与外力估计

| 文献 | 对 UR5e 的用途 | 全文与出处 |
| --- | --- | --- |
| Solà，2017。*Quaternion kinematics for the error-state Kalman filter* | IMU 比力、姿态坐标变换与误差状态估计。先用于检查重力符号和轴方向，扩到姿态估计时再参考完整滤波器。 | [本地 PDF](Sola-2017-Error-State-Kalman-Filter.pdf) · [公开版本](https://arxiv.org/abs/1711.02508) |
| Chen、Ballance、Gawthrop、O’Reilly，2000。*A nonlinear disturbance observer for robotic manipulators* | 用关节模型估计扰动。论文的实验是两连杆摩擦补偿。本项目可沿此思路单独验证未知工具外力的恢复效果。 | [本地 PDF](Chen-2000-Robotic-Manipulator-Disturbance-Observer.pdf) · [格拉斯哥大学论文库](https://eprints.gla.ac.uk/320/) |

Gazebo Classic 支持 IMU 噪声和偏置，Noetic 支持给连杆施加独立力和力矩。Gazebo 关节位置 PID 与直接力矩接口的实现可用于设计公平的控制器对照。 [IMU 噪声教程](https://classic.gazebosim.org/tutorials?cat=sensors&tut=sensor_noise) · [ApplyBodyWrench 定义](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_msgs/srv/ApplyBodyWrench.srv) · [Noetic 关节接口源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp)

## 补充期刊全文

- **Woolfrey、Lu、Liu，2021。** *Predictive End-Effector Control of Manipulators on Moving Platforms Under Disturbance*。论文把世界系目标变换到预测的移动基座坐标系，再用二次规划跟踪并约束实际执行的第一步。适合用作可预测扰动下的运动学对照。 [本地期刊 PDF](Predictive_End-Effector_Control_of_Manipulators_on_Moving_Platforms_Under_Disturbance.pdf) · [IEEE DOI](https://doi.org/10.1109/TRO.2021.3072544)
- **Liu、Yu、Liu、Wang，2026。** *A feedforward–feedback control framework for disturbance rejection in multi-loop PI controlled servo actuators under varying disturbance sensing conditions*。第 3 节用闭环误差模型设计前馈和反馈，分别处理扰动位移可测及只有加速度可测两类输入。第 4 节还对比了扫频、非周期扰动和不同负载。本项目已有基座姿态和速度，适合先将速度求导与 IMU 直接测加速度分开比较。 [出版社全文](https://www.sciencedirect.com/science/article/pii/S0019057826001539) · [期刊 DOI](https://doi.org/10.1016/j.isatra.2026.03.031)
