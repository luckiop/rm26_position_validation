# RM27 重载机械臂演示

本目录导入自 `rm_tech_core_trajectory_solve` 主线提交 `9a08859c33508e890363e767eaa14e440b882d0c` 的 `src/demos/rm27_heavy_arm_demo/`。模型资源逐字节保留；`source_manifest.json` 记录全部上游文件的 SHA-256。为兼容当前验证工作区的旧版规划器，`make_plan_options()` 使用当前的 `PredictorCorrectorWithCollisionCache` 方法与 `TimingOptions::limits` 接口，沿用当前零端点速度三次时间参数化；另在 CMake 中安装本说明与来源清单。

在本工作区使用根目录 `./scripts/build.sh` 构建；批量验证选用 `--arm-variant src/rm26_level3_validation/config/arm_rm27.json`。该入口使用本包的关节配置、安装变换和起始姿态，但任务模式及预算由验证工具决定。以下上游规划记录用于说明模型来源，不能代替本工作区的验证结果。

本包构建机械臂配置静态库 `rm27_heavy_arm_demo` 和规划入口 `rm27_planner_demo`，
并安装 URDF、MJCF、网格和 RViz 显示配置。C++ 代码使用 C++20，依赖
`tech_core_planner` 和 `ament_index_cpp`；规划库会传递引入 Eigen 和 MuJoCo。
`display.launch.py` 仍使用 URDF 显示模型。

## 安装依赖、构建与启动

在仓库根目录执行（以 ROS 2 Jazzy 为例）：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/demos/rm27_heavy_arm_demo src/tech_core_planner thrid_party/mujoco --ignore-src -r -y
colcon build --symlink-install --packages-up-to rm27_heavy_arm_demo
source install/setup.bash
ros2 launch rm27_heavy_arm_demo display.launch.py
```

默认启动关节滑条窗口和 RViz，需要桌面环境。`gui:=false` 选择无窗口的
`joint_state_publisher`，`rviz:=false` 关闭 RViz。

构建成功后，可单独运行 C++ 规划演示；当前入口启用规划查看器：

```bash
ros2 run rm27_heavy_arm_demo rm27_planner_demo
```

静态库和公开头文件随包安装，下游 CMake 使用
`find_package(rm27_heavy_arm_demo REQUIRED)` 后，可链接
`rm27_heavy_arm_demo::rm27_heavy_arm_demo`。

## 复现一次完整规划

保留 `make_plan_request()` / `planning_home_joints()` 中的自定义场景；使用
`--reference-scene` 显式切换到已验证的参考核心位姿与准备姿态：

```bash
source install/setup.bash
ros2 run rm27_heavy_arm_demo rm27_planner_demo --headless --reference-scene
```

去掉 `--headless` 可查看同一场景，关闭查看器后程序退出，不再忙等。
规划成功返回 0，失败返回 1，参数错误返回 2。

- 核心系原点在世界系中的位置：`(0.45, 0, 0.4) m`，绕世界系 +Z 旋转 `π rad`。
- `joint_1` 至 `joint_7` 起点：`[0.7, -0.5, -0.4, 1.9, 1.9, 0.4, 0.9] rad`，不是 CAD 零位。
- 保持 `Full / MaximumRange`、请求扭转区间 `[-π/2, π/2] rad` 和原有碰撞、局部检查阈值。
- 参考场景用于确认插入、平移、上翻完成及实际 Q 覆盖。输出中的 `planning_ok=1`、
  三个完成标志与扭转区间用于确认实际结果；不能仅凭请求模式推定完整覆盖。
- 独立整轨验收已移除，不再输出 `validation_ok`、`rule_success`、整轨验收采样数或误差峰值。
  `fk_evaluations`、`collision_states`、`collision_calls` 和 `elapsed_s` 用于比较规划工作量。

参考场景成功不代表任意核心位姿都可达。例如原先 `(0.5, 0, 0.7) m` 的核心位置
会要求工具到达 `(0.561, 0, 0.977) m` 的上翻终点，超过当前机械臂的臂展。
此类失败不能通过忽略碰撞或仅增加求解时间修复。

## 腕部板件修整

`link_5` 与 `link_7` 相邻的板件已在 J6 轴附近修成圆弧，MJCF 与 URDF 同时使用
`model/meshes/visual/link_5_0_rounded.obj`，原始 `link_5_0.stl` 保留以便核对。
`model/meshes/collision/link_5_plate.obj` 同步重建，不只改变显示外观：

- 圆心在 `link_5` 局部系 XZ 平面的位置为 `(0.12124355653, -0.07) m`，半径 `0.018 m`。
- 圆弧用 49 个相切裁切平面近似；全部 9 个通孔保留，网格保持闭合。
- 碰撞网格是修整后可视板件的保守凸包，并外扩 `2e-6 m`；没有新增碰撞排除，
  普通碰撞间隙仍为 `0.001 m`。并非所有腕部转角都因此无碰撞。

本次改变的是规划模型，板件体积约减少 24.34%；原 CAD 惯量暂未重新计算。
这不是经过刚度、强度或动力学认证的实物改造方案。

## 第二连杆碰撞精细化

原 `link_2_c0` 把两根长杆与六个夹块合成一个长凸包，把夹块之间本应空出的空间
也当作实体。起点 `[0, 1.39, -1.3, 0, 0, 0.3, 0] rad` 因此会对 J2/J6 误报碰撞。

目前按原始可视 STL 的连通实体拆为 `link_2_rod_0/1.obj` 和
`link_2_clamp_0` 至 `link_2_clamp_5.obj` 共 8 个凸网格，MJCF 与 URDF 同步引用。
每个网格完整包围对应源实体并外扩 `2e-6 m`；没有修改可视模型、惯量或碰撞阈值，
也没有排除 J2/J6 的碰撞。不再使用的旧粗包络已移除，避免误加载。

该起点不再误报原来 `-0.00366318 m` 的 J2/J6 穿透，完整场景碰撞检查通过。
每个独立实体内部的孔洞仍采用保守凸包，并非全模型已经
变成精确三角网格碰撞；其他姿态仍需正常检查。

该模型修改时的历史规划记录：在核心位置 `(0.3, 0, 0.4) m`、绕世界系 +Z 旋转 `π rad` 时，
该起点的 `Full / MaximumRange` 规划成功，插入、平移、上翻完成，
扭转实际覆盖 `[-1.57079632679, 1.02974425868] rad`，并未宣称覆盖完整正向请求范围。
修改模型后需重启演示进程，使已有查看器和规划场景重新加载几何。

## 全臂精细化与简化

基座及 J1–J7 统一使用结构分组凸包：折弯支架分段、夹爪四片板分开、主要开口保留，
电机和贴近其外壳的紧固件合并。整臂共 48 个碰撞体、13878 个三角面，约为可视模型
面数的 6%，不为每个螺钉单独创建碰撞体。

网格完整包围对应来源实体，保留 `2e-6 m` 支撑平面外扩；简化误差上限为 `5e-5 m`，
**相对每个分组的精确凸包，而非真实 CAD 凹面**。内部孔洞仍保守填充。
没有改变可视实体、惯量、关节或碰撞规则。

2026-10-31 的另一条历史规划记录：同一起点、核心位置 `(0.2, 0, 0.4) m`、绕世界系 +Z 旋转
`π rad`，插入/平移/上翻完成，扭转覆盖整个 `[-π/2, π/2] rad`。
这些是当时模型和算法版本的记录，不是当前输出的保证；旧版验收采样数不再作为当前指标，
当前版本应运行前述参考场景，以规划状态、阶段完成、实际 Q 覆盖及 FK/碰撞查询计数检查结果。
生成方法、分组表和离线维护命令见 `docs/collision_geometry.md`；网格清单与误差在
`model/meshes/collision/manifest.json` 中。

## 排查启动失败

- `Package not found`：先确认此包构建成功，再在同一终端加载当前仓库的
  `install/setup.bash`。`setup.bash` 存在不代表包已成功安装。
- 找不到 `joint_state_publisher_gui` 或 `joint_state_publisher`：执行上述
  `rosdep install`；Jazzy 下也可直接安装
  `sudo apt install ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui`。
- 模型位于包共享目录的 `model/urdf/`，不是 `urdf/`。

无需启动节点或窗口即可检查包索引和启动描述：

```bash
ros2 pkg prefix rm27_heavy_arm_demo
ros2 launch rm27_heavy_arm_demo display.launch.py --show-args
```
