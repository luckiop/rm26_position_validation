# RM26 固定基座三级任务批量验证

本包默认使用 `rm26_engineering_arm_demo` 的 J0–J6 精简七关节模型和 `tech_core_planner::Planner`，对规则表中的科技核心位姿进行确定性六维拉丁超立方采样。每个请求从七关节零位开始；三级任务使用 `RequiredRange`，须完整覆盖 Q `[-π/2, π/2] rad`，两侧由同一个已验收的 Q 剖面获得。最终角驻留由操作手完成，不包括在规划结果中。

## 构建和运行

从本仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
./scripts/build.sh
source install-integrated/setup.bash
colcon test --build-base build-integrated --base-paths rm_tech_core_trajectory_solve/src/tech_core_planner rm_tech_core_trajectory_solve/src/rm26_engineering_arm_demo rm_tech_core_trajectory_solve/thrid_party/mujoco src/rm26_level3_validation --packages-select rm26_level3_validation
colcon test-result --test-result-base build-integrated --verbose

# 输出目录必须尚不存在。先运行 10 个检查案例，再运行 100 个正式案例。
install-integrated/rm26_level3_validation/bin/level3_batch run --config src/rm26_level3_validation/config/default.json --set quick --output artifacts/level3-quick
install-integrated/rm26_level3_validation/bin/level3_batch run --config src/rm26_level3_validation/config/default.json --set full --output artifacts/level3-full

# 打开批次案例浏览窗口；点击案例启动三维预览。
install-integrated/rm26_level3_validation/bin/level3_batch browse --batch artifacts/level3-full

# 从已完成的批次中选择案例，重新规划并打开查看器。
install-integrated/rm26_level3_validation/bin/level3_batch replay --batch artifacts/level3-full --case case_0000
```

运行前要安装 git-lfs 并执行 `git lfs pull`，以获取机械臂和场景的网格。工具会检查安装资源中未下载的 LFS 指针。运行环境需加载 colcon 的 `install/setup.bash`，MuJoCo 构建依赖参见仓库根目录 Readme。

`--config` 指定 JSON 配置；上层源码配置在 `src/rm26_level3_validation/config/default.json`。未指定时读取构建时安装的副本，修改源码配置后需重建或通过 `--config` 显式指定。可修改样本数量、快速子集数量、随机种子、每次预算、各维边界和装配系在基座系中的位姿。`--workers N` 控制并发数，默认 1；并发会改变运行负载及有墙钟预算的结果。每次运行都写入新的输出目录，不覆盖已有批次。

默认模型仅保留 J0–J6 及其夹持几何，空 `base_link` 仅用于保留原基座坐标；车体、轮组、云台与相机的视觉和碰撞体不参与规划。旧批次的案例重算使用当前模型，可能与原结果不同；更改模型后建议重新运行批次。

## 机械臂自碰撞

配置 `"check_self_collision": true` 默认检查机械臂连杆之间的碰撞；设为 `false` 则忽略机械臂连杆相互碰撞，仍检查机械臂、能量单元与科技核心等其他碰撞。此设置随案例与配置快照保存，浏览、重算和交互编辑使用同一设置；旧批次未记录时默认开启。修改后需新建批次。关闭时规划结果不保证机械臂自身无碰撞。

## 夹爪与装配柱碰撞

配置 `"check_gripper_pillar_collision": false` 可忽略夹爪连杆与装配柱的碰撞；设为 `true` 恢复检查，旧批次未记录时默认检查。当前 RM26 夹爪连杆为 `j6_link`，装配柱包括 `core_peg` 与 `core_pillar_*` 几何；核心外壳、能量单元及其他机械臂连杆的碰撞仍按原规则检查。该设置随案例快照保存，重新规划与编辑沿用；修改后需新建批次。

## 坐标与任务

输入规则范围为装配系 `x/y/z`，以及 `theta/phi/alpha`。核心方向约定为 `Rz(theta) Ry(phi - π/2) Rx(alpha)`；装配系在基座系中的旋转采用 `Rz(yaw) Ry(pitch) Rx(roll)`。当前参数以配置文件为准。这是一组仿真假设，实际安装必须测量后更新。

示例中的虚拟抓持固定；速度 `0.5 rad/s`、加速度 `1.0 rad/s²` 是仿真限值，不是实机参数。规划器使用位置间距 `0.005 m`、角度间距 `0.05 rad` 和默认碰撞/跟踪验收配置。仅按照规则表范围采样，不检查场地底座正面平面；参数范围内的组合不保证实际赛场结构可实现。

`cases.json` 保存规则位姿与基座系位置、四元数，`config.json` 和 `metadata.json` 保存实验设置，`results.json` / `results.csv` 保存逐例信息，`summary.json` 统计全部请求的成功率、状态和耗时。每个案例的 `candidate.csv` 是失败候选，禁止执行；成功时 `prefix.csv` 是前段，`negative.csv` 和 `positive.csv` 是从实际起点到对应 Q 端点的两条已验收轨迹，不应简单按文件顺序拼接。

案例查看会在批次下新建 `replays/` 目录，显示原始批次结果，并按保存的请求重新规划；重新规划受运行时负载及预算影响，结果可能不同，不是原始轨迹回放。查看器只回放仿真运动学状态。

## 结果边界

所有请求包括失败、异常和超时都留在分母；缺失模型资源属于运行环境错误，在创建批次前报告。`status_counts` 使用规划器的结构化状态；`failure_hints` 从非结构化错误消息归纳，仅供排查，原始消息保留在逐例结果中。`candidate_last_phase` 只是返回候选中最后一个阶段，并非可靠的失败发生阶段；具体状态、消息和最近一次失败单独记录。严格覆盖失败时规划器可能不返回部分 Q 区间，此时 `q_coverage_rad` 为 `null`，不能推断为零。有限时间与初值下求解失败不证明物理不可达。碰撞依赖当前代理几何与离散采样；不验证真实抓持、动力学、硬件精度、驻留或完整赛场环境。


`q_diagnostic` 记录最后一个候选的两侧 Q 扩展：`reached_rad` 是局部扩展最后到达的角度，`failed_target_rad` 是未能扩展到的目标角，`failure_status` 和 `failure_message` 是该侧最近一次扩展失败原因。未进入 Q 阶段时整体为 `null`；没有实际失败的目标（例如预算先用尽）时目标为 `null`。这些数据未经过完整轨迹验收，也不是所有候选中最远的覆盖。旧批次不会自动补充诊断，重新规划或重新运行批次才会生成。

案例浏览窗口左右等宽，左侧每个案例显示为自适应方格：按已完成进度分色：4分绿色（全部通过），3分黄绿色（已完成P上翻），2分黄色（已完成平移），1分橙色（已完成插入），0分红色（尚未完成插入），窗口顶部显示图例。灰色表示未运行，深灰表示阶段未知。每个阶段通过后才增加进度，Q旋转或最终验收失败仍为3分；点击案例后重算并打开三维窗口，原结果和本次结果分别显示。失败步骤为最后一个候选执行时的阶段，旧批次只在原消息能明确判断时标注，否则显示未知。重算结果在预览窗口打开期间保存至 `replays/`。点击下一个案例会自动关闭上一三维预览及其重算进程，再打开新案例；连续点击以最后一次选择为准。关闭浏览器也会关闭它启动的预览。需要图形显示及 `python3-tk`。

案例查看器的 `Core XYZ (assembly)` 和 `Core theta / phi / alpha (assembly)` 使用批次配置的装配系。规则角以角度显示，顺序和含义分别对应配置中的 `theta_rad`、`phi_rad`、`alpha_rad`；方向约定为 `Rz(theta) Ry(phi - π/2) Rx(alpha)`。直接调用没有装配系时，界面标记规则角不可用。在 phi 为 0 或 π 的奇异位姿，theta 和 alpha 不唯一，界面显示其中一组等价角度。JSON 和规划接口保持 rad。

`parking_offset_world_m` 为停车位置偏移 `[x, y, z]`，单位 m；`parking_yaw_world_rad` 为车体绕场地世界系 Z 轴的旋转角，右手定则，单位 rad。当前配置为 `-π/2 rad`。偏移在场地轴上给出；核心、装配系与地面保持固定。`assembly_in_base` 保留原配置语义，描述未停车、未旋转时基座系中的装配位姿，因此同时是固定场地位姿。程序将核心和装配系姿态转换到停车后的基座系，显示与规划一致。`cases.json` 保存固定的 `core_in_world`、停车后的 `core_in_base`、停车偏移与 yaw；旧案例没有 yaw 时按零处理。修改这些参数后需新建批次。

右侧按每个停车点分别展示原批次扇形图：包括各进度分数，逐项显示案例数及百分比，分母为该停车点已运行案例总数；未运行数量单独显示，不计入占比。未知阶段保留为独立项，没有已运行案例时显示“暂无已运行案例”。右侧内容支持滚动；点击案例重算不会改写原批次统计。


每批测试保留独立的 `config.json` 参数快照。每个案例目录 `trajectories/<case_id>/` 还会保存 `config.json`、`case.json`、`request.json` 和 `result.json`；输入快照在求解开始前落盘，进程错误或超时也保留快照和结果。每次重算在独立的 `replays/<case_id>_<时间>/` 下保存同样的快照，保留原案例文件。旧批次选择案例时会从该批次保存的数据补齐缺失的案例快照。

右侧顶部的“复制案例编号和路径”按钮将当前案例的 `case_id` 和原案例目录的绝对路径复制到剪贴板；复制目标始终为 `trajectories/<case_id>/`，可从该目录读取参数、位姿与原结果。未选择案例或切换过程中按钮禁用。


## 停车位置网格

修改源码 `config/default.json` 后构建，或通过 `run --config <文件>` 指定独立配置：

```json
"parking_yaw_world_rad": -1.5707963267948966,
"parking_offset_world_m": [0.0, 0.1, 0.1],
"parking_grid": {"positions": 9, "step_x_m": 0.1, "step_y_m": 0.15}
```

`positions` 支持 `1`（默认/不提供网格配置）、`4`、`9`。步长沿场地世界系 X、Y 方向，单位 m，可为负，多点时必须非零；所有点 Z 保持原点高度。4 点索引为 `(0,0)、(1,0)、(0,1)、(1,1)`，即原点是网格角点；9 点索引为 `{0,1,2} × {0,1,2}`，原点也是角点。位置计算为 `原点 + [索引X × step_x_m, 索引Y × step_y_m, 0]`。

`samples` 和 `quick_samples` 为每个停车点的核心样本数。多点使用完全相同的场地核心姿态，full 总请求数为 `samples × positions`，quick 为 `quick_samples × positions`。所有停车点使用同一个配置的机械臂朝向。多点案例编号为 `case_0000_p00`，另存 `sample_id`、`parking_id`、`parking_grid_index` 和实际停车偏移，输入快照、重算和三维预览均使用该案例的实际停车点。

`summary.json` 的 `parking_results` 提供逐停车点统计，`results.csv` 附带样本编号、停车编号和 XYZ，浏览器右侧显示逐点成功数量。总成功率统计所有停车点请求，不表示任一停车点成功即可的覆盖率。旧批次继续读取自己的配置快照；修改默认配置后需新建批次。

### 从案例继续键鼠调整

`level3_batch replay` 和案例浏览器打开的三维窗口支持直接继续编辑当前 case 的核心位姿，RM26 planner demo 也使用同样控制：

- 按住 `I/K`、`J/L`、`U/O`：沿机械臂世界系的 X、Y、Z 正负方向平移核心，速度 `0.1 m/s`。
- 配合 `Shift`：分别绕世界系 X、Y、Z 轴旋转，速度 `1 rad/s`。
- 按 `P`：使用当前核心位姿重新规划；移动后也可以按 `Enter` 规划，规划完成后 `Enter` 播放通过验收的轨迹。
- 鼠标继续控制相机；调整和重算均保留当前相机视角。求解期间暂停核心编辑。

初始核心位姿、停车偏移、机械臂起点和任务配置来自所选 case。调整核心后立即清除旧轨迹；重新求解仍从该 case 原始机械臂起点开始。回放动画中的机械臂位置不会自动成为新的规划起点。

首次重算仍保存在本次 `replays/<case_id>_<时间>/`；后续编辑分别保存至其 `interactive/edit_1/`、`edit_2/` 等目录，包含 `core_pose.json`、`request.json`、`result.json` 和轨迹。`request.json` 保存编辑后的可重跑命令；若原快照存在，还会复制 `config.json` 和 `source_case.json`。浏览器右侧持续显示最新重算结果，原批次结果保留。

```bash
install-integrated/rm26_level3_validation/bin/level3_batch replay \
  --batch artifacts/level3-full-20261102-185008 --case case_0007
```

### 碰撞体与失败诊断

在案例浏览器中点击案例，或通过 `level3_batch replay` 打开三维窗口后，按 `C` 切换碰撞体显示：青色半透明实体是实际参与检测的碰撞几何，外观模型淡化显示。

当本次规划失败且最终消息或最近一次具体失败消息包含碰撞原因时，GUI 自动额外进行一次忽略碰撞的诊断求解，使用相同起点、核心位姿、限位、任务和求解预算。诊断仍可能因 IK、限位或预算失败；此时显示已有候选，不宣称找到完整解；若没有候选，则保留实际起点用于检查起点碰撞。诊断结束后按原始接触规则逐节点复查，允许的夹持和装配接触不会被当作违规高亮。

窗口自动打开碰撞体显示，并暂停在首个违规节点。红色实体是当前节点的违规碰撞体；右侧显示几何体名称、有符号距离与要求距离（m）。距离为负表示穿透；距离为正但小于要求值表示间隙不足。按左右方向键逐节点查看，`N/B` 跳到下一个/上一个有违规配对的节点，空格播放/暂停离散诊断候选，`C` 返回外观对照。屏幕最多列出三对，完整列表保存在 JSON 中。

原始 `result.json` 和批次统计保留真实规划结果。诊断另外保存为 `collision_diagnostic.csv`、`collision_diagnostic.json` 和 `collision_diagnostic_result.json`，编辑核心后的诊断位于对应 `interactive/edit_N/` 中。JSON 的 `solution_found` 只表示忽略碰撞后任务是否求解成功；所有诊断均为 `diagnostic_only: true`、`accepted: false`。诊断轨迹仅供查看，不生成验收通过的前段或 Q 轨迹；诊断结束或异常退出后恢复正常碰撞检查。

`j1_max_from_vertical_rad`：J1 连杆相对竖直向上的最大偏角，单位 rad，默认 `1.5707963267948966`，允许 `(0, π/2]`。连杆方向按 J1 转轴指向 J2 转轴定义，禁止 J2 转轴低于经过 J1 转轴的水平面；不表示整块连杆网格必须处于该平面上方。程序从当前 MJCF 正运动学计算竖直零偏和水平边界，当前模型关节 `j1` 的默认范围约为 `[-0.523599, 2.617992]` rad。限位用于逆解、路径搜索和连续轨迹校验。批量、重算、交互规划共用，结果记录实际关节限位 `j1_joint_limits_rad`。旧案例缺省采用 π/2；旧参数 `j1_max_abs_rad` 已由此参数替代。直接调用可追加 `--j1-max-from-vertical-rad=1.5707963267948966`。

## 批次结果绘图

安装 Python 依赖 `numpy`、`matplotlib` 后，在工作区根目录运行：

```bash
python3 rm26_position_validation/src/rm26_level3_validation/src/plot_validation.py \
  rm26_position_validation/artifacts/<批次目录> \
  --out rm26_position_validation/artifacts/<批次目录>/plots
```

省略 `--out` 时输出到批次的 `plots/`。生成 PNG 和 PDF：

- `xyz_projections`：机械臂基座系 XY、XZ、YZ 散点投影，每行一个停车点，所有停车点使用相同坐标范围和等比例坐标。
- `parking_sample_matrix`：停车点 × 核心样本结果矩阵，右侧显示成功数/已测数；标题显示所有停车点均已测样本的联合覆盖数/样本数。

颜色沿用案例浏览器的结果分类；颜色表示已完成进度，不代表空间可达边界。图例使用英文以避免系统中文字体依赖。脚本读取该批次的 `results.json`、`cases.json` 和 `config.json`，缺测显示灰色；旧批次缺少 `cases.json` 时只能显示结果中已有的样本。没有 `config.json` 时检查开关标为 unknown。

### 杆长探索用进度统计

所有指标使用0～4分进度。UI右侧显示各停车点平均分、世界系XY停车网格平均分热区图，以及逐任务停车适应分。热区图每格标注停车点编号、平均分、有效样本数和高度；绘图脚本额外导出 `parking_mean_score_heatmap.png/pdf`。

停车点平均分只计算已知分数，缺测和进度未知分别统计、不当作0分。总平均分为各有数据停车点平均分的等权平均。每个任务复用同一核心场地位姿，停车适应分是得到3分或4分的停车点数量（范围0～停车点总数，越高越好）；这表示完成P上翻或全部验收，并非全部任务均成功。适应分均值和数量直方图仅计入所有停车点分数均已知的任务；不完整任务单独列出，逐任务数量标为暂计。

批次保存报告时，`summary.json` 的 `progress` 字段、`progress_statistics.json` 和文本 `progress_statistics.log` 保存以上统计与逐任务适应分。浏览旧批次时UI直接依据原始结果计算统计，预览重算不改写原批次统计。

失败原因独立于进度分数统计：请求/进程超时、IK迭代上限、搜索预算耗尽归为“不可达（迭代超时/预算不足）”，这是本次测试的分类，不表示数学上的不可达证明，已完成进度仍保留。碰撞按报错的物体对区分“能量单元与机械臂”“机械臂与科技核心”；其余或未明确物体对的碰撞归为其他碰撞，模型/进程等问题归为其他失败。终止超时优先于残留候选碰撞消息。UI展示总数、各停车点数量和选中案例分类；`progress_statistics.json`、`summary.json.progress.failure_reasons` 与 `progress_statistics.log` 保存对应统计。历史结果缺少明确证据时不猜测物体对。

## 杆长变体：原始 / J1中段缩短

J1指 `j1_link` 上J1→J2的长杆。新增变体配置独立于任务采样配置：

```json
{
  "id": "j1-short-10cm",
  "changes": [
    {"link": "j1_link", "method": "remove_middle", "shorten_m": 0.1, "cut_center_m": 0.21}
  ]
}
```

`cut_center_m` 是从J1原点沿该杆在J1局部XY平面的轴向投影测量的切除中心。示例去掉轴向0.16～0.26m的一段，并将远端向近端平移0.1m；保留关节Z向偏置。两端各0.06m为禁止裁切区。支持J1、J4、J5、J6，可在同一变体中组合不同杆件；同一杆件只能修改一次。`remove_middle` 表示中段切除，`stretch_axial` 表示从杆件近端沿轴向拉伸外观及碰撞网格，并移动下游机构。变体的总杆长净变化不得超过原始四段轴向总长的±30%。UI显示每根杆的改动；空 `changes` 显示“原始”。

在 `rm26_position_validation` 根目录完成构建并加载环境后：

```bash
source /opt/ros/jazzy/setup.bash
./scripts/build.sh --packages-select rm26_level3_validation
source install-integrated/setup.bash

# 新目录；默认任务配置已显式标记原始臂
install-integrated/rm26_level3_validation/bin/level3_batch run \
  --config src/rm26_level3_validation/config/default.json \
  --arm-variant src/rm26_level3_validation/config/arm_original.json \
  --set quick --output artifacts/length-original-quick

install-integrated/rm26_level3_validation/bin/level3_batch run \
  --config src/rm26_level3_validation/config/default.json \
  --arm-variant src/rm26_level3_validation/config/arm_j1_short_10cm.json \
  --set quick --output artifacts/length-j1-short-10cm-quick

install-integrated/rm26_level3_validation/bin/level3_batch browse \
  --batch artifacts/length-j1-short-10cm-quick
```

可复制变体JSON，将 `shorten_m` 改成0.05等长度，或使用 `lengthen_m` / `lengthen_fraction` 加长轴向长度，并修改 `id`。多杆改动分别列入 `changes`，如J1加长10cm并同时缩短J5 10%：

```json
{
  "id": "j1-extend-10cm-j5-short-10pct",
  "changes": [
    {"link": "j1_link", "method": "stretch_axial", "lengthen_m": 0.1},
    {"link": "j5_link", "method": "remove_middle", "shorten_fraction": 0.1}
  ]
}
```

对 `remove_middle`，`shorten_m` 与 `shorten_fraction` 必须且只能选一个，且可用 `cut_center_m` 指定切除中心。对 `stretch_axial`，`lengthen_m` 与 `lengthen_fraction` 必须且只能选一个；沿轴向缩放杆件网格，并将子关节或J6末端挂载点同步外移。命令行 `--arm-variant` 优先。缩短量触及关节端保护区，或组合的总杆长净变化超出±30%，会拒绝生成。几何生成依赖 `python3-numpy` 和 `python3-scipy`。

每次运行在 `<批次>/arm_model/` 冻结XML及全部引用网格，`variant_config.json`保存生成参数，`manifest.json`记录来源文件哈希、裁切轴与区间、J2修改前后偏移以及生成资源的SHA256。`config.json`、`metadata.json`、每例 `case.json/request.json/result.json`、`results.csv`、`summary.json`、`progress_statistics.log` 均记录机械臂身份；求解器返回实际加载路径，批次层核对并写入 `arm_model_loaded`。重算使用该批次的模型快照，文件或清单哈希不匹配则拒绝启动。旧批次显示“原始（历史批次未记录模型指纹）”，不能追认其精确模型版本。UI顶部突出显示臂名称和指纹。

`remove_middle` 对外观三角面和原碰撞凸块同步裁切、封口，远端移动并同步下游机构；`stretch_axial` 沿轴向拉伸原外观及每个碰撞凸块，不在杆中留下几何空隙。保留原质量和惯量，清单明确标记：仅供当前运动学及碰撞规划，不用于动力学/制造结论。相同样本和停车点配置可公平比较任务分、适应分；短预算smoke只验证加载和数据链路，不用于评估长度优劣。


### J5缩短20% / 40%

新增 `config/arm_j5_short_20pct.json` 和 `config/arm_j5_short_40pct.json`。J5遵循与J1相同命名：表示 `j5_link` 上J5→J6这一段。

```json
{"id": "j5-short-20pct", "changes": [
  {"link": "j5_link", "method": "remove_middle", "shorten_fraction": 0.2}
]}
```

比例相对于原始J5→J6的局部−Z轴向间距0.0755m计算，保留X向0.012369m偏置，非整块网格包围盒尺寸或三维关节距离的比例。20%切除0.0151m，剩余轴向间距0.0604m；40%切除0.0302m，剩余0.0453m。默认从轴向中点0.03775m裁切；J5两端保护区各0.012m。J6位姿平移随动，J6局部旋转、夹爪及能量单元挂载关系保持原定义。外观和碰撞网格同步裁切，保留原质量/惯量的政策不变。

`shorten_m` 与 `shorten_fraction` 必须且只能选一个；可选 `cut_center_m` 覆盖轴向切除中心。批次清单同时记录输入比例、实际切除长度与修改前后关节偏移，UI显示“J5缩短20%”或“J5缩短40%”。运行时使用 `--arm-variant src/rm26_level3_validation/config/arm_j5_short_20pct.json`（或40pct）；建议批次名称包含 `j5-short-20pct` / `j5-short-40pct`。

### J4、J6各自缩短20% / 40%

四个新增配置为 `arm_j4_short_20pct.json`、`arm_j4_short_40pct.json`、`arm_j6_short_20pct.json`、`arm_j6_short_40pct.json`，每份只改一个杆件。

| 杆件 | 比例基准（轴向） | 20%缩短后 | 40%缩短后 |
| --- | --- | --- | --- |
| J4→J5，局部+X | 0.1211m | 0.09688m | 0.07266m |
| J6→夹爪内垫中心，局部+Z | 0.173794121m | 0.1390352968m | 0.1042764726m |

J4保留J5的局部Z偏置0.0245m，其后所有机构随J5平移。J4两端各0.018m为保护区。J6以原始夹爪内垫中心的Z坐标为末端轴向长度，保留横向偏置，J6两端各0.025m为保护区；并非以包围盒高度或J6到能量单元孔口的三维距离为比例基准。

J6没有下一级旋转关节，生成模型增加固定子体 `variant_energy_mount`，其位置为裁切引起的负Z平移。批次自动向求解器传入 `--arm-attach-link=variant_energy_mount`，能量单元通过该子体挂载，原来的能量单元局部安装变换不变。因此能量单元、孔口工具点和夹爪远端一起平移，不能仅缩短网格而保留原工具点。清单记录挂载体及位移；求解器报告实际 `arm_attach_link`，与批次身份不一致时拒绝采信结果。原始、J1、J4和J5变体继续挂载到j6_link。

运行时选择对应的 `--arm-variant src/rm26_level3_validation/config/arm_j4_short_20pct.json` 等配置；名称建议使用 `level3-full-j4-short-20pct-<时间戳>` 等格式。UI自动显示相应杆件及比例，所有身份/指纹/统计保存流程沿用。
