# RM26 机械臂位置验证

上层仓库提供三级任务批量验证、停车点网格、统计和案例浏览；子模块 `rm_tech_core_trajectory_solve` 提供规划器、模型以及交互查看器和碰撞诊断。查看器以角度显示核心规则角 theta、phi、alpha，不显示方向圆弧；姿态采样和编辑保留。

子模块分支 `feature/position-validation` 基于本机 `dev-构型验证`，包含该分支已有的交互功能；它**不是当前 master 的后代**，与 master 合并会涉及规划器、查看器及模型的冲突。新分支及其依赖的本地提交尚未推送，其他机器需要先能从子模块远端获取相应提交。

已从上游主线 `9a08859` 导入独立的 RM27 重载七关节机械臂，位于 `src/rm27_heavy_arm_demo/model/`，同时包含 MJCF、URDF、视觉与碰撞网格。`source_manifest.json` 记录源提交及原始文件哈希。构建后使用以下命令选择 RM27：

```bash
install-integrated/rm26_level3_validation/bin/level3_batch run \
  --config src/rm26_level3_validation/config/default.json \
  --arm-variant src/rm26_level3_validation/config/arm_rm27.json \
  --set quick --output artifacts/rm27-quick
```

RM27 使用上游的 `joint_1`–`joint_7`、`link_7` 夹爪挂载变换和起始关节角 `[0, 1.39, -1.3, 0, 0, 0.3, 0] rad`。批次冻结资源、记录模型身份，浏览与重放沿用 RM27 配置。RM26 的 J1 水平限位与杆长变体仅适用于 RM26；默认模型仍为 RM26。导入兼容当前验证工作区的规划器接口，上游成功案例不代表当前算法、采样范围与停车配置下的成功率。

## 构建与运行

依赖 ROS 2 Jazzy 环境、colcon、C++17、Eigen、GLFW、OpenGL、Python 3、Tk、Git LFS，以及子模块 MuJoCo 安装脚本所需工具。网络和依赖准备见子模块说明。

```bash
git submodule update --init --recursive
cd rm_tech_core_trajectory_solve && git lfs pull && cd ..
source /opt/ros/jazzy/setup.bash
./scripts/build.sh
source install-integrated/setup.bash

# 输出目录必须不存在
install-integrated/rm26_level3_validation/bin/level3_batch run --config src/rm26_level3_validation/config/default.json --set quick --output artifacts/quick
install-integrated/rm26_level3_validation/bin/level3_batch browse  # 自动打开 artifacts 中最近更新的批次
install-integrated/rm26_level3_validation/bin/level3_batch replay --batch artifacts/quick --case case_0000
install-integrated/rm26_level3_validation/bin/arm_dimensions_gui  # 滑块调姿并搜索最大伸展
```

脚本直接构建子模块与上层验证包，不复制或修改子模块源码。构建时请按子模块的文档安装依赖；首次运行可能下载 MuJoCo。默认使用仅保留 J0–J6 机械臂及空基座坐标系的模型；车辆其余连杆和碰撞体不参与规划。旧批次的案例重算使用当前模型，结果可能不同，建议重新运行批次。`check_self_collision` 可控制机械臂自碰撞检查，默认 `true`；设为 `false` 可关闭，核心及能量单元碰撞仍检查。`check_gripper_pillar_collision` 控制夹爪与装配柱碰撞检查；当前配置为 `false`。车辆默认绕场地世界 Z 轴旋转 −90°，可通过 `parking_yaw_world_rad`（rad）调整。修改 `src/rm26_level3_validation/config/default.json` 后，按示例使用 `--config` 指定该文件；不指定时使用构建时安装的配置副本。完整参数、结果语义、键鼠控制和诊断方式见 [验证工具说明](src/rm26_level3_validation/README.md)。

当前基线的真实模型集成测试仍未通过：参考案例报告机械臂与核心碰撞，尚未验证完整 Q 区间成功规划。批量结果可能全部失败，应根据实际场景检查模型、抓持关系和坐标配置；不能将失败候选当作成功轨迹。
