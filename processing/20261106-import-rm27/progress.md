# 进度记录

## 2026-11-06
- 已核对远端 master 9a08859c33508e890363e767eaa14e440b882d0c；源模型位于 src/demos/rm27_heavy_arm_demo。
- 当前验证仓库依赖旧版规划器接口；导入上层 src/rm27_heavy_arm_demo，适配模型配置与演示，不迁移规划器算法。
- RM27 使用 joint_1–joint_7，能量单元挂载在 link_7；沿用上游初始姿态和挂载变换。RM26 J1 水平限位仅适用于 RM26。
- 完成：从远端主线逐文件校验并复制 82 个文件；模型与网格保持上游字节内容，清单记录所有源文件哈希。配置库使用当前 PredictorCorrectorWithCollisionCache 与旧版 TimingOptions::limits。
- 完成：新增 arm_rm27.json，求解器按 model_family 选择关节与挂载变换；批次、浏览与重放保留模型身份并验证实际加载模型及初始姿态。RM27 不应用 RM26 J1 水平限制。
- 构建：./scripts/build.sh 通过，5 个包完成。新增 RM27 实际模型/重放检查，并开始验证包测试。
- 验证：RM27 专项 6 项全部通过，覆盖资源哈希、模型身份隔离、实际加载、起始关节姿态、快照重放、RM26 默认配置与 RM27 碰撞名称分类。
- 验证：验证包 CTest 共 8 项，7 项通过；原有 level3_real_model_integration 仍失败，错误为 Actual start collision: j2_link_collision_000|tech_core_planner_core/core_shell_collision_004，与仓库已有真实模型碰撞限制一致。其余杆长变体、J1 限位、批次合同、浏览器、交互及外壳碰撞回归通过。
- 批处理短预算检查：artifacts/rm27-import-checks/batch，1 个案例、0.25 s 预算，返回 Timeout；已确认 arm_model_loaded=true、model_family=rm27、link_7 挂载、正确非零初始姿态、J1 限位为 null，快照命令包含 --arm-family=rm27。此检查不证明完整任务成功。
- 独立演示入口 ros2 run rm27_heavy_arm_demo rm27_planner_demo --help 通过；git diff --check 通过。构建/测试日志位于 artifacts/rm27-import-checks/。未修改子模块源码。
- 最终总结：RM27 模型、URDF/MJCF/网格与兼容配置库已导入上层仓库，可使用 arm_rm27.json 在现有批量验证和重放中选择；模型资源与上游提交一致。完整任务效果尚需按实际采样范围与预算运行评估。
