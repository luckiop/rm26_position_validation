/**
 * @file rm27_heavy_arm_demo.hpp
 * @brief RM27 重载机械臂模型配置与规划请求接口。
 */
#pragma once

#include <tech_core_planner/planner.hpp>
#include <tech_core_planner/robot.hpp>
#include <tech_core_planner/types.hpp>

#include <filesystem>

/** @brief RM27 重载机械臂规划演示。 */
namespace rm27_heavy_arm_demo {

/**
 * @brief 查询安装后的机械臂 MJCF 模型文件路径。
 * @return 模型文件的绝对路径，无物理单位，不涉及坐标系。
 * @throws ament_index_cpp::PackageNotFoundError 无法在当前环境中找到本包。
 * @note 返回模型文件而非目录；调用前应加载工作空间的安装环境。
 */
std::filesystem::path installed_model_directory();

/**
 * @brief 创建机械臂模型配置。
 * @param[in] model_directory MJCF 模型文件路径；为空时使用已安装模型，无物理单位，不涉及坐标系。
 * @return 机械臂模型配置，世界系为 base_link；挂载变换的平移使用 m，关节角使用 rad。
 * @throws ament_index_cpp::PackageNotFoundError 使用默认模型时无法找到本包。
 */
tech_core_planner::ArmModelConfig make_arm_config(
  const std::filesystem::path & model_directory = {});

/**
 * @brief 创建机械臂规划时的起始关节位置。
 * @return 按 joint_1 至 joint_7 排列的预置起始关节角 [0, 1.39, -1.3, 0, 0, 0.3, 0]，单位 rad，相对各关节零位。
 */
tech_core_planner::JointVector planning_home_joints();

/**
 * @brief 创建机械臂规划算法参数。
 * @return 规划参数，时间使用 s，关节速度使用 rad/s，加速度使用 rad/s²；无独立坐标系。
 */
tech_core_planner::PlanOptions make_plan_options();

/**
 * @brief 创建机械臂规划请求。
 * @return 包含核心系在世界系中的位姿、起始关节角和任务参数的请求；长度使用 m，角度使用 rad，时间使用 s。
 */
tech_core_planner::PlanRequest make_plan_request();

} // rm27_heavy_arm_demo
