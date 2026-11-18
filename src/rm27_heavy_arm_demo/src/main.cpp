#include "rm27_heavy_arm_demo/rm27_heavy_arm_demo.hpp"

#include <tech_core_planner/planner.hpp>

#include <iostream>
#include <chrono>
#include <iomanip>
#include <string_view>
#include <thread>

/**
 * @brief 运行一次 RM27 机械臂规划并输出规划状态。
 * @param[in] argc 命令行参数数量，无物理单位，不涉及坐标系。
 * @param[in] argv 命令行参数数组；--headless 禁用查看器，--reference-scene 使用 README 中的参考场景，无物理单位。
 * @return 0 表示规划成功或请求帮助，1 表示规划失败，2 表示参数非法。
 */
int main(int argc, char ** argv){

    bool headless = false;
    bool reference_scene = false;
    for (int index = 1; index < argc; ++index) {
        const std::string_view argument(argv[index]);
        if (argument == "--headless") {
            headless = true;
        } else if (argument == "--reference-scene") {
            reference_scene = true;
        } else if (argument == "--help") {
            std::cout << "Usage: rm27_planner_demo [--headless] [--reference-scene]\n";
            return 0;
        } else {
            std::cerr << "Unknown argument: " << argument << '\n';
            return 2;
        }
    }

    auto arm_config = rm27_heavy_arm_demo::make_arm_config();

    auto planner_config = tech_core_planner::PlannerConfig{
        .arm = arm_config,
        .approach_offset = 0.04,
        .headless = headless
    };

    tech_core_planner::Planner planner(planner_config);

    auto request = rm27_heavy_arm_demo::make_plan_request();
    if (reference_scene) {
        request.scene.world_from_core = Eigen::Isometry3d::Identity();
        request.scene.world_from_core.translate(Eigen::Vector3d(0.45, 0.0, 0.4));
        request.scene.world_from_core.rotate(Eigen::AngleAxisd(
            tech_core_planner::kPi, Eigen::Vector3d::UnitZ()));
        request.start.resize(7);
        request.start << 0.7, -0.5, -0.4, 1.9, 1.9, 0.4, 0.9;
    }

    auto result = planner.plan(request);

    if(result.status.code == tech_core_planner::StatusCode::Ok){
        std::cout << "Planning succeeded." << std::endl;
    }else{
        std::cout << "Planning failed: " << result.status.message << std::endl;
    }

    std::cout << std::setprecision(12)
        << "core_position_m=" << request.scene.world_from_core.translation().transpose() << '\n'
        << "start_joints_rad=" << request.start.transpose() << '\n'
        << "planning_ok=" << (result.status.code == tech_core_planner::StatusCode::Ok) << '\n'
        << "inserted=" << result.completion.inserted
        << " lifted=" << result.completion.lifted
        << " flipped=" << result.completion.flipped
        << " trajectory_knots=" << result.trajectory.knots.size() << '\n'
        << "elapsed_s=" << result.metrics.total << '\n'
        << "fk_evaluations=" << result.metrics.scene.fk_evaluations
        << " collision_states=" << result.metrics.scene.collision_states
        << " collision_calls=" << result.metrics.scene.collision_calls << '\n';
    if (result.status.code == tech_core_planner::StatusCode::Ok && result.twist) {
        const auto covered = result.twist->covered();
        std::cout << "twist_range_rad=[" << covered.lower << ',' << covered.upper << "]\n";
    }

    while (!headless && planner.viewer_running()) {
        std::this_thread::sleep_for(std::chrono::duration<double>(0.05));
    }

    return result.status.code == tech_core_planner::StatusCode::Ok ? 0 : 1;

}
