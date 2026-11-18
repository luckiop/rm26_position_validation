#include "rm27_heavy_arm_demo/rm27_heavy_arm_demo.hpp"

#include <tech_core_planner/robot.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace rm27_heavy_arm_demo {

std::filesystem::path installed_model_directory(){

    static constexpr char PACKAGE_NAME[] = "rm27_heavy_arm_demo";

    return std::filesystem::path(ament_index_cpp::get_package_share_directory(PACKAGE_NAME)) / "model" / "mjcf" / "rm27_heavy_arm_demo.xml";

}

tech_core_planner::JointVector planning_home_joints(){




    return [](){
        tech_core_planner::JointVector home(7);
        home << 0.0, 1.39, -1.3, 0.0, 0.0, 0.3, 0.0;
        return home;
    }();
}


tech_core_planner::ArmModelConfig make_arm_config(
  const std::filesystem::path & model_directory){

    tech_core_planner::ArmModelConfig config;

    if(model_directory.empty()){
        config.model_path = installed_model_directory();
    }else{
        config.model_path = model_directory;
    }

    config.base_link = "base_link";

    config.planning_joints = {
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
        "joint_7"
    };

    config.attach_link = "link_7";

    config.attach_link_from_energy_unit = Eigen::Isometry3d::Identity();
    config.attach_link_from_energy_unit.rotate(Eigen::AngleAxisd(
        -tech_core_planner::kPi / 2.0, Eigen::Vector3d::UnitY()));
    config.attach_link_from_energy_unit.pretranslate(
        Eigen::Vector3d(0.1355042231, 0.0, -0.0725));

    config.grip_contact_links = {
        "link_7",
    };

    config.default_fixed_joints = {};

    return config;

}

tech_core_planner::PlanOptions make_plan_options(){

    tech_core_planner::PlanOptions options;

    options.method = tech_core_planner::PlannerMethod::PredictorCorrectorWithCollisionCache;
    options.timeout = 5.0;

    options.ik = tech_core_planner::IKOptions();
    options.approach = tech_core_planner::ApproachOptions();
    // 当前验证工作区使用旧版零端点速度三次时间参数化接口。
    options.timing.limits.velocity = tech_core_planner::JointVector::Constant(7, 5.0);
    options.timing.limits.acceleration = tech_core_planner::JointVector::Constant(7, 10.0);
    options.validation = tech_core_planner::ValidationOptions();
    options.optimize_joint_motion = true;
    options.approach_waypoint = std::nullopt;
    options.posture = tech_core_planner::PostureSearchOptions();
    options.tracking = tech_core_planner::LocalTrackingOptions();
    options.resources = tech_core_planner::PlanningResourceOptions();
    options.twist = tech_core_planner::TwistSearchOptions();

    return options;

}

tech_core_planner::PlanRequest make_plan_request(){

    tech_core_planner::PlanRequest request;

    Eigen::Isometry3d world_from_core = Eigen::Isometry3d::Identity();
    world_from_core.translate(Eigen::Vector3d(0.2, 0.0, 0.4));
    world_from_core.rotate(Eigen::AngleAxisd(
        tech_core_planner::kPi * 0.8, Eigen::Vector3d::UnitZ()));
    world_from_core.rotate(Eigen::AngleAxisd(
        tech_core_planner::kPi * 0.1, Eigen::Vector3d::UnitX()));

    request.scene = tech_core_planner::SceneConfig();
    request.scene.world_from_core = world_from_core;

    request.start = planning_home_joints();
    request.task = {
        .level = tech_core_planner::AssemblyLevel::Full,
        .twist_mode = tech_core_planner::TwistMode::MaximumRange,
        .twist_range = {
            .lower = -tech_core_planner::kPi / 2.0,
            .upper = tech_core_planner::kPi / 2.0
        },
        .position_spacing = 0.002,
        .angular_spacing = tech_core_planner::kPi / 180.0
    };
    request.options = make_plan_options();
    return request;
}

} // namespace rm27_heavy_arm_demo
