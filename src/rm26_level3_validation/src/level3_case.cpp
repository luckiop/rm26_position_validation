/** @file level3_case.cpp
 * @brief 单个 RM26 / RM27 三级请求的规划与轨迹导出；参数均为 m、rad、s。
 */
#include <rm26_engineering_arm_demo/rm26_arm.hpp>
#include <rm27_heavy_arm_demo/rm27_heavy_arm_demo.hpp>
#include <tech_core_planner/planner.hpp>

#include <mujoco/mujoco.h>
#include "visualization/visibility_evaluation.hpp"

#include <algorithm>
#include <memory>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>
#include <thread>

namespace {
namespace tcp = tech_core_planner;

/** @brief 检查模型目录中是否仍有未下载的 LFS 对象。 @param[in] directory 模型资源目录。 */
void check_resources(const std::filesystem::path & directory)
{
  if (!std::filesystem::is_directory(directory)) {throw std::runtime_error("missing model resources: " + directory.string());}
  for (const auto & entry : std::filesystem::recursive_directory_iterator(
      directory, std::filesystem::directory_options::follow_directory_symlink)) {
    if (!entry.is_regular_file()) {continue;}
    const auto extension = entry.path().extension();
    if (extension != ".obj" && extension != ".stl" && extension != ".STL" && extension != ".xml") {continue;}
    std::ifstream input(entry.path(), std::ios::binary);
    if (!input) {throw std::runtime_error("cannot read model resource: " + entry.path().string());}
    const std::string signature = "version https://git-lfs.github.com/spec/v1";
    std::string prefix(signature.size(), '\0');
    input.read(prefix.data(), static_cast<std::streamsize>(prefix.size()));
    if (prefix == signature) {
      throw std::runtime_error("Git LFS object missing: " + entry.path().string() + "; install git-lfs and run git lfs pull");
    }
  }
}

/**
 * @brief 从模型计算 J1 连杆朝上时的关节限位。
 * @param[in] path 固定底座机械臂 MJCF 路径。
 * @param[in] max_from_vertical 相对竖直向上的最大偏角，单位 rad。
 * @param[in] is_rm27 RM27 使用 joint_2 和 link_2 → link_3。
 * @return 肩部俯仰关节限位，单位 rad；保证肘部原点不低于肩部。
 * @note 连杆方向定义为 j1_link 原点指向 j2_link 原点，当前车辆仅绕世界 Z 轴停车旋转。
 */
tcp::JointRange j1_above_horizontal_limits(
  const std::filesystem::path & path, double max_from_vertical, bool is_rm27 = false)
{
  if (!(max_from_vertical > 0.0 && max_from_vertical <= tcp::kPi / 2)) {
    throw std::invalid_argument("J1 deviation must be in (0, pi/2] rad");
  }
  char error[1024] = {};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
    mj_loadXML(path.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
  if (!model) {throw std::runtime_error(error);}
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
  if (!data) {throw std::runtime_error("cannot allocate J1 model data");}
  const int joint = mj_name2id(model.get(), mjOBJ_JOINT, is_rm27 ? "joint_2" : "j1");
  const int proximal = mj_name2id(model.get(), mjOBJ_BODY, is_rm27 ? "link_2" : "j1_link");
  const int distal = mj_name2id(model.get(), mjOBJ_BODY, is_rm27 ? "link_3" : "j2_link");
  if (joint < 0 || proximal < 0 || distal < 0) {
    throw std::runtime_error("shoulder elevation constraint requires the shoulder joint and adjacent links");
  }
  const auto height = [&](double angle) {
    data->qpos[model->jnt_qposadr[joint]] = angle;
    mj_forward(model.get(), data.get());
    return data->xpos[3 * distal + 2] - data->xpos[3 * proximal + 2];
  };
  // Height is a*cos(q) + b*sin(q) + c. Account for the exported model's small axis tilt.
  const double h0 = height(0.0);
  const double hpi = height(tcp::kPi);
  const double c = (h0 + hpi) / 2;
  const double a = (h0 - hpi) / 2;
  const double b = height(tcp::kPi / 2) - c;
  const double radius = std::hypot(a, b);
  if (!(radius > std::abs(c))) {throw std::runtime_error("invalid J1 elevation geometry");}
  const double upright = std::atan2(b, a);
  const double horizontal_half_range = std::acos(-c / radius);
  const double half_range = std::min(max_from_vertical, horizontal_half_range);
  return {upright - half_range, upright + half_range};
}

/** @brief 返回状态码的稳定文本。 @param[in] code 状态码。 @return 状态名称。 */
const char * status_name(tcp::StatusCode code)
{
  switch (code) {
    case tcp::StatusCode::NotRun: return "NotRun";
    case tcp::StatusCode::Ok: return "Ok";
    case tcp::StatusCode::InvalidRequest: return "InvalidRequest";
    case tcp::StatusCode::ModelError: return "ModelError";
    case tcp::StatusCode::StartCollision: return "StartCollision";
    case tcp::StatusCode::IKFailed: return "IKFailed";
    case tcp::StatusCode::PlanningFailed: return "PlanningFailed";
    case tcp::StatusCode::ValidationFailed: return "ValidationFailed";
    case tcp::StatusCode::Timeout: return "Timeout";
    case tcp::StatusCode::Cancelled: return "Cancelled";
  }
  return "Unknown";
}

/** @brief 转义 JSON 字符串。 @param[in] text 原始文本。 @return JSON 字符串。 */
std::string json_string(const std::string & text)
{
  std::string result = "\"";
  constexpr char hex[] = "0123456789abcdef";
  for (unsigned char c : text) {
    if (c == '"' || c == '\\') {result += '\\'; result += static_cast<char>(c);}
    else if (c < 32) {result += "\\u00"; result += hex[c >> 4]; result += hex[c & 15];}
    else {result += static_cast<char>(c);}
  }
  return result + '"';
}

/** @brief 解析有限数。 @param[in] text 命令行文本。 @return 有限数。 */
double number(const char * text)
{
  std::size_t used = 0;
  const std::string input(text);
  const double result = std::stod(input, &used);
  if (used != input.size() || !std::isfinite(result)) {throw std::invalid_argument("invalid number");}
  return result;
}

/** @brief 序列化起始关节角。 @param[in] joints 按模型规划关节顺序排列的关节角，rad。 @return JSON 数组。 */
std::string joints_json(const tcp::JointVector & joints)
{
  std::ostringstream output;
  output << std::setprecision(17) << '[';
  for (Eigen::Index index = 0; index < joints.size(); ++index) {
    if (index) {output << ',';}
    output << joints[index];
  }
  output << ']';
  return output.str();
}

/** @brief 阶段名称。 @param[in] phase 阶段。 @return 阶段名称。 */
const char * phase_name(tcp::Phase phase)
{
  switch (phase) {
    case tcp::Phase::Approach: return "Approach";
    case tcp::Phase::Insert: return "Insert";
    case tcp::Phase::Lift: return "Lift";
    case tcp::Phase::Flip: return "Flip";
    case tcp::Phase::Twist: return "Twist";
  }
  return "Unknown";
}

/** @brief 保存轨迹节点。 @param[in] path 输出路径。 @param[in] trajectory 候选或已验收轨迹。 */
void write_trajectory(const std::filesystem::path & path, const tcp::Trajectory & trajectory)
{
  std::ofstream out(path);
  if (!out) {throw std::runtime_error("cannot write trajectory: " + path.string());}
  out << std::setprecision(17) << "time_s,phase,insertion_m,lift_m,flip_rad,twist_rad";
  for (const auto & name : trajectory.joint_names) {out << ',' << name;}
  out << ",target_x_m,target_y_m,target_z_m,axis_x,axis_y,axis_z\n";
  for (const auto & knot : trajectory.knots) {
    const auto & m = knot.task.mechanism;
    out << knot.time << ',' << phase_name(knot.task.phase) << ',' << m.insertion << ',' << m.lift
        << ',' << m.flip << ',' << m.twist;
    for (Eigen::Index i = 0; i < knot.q.size(); ++i) {out << ',' << knot.q[i];}
    for (int i = 0; i < 3; ++i) {out << ',' << knot.task.target.position[i];}
    for (int i = 0; i < 3; ++i) {out << ',' << knot.task.target.direction[i];}
    out << '\n';
  }
  if (!out) {throw std::runtime_error("failed writing trajectory: " + path.string());}
}
}  // namespace

/** @brief 求解指定世界系核心位姿，并打印 JSON 结果。 @return 0 表示调用完成，2 表示调用错误。 */
int main(int argc, char ** argv)
{
  try {
    std::filesystem::path visibility_track;
    bool visibility = false;
    std::filesystem::path arm_model_path;
    std::string arm_attach_link;
    std::string arm_family = "rm26";
    double j1_max_from_vertical_rad = tcp::kPi / 2;
    bool disable_self_collision = false;
    bool disable_gripper_pillar_collision = false;
    while (argc > 1) {
      const std::string option(argv[argc - 1]);
      if (option.rfind("--visibility-trajectory=", 0) == 0) {
        visibility = true; visibility_track = option.substr(24);
      }
      else if (option == "--disable-self-collision") {disable_self_collision = true;}
      else if (option == "--disable-gripper-pillar-collision") {disable_gripper_pillar_collision = true;}
      else if (option.rfind("--j1-max-from-vertical-rad=", 0) == 0) {
        j1_max_from_vertical_rad = number(option.substr(27).c_str());
      }
      else if (option.rfind("--arm-model-path=", 0) == 0) {
        arm_model_path = std::filesystem::absolute(option.substr(17));
      }
      else if (option.rfind("--arm-attach-link=", 0) == 0) {
        arm_attach_link = option.substr(18);
      }
      else if (option.rfind("--arm-family=", 0) == 0) {
        arm_family = option.substr(13);
      }
      else {break;}
      --argc;
    }
    if (arm_family != "rm26" && arm_family != "rm27") {
      throw std::invalid_argument("arm-family must be rm26 or rm27");
    }
    const bool is_rm27 = arm_family == "rm27";
    auto arm = is_rm27 ? rm27_heavy_arm_demo::make_arm_config() : rm26_engineering_arm_demo::make_arm_config();
    if (arm_model_path.empty()) {
      arm_model_path = is_rm27 ? arm.model_path :
        rm26_engineering_arm_demo::installed_model_directory().parent_path() / "rm26_arm_only" / "rm26_arm_only.xml";
    }
    arm.model_path = arm_model_path;
    if (!arm_attach_link.empty()) {arm.attach_link = arm_attach_link;}
    auto start_joints = is_rm27 ? rm27_heavy_arm_demo::planning_home_joints() :
      rm26_engineering_arm_demo::planning_home_joints();
    if (is_rm27) {
      const auto limits = j1_above_horizontal_limits(
        arm_model_path, j1_max_from_vertical_rad, true);
      // Keep the default start inside tightened shoulder limits, with a small margin.
      const double margin = std::min(0.02, (limits.upper - limits.lower) * 0.1);
      start_joints[1] = std::clamp(start_joints[1], limits.lower + margin, limits.upper - margin);
    }
    if (argc == 2 && std::string(argv[1]) == "--print-arm-config") {
      std::cout << "{\"model_family\":" << json_string(arm_family)
        << ",\"model_path\":" << json_string(arm_model_path.string())
        << ",\"attach_link\":" << json_string(arm.attach_link)
        << ",\"start_joints_rad\":" << joints_json(start_joints) << ",\"planning_joints\":[";
      for (std::size_t index = 0; index < arm.planning_joints.size(); ++index) {
        if (index) {std::cout << ',';}
        std::cout << json_string(arm.planning_joints[index]);
      }
      std::cout << "]}\n";
      return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--print-arm-model-path") {
      std::cout << arm_model_path.string() << '\n';
      return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--check-j1-limits") {
      const auto limits = j1_above_horizontal_limits(arm_model_path, j1_max_from_vertical_rad, is_rm27);
      std::cout << std::setprecision(17) << "[" << limits.lower << "," << limits.upper << "]\n";
      return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--check-resources") {
      check_resources(is_rm27 ? rm27_heavy_arm_demo::installed_model_directory().parent_path().parent_path() :
        rm26_engineering_arm_demo::installed_model_directory());
      check_resources(arm_model_path.parent_path());
      check_resources(LEVEL3_ASSET_DIR);
      std::cout << "resources ok\n";
      return 0;
    }
    if (argc != 11 && argc != 14 && argc != 15 && argc != 17 && argc != 20 && argc != 21) {
      throw std::invalid_argument("usage: level3_case x y z qw qx qy qz timeout_s output_dir gui(0|1) [assembly_x assembly_y assembly_z roll pitch yaw [parking_x parking_y parking_z [parking_yaw]]] [--arm-family=rm26|rm27] [--arm-model-path=PATH] [--disable-self-collision] [--disable-gripper-pillar-collision]");
    }
    if (!(j1_max_from_vertical_rad > 0.0 && j1_max_from_vertical_rad <= tcp::kPi / 2)) {
      throw std::invalid_argument("j1_max_from_vertical_rad must be in (0, pi/2] rad");
    }
    tcp::PlanRequest request;
    request.scene.collision.check_self_collision = !disable_self_collision;
    request.scene.collision.check_gripper_pillar_collision = !disable_gripper_pillar_collision;
    request.scene.world_from_core.translation() = Eigen::Vector3d(number(argv[1]), number(argv[2]), number(argv[3]));
    Eigen::Quaterniond rotation(number(argv[4]), number(argv[5]), number(argv[6]), number(argv[7]));
    if (std::abs(rotation.norm() - 1.0) > 1e-8) {throw std::invalid_argument("invalid unit quaternion");}
    request.scene.world_from_core.linear() = rotation.normalized().toRotationMatrix();
    request.options.timeout = number(argv[8]);
    const std::filesystem::path output = argv[9];
    const std::string gui = argv[10];
    if (gui != "0" && gui != "1") {throw std::invalid_argument("gui must be 0 or 1");}
    if (argc >= 17) {
      tcp::RigidTransform assembly = tcp::RigidTransform::Identity();
      assembly.translation() = Eigen::Vector3d(number(argv[11]), number(argv[12]), number(argv[13]));
      assembly.linear() = (Eigen::AngleAxisd(number(argv[16]), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(number(argv[15]), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(number(argv[14]), Eigen::Vector3d::UnitX())).toRotationMatrix();
      request.display_world_from_assembly = assembly;
    }
    request.start = start_joints;
    request.task.level = tcp::AssemblyLevel::Full;
    request.task.twist_mode = tcp::TwistMode::RequiredRange;
    request.task.twist_range = {-tcp::kPi / 2, tcp::kPi / 2};
    request.task.position_spacing = 0.005;
    request.task.angular_spacing = 0.05;
    request.options.timing.limits.velocity = tcp::JointVector::Constant(7, 0.5);
    request.options.timing.limits.acceleration = tcp::JointVector::Constant(7, 1.0);
    tcp::PlannerConfig config;
    config.arm = arm;
    config.arm.default_fixed_joints.clear();
    const std::string elevation_joint = is_rm27 ? "joint_2" : "j1";
    config.arm.joint_limits[elevation_joint] = j1_above_horizontal_limits(
      config.arm.model_path, j1_max_from_vertical_rad, is_rm27);
    if (argc == 14 || argc == 15 || argc == 20 || argc == 21) {
      const int start = argc <= 15 ? 11 : 17;
      config.arm.parking_offset_world = Eigen::Vector3d(
        number(argv[start]), number(argv[start + 1]), number(argv[start + 2]));
    }
    if (argc == 15 || argc == 21) {config.arm.parking_yaw_world = number(argv[argc - 1]);}
    config.headless = gui == "0";
    config.interactive = gui == "1";
    if (visibility) {
      if (const char * saved_start = std::getenv("TECH_CORE_CAMERA_START")) {
        std::istringstream values(saved_start);
        for (Eigen::Index index = 0; index < request.start.size(); ++index) {
          if (!(values >> request.start[index]) || !std::isfinite(request.start[index])) {
            throw std::runtime_error("invalid saved initial joints");
          }
        }
      }
      tcp::PlanningScene scene(config.arm);
      const auto configured = scene.configure(request.scene);
      if (configured.code != tcp::StatusCode::Ok) {throw std::runtime_error(configured.message);}
      const auto snapshot = tcp::internal::SceneSnapshotAccess::model(scene);
      const auto frame = tcp::internal::SceneSnapshotAccess::request(scene, request);
      tcp::internal::evaluate_visibility(snapshot, frame, config.arm.planning_joints,
        visibility_track, output, gui == "1");
      return 0;
    }
    tcp::Planner planner(config);
    const auto j1_limits = config.arm.joint_limits.find(elevation_joint);
    const auto solve = [&](const std::filesystem::path & output, std::size_t revision) {
      std::filesystem::create_directories(output);
      const Eigen::Quaterniond core_rotation(request.scene.world_from_core.linear());
      std::ostringstream pose;
      pose << std::setprecision(17) << "{\"position_m\":[" << request.scene.world_from_core.translation().x()
        << ',' << request.scene.world_from_core.translation().y() << ',' << request.scene.world_from_core.translation().z()
        << "],\"quaternion_wxyz\":[" << core_rotation.w() << ',' << core_rotation.x()
        << ',' << core_rotation.y() << ',' << core_rotation.z() << "]}";
      std::ofstream(output / "core_pose.json") << pose.str() << '\n';
      if (revision != 0) {
        const auto original = std::filesystem::path(argv[9]);
        for (const auto & name : {"config.json", "case.json"}) {
          if (std::filesystem::exists(original / name)) {
            std::filesystem::copy_file(original / name,
              output / (std::string(name) == "case.json" ? "source_case.json" : name));
          }
        }
        std::ofstream saved_request(output / "request.json");
        saved_request << "{\"core_in_base\":" << pose.str() << ",\"command\":[";
        const double components[] = {request.scene.world_from_core.translation().x(),
          request.scene.world_from_core.translation().y(), request.scene.world_from_core.translation().z(),
          core_rotation.w(), core_rotation.x(), core_rotation.y(), core_rotation.z()};
        for (int argument = 0; argument < argc; ++argument) {
          if (argument != 0) {saved_request << ',';}
          std::ostringstream value;
          value << std::setprecision(17);
          if (argument >= 1 && argument <= 7) {value << components[argument - 1];}
          else if (argument == 9) {value << output.string();}
          else if (argument == 10) {value << '0';}
          else {value << argv[argument];}
          saved_request << json_string(value.str());
        }
        if (disable_self_collision) {saved_request << ",\"--disable-self-collision\"";}
        if (disable_gripper_pillar_collision) {saved_request << ",\"--disable-gripper-pillar-collision\"";}
        saved_request << ',' << json_string("--arm-family=" + arm_family)
          << ',' << json_string("--arm-model-path=" + config.arm.model_path.string())
          << ',' << json_string("--arm-attach-link=" + config.arm.attach_link);
        saved_request << "]}\n";
      }
      std::ostringstream report;
      const auto result = planner.plan(request);
      const bool success = result.status.code == tcp::StatusCode::Ok && result.twist &&
        result.completion.required_range_covered && result.validation.rule_success;
      write_trajectory(output / (success ? "prefix.csv" : "candidate.csv"), result.trajectory);
      if (success) {
        for (const auto & endpoint : {std::pair<const char *, double>{"negative.csv", -tcp::kPi / 2},
          {"positive.csv", tcp::kPi / 2}})
        {
          const auto trajectory = result.twist->full(endpoint.second);
          if (trajectory.status.code != tcp::StatusCode::Ok) {throw std::runtime_error("validated Q endpoint unavailable");}
          write_trajectory(output / endpoint.first, trajectory.trajectory);
        }
      }
      report << std::setprecision(17) << "{\"success\":" << (success ? "true" : "false")
        << ",\"arm_model_family\":" << json_string(arm_family)
        << ",\"start_joints_rad\":" << joints_json(request.start)
        << ",\"elevation_joint\":" << json_string(elevation_joint)
        << ",\"j1_joint_limits_rad\":";
      if (j1_limits != config.arm.joint_limits.end()) {
        report << '[' << j1_limits->second.lower << ',' << j1_limits->second.upper << ']';
      } else {report << "null";}
      report << ",\"check_self_collision\":" << (request.scene.collision.check_self_collision ? "true" : "false")
        << ",\"check_gripper_pillar_collision\":" << (request.scene.collision.check_gripper_pillar_collision ? "true" : "false")
        << ",\"status\":" << json_string(status_name(result.status.code))
        << ",\"message\":" << json_string(result.status.message)
        << ",\"last_failure\":" << json_string(status_name(result.last_failure.code))
        << ",\"last_failure_message\":" << json_string(result.last_failure.message)
        << ",\"failure_stage\":" << json_string(result.failure_stage)
        << ",\"candidate_last_phase\":" << json_string(result.trajectory.knots.empty() ? "unknown" : phase_name(result.trajectory.knots.back().task.phase))
        << ",\"planning_s\":" << result.metrics.total << ",\"posture_s\":" << result.metrics.posture_search
        << ",\"approach_s\":" << result.metrics.approach << ",\"tracking_s\":" << result.metrics.tracking
        << ",\"twist_s\":" << result.metrics.twist << ",\"validation_s\":" << result.metrics.validation
        << ",\"attempts\":" << result.metrics.attempts << ",\"ik_calls\":" << result.metrics.ik_calls
        << ",\"validation_status\":" << json_string(status_name(result.validation.status.code))
        << ",\"required_range_covered\":" << (result.completion.required_range_covered ? "true" : "false")
        << ",\"q_coverage_rad\":";
      if (result.completion.achieved_twist) {
        report << '[' << result.completion.achieved_twist->lower << ',' << result.completion.achieved_twist->upper << ']';
      } else {report << "null";}
      report << ",\"q_diagnostic\":";
      if (result.twist_diagnostic) {
        const auto print_side = [&](const tcp::TwistBranchDiagnostic & side) {
            report << "{\"reached_rad\":" << side.reached_rad << ",\"failed_target_rad\":";
            if (side.failed_target_rad) {report << *side.failed_target_rad;} else {report << "null";}
            report << ",\"failure_status\":" << json_string(status_name(side.failure.code))
                      << ",\"failure_message\":" << json_string(side.failure.message) << '}';
          };
        report << "{\"negative\":";
        print_side(result.twist_diagnostic->negative);
        report << ",\"positive\":";
        print_side(result.twist_diagnostic->positive);
        report << '}';
      } else {report << "null";}
      report << ",\"arm_attach_link\":" << std::quoted(config.arm.attach_link);
      report << ",\"arm_model_path\":" << std::quoted(config.arm.model_path.string());
      report << ",\"revision\":" << revision << ",\"core_in_base\":" << pose.str()
        << ",\"output_directory\":" << json_string(output.string()) << "}\n";
      std::ofstream(output / "result.json") << report.str();
      std::cout << report.str() << std::flush;
      const auto collision_reason = [](const std::string & message) {
          return message.find("Collision") != std::string::npos || message.find("collision") != std::string::npos;
        };
      if (gui == "1" && !success &&
        (collision_reason(result.status.message) || collision_reason(result.last_failure.message)))
      {
        const auto diagnostic = planner.diagnose_collisions(request);
        write_trajectory(output / "collision_diagnostic.csv", diagnostic.trajectory);
        std::ostringstream details;
        details << std::setprecision(17) << "{\"diagnostic_only\":true,\"accepted\":false,\"solution_found\":"
          << (diagnostic.diagnostic_solution_found ? "true" : "false")
          << ",\"original_failure\":" << json_string(result.status.message)
          << ",\"message\":" << json_string(diagnostic.status.message) << ",\"nodes\":[";
        for (std::size_t i = 0; i < diagnostic.diagnostic_collisions.size(); ++i) {
          if (i) {details << ',';}
          const auto & knot = diagnostic.trajectory.knots[i];
          details << "{\"node\":" << i << ",\"phase\":" << json_string(phase_name(knot.task.phase))
            << ",\"violations\":[";
          const auto & violations = diagnostic.diagnostic_collisions[i].violations;
          for (std::size_t j = 0; j < violations.size(); ++j) {
            if (j) {details << ',';}
            const auto & pair = violations[j];
            details << "{\"first_geom\":" << json_string(pair.first_geom)
              << ",\"second_geom\":" << json_string(pair.second_geom)
              << ",\"distance_m\":" << pair.distance << ",\"required_distance_m\":" << pair.required_distance << '}';
          }
          details << "]}";
        }
        details << "]}";
        std::ofstream(output / "collision_diagnostic.json") << details.str() << '\n';
        auto enriched = report.str();
        enriched.erase(enriched.find_last_of('}'));
        enriched += ",\"collision_diagnostic\":" + details.str() + "}\n";
        std::ofstream(output / "collision_diagnostic_result.json") << enriched;
        std::cout << enriched << std::flush;
      }
    };
    solve(output, 0);
    std::size_t revision = 0;
    while (gui == "1" && planner.viewer_running()) {
      if (const auto core = planner.take_viewer_core_pose()) {
        request.scene.world_from_core = *core;
        const auto edited_output = output / "interactive" / ("edit_" + std::to_string(++revision));
        solve(edited_output, revision);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "level3_case: " << error.what() << '\n';
    return 2;
  }
}
