/** @file arm_dimensions_fk.cpp
 * @brief RM27 模型正运动学流接口；XY 以 J1 轴心为原点，Z 以 base 高度为原点。
 */
#include <rm27_heavy_arm_demo/rm27_heavy_arm_demo.hpp>

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

constexpr std::array<const char *, 7> kJointNames{
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
constexpr std::array<const char *, 7> kBodyNames{
  "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "link_7"};

struct ArmModel
{
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model{nullptr, mj_deleteModel};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data{nullptr, mj_deleteData};
  std::array<int, kJointNames.size()> joint_ids{};
  std::array<int, kJointNames.size()> qpos_addresses{};
  std::array<int, kBodyNames.size()> body_ids{};
  int base_body_id = -1;
  int tool_site_id = -1;

  explicit ArmModel(const std::string & path)
  {
    char error[1024] = {};
    model.reset(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)));
    if (!model) {throw std::runtime_error(error);}
    data.reset(mj_makeData(model.get()));
    if (!data) {throw std::runtime_error("cannot allocate MuJoCo data");}
    base_body_id = mj_name2id(model.get(), mjOBJ_BODY, "base_link");
    if (base_body_id < 0) {throw std::runtime_error("model is missing base_link");}
    tool_site_id = mj_name2id(model.get(), mjOBJ_SITE, "tool0");
    if (tool_site_id < 0) {throw std::runtime_error("model is missing tool0 site");}
    for (std::size_t index = 0; index < kJointNames.size(); ++index) {
      joint_ids[index] = mj_name2id(model.get(), mjOBJ_JOINT, kJointNames[index]);
      if (joint_ids[index] < 0) {throw std::runtime_error(std::string("model is missing joint ") + kJointNames[index]);}
      qpos_addresses[index] = model->jnt_qposadr[joint_ids[index]];
      body_ids[index] = mj_name2id(model.get(), mjOBJ_BODY, kBodyNames[index]);
      if (body_ids[index] < 0) {throw std::runtime_error(std::string("model is missing body ") + kBodyNames[index]);}
    }
  }

  void print_limits()
  {
    std::cout << std::setprecision(17);
    for (std::size_t index = 0; index < kJointNames.size(); ++index) {
      const int joint = joint_ids[index];
      const double model_lower = model->jnt_limited[joint] ? model->jnt_range[2 * joint] : -6.283185307179586;
      const double model_upper = model->jnt_limited[joint] ? model->jnt_range[2 * joint + 1] : 6.283185307179586;
      std::cout << kJointNames[index] << ',' << model_lower << ',' << model_upper << ','
                << model->qpos0[qpos_addresses[index]] << '\n';
    }
  }

  void print_positions(const std::array<double, kJointNames.size()> & joints)
  {
    std::copy(model->qpos0, model->qpos0 + model->nq, data->qpos);
    for (std::size_t index = 0; index < joints.size(); ++index) {
      data->qpos[qpos_addresses[index]] = joints[index];
    }
    mj_kinematics(model.get(), data.get());

    const double * base_position = data->xpos + 3 * base_body_id;
    const double * base_rotation = data->xmat + 9 * base_body_id;
    const double * j1_position = data->xpos + 3 * body_ids[1];
    const double reference_origin[3]{j1_position[0], j1_position[1], base_position[2]};
    const double reference_offset[3]{
      j1_position[0] - base_position[0], j1_position[1] - base_position[1], 0.0};
    for (int axis = 0; axis < 3; ++axis) {
      double coordinate = 0.0;
      for (int row = 0; row < 3; ++row) {
        coordinate += base_rotation[3 * row + axis] * reference_offset[row];
      }
      std::cout << std::setprecision(17) << coordinate << ',';
    }
    const auto print_relative = [&](const double * world_position) {
      const double delta[3]{
        world_position[0] - reference_origin[0],
        world_position[1] - reference_origin[1],
        world_position[2] - reference_origin[2]};
      for (int axis = 0; axis < 3; ++axis) {
        double coordinate = 0.0;
        for (int row = 0; row < 3; ++row) {coordinate += base_rotation[3 * row + axis] * delta[row];}
        std::cout << coordinate << ',';
      }
    };

    for (int body_id : body_ids) {print_relative(data->xpos + 3 * body_id);}

    const double * tool_world_position = data->site_xpos + 3 * tool_site_id;
    for (int axis = 0; axis < 3; ++axis) {
      double coordinate = 0.0;
      for (int row = 0; row < 3; ++row) {
        coordinate += base_rotation[3 * row + axis] * (tool_world_position[row] - reference_origin[row]);
      }
      std::cout << coordinate << (axis == 2 ? '\n' : ',');
    }
    std::cout.flush();
  }
};

std::array<double, kJointNames.size()> parse_joints(const std::string & line)
{
  std::array<double, kJointNames.size()> joints{};
  std::size_t start = 0;
  for (std::size_t index = 0; index < joints.size(); ++index) {
    const std::size_t end = line.find(',', start);
    if ((index + 1 < joints.size() && end == std::string::npos) ||
      (index + 1 == joints.size() && end != std::string::npos)) {
      throw std::invalid_argument("expected seven comma-separated joint angles in rad");
    }
    const std::string token = line.substr(start, end == std::string::npos ? end : end - start);
    std::size_t used = 0;
    joints[index] = std::stod(token, &used);
    if (used != token.size() || !std::isfinite(joints[index])) {
      throw std::invalid_argument("joint angles must be finite numbers in rad");
    }
    start = end == std::string::npos ? line.size() : end + 1;
  }
  return joints;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto config = rm27_heavy_arm_demo::make_arm_config();
    if (argc != 1 && !(argc == 2 && (std::string(argv[1]) == "--limits" ||
      std::string(argv[1]) == "--mount-pose"))) {
      throw std::invalid_argument("usage: arm_dimensions_fk [--limits|--mount-pose]");
    }
    if (argc == 2 && std::string(argv[1]) == "--mount-pose") {
      const auto & pose = config.attach_link_from_energy_unit;
      const Eigen::Quaterniond rotation(pose.rotation());
      std::cout << std::setprecision(17) << pose.translation().x() << ',' << pose.translation().y() << ','
                << pose.translation().z() << ',' << rotation.x() << ',' << rotation.y() << ','
                << rotation.z() << ',' << rotation.w() << '\n';
      return 0;
    }
    ArmModel arm(config.model_path.string());
    if (argc == 2) {
      if (std::string(argv[1]) != "--limits") {throw std::invalid_argument("unknown option");}
      arm.print_limits();
      return 0;
    }
    std::string line;
    while (std::getline(std::cin, line)) {
      try {
        arm.print_positions(parse_joints(line));
      } catch (const std::exception & error) {
        std::cout << "ERROR," << error.what() << '\n' << std::flush;
      }
    }
  } catch (const std::exception & error) {
    std::cerr << "arm_dimensions_fk: " << error.what() << '\n';
    return 2;
  }
  return 0;
}
