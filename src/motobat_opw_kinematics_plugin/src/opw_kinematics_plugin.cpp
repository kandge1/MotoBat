// See the header for what this plugin does and why it exists.

#include <motobat_opw_kinematics_plugin/opw_kinematics_plugin.hpp>

#include <algorithm>
#include <cmath>

#include <class_loader/class_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <opw_kinematics/opw_utilities.h>
#include <random_numbers/random_numbers.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace motobat_opw_kinematics_plugin
{
namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("motobat_opw_kinematics_plugin");
}

constexpr double TWO_PI = 2.0 * M_PI;
constexpr double CHECK_POS_TOL = 1e-6;  // m
constexpr double CHECK_ROT_TOL = 1e-6;  // rad
constexpr int CHECK_SAMPLES = 200;

template <typename T>
bool getParam(const rclcpp::Node::SharedPtr& node, const std::string& name, T& value)
{
  if (!node->has_parameter(name))
  {
    try
    {
      // No default: the value must come from an override (kinematics.yaml). dynamic_typing
      // allows declaring without a type; the override then sets it.
      rcl_interfaces::msg::ParameterDescriptor d;
      d.dynamic_typing = true;
      node->declare_parameter(name, rclcpp::ParameterValue{}, d);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
    {
    }
  }
  rclcpp::Parameter p;
  if (!node->get_parameter(name, p) || p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    RCLCPP_ERROR(logger(), "missing parameter '%s'", name.c_str());
    return false;
  }
  try
  {
    value = p.get_value<T>();
  }
  catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
  {
    RCLCPP_ERROR(logger(), "parameter '%s' has the wrong type (%s): %s", name.c_str(), p.get_type_name().c_str(),
                 e.what());
    return false;
  }
  return true;
}

double rotationAngle(const Eigen::Matrix3d& R)
{
  // atan2 form: acos((tr - 1) / 2) cannot resolve angles below ~1e-8 rad.
  const Eigen::Vector3d w(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  return std::atan2(0.5 * w.norm(), 0.5 * (R.trace() - 1.0));
}
}  // namespace

bool OPWKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                                     const std::string& group_name, const std::string& base_frame,
                                     const std::vector<std::string>& tip_frames, double search_discretization)
{
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
  jmg_ = robot_model_->getJointModelGroup(group_name);
  if (!jmg_)
  {
    RCLCPP_ERROR(logger(), "no joint model group '%s'", group_name.c_str());
    return false;
  }
  if (tip_frames_.size() != 1)
  {
    RCLCPP_ERROR(logger(), "group '%s': OPW solves one tip, got %zu", group_name.c_str(), tip_frames_.size());
    return false;
  }
  const auto& joints = jmg_->getActiveJointModels();
  if (!jmg_->isChain() || joints.size() != 6 || jmg_->getVariableCount() != 6)
  {
    RCLCPP_ERROR(logger(), "group '%s' must be a chain of 6 single-variable joints", group_name.c_str());
    return false;
  }
  joint_names_.clear();
  lower_.clear();
  upper_.clear();
  for (const auto* j : joints)
  {
    if (j->getType() != moveit::core::JointModel::REVOLUTE)
    {
      RCLCPP_ERROR(logger(), "joint '%s' is not revolute", j->getName().c_str());
      return false;
    }
    joint_names_.push_back(j->getName());
    const auto& b = j->getVariableBounds()[0];
    lower_.push_back(b.position_bounded_ ? b.min_position_ : -std::numeric_limits<double>::infinity());
    upper_.push_back(b.position_bounded_ ? b.max_position_ : std::numeric_limits<double>::infinity());
  }
  link_names_ = { tip_frames_[0] };
  opw_base_link_ = joints[0]->getParentLinkModel()->getName();

  if (!loadParameters(node))
    return false;

  // Base side: measured directly from the robot model (fixed joints only; selfCheck() proves it).
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.updateLinkTransforms();
  base_to_opw_ = state.getGlobalLinkTransform(base_frame_).inverse() * state.getGlobalLinkTransform(opw_base_link_);

  // Tool side: fitted at q = 0, then checked at random states.
  std::vector<double> zero(6, 0.0);
  state.setJointGroupPositions(jmg_, zero);
  state.updateLinkTransforms();
  const Eigen::Isometry3d tip_in_opw =
      state.getGlobalLinkTransform(opw_base_link_).inverse() * state.getGlobalLinkTransform(tip_frames_[0]);
  opw_tool_to_tip_ = opwForward(zero).inverse() * tip_in_opw;

  if (!selfCheck())
    return false;
  RCLCPP_INFO(logger(),
              "group '%s': OPW a1 %.4f a2 %.4f b %.4f c1 %.4f c2 %.4f c3 %.4f c4 %.4f; base '%s', OPW base '%s', "
              "tip '%s'; self-check over %d states: %.1e m, %.1e rad",
              group_name.c_str(), params_.a1, params_.a2, params_.b, params_.c1, params_.c2, params_.c3, params_.c4,
              base_frame_.c_str(), opw_base_link_.c_str(), tip_frames_[0].c_str(), CHECK_SAMPLES, check_dp_,
              check_dr_);
  return true;
}

bool OPWKinematicsPlugin::loadParameters(const rclcpp::Node::SharedPtr& node)
{
  const std::string prefix = "robot_description_kinematics." + group_name_ + ".";
  const std::string geo = prefix + "opw_kinematics_geometric_parameters.";
  bool ok = true;
  ok &= getParam(node, geo + "a1", params_.a1);
  ok &= getParam(node, geo + "a2", params_.a2);
  ok &= getParam(node, geo + "b", params_.b);
  ok &= getParam(node, geo + "c1", params_.c1);
  ok &= getParam(node, geo + "c2", params_.c2);
  ok &= getParam(node, geo + "c3", params_.c3);
  ok &= getParam(node, geo + "c4", params_.c4);
  std::vector<double> offsets;
  std::vector<int64_t> signs;
  ok &= getParam(node, prefix + "opw_kinematics_joint_offsets", offsets);
  ok &= getParam(node, prefix + "opw_kinematics_joint_sign_corrections", signs);
  if (!ok)
    return false;
  if (offsets.size() != 6 || signs.size() != 6)
  {
    RCLCPP_ERROR(logger(), "joint offsets and sign corrections need 6 entries each (got %zu, %zu)", offsets.size(),
                 signs.size());
    return false;
  }
  for (size_t i = 0; i < 6; ++i)
  {
    if (signs[i] != 1 && signs[i] != -1)
    {
      RCLCPP_ERROR(logger(), "sign correction %zu is %ld, must be +1 or -1", i, signs[i]);
      return false;
    }
    params_.offsets[i] = offsets[i];
    params_.sign_corrections[i] = static_cast<signed char>(signs[i]);
  }
  return true;
}

Eigen::Isometry3d OPWKinematicsPlugin::opwForward(const std::vector<double>& q) const
{
  std::array<double, 6> qa;
  std::copy_n(q.begin(), 6, qa.begin());
  return opw_kinematics::forward(params_, qa);
}

bool OPWKinematicsPlugin::selfCheck()
{
  // Random states inside the limits (seeded: the check is the same on every start).
  random_numbers::RandomNumberGenerator rng(42);
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  check_dp_ = check_dr_ = 0.0;
  for (int n = 0; n < CHECK_SAMPLES; ++n)
  {
    state.setToRandomPositions(jmg_, rng);  // only this group moves
    state.updateLinkTransforms();
    std::vector<double> q;
    state.copyJointGroupPositions(jmg_, q);
    const Eigen::Isometry3d model = state.getGlobalLinkTransform(base_frame_).inverse() *
                                    state.getGlobalLinkTransform(tip_frames_[0]);
    const Eigen::Isometry3d opw = base_to_opw_ * opwForward(q) * opw_tool_to_tip_;
    check_dp_ = std::max(check_dp_, (model.translation() - opw.translation()).norm());
    check_dr_ = std::max(check_dr_, rotationAngle(model.linear().transpose() * opw.linear()));
  }
  if (check_dp_ > CHECK_POS_TOL || check_dr_ > CHECK_ROT_TOL)
  {
    RCLCPP_ERROR(logger(),
                 "group '%s': OPW parameters disagree with the robot model by up to %.3g m / %.3g rad over %d "
                 "random states (limit %.0e / %.0e). Wrong parameters or signs? Re-derive with "
                 "motobat_description/scripts/derive_opw.py. Refusing to load.",
                 group_name_.c_str(), check_dp_, check_dr_, CHECK_SAMPLES, CHECK_POS_TOL, CHECK_ROT_TOL);
    return false;
  }
  return true;
}

std::vector<std::vector<double>> OPWKinematicsPlugin::solveAll(const Eigen::Isometry3d& tip_in_base,
                                                               const std::vector<double>& seed) const
{
  const Eigen::Isometry3d tool_in_opw = base_to_opw_.inverse() * tip_in_base * opw_tool_to_tip_.inverse();
  const auto branches = opw_kinematics::inverse(params_, tool_in_opw);

  std::vector<std::vector<double>> out;
  for (const auto& br : branches)
  {
    if (!opw_kinematics::isValid(br))
      continue;
    std::vector<double> q(6);
    bool ok = true;
    for (size_t i = 0; i < 6 && ok; ++i)
    {
      // nearest 2*pi-equivalent of br[i] to the seed, then pushed inside the limits if one exists
      const double s = seed.size() == 6 ? seed[i] : 0.0;
      double v = br[i] + TWO_PI * std::round((s - br[i]) / TWO_PI);
      if (v > upper_[i])
        v -= TWO_PI * std::ceil((v - upper_[i]) / TWO_PI);
      if (v < lower_[i])
        v += TWO_PI * std::ceil((lower_[i] - v) / TWO_PI);
      ok = v >= lower_[i] - 1e-12 && v <= upper_[i] + 1e-12;
      q[i] = std::clamp(v, lower_[i], upper_[i]);
    }
    if (ok)
      out.push_back(std::move(q));
  }
  auto dist = [&seed](const std::vector<double>& q) {
    double d = 0.0;
    for (size_t i = 0; i < 6; ++i)
      d += std::pow(q[i] - (seed.size() == 6 ? seed[i] : 0.0), 2);
    return d;
  };
  std::sort(out.begin(), out.end(), [&](const auto& a, const auto& b) { return dist(a) < dist(b); });
  return out;
}

bool OPWKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                        const std::vector<double>& ik_seed_state, std::vector<double>& solution,
                                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                                        const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, std::vector<double>(), solution, IKCallbackFn(),
                          error_code, options);
}

bool OPWKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::msg::Pose>& ik_poses,
                                        const std::vector<double>& ik_seed_state,
                                        std::vector<std::vector<double>>& solutions,
                                        kinematics::KinematicsResult& result,
                                        const kinematics::KinematicsQueryOptions& /*options*/) const
{
  if (ik_poses.empty())
  {
    result.kinematic_error = kinematics::KinematicErrors::EMPTY_TIP_POSES;
    return false;
  }
  if (ik_poses.size() > 1)
  {
    result.kinematic_error = kinematics::KinematicErrors::MULTIPLE_TIPS_NOT_SUPPORTED;
    return false;
  }
  Eigen::Isometry3d pose;
  tf2::fromMsg(ik_poses[0], pose);
  solutions = solveAll(pose, ik_seed_state);
  result.solution_percentage = solutions.empty() ? 0.0 : 1.0;
  result.kinematic_error = solutions.empty() ? kinematics::KinematicErrors::NO_SOLUTION : kinematics::KinematicErrors::OK;
  return !solutions.empty();
}

bool OPWKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, std::vector<double>(), solution, IKCallbackFn(),
                          error_code, options);
}

bool OPWKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           const std::vector<double>& consistency_limits,
                                           std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool OPWKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, std::vector<double>(), solution, solution_callback,
                          error_code, options);
}

bool OPWKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double /*timeout*/,
                                           const std::vector<double>& consistency_limits,
                                           std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& /*options*/) const
{
  Eigen::Isometry3d pose;
  tf2::fromMsg(ik_pose, pose);
  const auto candidates = solveAll(pose, ik_seed_state);
  bool any_consistent = false;
  for (const auto& q : candidates)
  {
    if (consistency_limits.size() == 6 && ik_seed_state.size() == 6)
    {
      bool inside = true;
      for (size_t i = 0; i < 6 && inside; ++i)
        inside = std::fabs(q[i] - ik_seed_state[i]) <= consistency_limits[i];
      if (!inside)
        continue;
    }
    any_consistent = true;
    if (solution_callback)
    {
      solution_callback(ik_pose, q, error_code);
      if (error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        continue;  // e.g. in collision: try the next branch
    }
    solution = q;
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
  }
  // Every branch was out of reach, out of limits or outside the consistency limits,
  // or each one failed the callback (the callback has already set its own code then).
  if (candidates.empty() || !any_consistent || !solution_callback)
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  return false;
}

bool OPWKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                        const std::vector<double>& joint_angles,
                                        std::vector<geometry_msgs::msg::Pose>& poses) const
{
  if (joint_angles.size() != 6)
    return false;
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(jmg_, joint_angles);
  state.updateLinkTransforms();
  const Eigen::Isometry3d base_inv = state.getGlobalLinkTransform(base_frame_).inverse();
  poses.clear();
  for (const auto& name : link_names)
  {
    if (!robot_model_->hasLinkModel(name))
    {
      RCLCPP_ERROR(logger(), "getPositionFK: unknown link '%s'", name.c_str());
      return false;
    }
    poses.push_back(tf2::toMsg(base_inv * state.getGlobalLinkTransform(name)));
  }
  return true;
}

}  // namespace motobat_opw_kinematics_plugin

CLASS_LOADER_REGISTER_CLASS(motobat_opw_kinematics_plugin::OPWKinematicsPlugin, kinematics::KinematicsBase)
