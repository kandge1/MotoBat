// MoveIt 2 kinematics plugin: closed-form OPW inverse kinematics (tasks 3.1, 3.3, 3.3b).
//
// Wraps Jmeyer's opw_kinematics (third_party/, Apache-2.0) for arms with an
// ortho-parallel basis and a spherical wrist: here the Yaskawa GP4, GP7 and
// GP25. No MoveIt 2 port of the ROS 1 moveit_opw_kinematics_plugin exists
// (checked 26 Nov 2026), so this is a fresh implementation of the same idea.
//
// What it does per IK query:
//   1. move the target from the group's base frame into the OPW base (the
//      parent link of joint 1) and from the tip back to the OPW tool frame;
//   2. solve all 8 branches in closed form (shoulder x elbow x wrist flip);
//   3. per joint, pick the 2*pi-equivalent angle nearest the seed that lies
//      inside the URDF limits (joint 6 spans +-455 deg on these arms, so a
//      branch can have more than one legal angle); drop branches with none;
//   4. sort by distance to the seed and hand them, nearest first, to MoveIt's
//      validity callback (collision checking) until one passes.
// There is no timeout to spend and no random restart: the answer is exact or
// does not exist.
//
// Safety: initialize() compares OPW forward kinematics with MoveIt's robot
// model at random joint states and refuses to load if they disagree by more
// than 1 um / 1 urad. A wrong or mis-signed parameter file cannot reach the
// planner. The two fixed frames around the OPW chain (base frame -> OPW base,
// OPW tool -> tip) are measured from the robot model, not configured; the
// tool side is fitted at q = 0 and then checked, which also absorbs any
// constant tool-frame difference (e.g. the vendor's 7-digit pi in tool0's rpy).
//
// Parameters, under robot_description_kinematics.<group>. (kinematics.yaml):
//   opw_kinematics_geometric_parameters.{a1,a2,b,c1,c2,c3,c4}   double, metres
//   opw_kinematics_joint_offsets            double[6], radians (no deg())
//   opw_kinematics_joint_sign_corrections   integer[6], +1 / -1
// Values come from motobat_description/scripts/derive_opw.py.

#pragma once

#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>

#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <opw_kinematics/opw_kinematics.h>

namespace motobat_opw_kinematics_plugin
{

class OPWKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                  const std::string& group_name, const std::string& base_frame,
                  const std::vector<std::string>& tip_frames, double search_discretization) override;

  bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                     std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                     const kinematics::KinematicsQueryOptions& options =
                         kinematics::KinematicsQueryOptions()) const override;

  // All valid solutions for one pose, nearest the seed first (as IKFast plugins do).
  bool getPositionIK(const std::vector<geometry_msgs::msg::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                     std::vector<std::vector<double>>& solutions, kinematics::KinematicsResult& result,
                     const kinematics::KinematicsQueryOptions& options) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, std::vector<double>& solution, const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution,
                        const IKCallbackFn& solution_callback, moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override;

  const std::vector<std::string>& getJointNames() const override
  {
    return joint_names_;
  }
  const std::vector<std::string>& getLinkNames() const override
  {
    return link_names_;
  }

  // Exposed for the unit test: every in-limit solution for a tip pose given in
  // the base frame, nearest the seed first. Empty if out of reach.
  std::vector<std::vector<double>> solveAll(const Eigen::Isometry3d& tip_in_base,
                                            const std::vector<double>& seed) const;

  // Largest FK disagreement found by the start-up self-check (m, rad).
  double selfCheckPositionError() const
  {
    return check_dp_;
  }
  double selfCheckRotationError() const
  {
    return check_dr_;
  }

private:
  bool loadParameters(const rclcpp::Node::SharedPtr& node);
  bool selfCheck();
  Eigen::Isometry3d opwForward(const std::vector<double>& q) const;

  opw_kinematics::Parameters<double> params_;
  const moveit::core::JointModelGroup* jmg_ = nullptr;
  std::vector<std::string> joint_names_, link_names_;
  std::vector<double> lower_, upper_;
  std::string opw_base_link_;
  Eigen::Isometry3d base_to_opw_ = Eigen::Isometry3d::Identity();      // base frame -> OPW base
  Eigen::Isometry3d opw_tool_to_tip_ = Eigen::Isometry3d::Identity();  // OPW tool -> tip frame
  double check_dp_ = 0.0, check_dr_ = 0.0;
};

}  // namespace motobat_opw_kinematics_plugin
