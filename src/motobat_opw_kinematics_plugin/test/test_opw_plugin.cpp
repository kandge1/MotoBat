// Tests for the OPW kinematics plugin, on the real vendor URDFs (GP4, GP7, GP25).
//
// The plugin is constructed directly (not through pluginlib) on a RobotModel built
// from each URDF plus a one-group SRDF, with the parameters derive_opw.py produces.
//
//   * self-check: OPW FK matches the robot model (< 1e-9 here; the plugin's own
//     limit is 1e-6)
//   * round trip: 2,000 random in-limit states q per arm; IK of FK(q) seeded at q
//     returns q itself first, and every returned branch reproduces the pose
//   * safety: a wrong sign or a wrong length makes initialize() refuse to load
//   * callback: a callback that rejects the nearest branch gets the next one
//   * reach: a pose 3 m away returns NO_IK_SOLUTION
//   * timing: median IK call, printed for the record

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <sstream>

#include <motobat_opw_kinematics_plugin/opw_kinematics_plugin.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <urdf_parser/urdf_parser.h>

using motobat_opw_kinematics_plugin::OPWKinematicsPlugin;

namespace
{
struct Arm
{
  std::string name;
  double a1, a2, b, c1, c2, c3, c4;
};

// From derive_opw.py (GP4 = the vendor file).
const std::vector<Arm> ARMS = {
  { "gp4", 0.000, -0.015, 0.0, 0.330, 0.260, 0.290, 0.072 },
  { "gp7", 0.040, -0.040, 0.0, 0.330, 0.445, 0.440, 0.080 },
  { "gp25", 0.150, -0.200, 0.0, 0.505, 0.760, 0.795, 0.100 },
};
const std::vector<double> OFFSETS = { 0.0, 0.0, -M_PI / 2, 0.0, 0.0, M_PI };
const std::vector<int64_t> SIGNS = { 1, 1, -1, -1, -1, -1 };
const std::string GROUP = "manipulator";

std::string readFile(const std::string& path)
{
  std::ifstream f(path);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

moveit::core::RobotModelPtr loadModel(const std::string& arm)
{
  const std::string urdf_xml = readFile(std::string(TEST_URDF_DIR) + "/" + arm + ".urdf");
  auto urdf = urdf::parseURDF(urdf_xml);
  if (!urdf)
    return nullptr;
  auto srdf = std::make_shared<srdf::Model>();
  const std::string srdf_xml = "<robot name=\"" + urdf->getName() + "\"><group name=\"" + GROUP +
                               "\"><chain base_link=\"base_link\" tip_link=\"tool0\"/></group></robot>";
  if (!srdf->initString(*urdf, srdf_xml))
    return nullptr;
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

rclcpp::Node::SharedPtr makeNode(const Arm& a, std::vector<int64_t> signs = SIGNS)
{
  const std::string p = "robot_description_kinematics." + GROUP + ".";
  const std::string g = p + "opw_kinematics_geometric_parameters.";
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      { g + "a1", a.a1 },
      { g + "a2", a.a2 },
      { g + "b", a.b },
      { g + "c1", a.c1 },
      { g + "c2", a.c2 },
      { g + "c3", a.c3 },
      { g + "c4", a.c4 },
      { p + "opw_kinematics_joint_offsets", OFFSETS },
      { p + "opw_kinematics_joint_sign_corrections", signs },
  });
  static int n = 0;
  return std::make_shared<rclcpp::Node>("opw_test_" + std::to_string(n++), opts);
}

double rotationAngle(const Eigen::Matrix3d& R)
{
  const Eigen::Vector3d w(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  return std::atan2(0.5 * w.norm(), 0.5 * (R.trace() - 1.0));
}
}  // namespace

class OPWPlugin : public ::testing::TestWithParam<Arm>
{
};

TEST_P(OPWPlugin, RoundTrip)
{
  const Arm& a = GetParam();
  auto model = loadModel(a.name);
  ASSERT_TRUE(model) << a.name;
  OPWKinematicsPlugin ik;
  ASSERT_TRUE(ik.initialize(makeNode(a), *model, GROUP, "base_link", { "tool0" }, 0.1));
  EXPECT_LT(ik.selfCheckPositionError(), 1e-9);
  EXPECT_LT(ik.selfCheckRotationError(), 1e-9);

  const auto* jmg = model->getJointModelGroup(GROUP);
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  random_numbers::RandomNumberGenerator rng(7);
  double worst = 0.0, worst_first = 0.0;
  size_t total = 0;
  std::vector<double> times;
  for (int n = 0; n < 2000; ++n)
  {
    state.setToRandomPositions(jmg, rng);
    state.updateLinkTransforms();
    std::vector<double> q;
    state.copyJointGroupPositions(jmg, q);
    const Eigen::Isometry3d pose = state.getGlobalLinkTransform("tool0");  // base_link is the model root

    const auto t0 = std::chrono::steady_clock::now();
    const auto sols = ik.solveAll(pose, q);
    times.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());

    ASSERT_FALSE(sols.empty()) << a.name << " sample " << n;
    total += sols.size();
    for (size_t i = 0; i < 6; ++i)
      worst_first = std::max(worst_first, std::fabs(sols[0][i] - q[i]));
    moveit::core::RobotState check(state);
    for (const auto& s : sols)
    {
      check.setJointGroupPositions(jmg, s);
      check.updateLinkTransforms();
      const Eigen::Isometry3d got = check.getGlobalLinkTransform("tool0");
      worst = std::max({ worst, (got.translation() - pose.translation()).norm(),
                         rotationAngle(got.linear().transpose() * pose.linear()) });
      EXPECT_TRUE(jmg->satisfiesPositionBounds(s.data())) << a.name;
    }
  }
  std::sort(times.begin(), times.end());
  std::printf("[%s] 2000 poses: %.2f in-limit branches per pose, worst pose error %.1e, nearest branch = source "
              "within %.1e rad, median solve %.1f us\n",
              a.name.c_str(), double(total) / 2000.0, worst, worst_first, times[times.size() / 2]);
  EXPECT_LT(worst, 1e-9);
  // The source q is in the solution set; seeded at q, it must come first (distance 0).
  // Near the wrist singularity t4/t6 are ill-conditioned, so allow 1e-6.
  EXPECT_LT(worst_first, 1e-6);
}

TEST_P(OPWPlugin, RefusesWrongParameters)
{
  const Arm& a = GetParam();
  auto model = loadModel(a.name);
  ASSERT_TRUE(model);
  {
    std::vector<int64_t> bad = SIGNS;
    bad[3] = -bad[3];  // joint 4 sign flipped
    OPWKinematicsPlugin ik;
    EXPECT_FALSE(ik.initialize(makeNode(a, bad), *model, GROUP, "base_link", { "tool0" }, 0.1));
  }
  {
    Arm wrong = a;
    wrong.c2 += 0.001;  // upper arm 1 mm long
    OPWKinematicsPlugin ik;
    EXPECT_FALSE(ik.initialize(makeNode(wrong), *model, GROUP, "base_link", { "tool0" }, 0.1));
  }
}

TEST_P(OPWPlugin, CallbackAndReach)
{
  const Arm& a = GetParam();
  auto model = loadModel(a.name);
  ASSERT_TRUE(model);
  OPWKinematicsPlugin ik;
  ASSERT_TRUE(ik.initialize(makeNode(a), *model, GROUP, "base_link", { "tool0" }, 0.1));

  // A comfortable pose: elbow bent, wrist pitched (all 8 branches usually exist).
  const std::vector<double> q0 = { 0.3, 0.2, -0.3, 0.4, -0.8, 0.5 };
  std::vector<geometry_msgs::msg::Pose> pose(1);
  ASSERT_TRUE(ik.getPositionFK({ "tool0" }, q0, pose));

  std::vector<double> sol;
  moveit_msgs::msg::MoveItErrorCodes err;
  ASSERT_TRUE(ik.searchPositionIK(pose[0], q0, 0.05, sol, err));
  for (size_t i = 0; i < 6; ++i)
    EXPECT_NEAR(sol[i], q0[i], 1e-9);

  // Reject whichever branch is offered first; the plugin must move on to the next.
  int calls = 0;
  std::vector<double> rejected;
  auto cb = [&](const geometry_msgs::msg::Pose&, const std::vector<double>& q, moveit_msgs::msg::MoveItErrorCodes& e) {
    if (calls++ == 0)
    {
      rejected = q;
      e.val = moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION;
    }
    else
      e.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  };
  ASSERT_TRUE(ik.searchPositionIK(pose[0], q0, 0.05, sol, cb, err));
  EXPECT_EQ(calls, 2);
  double diff = 0.0;
  for (size_t i = 0; i < 6; ++i)
    diff = std::max(diff, std::fabs(sol[i] - rejected[i]));
  EXPECT_GT(diff, 1e-3) << "the second branch must differ from the rejected one";

  // Every branch rejected: no solution.
  auto reject_all = [](const geometry_msgs::msg::Pose&, const std::vector<double>&,
                       moveit_msgs::msg::MoveItErrorCodes& e) { e.val = moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION; };
  EXPECT_FALSE(ik.searchPositionIK(pose[0], q0, 0.05, sol, reject_all, err));

  // Out of reach.
  geometry_msgs::msg::Pose far = pose[0];
  far.position.x = 3.0;
  EXPECT_FALSE(ik.searchPositionIK(far, q0, 0.05, sol, err));
  EXPECT_EQ(err.val, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);

  // All-solutions query: every branch, nearest first.
  std::vector<std::vector<double>> sols;
  kinematics::KinematicsResult res;
  ASSERT_TRUE(ik.getPositionIK(pose, q0, sols, res, kinematics::KinematicsQueryOptions()));
  EXPECT_EQ(res.kinematic_error, kinematics::KinematicErrors::OK);
  EXPECT_GE(sols.size(), 2u);
}

INSTANTIATE_TEST_SUITE_P(Yaskawa, OPWPlugin, ::testing::ValuesIn(ARMS),
                         [](const ::testing::TestParamInfo<Arm>& i) { return i.param.name; });

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
