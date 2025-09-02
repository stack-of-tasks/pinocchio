//
// Copyright (c) 2024-2025 INRIA
//

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/graph/joints.hpp"
#include "pinocchio/parsers/graph/model-graph-algo.hpp"
#include "pinocchio/parsers/graph/model-configuration-converter.hpp"

#if defined(PINOCCHIO_WITH_HPP_FCL)
  #include "pinocchio/parsers/graph/model-graph-algo-geometry.hpp"
#endif
#include <boost/test/unit_test.hpp>
#include <stdexcept>

pinocchio::graph::ModelGraph buildReversableModelGraph(const pinocchio::graph::JointVariant & joint)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addFrame("body1", BodyFrame(pinocchio::Inertia::Identity()));
  g.addFrame(
    "body2", BodyFrame(pinocchio::Inertia(
               4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero())));

  /////////////////////////////////////// Joints
  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 poseBody2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.));

  g.edgeBuilder()
    .withName("body1_to_body2")
    .withSourceVertex("body1")
    .withSourcePose(poseBody1)
    .withTargetVertex("body2")
    .withTargetPose(poseBody2)
    .withJointType(joint)
    .build();

  return g;
}

/// function isApprox better to avoid problem with zero precision
bool SE3isApprox(
  const pinocchio::SE3 & s1,
  const pinocchio::SE3 & s2,
  const double prec = Eigen::NumTraits<double>::dummy_precision())
{
  return s1.rotation().isApprox(s2.rotation())
         && (s1.translation() - s2.translation()).isZero(prec);
}

BOOST_AUTO_TEST_SUITE(ModelGraphTest)

/// @brief test if vertex are added to the graph
BOOST_AUTO_TEST_CASE(test_add_vertex)
{
  using namespace pinocchio::graph;

  ModelGraph g;

  g.addFrame("body1", BodyFrame(pinocchio::Inertia::Identity()));
  BOOST_CHECK(g.name_to_vertex.find("body1") != g.name_to_vertex.end());
}

/// @brief Test if edges and their reverse are added correctly to the graph and the following edge
/// case:
///  - source body doesn't exists
///  - target body doesn't exists
///  - joint name already exists
///  - only one joint between two body
BOOST_AUTO_TEST_CASE(test_add_joint)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  ////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody("body2", pinocchio::Inertia::Identity());

  /////////////////////////////////////// Joints
  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 poseBody2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.));

  g.edgeBuilder()
    .withName("body1_to_body2")
    .withSourceVertex("body1")
    .withSourcePose(poseBody1)
    .withTargetVertex("body2")
    .withTargetPose(poseBody2)
    .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
    .build();

  auto v_out = g.name_to_vertex["body1"];
  auto v_in = g.name_to_vertex["body2"];
  BOOST_CHECK(boost::edge(v_out, v_in, g.graph).second);
  BOOST_CHECK(boost::edge(v_in, v_out, g.graph).second);

  /////////////////////////////////////// Edge cases
  BOOST_CHECK_THROW(
    g.edgeBuilder()
      .withName("body3_to_body2")
      .withSourceVertex("body3")
      .withSourcePose(poseBody1)
      .withTargetVertex("body2")
      .withTargetPose(poseBody2)
      .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
      .build(),
    std::invalid_argument);
  BOOST_CHECK_THROW(
    g.edgeBuilder()
      .withName("body1_to_body3")
      .withSourceVertex("body1")
      .withSourcePose(poseBody1)
      .withTargetVertex("body3")
      .withTargetPose(poseBody2)
      .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
      .build(),
    std::invalid_argument);
  BOOST_CHECK_THROW(
    g.edgeBuilder()
      .withName("body1_to_body2")
      .withSourceVertex("body1")
      .withSourcePose(poseBody1)
      .withTargetVertex("body2")
      .withTargetPose(poseBody2)
      .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
      .build(),
    std::invalid_argument);
  BOOST_CHECK_THROW(
    g.edgeBuilder()
      .withName("body1_to_body2_bis")
      .withSourceVertex("body1")
      .withSourcePose(poseBody1)
      .withTargetVertex("body2")
      .withTargetPose(poseBody2)
      .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
      .build(),
    std::invalid_argument);
}

/// @brief Test of simple 2R robot to try out kinematics and what happens when we use different body
/// as root body body1 --- body2
BOOST_AUTO_TEST_CASE(test_linear_2D_robot)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointRevolute(Eigen::Vector3d::UnitY()));

  ///////////////// Model
  pinocchio::Model m = buildModel(g, "body1", pinocchio::SE3::Identity());
  BOOST_CHECK(m.jointPlacements[m.getJointId("body1_to_body2")].isApprox(
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2, 0, 0))));

  // Forward kinematics
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m.nq);
  q[0] = M_PI / 2;

  // Compute forward kinematics
  pinocchio::Data d(m);
  pinocchio::framesForwardKinematics(m, d, q);

  pinocchio::Model m1 = buildModel(g, "body2", pinocchio::SE3::Identity());
  // Compute forward kinematics
  pinocchio::Data d1(m1);
  pinocchio::framesForwardKinematics(m1, d1, -q);

  // World to Body1 (Identity)
  pinocchio::SE3 X1 = pinocchio::SE3::Identity();
  // Body1 to Joint1 (translation of 2 along X)
  pinocchio::SE3 X2 = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2.0, 0.0, 0.0));
  // Rotation around Y by q = pi/2
  Eigen::AngleAxisd R_y(q[0], Eigen::Vector3d::UnitY());
  pinocchio::SE3 X3 = pinocchio::SE3(R_y.toRotationMatrix(), Eigen::Vector3d::Zero());
  // Joint1 to Body2 (translation of 2 along Y)
  pinocchio::SE3 X4 = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 3.0, 0.0));
  // Final transformation
  pinocchio::SE3 bodyPose = X1 * X2 * X3 * X4;
  BOOST_CHECK(d.oMf[m.getFrameId("body2", pinocchio::BODY)].isApprox(bodyPose));

  // World to Body2 (Identity)
  pinocchio::SE3 X1_ = pinocchio::SE3::Identity();
  // Body2 to Joint1
  pinocchio::SE3 X2_ = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, -3.0, 0.0));
  // Rotation around Y by q = -pi/2 (reverse joint)
  Eigen::AngleAxisd R_y_(-q[0], Eigen::Vector3d::UnitY());
  pinocchio::SE3 X3_ = pinocchio::SE3(R_y_.toRotationMatrix(), Eigen::Vector3d::Zero());
  // Joint1 to Body1
  pinocchio::SE3 X4_ = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-2.0, 0.0, 0.0));
  // Final transformation
  pinocchio::SE3 bodyPose1 = X1_ * X2_ * X3_ * X4_;
  BOOST_CHECK(d1.oMf[m1.getFrameId("body1", pinocchio::BODY)].isApprox(bodyPose1));
}

/// @brief Test of simple loop inside the model graph
///      b1
///     |  |
///    b2 - b3
BOOST_AUTO_TEST_CASE(test_loop)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody("body2", pinocchio::Inertia::Identity());
  g.addBody("body3", pinocchio::Inertia::Identity());

  /////////////////////////////////////// Joints
  g.edgeBuilder()
    .withName("body1_to_body2")
    .withSourceVertex("body1")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("body2")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
    .build();

  g.edgeBuilder()
    .withName("body1_to_body3")
    .withSourceVertex("body1")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("body3")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
    .build();

  g.edgeBuilder()
    .withName("body2_to_body3")
    .withSourceVertex("body2")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("body3")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
    .build();

  ///////////////// Model
  BOOST_CHECK_THROW(buildModel(g, "body1", pinocchio::SE3::Identity()), std::invalid_argument);
  BOOST_CHECK_THROW(buildModel(g, "body3", pinocchio::SE3::Identity()), std::invalid_argument);
}

/// @brief Test of multi-body loop inside the model graph
///      b1
///     |  |
///    b2  b4
///     |  |
///      b3
BOOST_AUTO_TEST_CASE(test_giant_loop)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody("body2", pinocchio::Inertia::Identity());
  g.addBody("body3", pinocchio::Inertia::Identity());
  g.addBody("body4", pinocchio::Inertia::Identity());

  /////////////////////////////////////// Joints
  g.edgeBuilder()
    .withName("body1_to_body2")
    .withSourceVertex("body1")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("body2")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
    .build();

  g.edgeBuilder()
    .withName("body2_to_body3")
    .withSourceVertex("body2")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("body3")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
    .build();

  g.edgeBuilder()
    .withName("body1_to_body4")
    .withSourceVertex("body1")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("body4")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
    .build();

  g.edgeBuilder()
    .withName("body4_to_body3")
    .withSourceVertex("body4")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("body3")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
    .build();

  ///////////////// Model
  BOOST_CHECK_THROW(buildModel(g, "body1", pinocchio::SE3::Identity()), std::invalid_argument);
}

/// @brief Test out the fixed joint.
BOOST_AUTO_TEST_CASE(test_fixed_joint)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody("body2", pinocchio::Inertia::Identity());
  g.addBody("body3", pinocchio::Inertia::Identity());

  /////////////////////////////////////// Joints
  g.edgeBuilder()
    .withName("body1_to_body2")
    .withSourceVertex("body1")
    .withSourcePose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)))
    .withTargetVertex("body2")
    .withTargetPose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)))
    .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
    .build();

  g.edgeBuilder()
    .withName("body2_to_body3")
    .withSourceVertex("body2")
    .withSourcePose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., -3., 0.)))
    .withTargetVertex("body3")
    .withTargetPose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4., -5., 0.)))
    .withJointType(JointFixed())
    .build();

  ///////////////// Model
  pinocchio::Model m = buildModel(g, "body1", pinocchio::SE3::Identity(), JointFreeFlyer());

  BOOST_CHECK(m.njoints == 3);
  pinocchio::SE3 body3_placement =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.))
    * pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., -3., 0.));
  BOOST_CHECK(m.frames[m.getFrameId("body2_to_body3", pinocchio::FIXED_JOINT)].placement.isApprox(
    body3_placement));
}

/// @brief test construction of model with a mimic
BOOST_AUTO_TEST_CASE(test_mimic_joint)
{
  using namespace pinocchio::graph;
  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addFrame("body2", BodyFrame(pinocchio::Inertia::Identity()));
  g.addBody("body3", pinocchio::Inertia::Identity());

  /////////////////////////////////////// Joints
  pinocchio::SE3 pose_body1_joint1(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 pose_body2_joint1(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 4., 0.));

  g.edgeBuilder()
    .withName("body1_to_body2")
    .withSourceVertex("body1")
    .withSourcePose(pose_body1_joint1)
    .withTargetVertex("body2")
    .withTargetPose(pose_body2_joint1)
    .withJointType(JointRevolute(Eigen::Vector3d::UnitX()))
    .build();

  pinocchio::SE3 pose_body2_joint2(Eigen::Matrix3d::Identity(), Eigen::Vector3d(5., 0., 0.));
  pinocchio::SE3 pose_body3_joint2(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 1.));
  double scaling = 2.0;
  double offset = 0.5;
  g.edgeBuilder()
    .withName("body2_to_body3")
    .withSourceVertex("body2")
    .withSourcePose(pose_body2_joint2)
    .withTargetVertex("body3")
    .withTargetPose(pose_body3_joint2)
    .withJointType(
      JointMimic(JointRevolute(Eigen::Vector3d::UnitY()), "body1_to_body2", scaling, offset))
    .build();

  ///////////////// Model
  pinocchio::SE3 pose_body1_universe = pinocchio::SE3::Identity();
  pinocchio::Model m = buildModel(g, "body1", pose_body1_universe);

  Eigen::VectorXd q(m.nq);
  q << M_PI / 2;

  pinocchio::Data d(m);
  pinocchio::framesForwardKinematics(m, d, q);

  // First revolute around X
  Eigen::AngleAxisd R_x(q[0], Eigen::Vector3d::UnitX());
  pinocchio::SE3 X_joint1 = pinocchio::SE3(R_x.toRotationMatrix(), Eigen::Vector3d::Zero());
  Eigen::AngleAxisd R_y(scaling * q[0] + offset, Eigen::Vector3d::UnitY());
  pinocchio::SE3 X_joint2 = pinocchio::SE3(R_y.toRotationMatrix(), Eigen::Vector3d::Zero());
  // Final transformation
  pinocchio::SE3 bodyPose = pose_body1_universe * pose_body1_joint1 * X_joint1 * pose_body2_joint1
                            * pose_body2_joint2 * X_joint2 * pose_body3_joint2;

  BOOST_CHECK(d.oMf[m.getFrameId("body3", pinocchio::BODY)].isApprox(bodyPose));
}

/// @brief test out reverse joint for revolute
BOOST_AUTO_TEST_CASE(test_reverse_revolute)
{
  using namespace pinocchio::graph;
  ModelGraph g = buildReversableModelGraph(JointRevolute(Eigen::Vector3d::UnitY()));

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q[0] = M_PI / 2;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, -q);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief Test for a reverse revolute unbounded
BOOST_AUTO_TEST_CASE(test_reverse_revolute_unbounded)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointRevoluteUnbounded(Eigen::Vector3d::UnitY()));

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  double ca, sa;
  pinocchio::SINCOS(M_PI / 4, &ca, &sa);
  q[0] = ca;
  q[1] = sa;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  Eigen::VectorXd q_reverse = Eigen::VectorXd::Zero(m_reverse.nq);
  q_reverse[0] = q[0];  // cos(-a) = cos(a)
  q_reverse[1] = -q[1]; // sin(-a) = -sin(a)
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reverse);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test out reverse joint for prismatic
BOOST_AUTO_TEST_CASE(test_reverse_prismatic)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointPrismatic(Eigen::Vector3d::UnitY()));

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q[0] = 0.2;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, -q);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test out reverse joint for helical
BOOST_AUTO_TEST_CASE(test_reverse_helical)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointHelical(Eigen::Vector3d::UnitY(), 2.3));

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q[0] = M_PI / 3;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, -q);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test reversing helical joint on a simple linear robot
BOOST_AUTO_TEST_CASE(test_reverse_universal)
{
  using namespace pinocchio::graph;

  ModelGraph g =
    buildReversableModelGraph(JointUniversal(Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitX()));

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q[0] = M_PI / 2;
  q[1] = M_PI / 4;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  Eigen::VectorXd q_reverse = Eigen::VectorXd::Zero(m_forward.nq);
  q_reverse[0] = q[1];
  q_reverse[1] = q[0];

  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reverse);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief compare reverse model with spherical
BOOST_AUTO_TEST_CASE(test_reverse_spherical)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointSpherical());

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  Eigen::AngleAxisd rollAngle(1, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yawAngle(0.4, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(1, Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q_sph = rollAngle * yawAngle * pitchAngle;

  Eigen::VectorXd q(m_forward.nq);
  q << q_sph.x(), q_sph.y(), q_sph.z(), q_sph.w();

  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  Eigen::VectorXd q_reverse(m_reverse.nq);
  q_reverse << q_sph.inverse().x(), q_sph.inverse().y(), q_sph.inverse().z(), q_sph.inverse().w();

  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reverse);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test if reversing of a composite joint is correct.
BOOST_AUTO_TEST_CASE(test_reverse_spherical_zyx)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointSphericalZYX());

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  // config vector forward model ZYX
  Eigen::Vector3d q(m_forward.nq);
  q << M_PI / 4, M_PI, M_PI / 2;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  // rotation matrix for spherique xyz for inverting spherical zyx
  Eigen::AngleAxisd Rx(-q[2], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd Ry(-q[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rz(-q[0], Eigen::Vector3d::UnitZ());
  // Eigen convention is right multiply
  Eigen::Matrix3d R = Rx.toRotationMatrix() * Ry.toRotationMatrix() * Rz.toRotationMatrix();

  Eigen::Vector3d q_reverse = R.eulerAngles(2, 1, 0);

  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reverse);
  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test if reversing of a composite joint is correct.
BOOST_AUTO_TEST_CASE(test_reverse_composite)
{
  using namespace pinocchio::graph;

  JointComposite jmodel;
  pinocchio::SE3 jPose1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.)); // from body to j1
  pinocchio::SE3 jPose2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)); // from j1 to j2
  pinocchio::SE3 jPose3 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 1.)); // from j2 to j3
  jmodel.addJoint(JointRevolute(Eigen::Vector3d::UnitX()), jPose1);
  jmodel.addJoint(JointRevolute(Eigen::Vector3d::UnitZ()), jPose2);
  jmodel.addJoint(JointPrismatic(Eigen::Vector3d::UnitY()), jPose3);

  ModelGraph g = buildReversableModelGraph(jmodel);

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q << M_PI / 2, M_PI / 2, 0.4;

  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);

  Eigen::VectorXd q_reverse(m_reverse.nq);
  q_reverse << -q[2], -q[1], -q[0];
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reverse);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test if reversing of a composite joint is correct.
BOOST_AUTO_TEST_CASE(test_reverse_planar)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointPlanar());

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  double ca, sa;
  pinocchio::SINCOS(M_PI / 3, &ca, &sa);
  q << 2, 1, ca, sa;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);
  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);

  // Compute reverse coordinate
  Eigen::Vector3d trans;
  trans << q[0], q[1], 0;
  Eigen::VectorXd q_reverse = Eigen::VectorXd::Zero(m_reverse.nq);
  Eigen::Matrix3d R;
  R << ca, sa, 0, -sa, ca, 0, 0, 0, 1;
  Eigen::Vector3d trans_rev = -R * trans;
  q_reverse << trans_rev[0], trans_rev[1], ca, -sa;

  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reverse);
  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test if reversing of a composite joint is correct.
BOOST_AUTO_TEST_CASE(test_reverse_mimic)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addFrame("body2", BodyFrame(pinocchio::Inertia::Identity()));
  g.addBody("body3", pinocchio::Inertia::Identity());

  /////////////////////////////////////// Joints
  pinocchio::SE3 pose_body1_joint1(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 pose_body2_joint1(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 4., 0.));

  g.edgeBuilder()
    .withName("body1_to_body2")
    .withSourceVertex("body1")
    .withSourcePose(pose_body1_joint1)
    .withTargetVertex("body2")
    .withTargetPose(pose_body2_joint1)
    .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
    .build();

  pinocchio::SE3 pose_body2_joint2(Eigen::Matrix3d::Identity(), Eigen::Vector3d(5., 0., 0.));
  pinocchio::SE3 pose_body3_joint2(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 1.));
  double scaling = 2.0;
  double offset = 0.5;
  g.edgeBuilder()
    .withName("body2_to_body3")
    .withSourceVertex("body2")
    .withSourcePose(pose_body2_joint2)
    .withTargetVertex("body3")
    .withTargetPose(pose_body3_joint2)
    .withJointType(
      JointMimic(JointRevolute(Eigen::Vector3d::UnitY()), "body1_to_body2", scaling, offset))
    .build();

  ///////////////// Model
  BOOST_CHECK_THROW(buildModel(g, "body3", pinocchio::SE3::Identity()), std::invalid_argument);
}

/// @brief Test out if inertias are well placed on the model
BOOST_AUTO_TEST_CASE(test_inertia)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  pinocchio::Inertia inert = pinocchio::Inertia(
    1., pinocchio::Inertia::Vector3(0., -2., 1.), pinocchio::Symmetric3::Random());
  g.addBody("body1", inert);
  g.addBody(
    "body2", pinocchio::Inertia(
               4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Random()));
  g.addBody("body3", inert);

  /////////////////////////////////////// Joints
  g.edgeBuilder()
    .withName("body1_to_body2")
    .withSourceVertex("body1")
    .withSourcePose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)))
    .withTargetVertex("body2")
    .withTargetPose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)))
    .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
    .build();

  g.edgeBuilder()
    .withName("body2_to_body3")
    .withSourceVertex("body2")
    .withSourcePose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., -2., 0.)))
    .withTargetVertex("body3")
    .withTargetPose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-2., 0., 0.)))
    .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
    .build();

  pinocchio::Model m = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Model m1 = buildModel(g, "body3", pinocchio::SE3::Identity());

  Eigen::VectorXd q = Eigen::VectorXd::Random(m.nq);

  pinocchio::Data d(m);
  pinocchio::Data d1(m1);

  pinocchio::crba(m, d, q);
  pinocchio::crba(m1, d1, -q);

  BOOST_CHECK(d.M.isApprox(d1.M));
}

/// @brief test if joint limits are parsed correctly (forward and backward)
BOOST_AUTO_TEST_CASE(test_joint_limits)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addFrame("body1", BodyFrame(pinocchio::Inertia::Identity()));
  g.addFrame("body2", BodyFrame(pinocchio::Inertia::Identity()));

  /////////////////////////////////////// Joints
  g.edgeBuilder()
    .withName("body1_to_body2")
    .withSourceVertex("body1")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("body2")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(JointRevolute(Eigen::Vector3d::UnitX()))
    .withMaxConfig(Eigen::VectorXd::Constant(1, M_PI))
    .withMinConfig(Eigen::VectorXd::Constant(1, -M_PI / 4))
    .withMaxVel(Eigen::VectorXd::Constant(1, 0.3))
    .build();

  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);

  pinocchio::framesForwardKinematics(m_forward, d_f, m_forward.lowerPositionLimit);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);

  pinocchio::framesForwardKinematics(m_reverse, d_reverse, m_reverse.upperPositionLimit);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test if joint limits are parsed correctly (forward and backward)
BOOST_AUTO_TEST_CASE(test_joint_limits_universal)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addFrame("body1", BodyFrame(pinocchio::Inertia::Identity()));
  g.addFrame("body2", BodyFrame(pinocchio::Inertia::Identity()));

  Eigen::Vector2d minConfig;
  minConfig << -M_PI / 3, 0;
  Eigen::Vector2d maxConfig;
  maxConfig << M_PI / 2, M_PI;

  /////////////////////////////////////// Joints
  g.edgeBuilder()
    .withName("body1_to_body2")
    .withSourceVertex("body1")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("body2")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(JointUniversal(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()))
    .withMaxConfig(maxConfig)
    .withMinConfig(minConfig)
    .build();

  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);

  pinocchio::framesForwardKinematics(m_forward, d_f, m_forward.lowerPositionLimit);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);

  pinocchio::framesForwardKinematics(m_reverse, d_reverse, m_reverse.lowerPositionLimit);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test if joint limits are parsed correctly (forward and backward)
BOOST_AUTO_TEST_CASE(test_joint_limits_planar)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addFrame("body1", BodyFrame(pinocchio::Inertia::Identity()));
  g.addFrame("body2", BodyFrame(pinocchio::Inertia::Identity()));

  Eigen::VectorXd minConfig(4);
  minConfig << 0.1, 0.2, 1, 0; // no limit for the Z rotation
  Eigen::VectorXd maxConfig(4);
  maxConfig << 0.9, 1.1, 1, 0;

  /////////////////////////////////////// Joints
  g.edgeBuilder()
    .withName("body1_to_body2")
    .withSourceVertex("body1")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("body2")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(JointPlanar())
    .withMaxConfig(maxConfig)
    .withMinConfig(minConfig)
    .build();

  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);

  pinocchio::framesForwardKinematics(m_forward, d_f, m_forward.lowerPositionLimit);
  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);

  pinocchio::framesForwardKinematics(m_reverse, d_reverse, m_reverse.upperPositionLimit);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test joint limits for composite joints (forward and backward)
BOOST_AUTO_TEST_CASE(test_joint_limits_composite)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addFrame("body1", BodyFrame(pinocchio::Inertia::Identity()));
  g.addFrame("body2", BodyFrame(pinocchio::Inertia::Identity()));

  JointComposite jmodel;
  pinocchio::SE3 jPose1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.)); // from body to j1
  pinocchio::SE3 jPose2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)); // from j1 to j2
  pinocchio::SE3 jPose3 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 1.)); // from j2 to j3
  jmodel.addJoint(JointRevolute(Eigen::Vector3d::UnitX()), jPose1);
  jmodel.addJoint(JointRevolute(Eigen::Vector3d::UnitZ()), jPose2);
  jmodel.addJoint(JointPrismatic(Eigen::Vector3d::UnitY()), jPose3);

  Eigen::Vector3d minConfig;
  minConfig << -M_PI / 4, -M_PI / 3, 0.3;
  Eigen::Vector3d maxConfig;
  maxConfig << M_PI / 2, M_PI, 0.6;

  /////////////////////////////////////// Joints
  g.edgeBuilder()
    .withName("body1_to_body2")
    .withSourceVertex("body1")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("body2")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(jmodel)
    .withMaxConfig(maxConfig)
    .withMinConfig(minConfig)
    .build();

  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);

  pinocchio::framesForwardKinematics(m_forward, d_f, m_forward.lowerPositionLimit);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);

  Eigen::Vector2d q_reverse;
  q_reverse << minConfig[1], minConfig[0];
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, m_reverse.upperPositionLimit);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test out a tree robot
///          /---- left leg
/// torso ---
///          \--- right leg
BOOST_AUTO_TEST_CASE(test_tree_robot)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  g.addBody("torso", pinocchio::Inertia::Identity());
  g.addBody("left_leg", pinocchio::Inertia::Identity());
  g.addBody("right_leg", pinocchio::Inertia::Identity());

  /////////////////////////////////////// Joints
  g.edgeBuilder()
    .withName("torso_to_left_leg")
    .withSourceVertex("torso")
    .withSourcePose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)))
    .withTargetVertex("left_leg")
    .withTargetPose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)))
    .withJointType(JointRevolute(Eigen::Vector3d::UnitX()))
    .build();

  g.edgeBuilder()
    .withName("torso_to_right_leg")
    .withSourceVertex("torso")
    .withSourcePose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-2., 2., 0.)))
    .withTargetVertex("right_leg")
    .withTargetPose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)))
    .withJointType(JointRevolute(Eigen::Vector3d::UnitZ()))
    .build();

  pinocchio::Model m = buildModel(g, "torso", pinocchio::SE3::Identity(), JointFreeFlyer());

  BOOST_CHECK(m.parents[m.getJointId("torso_to_left_leg")] == m.getJointId("root_joint"));
  BOOST_CHECK(m.parents[m.getJointId("torso_to_right_leg")] == m.getJointId("root_joint"));

  pinocchio::Model m1 = buildModel(g, "left_leg", pinocchio::SE3::Identity(), JointFreeFlyer());
  BOOST_CHECK(m1.parents[m.getJointId("torso_to_left_leg")] == m1.getJointId("root_joint"));
  BOOST_CHECK(m1.parents[m.getJointId("torso_to_right_leg")] == m1.getJointId("torso_to_left_leg"));
}

/// @brief Test to make sure that we can't chain sensor or OpFrame, that not fixed joint is
// created when adding a sensor frame
BOOST_AUTO_TEST_CASE(test_other_frame)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointRevolute(Eigen::Vector3d::UnitX()));

  g.addFrame("sensor1", SensorFrame());

  /////////////////////////////////////// Joints
  g.edgeBuilder()
    .withName("body2_to_sensor1")
    .withSourceVertex("body2")
    .withSourcePose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-2., 2., 0.)))
    .withTargetVertex("sensor1")
    .withTargetPose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)))
    .withJointType(JointFixed())
    .build();

  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());

  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q[0] = M_PI / 2;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "sensor1", d_f.oMf[m_forward.getFrameId("sensor1", pinocchio::SENSOR)]);
  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, -q);

  BOOST_CHECK(m_forward.frames.size() == 5); // All the bodies (2), joints (2) and one sensor
  BOOST_CHECK(m_reverse.frames.size() == 5); // same

  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));

  g.addFrame("sensor2", SensorFrame());
  BOOST_CHECK_THROW(
    g.edgeBuilder()
      .withName("sensor2_to_sensor1")
      .withSourceVertex("sensor2")
      .withSourcePose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-2., 2., 0.)))
      .withTargetVertex("sensor1")
      .withTargetPose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 5., 1.)))
      .withJointType(JointFixed())
      .build(),
    std::invalid_argument);
}

/// @brief Test q_ref positioning
BOOST_AUTO_TEST_CASE(test_q_ref_revolute)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody("body2", pinocchio::Inertia::Identity());

  /////////////////////////////////////// Joints
  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 poseBody2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.));
  Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(1);
  q_ref[0] = M_PI / 4;
  g.edgeBuilder()
    .withName("body1_to_body2")
    .withSourceVertex("body1")
    .withSourcePose(poseBody1)
    .withTargetVertex("body2")
    .withTargetPose(poseBody2)
    .withJointType(JointRevolute(Eigen::Vector3d::UnitX()))
    .withQref(q_ref)
    .build();

  /////////////////////////////////////// Joints
  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());

  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q[0] = M_PI / 4;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, -q);

  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief Test q_ref positioning for joint composite
BOOST_AUTO_TEST_CASE(test_q_ref_composite)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addFrame("body1", BodyFrame(pinocchio::Inertia::Identity()));
  g.addFrame("body2", BodyFrame(pinocchio::Inertia::Identity()));

  /////////////////////////////////////// Joints
  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 poseBody2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.));

  JointComposite jmodel;
  pinocchio::SE3 jPose1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.)); // from body to j1
  pinocchio::SE3 jPose2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)); // from j1 to j2
  pinocchio::SE3 jPose3 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 1.)); // from j2 to j3
  jmodel.addJoint(JointRevolute(Eigen::Vector3d::UnitX()), jPose1);
  jmodel.addJoint(JointRevolute(Eigen::Vector3d::UnitZ()), jPose2);
  jmodel.addJoint(JointPrismatic(Eigen::Vector3d::UnitY()), jPose3);

  Eigen::VectorXd q_ref = Eigen::Vector3d::Zero();
  q_ref[0] = M_PI / 4;
  q_ref[1] = -M_PI / 3;
  q_ref[2] = 1.5;

  g.edgeBuilder()
    .withName("body1_to_body2")
    .withSourceVertex("body1")
    .withSourcePose(poseBody1)
    .withTargetVertex("body2")
    .withTargetPose(poseBody2)
    .withJointType(jmodel)
    .withQref(q_ref)
    .build();

  pinocchio::Model m_forward = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q << M_PI / 2, M_PI / 5, 0.4;

  pinocchio::framesForwardKinematics(m_forward, d_f, q);
  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    buildModel(g, "body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);

  Eigen::VectorXd q_reverse(m_reverse.nq);
  q_reverse << -q[2], -q[1], -q[0];

  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reverse);
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief Test q_ref positioning with all joints
BOOST_AUTO_TEST_CASE(test_q_ref_all)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  pinocchio::Inertia I_I(pinocchio::Inertia::Identity());
  g.addBody("b1", I_I);
  g.addBody("b2", I_I);
  g.addBody("b3", I_I);
  g.addBody("b4", I_I);
  g.addBody("b5", I_I);
  g.addBody("b6", I_I);
  g.addBody("b7", I_I);
  g.addBody("b8", I_I);
  g.addBody("b9", I_I);
  g.addBody("b10", I_I);
  g.addBody("b11", I_I);
  g.addBody("b12", I_I);

  //////////////////////////////////////// Joints
  // We can't test mimic joint because backward construction is not supported
  g.edgeBuilder()
    .withName("j1")
    .withSourceVertex("b1")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("b2")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(pinocchio::graph::JointRevolute(Eigen::Vector3d::UnitX()))
    .withQref(Eigen::VectorXd::Random(1))
    .build();
  Eigen::VectorXd j2_q_ref(7);
  j2_q_ref << Eigen::Vector3d::Random(), Eigen::Vector4d::Random().normalized();
  g.edgeBuilder()
    .withName("j2")
    .withSourceVertex("b2")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("b3")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(pinocchio::graph::JointFreeFlyer())
    .withQref(j2_q_ref)
    .build();
  Eigen::VectorXd j3_q_ref(4);
  j3_q_ref << Eigen::Vector4d::Random().normalized();
  g.edgeBuilder()
    .withName("j3")
    .withSourceVertex("b3")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("b4")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(pinocchio::graph::JointSpherical())
    .withQref(j3_q_ref)
    .build();
  g.edgeBuilder()
    .withName("j4")
    .withSourceVertex("b4")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("b5")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(
      pinocchio::graph::JointUniversal(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()))
    .withQref(Eigen::Vector2d::Random())
    .build();
  Eigen::VectorXd j5_q_ref(2);
  j5_q_ref << std::cos(0.4), std::sin(0.4);
  g.edgeBuilder()
    .withName("j5")
    .withSourceVertex("b5")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("b6")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(pinocchio::graph::JointRevoluteUnbounded(Eigen::Vector3d::UnitX()))
    .withQref(j5_q_ref)
    .build();
  g.edgeBuilder()
    .withName("j6")
    .withSourceVertex("b6")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("b7")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(pinocchio::graph::JointPrismatic(Eigen::Vector3d::UnitX()))
    .withQref(Eigen::VectorXd::Random(1))
    .build();
  g.edgeBuilder()
    .withName("j7")
    .withSourceVertex("b7")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("b8")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(pinocchio::graph::JointHelical(Eigen::Vector3d::UnitX(), 0.1))
    .withQref(Eigen::VectorXd::Random(1))
    .build();
  g.edgeBuilder()
    .withName("j8")
    .withSourceVertex("b8")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("b9")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(pinocchio::graph::JointTranslation())
    .withQref(Eigen::Vector3d::Random())
    .build();
  g.edgeBuilder()
    .withName("j9")
    .withSourceVertex("b9")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("b10")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(pinocchio::graph::JointSphericalZYX())
    .withQref(Eigen::Vector3d::Random())
    .build();
  Eigen::VectorXd j10_q_ref(4);
  j10_q_ref << Eigen::Vector2d::Random(), std::cos(-0.4), std::sin(-0.4);
  g.edgeBuilder()
    .withName("j10")
    .withSourceVertex("b10")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("b11")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(pinocchio::graph::JointPlanar())
    .withQref(j10_q_ref)
    .build();
  pinocchio::graph::JointComposite joint_composite;
  joint_composite.addJoint(
    pinocchio::graph::JointRevolute(Eigen::Vector3d::UnitX()), pinocchio::SE3::Random());
  joint_composite.addJoint(pinocchio::graph::JointSpherical(), pinocchio::SE3::Random());
  joint_composite.addJoint(
    pinocchio::graph::JointRevoluteUnbounded(Eigen::Vector3d::UnitX()), pinocchio::SE3::Random());
  Eigen::VectorXd j11_q_ref(7);
  j11_q_ref << Eigen::VectorXd::Random(1), Eigen::Vector4d::Random().normalized(), std::cos(-0.9),
    std::sin(-0.9);
  g.edgeBuilder()
    .withName("j11")
    .withSourceVertex("b11")
    .withSourcePose(pinocchio::SE3::Random())
    .withTargetVertex("b12")
    .withTargetPose(pinocchio::SE3::Random())
    .withJointType(joint_composite)
    .withQref(j11_q_ref)
    .build();

  /////////////////////////////////////// Forward model
  auto forward_with_bi = buildModelWithBuildInfo(g, "b1", pinocchio::SE3::Identity());
  pinocchio::Model m_forward(forward_with_bi.model);

  const Eigen::VectorXd qmax = Eigen::VectorXd::Ones(m_forward.nq);
  const Eigen::VectorXd q = pinocchio::randomConfiguration(m_forward, -qmax, qmax);
  pinocchio::Data d_f(m_forward);
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  auto reverse_with_bi =
    buildModelWithBuildInfo(g, "b12", d_f.oMf[m_forward.getFrameId("b12", pinocchio::BODY)]);
  pinocchio::Model m_reverse(reverse_with_bi.model);
  auto forward_to_reverse_converter = pinocchio::graph::createConverter(
    m_forward, m_reverse, forward_with_bi.build_info, reverse_with_bi.build_info);
  const Eigen::VectorXd q_reversed(m_reverse.nq);
  forward_to_reverse_converter.convertConfigurationVector(q, q_reversed);

  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reversed);

  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("b1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("b1", pinocchio::BODY)]));
}

/// @brief  Test the algorithm to prefix frames and joints name
BOOST_AUTO_TEST_CASE(test_prefix_names)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  g.addBody("torso", pinocchio::Inertia::Identity());
  g.addBody("left_leg", pinocchio::Inertia::Identity());

  g.edgeBuilder()
    .withName("torso_to_left_leg")
    .withSourceVertex("torso")
    .withSourcePose(pinocchio::SE3::Identity())
    .withTargetVertex("left_leg")
    .withTargetPose(pinocchio::SE3::Identity())
    .withJointType(JointRevolute(Eigen::Vector3d::UnitX()))
    .build();

  ModelGraph g_prefixed = prefixNames(g, "prefix/");
  pinocchio::Model model = buildModel(g_prefixed, "prefix/torso", pinocchio::SE3::Identity());
  // Skip universe frame and joint
  for (std::size_t i = 1; i < model.frames.size(); ++i)
  {
    BOOST_CHECK(model.frames[i].name.find("prefix/") != std::string::npos);
  }
  for (std::size_t i = 1; i < model.names.size(); ++i)
  {
    BOOST_CHECK(model.names[i].find("prefix/") != std::string::npos);
  }
}

/// @brief  Test the algorithm to merge 2 graphs
BOOST_AUTO_TEST_CASE(test_merge_graphs)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  g.addBody("torso", pinocchio::Inertia::Identity());
  g.addBody("left_leg", pinocchio::Inertia::Identity());
  g.addBody("right_leg", pinocchio::Inertia::Identity());

  g.edgeBuilder()
    .withName("torso_to_left_leg")
    .withSourceVertex("torso")
    .withSourcePose(pinocchio::SE3::Identity())
    .withTargetVertex("left_leg")
    .withTargetPose(pinocchio::SE3::Identity())
    .withJointType(JointRevolute(Eigen::Vector3d::UnitX()))
    .build();

  g.edgeBuilder()
    .withName("torso_to_right_leg")
    .withSourceVertex("torso")
    .withSourcePose(pinocchio::SE3::Identity())
    .withTargetVertex("right_leg")
    .withTargetPose(pinocchio::SE3::Identity())
    .withJointType(JointRevolute(Eigen::Vector3d::UnitX()))
    .build();

  ModelGraph g1;
  g1.addBody("upper_arm", pinocchio::Inertia::Identity());
  g1.addBody("lower_arm", pinocchio::Inertia::Identity());
  g1.addBody("hand", pinocchio::Inertia::Identity());

  g1.edgeBuilder()
    .withName("upper2lower")
    .withSourceVertex("upper_arm")
    .withSourcePose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)))
    .withTargetVertex("lower_arm")
    .withTargetPose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)))
    .withJointType(JointRevolute(Eigen::Vector3d::UnitX()))
    .build();

  g1.edgeBuilder()
    .withName("lower2hand")
    .withSourceVertex("lower_arm")
    .withSourcePose(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 1., 0.)))
    .withTargetVertex("hand")
    .withTargetPose(pinocchio::SE3::Identity())
    .withJointType(JointRevolute(Eigen::Vector3d::UnitX()))
    .build();

  ModelGraph g_full = merge(
    g, g1, "torso", "upper_arm",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 4)),
    JointRevolute(Eigen::Vector3d::UnitY()));

  pinocchio::Model m = buildModel(g_full, "torso", pinocchio::SE3::Identity(), JointFreeFlyer());

  BOOST_CHECK(m.parents[m.getJointId("torso_to_left_leg")] == m.getJointId("root_joint"));
  BOOST_CHECK(m.parents[m.getJointId("torso_to_right_leg")] == m.getJointId("root_joint"));
  BOOST_CHECK(m.parents[m.getJointId("merging_joint")] == m.getJointId("root_joint"));
  BOOST_CHECK(m.parents[m.getJointId("upper2lower")] == m.getJointId("merging_joint"));
  BOOST_CHECK(m.parents[m.getJointId("lower2hand")] == m.getJointId("upper2lower"));

  BOOST_CHECK(
    m.frames[m.getFrameId("upper_arm", pinocchio::BODY)].placement
    == pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 4)));
}

/// @brief Test algorithm for locking joints
BOOST_AUTO_TEST_CASE(test_lock_joint)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointRevolute(Eigen::Vector3d::UnitZ()));

  std::vector<std::string> joints_to_lock;
  joints_to_lock.push_back("body1_to_body2");
  std::vector<Eigen::VectorXd> ref_configs;
  Eigen::VectorXd q_ref(1);
  q_ref[0] = M_PI / 6;
  ref_configs.push_back(q_ref);
  ModelGraph g_locked = lockJoints(g, joints_to_lock, ref_configs);

  pinocchio::Model m = buildModel(g_locked, "body1", pinocchio::SE3::Identity());

  BOOST_CHECK(m.njoints == 1);

  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));

  Eigen::AngleAxisd Rz(q_ref[0], Eigen::Vector3d::UnitZ());
  pinocchio::SE3 pose_fixed_joint = pinocchio::SE3(Rz.toRotationMatrix(), Eigen::Vector3d::Zero());

  BOOST_CHECK(m.frames[m.getFrameId("body1_to_body2", pinocchio::FIXED_JOINT)].placement.isApprox(
    poseBody1 * pose_fixed_joint));
}

/// @brief add geometries to a vertex
BOOST_AUTO_TEST_CASE(test_add_geometry)
{
  using namespace pinocchio::graph;
  ModelGraph g = buildReversableModelGraph(JointRevolute(Eigen::Vector3d::UnitY()));

  pinocchio::SE3 placement = pinocchio::SE3::Random();
  g.geometryBuilder()
    .withBody("body1")
    .withGeomType(GeomType::VISUAL)
    .withName("body1_geom1")
    .withPlacement(placement)
    .withGeom(Box(Eigen::Vector3d::Constant(2)))
    .build();

  g.geometryBuilder()
    .withBody("body1")
    .withGeomType(GeomType::VISUAL)
    .withName("body1_geom2")
    .withPlacement(placement)
    .withGeom(Sphere(4))
    .build();

  auto vertex_n = g.name_to_vertex.find("body1");
  auto vertex = g.graph[vertex_n->second];

  BOOST_CHECK(vertex.geometries.size() == 2);
}

#if defined(PINOCCHIO_WITH_HPP_FCL)
BOOST_AUTO_TEST_CASE(test_build_geometry_model)
{
  using namespace pinocchio::graph;
  ModelGraph g = buildReversableModelGraph(JointRevolute(Eigen::Vector3d::UnitY()));

  pinocchio::SE3 placement = pinocchio::SE3::Random();
  g.geometryBuilder()
    .withBody("body1")
    .withGeomType(GeomType::VISUAL)
    .withName("body1_geom1")
    .withPlacement(placement)
    .withGeom(Box(Eigen::Vector3d::Constant(2)))
    .build();

  g.geometryBuilder()
    .withBody("body1")
    .withGeomType(GeomType::COLLISION)
    .withName("body1_geom2")
    .withPlacement(placement)
    .withGeom(Sphere(4))
    .build();

  pinocchio::SE3 placement_b2 = pinocchio::SE3::Random();
  g.geometryBuilder()
    .withBody("body2")
    .withGeomType(GeomType::VISUAL)
    .withName("body2_geom2")
    .withPlacement(placement_b2)
    .withGeom(Cylinder(Eigen::Vector2d::Constant(2)))
    .build();

  g.geometryBuilder()
    .withBody("body2")
    .withGeomType(GeomType::COLLISION)
    .withName("body2_geom2")
    .withPlacement(placement_b2)
    .withGeom(Capsule(Eigen::Vector2d::Constant(2)))
    .build();

  g.geometryBuilder()
    .withBody("body2")
    .withGeomType(GeomType::BOTH)
    .withName("body2_geom3")
    .withPlacement(placement_b2)
    .withGeom(Capsule(Eigen::Vector2d::Constant(3)))
    .build();

  pinocchio::Model m = buildModel(g, "body1", pinocchio::SE3::Identity());
  pinocchio::GeometryModel m_visual = buildGeometryModel(g, m, pinocchio::VISUAL);
  pinocchio::GeometryModel m_col = buildGeometryModel(g, m, pinocchio::COLLISION);

  BOOST_CHECK(m_visual.ngeoms == 3);
  BOOST_CHECK(m_col.ngeoms == 3);

  // Check that the right geometries were added to visual
  auto * box = dynamic_cast<hpp::fcl::Box *>(m_visual.geometryObjects.at(0).geometry.get());
  BOOST_REQUIRE(box);
  Eigen::Vector3d sides = Eigen::Vector3d::Constant(1);
  BOOST_CHECK(box->halfSide == sides);

  auto * cyl = dynamic_cast<hpp::fcl::Cylinder *>(m_visual.geometryObjects.at(1).geometry.get());
  BOOST_REQUIRE(cyl);
  BOOST_CHECK(cyl->halfLength == 1);
  BOOST_CHECK(cyl->radius == 2);

  auto * cap = dynamic_cast<hpp::fcl::Capsule *>(m_visual.geometryObjects.at(2).geometry.get());
  BOOST_REQUIRE(cap);
  BOOST_CHECK(cap->halfLength == 1.5);
  BOOST_CHECK(cap->radius == 3);

  // Check that the right geometries were added to collision
  auto * s = dynamic_cast<hpp::fcl::Sphere *>(m_col.geometryObjects.at(0).geometry.get());
  BOOST_REQUIRE(s);
  BOOST_CHECK(s->radius == 4);

  auto * c = dynamic_cast<hpp::fcl::Capsule *>(m_col.geometryObjects.at(1).geometry.get());
  BOOST_REQUIRE(c);
  BOOST_CHECK(c->halfLength == 1);
  BOOST_CHECK(c->radius == 2);

  auto * cap2 = dynamic_cast<hpp::fcl::Capsule *>(m_col.geometryObjects.at(2).geometry.get());
  BOOST_REQUIRE(cap2);
  BOOST_CHECK(cap2->halfLength == 1.5);
  BOOST_CHECK(cap2->radius == 3);
}
#endif
BOOST_AUTO_TEST_SUITE_END()
