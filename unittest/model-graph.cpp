//
// Copyright (c) 2024-2025 INRIA
//
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "pinocchio/multibody/model-graph.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(ModelGraph)

/// @brief test if vertex are added to the graph
BOOST_AUTO_TEST_CASE(test_add_vertex)
{
  pinocchio::ModelGraph g;

  g.addBody("body1", pinocchio::Inertia::Identity());
  BOOST_CHECK(g.name_to_vertex.find("body1") != g.name_to_vertex.end());
}

/// @brief Test if edges and their reverse are added correctly to the graph
BOOST_AUTO_TEST_CASE(test_add_edges)
{
  typedef boost::adjacency_list<
    boost::vecS, boost::vecS, boost::directedS, pinocchio::ModelGraphVertex,
    pinocchio::ModelGraphEdge>
    Graph;
  typedef typename boost::graph_traits<Graph>::vertex_descriptor VertexDesc;

  pinocchio::ModelGraph g;
  ////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody("body2", pinocchio::Inertia::Identity());

  /////////////////////////////////////// Joints
  g.addJoint(
    "body1_to_body2", pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitZ()), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)));

  VertexDesc v_out = g.name_to_vertex["body1"];
  VertexDesc v_in = g.name_to_vertex["body2"];
  BOOST_CHECK(boost::edge(v_out, v_in, g.g).second);
  BOOST_CHECK(boost::edge(v_in, v_out, g.g).second);
}

/// @brief Test of simple 2R robot to try out kinematics and what happens when we use different body
/// as root body body1 --- body2
BOOST_AUTO_TEST_CASE(test_linear_2D_robot)
{
  pinocchio::ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  g.addJoint(
    "body1_to_body2", pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitY()), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 2., 0.)));

  ///////////////// Model
  pinocchio::Model m = g.buildModel("body1", pinocchio::SE3::Identity());

  BOOST_CHECK(m.jointPlacements[m.getJointId("body1_to_body2")].isApprox(
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2, 0, 0))));

  // Forward kinematics
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m.nq);
  q[0] = M_PI / 2;

  // Compute forward kinematics
  pinocchio::Data d(m);
  pinocchio::framesForwardKinematics(m, d, q);

  pinocchio::Model m1 = g.buildModel("body2", pinocchio::SE3::Identity());
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
  pinocchio::SE3 X4 = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 2.0, 0.0));
  // Final transformation
  pinocchio::SE3 bodyPose = X1 * X2 * X3 * X4;
  BOOST_CHECK(d.oMf[m.getFrameId("body2", pinocchio::BODY)].isApprox(bodyPose));

  // World to Body2 (Identity)
  pinocchio::SE3 X1_ = pinocchio::SE3::Identity();
  // Body2 to Joint1
  pinocchio::SE3 X2_ = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, -2.0, 0.0));
  // Rotation around Y by q = -pi/2 (reverse joint)
  Eigen::AngleAxisd R_y_(-q[0], Eigen::Vector3d::UnitY());
  pinocchio::SE3 X3_ = pinocchio::SE3(R_y_.toRotationMatrix(), Eigen::Vector3d::Zero());
  // Joint1 to Body1
  pinocchio::SE3 X4_ = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-2.0, 0.0, 0.0));
  // Final transformation
  pinocchio::SE3 bodyPose1 = X1_ * X2_ * X3_ * X4_;
  BOOST_CHECK(d1.oMf[m1.getFrameId("body1", pinocchio::BODY)].isApprox(bodyPose1));
}

/// @brief Test out the fixed joint.
BOOST_AUTO_TEST_CASE(test_fixed_joint)
{
  pinocchio::ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));
  g.addBody(
    "body3", pinocchio::Inertia(
               3., pinocchio::Inertia::Vector3(0., -3, -2.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  g.addJoint(
    "body1_to_body2", pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitZ()), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)));
  g.addJoint(
    "body2_to_body3", pinocchio::JointFixedGraph(), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., -3., 0.)), "body3",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4., -5., 0.)));

  ///////////////// Model
  pinocchio::Model m = g.buildModel(
    "body1", pinocchio::SE3::Identity(),
    pinocchio::JointGraphVariant(pinocchio::JointFreeFlyerGraph()));

  BOOST_CHECK(m.njoints == 3);
  BOOST_CHECK(m.frames[m.getFrameId("body2_to_body3", pinocchio::FIXED_JOINT)].placement.isApprox(
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4., -1., 0.))));
}

/// @brief test out reverse joint for revolute
BOOST_AUTO_TEST_CASE(test_reverse_revolute)
{
  pinocchio::ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 poseBody2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.));
  g.addJoint(
    "body1_to_body2", pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitX()), "body1", poseBody1,
    "body2", poseBody2);

  pinocchio::ModelGraph g1;
  //////////////////////////////////////// Bodies
  g1.addBody("body1", pinocchio::Inertia::Identity());
  g1.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  g1.addJoint(
    "body1_to_body2", pinocchio::JointRevoluteGraph(-Eigen::Vector3d::UnitX()), "body1",
    poseBody2.inverse(), "body2", poseBody1.inverse());

  //////////////////////////////////// Models
  pinocchio::Model m_reverse = g.buildModel("body2", pinocchio::SE3::Identity());
  pinocchio::Model m_equi = g1.buildModel("body1", pinocchio::SE3::Identity());

  // Forward kinematics
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_reverse.nq);
  q[0] = M_PI / 2;

  // Compute forward kinematics
  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, -q);

  pinocchio::Data d_equi(m_equi);
  pinocchio::framesForwardKinematics(m_equi, d_equi, q);

  BOOST_CHECK(d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)].isApprox(
    d_equi.oMf[m_equi.getFrameId("body2", pinocchio::BODY)]));
}

/// @brief Test for a reverse revolute unbounded
BOOST_AUTO_TEST_CASE(test_reverse_revolute_unbounded)
{
  pinocchio::ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 poseBody2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.));
  g.addJoint(
    "body1_to_body2", pinocchio::JointRevoluteUnboundedGraph(Eigen::Vector3d::UnitX()), "body1",
    poseBody1, "body2", poseBody2);

  pinocchio::ModelGraph g1;
  //////////////////////////////////////// Bodies
  g1.addBody("body1", pinocchio::Inertia::Identity());
  g1.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  g1.addJoint(
    "body1_to_body2", pinocchio::JointRevoluteUnboundedGraph(-Eigen::Vector3d::UnitX()), "body1",
    poseBody2.inverse(), "body2", poseBody1.inverse());

  //////////////////////////////////// Models
  pinocchio::Model m_reverse = g.buildModel("body2", pinocchio::SE3::Identity());
  pinocchio::Model m_equi = g1.buildModel("body1", pinocchio::SE3::Identity());

  // Forward kinematics
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_reverse.nq);
  q[0] = 0;
  q[1] = 1;

  // Compute forward kinematics
  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, -q);

  pinocchio::Data d_equi(m_equi);
  pinocchio::framesForwardKinematics(m_equi, d_equi, q);

  BOOST_CHECK(d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)].isApprox(
    d_equi.oMf[m_equi.getFrameId("body2", pinocchio::BODY)]));
}

/// @brief test out reverse joint for prismatic
BOOST_AUTO_TEST_CASE(test_reverse_prismatic)
{
  pinocchio::ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 poseBody2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.));
  g.addJoint(
    "body1_to_body2", pinocchio::JointPrismaticGraph(Eigen::Vector3d::UnitX()), "body1", poseBody1,
    "body2", poseBody2);

  pinocchio::ModelGraph g1;
  //////////////////////////////////////// Bodies
  g1.addBody("body1", pinocchio::Inertia::Identity());
  g1.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  g1.addJoint(
    "body1_to_body2", pinocchio::JointPrismaticGraph(-Eigen::Vector3d::UnitX()), "body1",
    poseBody2.inverse(), "body2", poseBody1.inverse());

  //////////////////////////////////// Models
  pinocchio::Model m_reverse = g.buildModel("body2", pinocchio::SE3::Identity());
  pinocchio::Model m_equi = g1.buildModel("body1", pinocchio::SE3::Identity());

  // Forward kinematics
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_reverse.nq);
  q[0] = M_PI / 2;

  // Compute forward kinematics
  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, -q);

  pinocchio::Data d_equi(m_equi);
  pinocchio::framesForwardKinematics(m_equi, d_equi, q);

  BOOST_CHECK(d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)].isApprox(
    d_equi.oMf[m_equi.getFrameId("body2", pinocchio::BODY)]));
}

/// @brief test out reverse joint for helical
BOOST_AUTO_TEST_CASE(test_reverse_helical)
{
  pinocchio::ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 poseBody2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.));
  g.addJoint(
    "body1_to_body2", pinocchio::JointHelicalGraph(Eigen::Vector3d::UnitX(), 0.1), "body1",
    poseBody1, "body2", poseBody2);

  pinocchio::ModelGraph g1;
  //////////////////////////////////////// Bodies
  g1.addBody("body1", pinocchio::Inertia::Identity());
  g1.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  g1.addJoint(
    "body1_to_body2", pinocchio::JointHelicalGraph(-Eigen::Vector3d::UnitX(), 0.1), "body1",
    poseBody2.inverse(), "body2", poseBody1.inverse());

  //////////////////////////////////// Models
  pinocchio::Model m_reverse = g.buildModel("body2", pinocchio::SE3::Identity());
  pinocchio::Model m_equi = g1.buildModel("body1", pinocchio::SE3::Identity());

  // Forward kinematics
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_reverse.nq);
  q[0] = M_PI / 2;

  // Compute forward kinematics
  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, -q);

  pinocchio::Data d_equi(m_equi);
  pinocchio::framesForwardKinematics(m_equi, d_equi, q);

  BOOST_CHECK(d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)].isApprox(
    d_equi.oMf[m_equi.getFrameId("body2", pinocchio::BODY)]));
}

/// @brief test reversing helical joint on a simple linear robot
/// body1 --- body2
BOOST_AUTO_TEST_CASE(test_reverse_universal)
{
  pinocchio::ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 poseBody2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.));
  g.addJoint(
    "body1_to_body2",
    pinocchio::JointUniversalGraph(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()), "body1",
    poseBody1, "body2", poseBody2);

  //////////////////////////////////// Models
  pinocchio::Model m_forward = g.buildModel("body1", pinocchio::SE3::Identity());
  pinocchio::Model m_backward = g.buildModel("body2", pinocchio::SE3::Identity());

  // Forward kinematics
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q[0] = M_PI / 2;
  q[1] = M_PI / 3;

  // Compute forward kinematics
  pinocchio::Data d_f(m_forward);
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  Eigen::VectorXd q_backward = Eigen::VectorXd::Zero(m_backward.nq);
  q_backward[0] = q[1];
  q_backward[1] = q[0];

  pinocchio::Data d_b(m_backward);
  pinocchio::framesForwardKinematics(m_backward, d_b, q_backward);

  // World to Body1 (Identity)
  pinocchio::SE3 X1 = pinocchio::SE3::Identity();
  // First Rotate around X and then rotate around Y
  Eigen::AngleAxisd R_y(q[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd R_x(q[0], Eigen::Vector3d::UnitX());
  pinocchio::SE3 X3 =
    pinocchio::SE3(R_x.toRotationMatrix() * R_y.toRotationMatrix(), Eigen::Vector3d::Zero());
  // Final transformation
  pinocchio::SE3 bodyPose = X1 * poseBody1 * X3 * poseBody2;
  BOOST_CHECK(d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)].isApprox(bodyPose));

  // World to Body2  = World to Body1
  // Body2 to Joint1 = poseBody2.inverse()
  // Rotation around -Y and then around -X (reverse joint universal)
  Eigen::AngleAxisd R_y_(q[1], -Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd R_x_(q[0], -Eigen::Vector3d::UnitX());
  pinocchio::SE3 X3_ =
    pinocchio::SE3(R_y_.toRotationMatrix() * R_x_.toRotationMatrix(), Eigen::Vector3d::Zero());
  // Joint1 to Body1 = poseBody.inverse
  // Final transformation
  pinocchio::SE3 bodyPose1 = X1 * poseBody2.inverse() * X3_ * poseBody1.inverse();

  BOOST_CHECK(d_b.oMf[m_backward.getFrameId("body1", pinocchio::BODY)].isApprox(bodyPose1));
}

/// @brief compare reverse model with spherical
BOOST_AUTO_TEST_CASE(test_reverse_spherical)
{
  pinocchio::ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  g.addJoint(
    "body1_to_body2", pinocchio::JointSphericalGraph(), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-2., 0., 0.)));

  ///////////////// Model
  pinocchio::Model m = g.buildModel("body1", pinocchio::SE3::Identity());
  ///////////////// Model
  pinocchio::Model m1 = g.buildModel("body2", pinocchio::SE3::Identity());

  Eigen::AngleAxisd rollAngle(1, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yawAngle(0.4, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(1, Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q_sph = rollAngle * yawAngle * pitchAngle;

  Eigen::VectorXd q(m.nq);
  q << q_sph.x(), q_sph.y(), q_sph.z(), q_sph.w();

  pinocchio::Data d(m);
  pinocchio::framesForwardKinematics(m, d, q);

  pinocchio::Data d1(m1);
  Eigen::VectorXd q_reverse(m.nq);
  q_reverse << q_sph.inverse().x(), q_sph.inverse().y(), q_sph.inverse().z(), q_sph.inverse().w();

  pinocchio::framesForwardKinematics(m1, d1, q_reverse);

  BOOST_CHECK(d.oMf[m.getFrameId("body1", pinocchio::BODY)].isApprox(
    d1.oMf[m1.getFrameId("body2", pinocchio::BODY)]));
}

/// @brief test if reversing of a composite joint is correct.
BOOST_AUTO_TEST_CASE(test_reverse_composite)
{
  pinocchio::ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  pinocchio::JointCompositeGraph jmodel;
  pinocchio::SE3 jPose1 = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.));
  pinocchio::SE3 jPose2 = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 jPose3 = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 1.));
  jmodel.addJoint(pinocchio::JointPrismaticGraph(Eigen::Vector3d::UnitX()), jPose1);
  jmodel.addJoint(pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitZ()), jPose2);
  jmodel.addJoint(pinocchio::JointPrismaticGraph(Eigen::Vector3d::UnitY()), jPose3);

  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 poseBody2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.));
  g.addJoint("body1_to_body2", jmodel, "body1", poseBody1, "body2", poseBody2);

  pinocchio::ModelGraph g1;
  //////////////////////////////////////// Bodies
  g1.addBody("body1", pinocchio::Inertia::Identity());
  g1.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  pinocchio::JointCompositeGraph jmodel1;
  jmodel1.addJoint(pinocchio::JointPrismaticGraph(-Eigen::Vector3d::UnitY()), jPose3.inverse());
  jmodel1.addJoint(pinocchio::JointRevoluteGraph(-Eigen::Vector3d::UnitZ()), jPose2.inverse());
  jmodel1.addJoint(pinocchio::JointPrismaticGraph(-Eigen::Vector3d::UnitX()), jPose1.inverse());

  g1.addJoint(
    "body1_to_body2", jmodel1, "body1", poseBody2.inverse(), "body2", poseBody1.inverse());

  ///////////////// Model
  pinocchio::Model m_backward = g.buildModel("body2", pinocchio::SE3::Identity());
  pinocchio::Data d_b(m_backward);
  ///////////////// Model
  pinocchio::Model m_equi = g1.buildModel("body1", pinocchio::SE3::Identity());
  pinocchio::Data d1(m_equi);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_backward.nq);
  q[0] = 0.6;
  q[1] = 0.1;
  q[2] = -0.2;
  pinocchio::framesForwardKinematics(m_backward, d_b, -q);
  pinocchio::framesForwardKinematics(m_equi, d1, q);

  BOOST_CHECK(d_b.oMf[m_backward.getFrameId("body1", pinocchio::BODY)].isApprox(
    d1.oMf[m_equi.getFrameId("body2", pinocchio::BODY)]));
}

/// @brief Test out if inertias are well placed on the model
BOOST_AUTO_TEST_CASE(test_inertia)
{
  pinocchio::ModelGraph g;
  //////////////////////////////////////// Bodies
  pinocchio::Inertia inert = pinocchio::Inertia(
    1., pinocchio::Inertia::Vector3(0., -2., 1.), pinocchio::Symmetric3::Random());
  g.addBody("body1", inert);
  g.addBody(
    "body2", pinocchio::Inertia(
               4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Random()));
  g.addBody("body3", inert);

  /////////////////////////////////////// Joints
  g.addJoint(
    "body1_to_body2", pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitZ()), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)));
  g.addJoint(
    "body2_to_body3", pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitZ()), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., -2., 0.)), "body3",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-2., 0., 0.)));

  pinocchio::Model m = g.buildModel("body1", pinocchio::SE3::Identity());
  pinocchio::Model m1 = g.buildModel("body3", pinocchio::SE3::Identity());

  Eigen::VectorXd q = Eigen::VectorXd::Random(m.nq);

  pinocchio::Data d(m);
  pinocchio::Data d1(m1);

  pinocchio::crba(m, d, q);
  pinocchio::crba(m1, d1, -q);

  BOOST_CHECK(d.M.isApprox(d1.M));
}

/// @brief test out a tree robot
///          /---- left leg
/// torso ---
///          \--- right leg
/// @param
BOOST_AUTO_TEST_CASE(test_tree_robot)
{
  pinocchio::ModelGraph g;
  g.addBody("torso", pinocchio::Inertia::Identity());
  g.addBody("left_leg", pinocchio::Inertia::Identity());
  g.addBody("right_leg", pinocchio::Inertia::Identity());
  g.addJoint(
    "torso_to_left_leg", pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitX()), "torso",
    pinocchio::SE3::Identity(), "left_leg", pinocchio::SE3::Identity());
  g.addJoint(
    "torso_to_right_leg", pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitX()), "torso",
    pinocchio::SE3::Identity(), "right_leg", pinocchio::SE3::Identity());

  pinocchio::Model m = g.buildModel(
    "torso", pinocchio::SE3::Identity(),
    pinocchio::JointGraphVariant(pinocchio::JointFreeFlyerGraph()));

  BOOST_CHECK(m.parents[m.getJointId("torso_to_left_leg")] == m.getJointId("root_joint"));
  BOOST_CHECK(m.parents[m.getJointId("torso_to_right_leg")] == m.getJointId("root_joint"));

  pinocchio::Model m1 = g.buildModel(
    "left_leg", pinocchio::SE3::Identity(),
    pinocchio::JointGraphVariant(pinocchio::JointFreeFlyerGraph()));
  BOOST_CHECK(m1.parents[m.getJointId("torso_to_left_leg")] == m1.getJointId("root_joint"));
  BOOST_CHECK(m1.parents[m.getJointId("torso_to_right_leg")] == m1.getJointId("torso_to_left_leg"));
}

/// @brief  Test the algorithm to merge 2 graphs
BOOST_AUTO_TEST_CASE(test_merge_graphs)
{
  pinocchio::ModelGraph g;
  g.addBody("torso", pinocchio::Inertia::Identity());
  g.addBody("left_leg", pinocchio::Inertia::Identity());
  g.addBody("right_leg", pinocchio::Inertia::Identity());
  g.addJoint(
    "torso2left_leg", pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitX()), "torso",
    pinocchio::SE3::Identity(), "left_leg", pinocchio::SE3::Identity());
  g.addJoint(
    "torso2right_leg", pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitX()), "torso",
    pinocchio::SE3::Identity(), "right_leg", pinocchio::SE3::Identity());

  pinocchio::ModelGraph g1;
  g1.addBody("upper_arm", pinocchio::Inertia::Identity());
  g1.addBody("lower_arm", pinocchio::Inertia::Identity());
  g1.addBody("hand", pinocchio::Inertia::Identity());
  g1.addJoint(
    "upper2lower", pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitZ()), "upper_arm",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "lower_arm",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)));
  g1.addJoint(
    "lower2hand", pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitX()), "lower_arm",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 1., 0.)), "hand",
    pinocchio::SE3::Identity());

  pinocchio::ModelGraph g_full = pinocchio::mergeGraphs(
    g, g1, "torso", "upper_arm",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 4)),
    pinocchio::JointGraphVariant(pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitY())));

  pinocchio::Model m = g_full.buildModel(
    "torso", pinocchio::SE3::Identity(),
    pinocchio::JointGraphVariant(pinocchio::JointFreeFlyerGraph()));

  BOOST_CHECK(m.parents[m.getJointId("torso2left_leg")] == m.getJointId("root_joint"));
  BOOST_CHECK(m.parents[m.getJointId("torso2right_leg")] == m.getJointId("root_joint"));
  BOOST_CHECK(m.parents[m.getJointId("merging_joint")] == m.getJointId("root_joint"));
  BOOST_CHECK(m.parents[m.getJointId("g2/upper2lower")] == m.getJointId("merging_joint"));
  BOOST_CHECK(m.parents[m.getJointId("g2/lower2hand")] == m.getJointId("g2/upper2lower"));

  BOOST_CHECK(
    m.frames[m.getFrameId("g2/upper_arm", pinocchio::BODY)].placement
    == pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 4)));
}

BOOST_AUTO_TEST_SUITE_END()
