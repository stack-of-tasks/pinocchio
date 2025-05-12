//
// Copyright (c) 2024-2025 INRIA
//
#include "pinocchio/multibody/data.hpp"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

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
/// to start
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
    "body1_to_body2", pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitZ()), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 2., 0.)));

  ///////////////// Model
  pinocchio::Model m = g.buildModel("body1", pinocchio::SE3::Identity());

  BOOST_CHECK(m.jointPlacements[m.getJointId("body1_to_body2")].isApprox(
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2, 0, 0))));

  // Forward kinematics
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m.nq);
  q[0] = M_PI;

  // Compute forward kinematics
  pinocchio::Data d(m);
  pinocchio::framesForwardKinematics(m, d, q);

  pinocchio::Model m1 = g.buildModel("body2", pinocchio::SE3::Identity());
  // Compute forward kinematics
  pinocchio::Data d1(m1);
  pinocchio::framesForwardKinematics(m1, d1, q);

  Eigen::Matrix3d rot;
  rot << -1, 0, 0, 0, -1, 0, 0, 0, 1;
  pinocchio::SE3 bodyPos(rot, Eigen::Vector3d(2, -2, 0));

  // Expected result for world position of frame body2 R = [[-1 0 0], [0 -1 0], [0 0 1]] p = 2 -2 0
  // because body1 is fixed in R = Identity p = 0 0 0 (X1)
  // joint1 position wrt body1 is R = Identity p = 2 0 0 (X2)
  // q = pi so T = [[-1 0 0 0], [0 -1 0 0], [0 0 1 0], [0 0 0 1]] (X3)
  // and body2 position wrt joint1 is R = Identity p = 0 2 0 (X4)
  // so world position of body2 = X1 * X2 * X3 * X4, so R = [[-1 0 0], [0 -1 0], [0 0 1]] p = 2 -2 0
  BOOST_CHECK(d.oMf[m.getFrameId("body2", pinocchio::BODY)].isApprox(bodyPos));

  // Expected result for world position of frame body1 R = [[-1 0 0], [0 -1 0], [0 0 1]] p = 2 -2 0
  // because body2 is fixed in R = Identity p = 0 0 0 (X1)
  // joint1 position wrt body2 is R = [[-1 0 0], [0 1 0], [0 0 -1]] p = 0 -2 0 (X2) (to keep
  // pinocchio optimization, frame are rotated around to always have a positive axis for joints) q =
  // pi so T = [[-1 0 0 0], [0 -1 0 0], [0 0 1 0], [0 0 0 1]] (X3) and body1 position wrt joint1 is
  // R = [[-1 0 0], [0 1 0], [0 0 -1]] p = 2 0 0 (X4) so world position of body1 = X1 * X2 * X3 *
  // X4, so R = [[-1 0 0], [0 -1 0], [0 0 1]] p = 2 -2 0
  BOOST_CHECK(d1.oMf[m1.getFrameId("body1", pinocchio::BODY)].isApprox(bodyPos));
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
  pinocchio::Model m =
    g.buildModel("body1", pinocchio::SE3::Identity(), pinocchio::JointModelFreeFlyer());

  BOOST_CHECK(m.njoints == 3);
  BOOST_CHECK(m.frames[m.getFrameId("body2_to_body3", pinocchio::FIXED_JOINT)].placement.isApprox(
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4., -1., 0.))));
}

// BOOST_AUTO_TEST_CASE(test_tree_robot)
// {
//   pinocchio::ModelGraph g;
//   g.addBody("torso", pinocchio::Inertia::Identity());
//   g.addBody("left_leg", pinocchio::Inertia::Identity());
//   g.addBody("right_leg", pinocchio::Inertia::Identity());
//   g.addJoint(
//     "torso_to_left_leg",
//     pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitX()), "torso",
//     pinocchio::SE3::Identity(), "left_leg", pinocchio::SE3::Identity());
//   g.addJoint(
//     "torso_to_right_leg",
//     pinocchio::JointRevoluteGraph(Eigen::Vector3d::UnitX()), "torso",
//     pinocchio::SE3::Identity(), "right_leg", pinocchio::SE3::Identity());

//   g.buildModel("torso", pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity());
// }

BOOST_AUTO_TEST_SUITE_END()
