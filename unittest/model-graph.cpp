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

BOOST_AUTO_TEST_CASE(test_add_vertex)
{
  pinocchio::ModelGraph g;

  g.addBody("body1", pinocchio::Inertia::Identity());
  BOOST_CHECK(g.name_to_vertex.find("body1") != g.name_to_vertex.end());
}

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
    "body1_to_body2",
    pinocchio::internal::JointRevoluteGraph("body1_to_body2", Eigen::Vector3d::UnitZ()), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)));

  VertexDesc v_out = g.name_to_vertex["body1"];
  VertexDesc v_in = g.name_to_vertex["body2"];
  BOOST_CHECK(boost::edge(v_out, v_in, g.g).second);
  BOOST_CHECK(boost::edge(v_in, v_out, g.g).second);
}

BOOST_AUTO_TEST_CASE(test_linear_robot)
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
    "body1_to_body2",
    pinocchio::internal::JointRevoluteGraph("body1_to_body2", Eigen::Vector3d::UnitZ()), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)));
  g.addJoint(
    "body2_to_body3",
    pinocchio::internal::JointRevoluteGraph("body2_to_body3", Eigen::Vector3d::UnitX()), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., -3., 0.)), "body3",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4., -3., 0.)));

  ///////////////// Model
  pinocchio::Model m = g.buildModel("body1", pinocchio::JointModelFreeFlyer());

  BOOST_CHECK(m.jointPlacements[m.getJointId("body1_to_body2")].isApprox(
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2, 0, 0))));
  BOOST_CHECK(m.jointPlacements[m.getJointId("body2_to_body3")].isApprox(
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4, -1, 0))));

  pinocchio::Model m1 = g.buildModel("body3", pinocchio::JointModelFreeFlyer());
  for (size_t i = 1; i < m.inertias.size(); i++)
  {
    BOOST_CHECK(m1.inertias[i].isApprox(m.inertias[m.inertias.size() - i]));
  }
}

BOOST_AUTO_TEST_CASE(test_flipping_axis)
{
  pinocchio::ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  g.addJoint(
    "body1_to_body2",
    pinocchio::internal::JointRevoluteGraph("body1_to_body2", -Eigen::Vector3d::UnitZ()), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)));

  ///////////////// Model
  pinocchio::Model m = g.buildModel("body1", pinocchio::JointModelFreeFlyer());
  pinocchio::Data d(m);
  // Create a random valid configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m.nq);
  q[m.idx_qs[m.getJointId("body1_to_body2")]] = 0.7;

  // Compute forward kinematics
  pinocchio::framesForwardKinematics(m, d, q);

  pinocchio::ModelGraph g1;
  //////////////////////////////////////// Bodies
  g1.addBody("body1", pinocchio::Inertia::Identity());
  g1.addBody(
    "body2",
    pinocchio::Inertia(4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));

  /////////////////////////////////////// Joints
  g1.addJoint(
    "body1_to_body2",
    pinocchio::internal::JointRevoluteGraph("body1_to_body2", Eigen::Vector3d::UnitZ()), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)));

  ///////////////// Model
  pinocchio::Model m1 = g1.buildModel("body1", pinocchio::JointModelFreeFlyer());
  pinocchio::Data d1(m1);
  // Compute forward kinematics
  // flip q value of joint body1_to_body2
  q[m1.idx_qs[m1.getJointId("body1_to_body2")]] = -q[m1.idx_qs[m1.getJointId("body1_to_body2")]];
  pinocchio::framesForwardKinematics(m1, d1, q);

  BOOST_CHECK(d.oMf[m.getFrameId("body2", pinocchio::BODY)].isApprox(
    d1.oMf[m1.getFrameId("body2", pinocchio::BODY)]));
}

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
    "body1_to_body2",
    pinocchio::internal::JointRevoluteGraph("body1_to_body2", Eigen::Vector3d::UnitZ()), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)));
  g.addJoint(
    "body2_to_body3", pinocchio::internal::JointFixedGraph("body2_to_body3"), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., -3., 0.)), "body3",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4., -5., 0.)));

  ///////////////// Model
  pinocchio::Model m = g.buildModel("body1", pinocchio::JointModelFreeFlyer());

  BOOST_CHECK(m.njoints == 3);
  BOOST_CHECK(m.frames[m.getFrameId("body2_to_body3", pinocchio::FIXED_JOINT)].placement.isApprox(
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4., -1., 0.))));
}

BOOST_AUTO_TEST_CASE(test_tree_robot)
{
  pinocchio::ModelGraph g;
  g.addBody("torso", pinocchio::Inertia::Identity());
  g.addBody("left_leg", pinocchio::Inertia::Identity());
  g.addBody("right_leg", pinocchio::Inertia::Identity());
  g.addJoint(
    "torso_to_left_leg",
    pinocchio::internal::JointRevoluteGraph("torso_left_leg", Eigen::Vector3d::UnitX()), "torso",
    pinocchio::SE3::Identity(), "left_leg", pinocchio::SE3::Identity());
  g.addJoint(
    "torso_to_right_leg",
    pinocchio::internal::JointRevoluteGraph("torso_right_leg", Eigen::Vector3d::UnitX()), "torso",
    pinocchio::SE3::Identity(), "right_leg", pinocchio::SE3::Identity());

  g.buildModel("torso", pinocchio::JointModelFreeFlyer());
}

BOOST_AUTO_TEST_SUITE_END()
