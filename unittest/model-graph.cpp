//
// Copyright (c) 2024 INRIA
//

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
  typedef typename boost::graph_traits<Graph>::edge_descriptor EdgeDesc;

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
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., -2., 0.)), "body3",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4., -3., 0.)));

  ///////////////// Model
  pinocchio::Model m = g.buildModel("body1", pinocchio::JointModelFreeFlyer());

  std::cout << m << std::endl;

  for (auto jp : m.inertias)
    std::cout << jp << std::endl;
  // pinocchio::Model m1 =g.buildModel(
  //   "body3",
  //   pinocchio::JointModelFreeFlyerTpl<pinocchio::context::Scalar,
  //   pinocchio::context::Options>());

  // std::cout << m1 << std::endl;
}

BOOST_AUTO_TEST_CASE(test_tree_robot)
{
  // pinoccModelGraph g;
  // g.addBody("torso", pinocchio::Inertia::Identity());
  // g.addBody("left_leg", pinocchio::Inertia::Identity());
  // g.addBody("right_leg", pinocchio::Inertia::Identity());
  // g.addJoint(
  //   "torso_to_left_leg", JointModel(pinocchio::JointModelRX()), "torso", SE3::Identity(),
  //   "left_leg", SE3::Identity());
  // g.addJoint(
  //   "torso_to_right_leg", JointModel(pinocchio::JointModelRX()), "torso", SE3::Identity(),
  //   "right_leg", SE3::Identity());

  //   g.buildModel(
  //     "torso",
  //     pinocchio::JointModelFreeFlyerTpl<pinocchio::context::Scalar,
  //     pinocchio::context::Options>());

  //   g.buildModel(
  //     "left_leg",
  //     pinocchio::JointModelFreeFlyerTpl<pinocchio::context::Scalar,
  //     pinocchio::context::Options>());
}

BOOST_AUTO_TEST_SUITE_END()
