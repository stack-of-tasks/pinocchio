//
// Copyright (c) 2024 INRIA
//

#include "pinocchio/multibody/model-graph.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(ModelGraph)

// using ModelGraph = pinocchio::ModelGraphTpl<
//   pinocchio::context::Scalar,
//   pinocchio::context::Options,
//   pinocchio::JointCollectionDefaultTpl>;
// using JointModel = typename ModelGraph::JointModel;
// using SE3 = typename ModelGraph::SE3;
// using Inertia = typename ModelGraph::Inertia;
// using Vector3 = typename Inertia::Vector3;
// using Symmetric3 = typename Inertia::Symmetric3;

BOOST_AUTO_TEST_CASE(test_add_body)
{
}

BOOST_AUTO_TEST_CASE(test_add_edge)
{
}

BOOST_AUTO_TEST_CASE(test_linear_robot)
{
  pinocchio::ModelGraph g;
  g.addBody("body1", pinocchio::Inertia::Identity());
  g.addBody("body2", pinocchio::Inertia(0., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero()));
  g.addBody("body3", pinocchio::Inertia(0., pinocchio::Inertia::Vector3(0., -3, -2.), pinocchio::Symmetric3::Zero()));


  g.addJoint(
    "body1_to_body2", pinocchio::internal::JointRevoluteGraph("body1_to_body2", Eigen::Vector3d::UnitZ()), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)));
  g.addJoint(
    "body2_to_body3", pinocchio::internal::JointRevoluteGraph("body2_to_body3", Eigen::Vector3d::UnitX()), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., -2., 0.)), "body3",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4., -3., 0.)));

  g.buildModel(
    "body1",
    pinocchio::JointModelFreeFlyerTpl<pinocchio::context::Scalar, pinocchio::context::Options>());
  g.buildModel(
    "body3",
    pinocchio::JointModelFreeFlyerTpl<pinocchio::context::Scalar, pinocchio::context::Options>());
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
  //     pinocchio::JointModelFreeFlyerTpl<pinocchio::context::Scalar, pinocchio::context::Options>());

  //   g.buildModel(
  //     "left_leg",
  //     pinocchio::JointModelFreeFlyerTpl<pinocchio::context::Scalar, pinocchio::context::Options>());
}

BOOST_AUTO_TEST_SUITE_END()
