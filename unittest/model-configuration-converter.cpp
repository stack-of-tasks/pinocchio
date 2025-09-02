//
// Copyright (c) 2025 INRIA
//

#include <boost/test/unit_test.hpp>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/parsers/graph/model-configuration-converter.hpp"
#include "pinocchio/parsers/graph/joints.hpp"
#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/graph/model-graph-algo.hpp"

BOOST_AUTO_TEST_SUITE(ModelConfigurationConverter)

// TODO: factorize
/// function isApprox better to avoid problem with zero precision
bool SE3isApprox(
  const pinocchio::SE3 & s1,
  const pinocchio::SE3 & s2,
  const double prec = Eigen::NumTraits<double>::dummy_precision())
{
  return s1.rotation().isApprox(s2.rotation())
         && (s1.translation() - s2.translation()).isZero(prec);
}

template<typename MatrixLike1, typename MatrixLike2>
bool isApproxOrZero(
  const Eigen::MatrixBase<MatrixLike1> & mat1,
  const Eigen::MatrixBase<MatrixLike2> & mat2,
  const typename MatrixLike1::RealScalar & prec =
    Eigen::NumTraits<typename MatrixLike1::RealScalar>::dummy_precision())
{
  const bool mat1_is_zero = mat1.isZero(prec);
  const bool mat2_is_zero = mat2.isZero(prec);

  const bool mat1_is_approx_mat2 = mat1.isApprox(mat2, prec);
  return mat1_is_approx_mat2 || (mat1_is_zero && mat2_is_zero);
}

BOOST_AUTO_TEST_CASE(test_create_converter)
{
  // Create the following model:
  //      j3    j4
  //   b4----b3----b5
  //        | j2
  //        b2
  //        | j1 (unbounded)
  //        b1
  //
  // The model will be created from b1, b4 and b5 and will have the following joint order:
  // - b1:    j1,  j2, j3, j4
  // - b4:    j3,  j2, j1, j4
  // - b5:    j4,  j2, j1, j3
  // - b1_ff: jff, j1, j2, j3, j4
  //
  // We should have the following configuration and tangent vector:
  // - configuration:
  //   - b1:    [j1[0], j1[2], j2, j3, j4]
  //   - b4:    [j3, j2, j1[0], j1[2], j4]
  //   - b5:    [j4, j2, j1[0], j1[2], j3]
  //   - b1_ff: [jff[0..7], j1[0], j1[2], j2, j3, j4]
  // - tangent:
  //   - b1:    [j1, j2, j3, j4]
  //   - b4:    [j3, j2, j1, j4]
  //   - b5:    [j4, j2, j1, j3]
  //   - b1_ff: [jff[0..8], j1[0], j1[2], j2, j3, j4]

  pinocchio::graph::ModelGraph g;
  pinocchio::Inertia I_I(pinocchio::Inertia::Identity());
  pinocchio::SE3 X_I(pinocchio::SE3::Identity());

  g.addBody("b1", I_I);
  g.addBody("b2", I_I);
  g.addBody("b3", I_I);
  g.addBody("b4", I_I);
  g.addBody("b5", I_I);

  pinocchio::graph::JointRevolute joint(Eigen::Vector3d::UnitX());
  g.addJoint(
    "j1", pinocchio::graph::JointRevoluteUnbounded(Eigen::Vector3d::UnitX()), "b1", X_I, "b2", X_I);
  g.addJoint("j2", joint, "b2", X_I, "b3", X_I);
  g.addJoint("j3", joint, "b3", X_I, "b4", X_I);
  g.addJoint("j4", joint, "b3", X_I, "b5", X_I);

  auto b1_ret = pinocchio::graph::buildModelWithBuildInfo(g, "b1", X_I);
  auto b4_ret = pinocchio::graph::buildModelWithBuildInfo(g, "b4", X_I);
  auto b5_ret = pinocchio::graph::buildModelWithBuildInfo(g, "b5", X_I);
  auto b1_ff_ret = pinocchio::graph::buildModelWithBuildInfo(
    g, "b1", X_I, pinocchio::graph::JointFreeFlyer(), "ff");

  auto b1_to_b4_converter = pinocchio::graph::createConverter(
    b1_ret.model, b4_ret.model, b1_ret.build_info, b4_ret.build_info);
  auto b1_to_b5_converter = pinocchio::graph::createConverter(
    b1_ret.model, b5_ret.model, b1_ret.build_info, b5_ret.build_info);
  auto b4_to_b5_converter = pinocchio::graph::createConverter(
    b4_ret.model, b5_ret.model, b4_ret.build_info, b5_ret.build_info);
  auto b1_ff_to_b1_converter = pinocchio::graph::createConverter(
    b1_ff_ret.model, b1_ret.model, b1_ff_ret.build_info, b1_ret.build_info);

  // Test b1 to b4
  BOOST_REQUIRE_EQUAL(b1_to_b4_converter._configuration_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b1_to_b4_converter._tangent_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b1_to_b4_converter._joint_mapping.size(), 4);

  //   j1
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[0].nq, 2);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[0].idx_qs_source, 0);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[0].idx_qs_target, 2);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[0].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[0].idx_vs_source, 0);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[0].idx_vs_target, 2);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._joint_mapping[0].same_direction, false);

  //   j2
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[1].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[1].idx_qs_source, 2);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[1].idx_qs_target, 1);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[1].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[1].idx_vs_source, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[1].idx_vs_target, 1);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._joint_mapping[1].same_direction, false);

  //   j3
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[2].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[2].idx_qs_source, 3);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[2].idx_qs_target, 0);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[2].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[2].idx_vs_source, 2);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[2].idx_vs_target, 0);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._joint_mapping[2].same_direction, false);

  //   j4
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[3].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[3].idx_qs_source, 4);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[3].idx_qs_target, 4);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[3].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[3].idx_vs_source, 3);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[3].idx_vs_target, 3);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._joint_mapping[3].same_direction, true);

  // Test b1 to b5
  BOOST_REQUIRE_EQUAL(b1_to_b5_converter._configuration_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b1_to_b5_converter._tangent_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b1_to_b5_converter._joint_mapping.size(), 4);

  //   j1
  BOOST_CHECK_EQUAL(b1_to_b5_converter._configuration_mapping[0].nq, 2);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._configuration_mapping[0].idx_qs_source, 0);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._configuration_mapping[0].idx_qs_target, 2);

  BOOST_CHECK_EQUAL(b1_to_b5_converter._tangent_mapping[0].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._tangent_mapping[0].idx_vs_source, 0);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._tangent_mapping[0].idx_vs_target, 2);

  BOOST_CHECK_EQUAL(b1_to_b5_converter._joint_mapping[0].same_direction, false);

  //   j2
  BOOST_CHECK_EQUAL(b1_to_b5_converter._configuration_mapping[1].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._configuration_mapping[1].idx_qs_source, 2);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._configuration_mapping[1].idx_qs_target, 1);

  BOOST_CHECK_EQUAL(b1_to_b5_converter._tangent_mapping[1].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._tangent_mapping[1].idx_vs_source, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._tangent_mapping[1].idx_vs_target, 1);

  BOOST_CHECK_EQUAL(b1_to_b5_converter._joint_mapping[1].same_direction, false);

  //   j3
  BOOST_CHECK_EQUAL(b1_to_b5_converter._configuration_mapping[2].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._configuration_mapping[2].idx_qs_source, 3);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._configuration_mapping[2].idx_qs_target, 4);

  BOOST_CHECK_EQUAL(b1_to_b5_converter._tangent_mapping[2].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._tangent_mapping[2].idx_vs_source, 2);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._tangent_mapping[2].idx_vs_target, 3);

  BOOST_CHECK_EQUAL(b1_to_b5_converter._joint_mapping[2].same_direction, true);

  //   j4
  BOOST_CHECK_EQUAL(b1_to_b5_converter._configuration_mapping[3].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._configuration_mapping[3].idx_qs_source, 4);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._configuration_mapping[3].idx_qs_target, 0);

  BOOST_CHECK_EQUAL(b1_to_b5_converter._tangent_mapping[3].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._tangent_mapping[3].idx_vs_source, 3);
  BOOST_CHECK_EQUAL(b1_to_b5_converter._tangent_mapping[3].idx_vs_target, 0);

  BOOST_CHECK_EQUAL(b1_to_b5_converter._joint_mapping[3].same_direction, false);

  // Test b4 to b5
  BOOST_REQUIRE_EQUAL(b4_to_b5_converter._configuration_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b4_to_b5_converter._tangent_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b4_to_b5_converter._joint_mapping.size(), 4);

  //   j3
  BOOST_CHECK_EQUAL(b4_to_b5_converter._configuration_mapping[0].nq, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._configuration_mapping[0].idx_qs_source, 0);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._configuration_mapping[0].idx_qs_target, 4);

  BOOST_CHECK_EQUAL(b4_to_b5_converter._tangent_mapping[0].nv, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._tangent_mapping[0].idx_vs_source, 0);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._tangent_mapping[0].idx_vs_target, 3);

  BOOST_CHECK_EQUAL(b4_to_b5_converter._joint_mapping[0].same_direction, false);

  //   j2
  BOOST_CHECK_EQUAL(b4_to_b5_converter._configuration_mapping[1].nq, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._configuration_mapping[1].idx_qs_source, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._configuration_mapping[1].idx_qs_target, 1);

  BOOST_CHECK_EQUAL(b4_to_b5_converter._tangent_mapping[1].nv, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._tangent_mapping[1].idx_vs_source, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._tangent_mapping[1].idx_vs_target, 1);

  BOOST_CHECK_EQUAL(b4_to_b5_converter._joint_mapping[1].same_direction, true);

  //   j1
  BOOST_CHECK_EQUAL(b4_to_b5_converter._configuration_mapping[2].nq, 2);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._configuration_mapping[2].idx_qs_source, 2);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._configuration_mapping[2].idx_qs_target, 2);

  BOOST_CHECK_EQUAL(b4_to_b5_converter._tangent_mapping[2].nv, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._tangent_mapping[2].idx_vs_source, 2);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._tangent_mapping[2].idx_vs_target, 2);

  BOOST_CHECK_EQUAL(b4_to_b5_converter._joint_mapping[2].same_direction, true);

  //   j4
  BOOST_CHECK_EQUAL(b4_to_b5_converter._configuration_mapping[3].nq, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._configuration_mapping[3].idx_qs_source, 4);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._configuration_mapping[3].idx_qs_target, 0);

  BOOST_CHECK_EQUAL(b4_to_b5_converter._tangent_mapping[3].nv, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._tangent_mapping[3].idx_vs_source, 3);
  BOOST_CHECK_EQUAL(b4_to_b5_converter._tangent_mapping[3].idx_vs_target, 0);

  BOOST_CHECK_EQUAL(b4_to_b5_converter._joint_mapping[3].same_direction, false);

  // Test b1_ff to b1
  BOOST_REQUIRE_EQUAL(b1_ff_to_b1_converter._configuration_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b1_ff_to_b1_converter._tangent_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b1_ff_to_b1_converter._joint_mapping.size(), 4);

  //   j1
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._configuration_mapping[0].nq, 2);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._configuration_mapping[0].idx_qs_source, 7);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._configuration_mapping[0].idx_qs_target, 0);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._tangent_mapping[0].nv, 1);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._tangent_mapping[0].idx_vs_source, 6);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._tangent_mapping[0].idx_vs_target, 0);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._joint_mapping[0].same_direction, true);

  //   j2
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._configuration_mapping[1].nq, 1);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._configuration_mapping[1].idx_qs_source, 9);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._configuration_mapping[1].idx_qs_target, 2);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._tangent_mapping[1].nv, 1);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._tangent_mapping[1].idx_vs_source, 7);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._tangent_mapping[1].idx_vs_target, 1);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._joint_mapping[1].same_direction, true);

  //   j3
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._configuration_mapping[2].nq, 1);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._configuration_mapping[2].idx_qs_source, 10);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._configuration_mapping[2].idx_qs_target, 3);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._tangent_mapping[2].nv, 1);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._tangent_mapping[2].idx_vs_source, 8);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._tangent_mapping[2].idx_vs_target, 2);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._joint_mapping[2].same_direction, true);

  //   j4
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._configuration_mapping[3].nq, 1);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._configuration_mapping[3].idx_qs_source, 11);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._configuration_mapping[3].idx_qs_target, 4);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._tangent_mapping[3].nv, 1);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._tangent_mapping[3].idx_vs_source, 9);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._tangent_mapping[3].idx_vs_target, 3);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter._joint_mapping[3].same_direction, true);
}

BOOST_AUTO_TEST_CASE(test_create_converter_composite)
{
  // Create the following model:
  //     j1      j2      j3
  // b1 ---- b2 ---- b3 ---- b4
  //
  // With j2 the following composite joint:
  //
  // revolute    unbounded
  //   j2_1 ------ j2_2
  //
  // The model will be created from b1 and b4
  // - b1: j1, j2, j3
  // - b4: j3, j2, j1
  //
  // We should have the following configuration and tangent vector:
  // - configuration:
  //   - b1: [j1, j2_1, j2_2[0], j2_2[1], j3]
  //   - b4: [j3, j2_2[0], j2_2[1], j2_1, j1]
  // - tangent:
  //   - b1: [j1, j2_1, j2_2, j3]
  //   - b4: [j3, j2_2, j2_1, j1]

  pinocchio::graph::ModelGraph g;
  pinocchio::Inertia I_I(pinocchio::Inertia::Identity());
  pinocchio::SE3 X_I(pinocchio::SE3::Identity());

  g.addBody("b1", I_I);
  g.addBody("b2", I_I);
  g.addBody("b3", I_I);
  g.addBody("b4", I_I);

  pinocchio::graph::JointRevolute joint(Eigen::Vector3d::UnitX());
  pinocchio::graph::JointComposite composite;
  composite.addJoint(joint);
  composite.addJoint(pinocchio::graph::JointRevoluteUnbounded(Eigen::Vector3d::UnitX()));
  g.addJoint("j1", joint, "b1", X_I, "b2", X_I);
  g.addJoint("j2", composite, "b2", X_I, "b3", X_I);
  g.addJoint("j3", joint, "b3", X_I, "b4", X_I);

  auto b1_ret = pinocchio::graph::buildModelWithBuildInfo(g, "b1", X_I);
  auto b4_ret = pinocchio::graph::buildModelWithBuildInfo(g, "b4", X_I);

  auto b1_to_b4_converter = pinocchio::graph::createConverter(
    b1_ret.model, b4_ret.model, b1_ret.build_info, b4_ret.build_info);

  // Test b1 to b4
  BOOST_REQUIRE_EQUAL(b1_to_b4_converter._configuration_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b1_to_b4_converter._tangent_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b1_to_b4_converter._joint_mapping.size(), 4);

  //   j1
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[0].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[0].idx_qs_source, 0);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[0].idx_qs_target, 4);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[0].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[0].idx_vs_source, 0);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[0].idx_vs_target, 3);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._joint_mapping[0].same_direction, false);

  //   j2_1
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[1].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[1].idx_qs_source, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[1].idx_qs_target, 3);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[1].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[1].idx_vs_source, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[1].idx_vs_target, 2);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._joint_mapping[1].same_direction, false);

  //   j2_2
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[2].nq, 2);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[2].idx_qs_source, 2);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[2].idx_qs_target, 1);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[2].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[2].idx_vs_source, 2);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[2].idx_vs_target, 1);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._joint_mapping[2].same_direction, false);

  //   j3
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[3].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[3].idx_qs_source, 4);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._configuration_mapping[3].idx_qs_target, 0);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[3].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[3].idx_vs_source, 3);
  BOOST_CHECK_EQUAL(b1_to_b4_converter._tangent_mapping[3].idx_vs_target, 0);

  BOOST_CHECK_EQUAL(b1_to_b4_converter._joint_mapping[3].same_direction, false);
}

BOOST_AUTO_TEST_CASE(test_convert_configuration)
{
  pinocchio::graph::ModelGraph g;
  pinocchio::Inertia I_I(pinocchio::Inertia::Identity());
  pinocchio::SE3 X_I(pinocchio::SE3::Identity());

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
  // We can't test mimic joint because backward construction is not supported
  g.addJoint(
    "j1", pinocchio::graph::JointRevolute(Eigen::Vector3d::UnitX()), "b1", pinocchio::SE3::Random(),
    "b2", pinocchio::SE3::Random());
  g.addJoint(
    "j2", pinocchio::graph::JointFreeFlyer(), "b2", pinocchio::SE3::Random(), "b3",
    pinocchio::SE3::Random());
  g.addJoint(
    "j3", pinocchio::graph::JointSpherical(), "b3", pinocchio::SE3::Random(), "b4",
    pinocchio::SE3::Random());
  g.addJoint(
    "j4", pinocchio::graph::JointUniversal(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()),
    "b4", pinocchio::SE3::Random(), "b5", pinocchio::SE3::Random());
  g.addJoint(
    "j5", pinocchio::graph::JointRevoluteUnbounded(Eigen::Vector3d::UnitX()), "b5",
    pinocchio::SE3::Random(), "b6", pinocchio::SE3::Random());
  g.addJoint(
    "j6", pinocchio::graph::JointPrismatic(Eigen::Vector3d::UnitX()), "b6",
    pinocchio::SE3::Random(), "b7", pinocchio::SE3::Random());
  g.addJoint(
    "j7", pinocchio::graph::JointHelical(Eigen::Vector3d::UnitX(), 0.1), "b7",
    pinocchio::SE3::Random(), "b8", pinocchio::SE3::Random());
  g.addJoint(
    "j8", pinocchio::graph::JointTranslation(), "b8", pinocchio::SE3::Random(), "b9",
    pinocchio::SE3::Random());
  g.addJoint(
    "j9", pinocchio::graph::JointSphericalZYX(), "b9", pinocchio::SE3::Random(), "b10",
    pinocchio::SE3::Random());
  g.addJoint(
    "j10", pinocchio::graph::JointPlanar(), "b10", pinocchio::SE3::Random(), "b11",
    pinocchio::SE3::Random());

  pinocchio::graph::JointComposite joint_composite;
  joint_composite.addJoint(
    pinocchio::graph::JointRevolute(Eigen::Vector3d::UnitX()), pinocchio::SE3::Random());
  joint_composite.addJoint(pinocchio::graph::JointSpherical(), pinocchio::SE3::Random());
  joint_composite.addJoint(
    pinocchio::graph::JointRevoluteUnbounded(Eigen::Vector3d::UnitX()), pinocchio::SE3::Random());
  g.addJoint(
    "j11", joint_composite, "b11", pinocchio::SE3::Random(), "b12", pinocchio::SE3::Random());

  const auto model_a_ret = pinocchio::graph::buildModelWithBuildInfo(g, "b1", X_I);
  const auto model_a = model_a_ret.model;
  pinocchio::Data data_a(model_a);
  const Eigen::VectorXd qmax = Eigen::VectorXd::Ones(model_a.nq);
  const Eigen::VectorXd q_a = pinocchio::randomConfiguration(model_a, -qmax, qmax);
  pinocchio::framesForwardKinematics(model_a, data_a, q_a);

  // Check joint mapping and backward conversion
  {
    const auto model_b_ret = pinocchio::graph::buildModelWithBuildInfo(
      g, "b12", data_a.oMf[model_a.getFrameId("b12", pinocchio::BODY)]);
    const auto model_b = model_b_ret.model;
    pinocchio::Data data_b(model_b);
    Eigen::VectorXd q_b = pinocchio::neutral(model_b);
    auto a_to_b_converter = pinocchio::graph::createConverter(
      model_a, model_b, model_a_ret.build_info, model_b_ret.build_info);
    a_to_b_converter.convertConfigurationVector(q_a, q_b);
    pinocchio::framesForwardKinematics(model_b, data_b, q_b);
    for (std::size_t i = 0; i < model_a.frames.size(); ++i)
    {
      const auto & frame = model_a.frames[i];
      if (frame.type == pinocchio::FrameType::BODY)
      {
        BOOST_CHECK(
          SE3isApprox(data_a.oMf[i], data_b.oMf[model_b.getFrameId(frame.name, frame.type)]));
      }
    }
  }

  // Check forward conversion
  {
    pinocchio::Data data_a2(model_a);
    Eigen::VectorXd q_a2 = pinocchio::neutral(model_a);
    auto a_to_a_converter = pinocchio::graph::createConverter(
      model_a, model_a, model_a_ret.build_info, model_a_ret.build_info);
    a_to_a_converter.convertConfigurationVector(q_a, q_a2);
    pinocchio::framesForwardKinematics(model_a, data_a2, q_a2);
    for (std::size_t i = 0; i < model_a.frames.size(); ++i)
    {
      const auto & frame = model_a.frames[i];
      if (frame.type == pinocchio::FrameType::BODY)
      {
        BOOST_CHECK(SE3isApprox(data_a.oMf[i], data_a2.oMf[i]));
      }
    }
  }

  // Check forward conversion with custom root joint
  {
    const auto model_a_ff_ret = pinocchio::graph::buildModelWithBuildInfo(
      g, "b1", X_I, pinocchio::graph::JointFreeFlyer(), "ff");
    const auto model_a_ff = model_a_ff_ret.model;
    pinocchio::Data data_a_ff(model_a_ff);
    const Eigen::VectorXd qmax_ff = Eigen::VectorXd::Ones(model_a_ff.nq);
    Eigen::VectorXd q_a_ff = pinocchio::randomConfiguration(model_a_ff, -qmax_ff, qmax_ff);
    q_a_ff.head<7>() << 0., 0., 0., 0., 0., 0., 1.;
    pinocchio::framesForwardKinematics(model_a_ff, data_a_ff, q_a_ff);
    auto a_ff_to_a_converter = pinocchio::graph::createConverter(
      model_a_ff, model_a, model_a_ff_ret.build_info, model_a_ret.build_info);
    a_ff_to_a_converter.convertConfigurationVector(q_a_ff, q_a);
    pinocchio::framesForwardKinematics(model_a, data_a, q_a);
    for (std::size_t i = 0; i < model_a.frames.size(); ++i)
    {
      const auto & frame = model_a.frames[i];
      if (frame.type == pinocchio::FrameType::BODY)
      {
        BOOST_CHECK(
          SE3isApprox(data_a.oMf[i], data_a_ff.oMf[model_a_ff.getFrameId(frame.name, frame.type)]));
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(test_convert_tangent)
{
  pinocchio::graph::ModelGraph g;
  pinocchio::Inertia I_I(pinocchio::Inertia::Identity());
  pinocchio::SE3 X_I(pinocchio::SE3::Identity());

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
  // We can't test mimic joint because backward construction is not supported
  g.addJoint(
    "j1", pinocchio::graph::JointRevolute(Eigen::Vector3d::UnitX()), "b1", pinocchio::SE3::Random(),
    "b2", pinocchio::SE3::Random());
  g.addJoint(
    "j2", pinocchio::graph::JointFreeFlyer(), "b2", pinocchio::SE3::Random(), "b3",
    pinocchio::SE3::Random());
  g.addJoint(
    "j3", pinocchio::graph::JointSpherical(), "b3", pinocchio::SE3::Random(), "b4",
    pinocchio::SE3::Random());
  g.addJoint(
    "j4", pinocchio::graph::JointUniversal(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()),
    "b4", pinocchio::SE3::Random(), "b5", pinocchio::SE3::Random());
  g.addJoint(
    "j5", pinocchio::graph::JointRevoluteUnbounded(Eigen::Vector3d::UnitX()), "b5",
    pinocchio::SE3::Random(), "b6", pinocchio::SE3::Random());
  g.addJoint(
    "j6", pinocchio::graph::JointPrismatic(Eigen::Vector3d::UnitX()), "b6",
    pinocchio::SE3::Random(), "b7", pinocchio::SE3::Random());
  g.addJoint(
    "j7", pinocchio::graph::JointHelical(Eigen::Vector3d::UnitX(), 0.1), "b7",
    pinocchio::SE3::Random(), "b8", pinocchio::SE3::Random());
  g.addJoint(
    "j8", pinocchio::graph::JointTranslation(), "b8", pinocchio::SE3::Random(), "b9",
    pinocchio::SE3::Random());
  g.addJoint(
    "j9", pinocchio::graph::JointSphericalZYX(), "b9", pinocchio::SE3::Random(), "b10",
    pinocchio::SE3::Random());
  g.addJoint(
    "j10", pinocchio::graph::JointPlanar(), "b10", pinocchio::SE3::Random(), "b11",
    pinocchio::SE3::Random());

  pinocchio::graph::JointComposite joint_composite;
  joint_composite.addJoint(
    pinocchio::graph::JointRevolute(Eigen::Vector3d::UnitX()), pinocchio::SE3::Random());
  joint_composite.addJoint(pinocchio::graph::JointSpherical(), pinocchio::SE3::Random());
  joint_composite.addJoint(
    pinocchio::graph::JointRevoluteUnbounded(Eigen::Vector3d::UnitX()), pinocchio::SE3::Random());
  g.addJoint(
    "j11", joint_composite, "b11", pinocchio::SE3::Random(), "b12", pinocchio::SE3::Random());

  const auto model_a_ret = pinocchio::graph::buildModelWithBuildInfo(g, "b1", X_I);
  const auto model_a = model_a_ret.model;
  pinocchio::Data data_a(model_a);
  const Eigen::VectorXd qmax = Eigen::VectorXd::Ones(model_a.nq);
  const Eigen::VectorXd q_a = pinocchio::randomConfiguration(model_a, -qmax, qmax);
  const Eigen::VectorXd v_a(Eigen::VectorXd::Random(model_a.nv));
  pinocchio::forwardKinematics(model_a, data_a, q_a, v_a);
  pinocchio::updateFramePlacements(model_a, data_a);

  // Check joint mapping and backward conversion
  {
    // To have the same velocity between frame in model_a and model_b
    // We create model_b with a freeflyer base link and set his velocity to
    // model_a end effector velocity.
    const std::string end_effector = "b12";
    const auto end_effector_frame_id = model_a.getFrameId(end_effector, pinocchio::BODY);
    const auto model_b_ret = pinocchio::graph::buildModelWithBuildInfo(
      g, end_effector, data_a.oMf[end_effector_frame_id], pinocchio::graph::JointFreeFlyer());
    const auto model_b = model_b_ret.model;
    pinocchio::Data data_b(model_b);
    Eigen::VectorXd q_b = pinocchio::neutral(model_b);
    Eigen::VectorXd v_b(Eigen::VectorXd::Zero(model_b.nv));
    v_b.segment<6>(0) =
      pinocchio::getFrameVelocity(model_a, data_a, end_effector_frame_id).toVector();

    auto a_to_b_converter = pinocchio::graph::createConverter(
      model_a, model_b, model_a_ret.build_info, model_b_ret.build_info);
    a_to_b_converter.convertConfigurationVector(q_a, q_b);
    a_to_b_converter.convertTangentVector(q_a, v_a, v_b);
    pinocchio::forwardKinematics(model_b, data_b, q_b, v_b);
    pinocchio::updateFramePlacements(model_b, data_b);
    for (std::size_t i = 0; i < model_a.frames.size(); ++i)
    {
      const auto & frame = model_a.frames[i];
      if (frame.type == pinocchio::FrameType::BODY)
      {
        auto i_b = model_b.getFrameId(frame.name, frame.type);
        auto motion_a = pinocchio::getFrameVelocity(model_a, data_a, i);
        auto motion_b = pinocchio::getFrameVelocity(model_b, data_b, i_b);

        BOOST_CHECK(SE3isApprox(data_a.oMf[i], data_b.oMf[i_b]));
        BOOST_CHECK(isApproxOrZero(motion_a.toVector(), motion_b.toVector()));
      }
    }
  }

  // Check forward conversion
  {
    pinocchio::Data data_a2(model_a);
    Eigen::VectorXd q_a2 = pinocchio::neutral(model_a);
    Eigen::VectorXd v_a2(Eigen::VectorXd::Zero(model_a.nv));
    auto a_to_a_converter = pinocchio::graph::createConverter(
      model_a, model_a, model_a_ret.build_info, model_a_ret.build_info);
    a_to_a_converter.convertConfigurationVector(q_a, q_a2);
    a_to_a_converter.convertTangentVector(q_a, v_a, v_a2);
    pinocchio::forwardKinematics(model_a, data_a2, q_a2, v_a2);
    pinocchio::updateFramePlacements(model_a, data_a2);
    for (std::size_t i = 0; i < model_a.frames.size(); ++i)
    {
      const auto & frame = model_a.frames[i];
      if (frame.type == pinocchio::FrameType::BODY)
      {
        auto motion_a = pinocchio::getFrameVelocity(model_a, data_a, i);
        auto motion_a2 = pinocchio::getFrameVelocity(model_a, data_a2, i);

        BOOST_CHECK(isApproxOrZero(motion_a.toVector(), motion_a2.toVector()));
      }
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
