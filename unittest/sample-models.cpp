//
// Copyright (c) 2015-2018 CNRS
//

#include <iostream>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/variant.hpp>

// Helper functions to map reduced to full model
Eigen::VectorXd
toFull(const Eigen::VectorXd & vec, int mimPrim, int mimSec, double scaling, double offset)
{
  Eigen::VectorXd vecFull(vec.size() + 1);
  vecFull << vec.head(mimSec), scaling * vec[mimPrim] + offset, vec.tail(vec.size() - mimSec);
  return vecFull;
}

Eigen::MatrixXd create_G(const pinocchio::Model & model, const pinocchio::Model & modelMimic)
{
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(model.nv, modelMimic.nv);
  for (int i = 0; i < modelMimic.nv; ++i)
    G(i, i) = 1;

  return G;
}

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(build_model_sample_humanoid_random)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);

  BOOST_CHECK(model.nq == 33);
  BOOST_CHECK(model.nv == 32);

  pinocchio::Model modelff;
  pinocchio::buildModels::humanoidRandom(modelff, false);

  BOOST_CHECK(modelff.nq == 32);
  BOOST_CHECK(modelff.nv == 32);

  pinocchio::Model modelMimic;
  pinocchio::buildModels::humanoidRandom(modelMimic, true, true);

  BOOST_CHECK(modelMimic.nq == 32);
  BOOST_CHECK(modelMimic.nv == 31);

  pinocchio::Model modelMimicff;
  pinocchio::buildModels::humanoidRandom(modelMimicff, false, true);

  BOOST_CHECK(modelMimicff.nq == 31);
  BOOST_CHECK(modelMimicff.nv == 31);
}

BOOST_AUTO_TEST_CASE(build_model_sample_manipulator)
{
  pinocchio::Model model;
  pinocchio::buildModels::manipulator(model, true);

  BOOST_CHECK(model.nq == 5);
  BOOST_CHECK(model.nv == 5);

#ifdef PINOCCHIO_WITH_HPP_FCL
  pinocchio::Data data(model);
  pinocchio::GeometryModel geom;
  pinocchio::buildModels::manipulatorGeometries(model, geom);
#endif
}

BOOST_AUTO_TEST_CASE(build_model_sample_humanoid)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoid(model);
  pinocchio::Data data(model);

  BOOST_CHECK(model.nq == 35);
  BOOST_CHECK(model.nv == 34);

#ifdef PINOCCHIO_WITH_HPP_FCL
  pinocchio::GeometryModel geom;
  pinocchio::buildModels::humanoidGeometries(model, geom);
  pinocchio::GeometryData geomdata(geom);

  Eigen::VectorXd q = pinocchio::neutral(model);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateGeometryPlacements(model, data, geom, geomdata, q);
#endif

  /* We might want to check here the joint namings, and validate the
   * direct geometry with respect to reference values. */
}

BOOST_AUTO_TEST_CASE(compare_mimic)
{
  pinocchio::Model model, model_m;
  pinocchio::buildModels::manipulator(model);
  pinocchio::buildModels::manipulator(model_m, true);

  // Reduced model configuration (with the mimic joint)
  Eigen::VectorXd q_reduced = pinocchio::neutral(model_m);
  Eigen::VectorXd v_reduced = Eigen::VectorXd::Random(model_m.nv);
  Eigen::VectorXd a_reduced = Eigen::VectorXd::Random(model_m.nv);
  Eigen::VectorXd tau_reduced = Eigen::VectorXd::Random(model_m.nv);

  auto jointMimic = boost::get<pinocchio::JointModelMimic>(
    model_m.joints[model_m.getJointId("wrist1_mimic_joint")]);
  double scaling = jointMimic.scaling();
  double offset = jointMimic.offset();
  // Reduced model configuration (without the mimic joint)
  Eigen::VectorXd q_full = toFull(q_reduced, 4, 5, scaling, offset);
  Eigen::VectorXd v_full = toFull(v_reduced, 4, 5, scaling, 0);
  Eigen::VectorXd a_full = toFull(a_reduced, 4, 5, scaling, 0);

  pinocchio::Data dataFKFull(model);
  pinocchio::forwardKinematics(model, dataFKFull, q_full, v_full, a_full);

  pinocchio::Data dataFKRed(model_m);
  pinocchio::forwardKinematics(model_m, dataFKRed, q_reduced, v_reduced, a_reduced);

  for (int i = 0; i < model.njoints; i++)
  {
    BOOST_CHECK(dataFKRed.oMi[i].isApprox(dataFKFull.oMi[i]));
    BOOST_CHECK(dataFKRed.liMi[i].isApprox(dataFKFull.liMi[i]));
    BOOST_CHECK(model.inertias[i].isApprox(model_m.inertias[i]));
  }

  // Build G matrix to go from full to reduced system
  Eigen::MatrixXd G = create_G(model, model_m);
  G(model.nv - 1, model_m.nv - 1) = scaling;

  // Test crba
  pinocchio::Data dataCRBAMimic(model_m);
  pinocchio::crba(model_m, dataCRBAMimic, q_reduced);
  dataCRBAMimic.M.triangularView<Eigen::StrictlyLower>() =
    dataCRBAMimic.M.transpose().triangularView<Eigen::StrictlyLower>();

  pinocchio::Data dataCRBAFull(model);
  pinocchio::crba(model, dataCRBAFull, q_full);
  dataCRBAFull.M.triangularView<Eigen::StrictlyLower>() =
    dataCRBAFull.M.transpose().triangularView<Eigen::StrictlyLower>();

  Eigen::MatrixXd M_reduced_computed = (G.transpose() * dataCRBAFull.M * G);

  BOOST_CHECK((M_reduced_computed - dataCRBAMimic.M).isZero());

  // Test rnea
  pinocchio::Data dataBiasFull(model);
  pinocchio::nonLinearEffects(model, dataBiasFull, q_full, v_full);
  Eigen::VectorXd C_full = dataBiasFull.nle;

  // // Use equation of motion to compute tau from a_reduced
  Eigen::VectorXd tau_reduced_computed = M_reduced_computed * a_reduced + (G.transpose() * C_full);

  pinocchio::Data dataRneaRed(model_m);
  pinocchio::rnea(model_m, dataRneaRed, q_reduced, v_reduced, a_reduced);

  BOOST_CHECK((dataRneaRed.tau - tau_reduced_computed).isZero());
}

BOOST_AUTO_TEST_SUITE_END()
