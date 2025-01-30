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
  pinocchio::buildModels::manipulator(model);

  BOOST_CHECK(model.nq == 6);
  BOOST_CHECK(model.nv == 6);

  pinocchio::Model model_m;
  pinocchio::buildModels::manipulator(model_m, true);

  BOOST_CHECK(model_m.nq == 5);
  BOOST_CHECK(model_m.nv == 5);

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

BOOST_AUTO_TEST_SUITE_END()
