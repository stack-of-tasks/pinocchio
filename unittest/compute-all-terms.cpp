//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

void run_test(const Model & model, const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  pinocchio::Data data(model);
  pinocchio::Data data_other(model);

  computeAllTerms(model, data, q, v);

  nonLinearEffects(model, data_other, q, v);
  crba(model, data_other, q, Convention::WORLD);
  getJacobianComFromCrba(model, data_other);
  computeJointJacobiansTimeVariation(model, data_other, q, v);
  centerOfMass(model, data_other, q, v, true);
  computeKineticEnergy(model, data_other, q, v);
  computePotentialEnergy(model, data_other, q);
  dccrba(model, data_other, q, v);
  computeGeneralizedGravity(model, data_other, q);

  BOOST_CHECK(data.nle.isApprox(data_other.nle));
  BOOST_CHECK(Eigen::MatrixXd(data.M.triangularView<Eigen::Upper>())
                .isApprox(Eigen::MatrixXd(data_other.M.triangularView<Eigen::Upper>())));
  BOOST_CHECK(data.J.isApprox(data_other.J));
  BOOST_CHECK(data.dJ.isApprox(data_other.dJ));
  BOOST_CHECK(data.Jcom.isApprox(data_other.Jcom));
  BOOST_CHECK(data.Ag.isApprox(data_other.Ag));
  BOOST_CHECK(data.dAg.isApprox(data_other.dAg));
  BOOST_CHECK(data.hg.isApprox(data_other.hg));
  BOOST_CHECK(data.Ig.isApprox(data_other.Ig));
  BOOST_CHECK(data.g.isApprox(data_other.g));

  for (int k = 0; k < model.njoints; ++k)
  {
    BOOST_CHECK(data.com[(size_t)k].isApprox(data_other.com[(size_t)k]));
    BOOST_CHECK(data.vcom[(size_t)k].isApprox(data_other.vcom[(size_t)k]));
    BOOST_CHECK_CLOSE(data.mass[(size_t)k], data_other.mass[(size_t)k], 1e-12);
  }

  BOOST_CHECK_CLOSE(data.kinetic_energy, data_other.kinetic_energy, 1e-12);
  BOOST_CHECK_CLOSE(data.potential_energy, data_other.potential_energy, 1e-12);
}

BOOST_AUTO_TEST_CASE(test_against_algo)
{
  using namespace Eigen;

  pinocchio::Model model;
  buildModels::humanoidRandom(model);

  VectorXd q(VectorXd::Random(model.nq));
  VectorXd v(VectorXd::Random(model.nv));

  // -------
  q.setZero();
  v.setZero();

  run_test(model, q, v);

  // -------
  q.setZero();
  v.setOnes();

  run_test(model, q, v);

  // -------
  q.setOnes();
  q.segment<4>(3).normalize();
  v.setOnes();

  run_test(model, q, v);

  // -------
  q.setRandom();
  q.segment<4>(3).normalize();
  v.setRandom();

  run_test(model, q, v);
}

BOOST_AUTO_TEST_SUITE_END()
