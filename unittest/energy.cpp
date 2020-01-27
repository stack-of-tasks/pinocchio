//
// Copyright (c) 2016-2020 CNRS INRIA
//

#include "pinocchio/algorithm/energy.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"

#include "pinocchio/parsers/sample-models.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include <boost/test/floating_point_comparison.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE(test_kinetic_energy)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  pinocchio::Data data(model);
  
  const VectorXd qmax = VectorXd::Ones(model.nq);
  VectorXd q = randomConfiguration(model,-qmax,qmax);
  VectorXd v = VectorXd::Ones(model.nv);

  data.M.fill(0);  crba(model,data,q);
  data.M.triangularView<Eigen::StrictlyLower>()
  = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  double kinetic_energy_ref = 0.5 * v.transpose() * data.M * v;
  double kinetic_energy = computeKineticEnergy(model, data, q, v);
  
  BOOST_CHECK_SMALL(kinetic_energy_ref - kinetic_energy, 1e-12);
}

BOOST_AUTO_TEST_CASE(test_potential_energy)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  pinocchio::Data data(model), data_ref(model);
  
  const VectorXd qmax = VectorXd::Ones(model.nq);
  VectorXd q = randomConfiguration(model,-qmax,qmax);
  
  double potential_energy = computePotentialEnergy(model, data, q);
  centerOfMass(model,data_ref,q);
  
  double potential_energy_ref = -data_ref.mass[0] * (data_ref.com[0].dot(model.gravity.linear()));
  
  BOOST_CHECK_SMALL(potential_energy_ref - potential_energy, 1e-12);
}

BOOST_AUTO_TEST_SUITE_END()
