//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/impulse-dynamics.hpp"
#include "pinocchio/algorithm/impulse-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

using namespace Eigen;
using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_sparse_impulse_dynamics_derivatives_no_contact)
{
  //result: (dMdq)(dqafter-v) = drnea(q,0,dqafter-v)
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  
  // Contact models and data
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) empty_contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) empty_contact_data;

  const Eigen::DenseIndex constraint_dim = 0;
  const double mu0 = 0.;
  const double r_coeff = 0.;
  
  initContactDynamics(model,data,empty_contact_models);
  impulseDynamics(model,data,q,v,empty_contact_models,empty_contact_data,r_coeff, mu0);

  const Eigen::VectorXd dv = data.dq_after-v;
  
  data.M.triangularView<Eigen::StrictlyLower>()
    = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeImpulseDynamicsDerivatives(model,data,empty_contact_models,empty_contact_data,mu0);

  Motion gravity_bk = model.gravity;
  model.gravity.setZero();
  computeRNEADerivatives(model, data_ref, q, Eigen::VectorXd::Zero(model.nv), dv);
  // Reference values
  BOOST_CHECK(data_ref.ddq_dq.isApprox(data.ddq_dq));
}

BOOST_AUTO_TEST_SUITE_END ()
