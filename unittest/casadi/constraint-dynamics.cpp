//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/autodiff/casadi.hpp"
#include "pinocchio/autodiff/casadi-algo.hpp"

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/multibody/sample-models.hpp"

#include <casadi/casadi.hpp>

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_constraintDynamics_casadi_algo)
{
  typedef double Scalar;
  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef pinocchio::DataTpl<Scalar> Data;
  typedef typename Model::ConfigVectorType ConfigVector;
  typedef typename Model::TangentVectorType TangentVector;

  const Scalar prec = Eigen::NumTraits<Scalar>::dummy_precision();

  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  Data data(model);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) contact_data;

  pinocchio::RigidConstraintModel ci_RF(
    pinocchio::CONTACT_3D, model, RF_id, pinocchio::LOCAL_WORLD_ALIGNED);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF);
  contact_data.push_back(pinocchio::RigidConstraintData(ci_RF));

  pinocchio::RigidConstraintModel ci_LF(
    pinocchio::CONTACT_6D, model, LF_id, pinocchio::LOCAL_WORLD_ALIGNED);
  ci_LF.joint1_placement.setRandom();
  contact_models.push_back(ci_LF);
  contact_data.push_back(pinocchio::RigidConstraintData(ci_LF));
  const double mu0 = 0.;
  ConfigVector q(model.nq);
  q = pinocchio::randomConfiguration(model);
  TangentVector v(TangentVector::Random(model.nv));
  TangentVector tau(TangentVector::Random(model.nv));

  pinocchio::initConstraintDynamics(model, data, contact_models);
  pinocchio::constraintDynamics(model, data, q, v, tau, contact_models, contact_data);
  pinocchio::computeConstraintDynamicsDerivatives(model, data, contact_models, contact_data);
  pinocchio::casadi::AutoDiffConstraintDynamics<Scalar> ad_casadi(model, contact_models);
  ad_casadi.initLib();
  ad_casadi.loadLib();

  ad_casadi.evalFunction(q, v, tau);
  BOOST_CHECK(ad_casadi.ddq.isApprox(data.ddq, 1e2 * prec));
  BOOST_CHECK(ad_casadi.lambda_c.isApprox(data.lambda_c, 1e2 * prec));
  ad_casadi.evalJacobian(q, v, tau);

  BOOST_CHECK(ad_casadi.ddq_dq.isApprox(data.ddq_dq, 1e2 * prec));
  BOOST_CHECK(ad_casadi.ddq_dv.isApprox(data.ddq_dv, 1e2 * prec));
  BOOST_CHECK(ad_casadi.ddq_dtau.isApprox(data.ddq_dtau, 1e2 * prec));
  BOOST_CHECK(ad_casadi.dlambda_dq.isApprox(data.dlambda_dq, 1e2 * prec));
  BOOST_CHECK(ad_casadi.dlambda_dv.isApprox(data.dlambda_dv, 1e2 * prec));
  BOOST_CHECK(ad_casadi.dlambda_dtau.isApprox(data.dlambda_dtau, 1e2 * prec));
}

BOOST_AUTO_TEST_SUITE_END()
