//
// Copyright (c) 2021 INRIA
//

#include "pinocchio/codegen/cppadcg.hpp"
#include "pinocchio/codegen/code-generator-algo.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_code_generation)
{
  typedef double Scalar;
  typedef CppAD::cg::CG<Scalar> CGScalar;
  typedef CppAD::AD<CGScalar> ADScalar;

  typedef Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ADVector;

  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef Model::Data Data;

  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  typedef ADModel::Data ADData;

  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  Data data(model);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_6D3D;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas_6D3D;

  RigidConstraintModel ci_RF(CONTACT_6D, model.getJointId(RF), LOCAL);
  contact_models_6D3D.push_back(ci_RF);
  contact_datas_6D3D.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_3D, model.getJointId(LF), LOCAL);
  contact_models_6D3D.push_back(ci_LF);
  contact_datas_6D3D.push_back(RigidConstraintData(ci_LF));

  Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);
  std::vector<Eigen::VectorXd> results_q(2, Eigen::VectorXd::Zero(model.nq));
  std::vector<Eigen::VectorXd> results_v(2, Eigen::VectorXd::Zero(model.nv));

  CodeGenConstraintDynamics<double> cg_constraintDynamics(model, contact_models_6D3D);
  cg_constraintDynamics.initLib();
  cg_constraintDynamics.loadLib();
  cg_constraintDynamics.evalFunction(q, v, tau);

  pinocchio::initConstraintDynamics(model, data, contact_models_6D3D);
  pinocchio::constraintDynamics(
    model, data, q, v, tau, contact_models_6D3D, contact_datas_6D3D, 0.);
  BOOST_CHECK(data.ddq.isApprox(cg_constraintDynamics.ddq));
  BOOST_CHECK(data.lambda_c.isApprox(cg_constraintDynamics.lambda_c));
}

BOOST_AUTO_TEST_SUITE_END()
