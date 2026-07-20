//
// Copyright (c) 2021 INRIA
//

#define BOOST_TEST_MODULE contact_dynamics

#include "pinocchio/codegen/cppadcg.hpp"
#include "pinocchio/codegen/cppadcg-algo.hpp"

#include "pinocchio/multibody.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_code_generation)
{
  typedef double Scalar;

  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef Model::Data Data;

  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  Data data(model);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  std::vector<RigidConstraintModel> contact_models_6D3D;
  std::vector<RigidConstraintData> contact_datas_6D3D;

  RigidConstraintModel ci_RF(CONTACT_6D, model, model.getJointId(RF), LOCAL);
  ci_RF.joint1_placement.setRandom();
  contact_models_6D3D.push_back(ci_RF);
  contact_datas_6D3D.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_3D, model, model.getJointId(LF), LOCAL);
  ci_LF.joint1_placement.setRandom();
  contact_models_6D3D.push_back(ci_LF);
  contact_datas_6D3D.push_back(RigidConstraintData(ci_LF));

  Eigen::VectorXd q = pinocchio::randomConfiguration(model);
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);
  std::vector<Eigen::VectorXd> results_q(2, Eigen::VectorXd::Zero(model.nq));
  std::vector<Eigen::VectorXd> results_v(2, Eigen::VectorXd::Zero(model.nv));

  CodeGenConstraintDynamics<double> cg_constraintDynamics(model, contact_models_6D3D);
  cg_constraintDynamics.initLib();
  cg_constraintDynamics.compileAndLoadLib(PINOCCHIO_CXX_COMPILER);
  cg_constraintDynamics.evalFunction(q, v, tau);

  pinocchio::initConstraintDynamics(model, data, contact_models_6D3D, contact_datas_6D3D);
  pinocchio::constraintDynamics(model, data, q, v, tau, contact_models_6D3D, contact_datas_6D3D);
  BOOST_CHECK(data.ddq.isApprox(cg_constraintDynamics.ddq));
  BOOST_CHECK(data.lambda_c.isApprox(cg_constraintDynamics.lambda_c));
}
