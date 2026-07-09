//
// Copyright (c) 2026 INRIA
//

#define BOOST_TEST_MODULE serialization_algorithm

#include <iostream>

#include "pinocchio/multibody.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/delassus-operator.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/constraints.hpp"

#include "pinocchio/serialization.hpp"
#include "serialization.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(test_delassus_operator_dense_serialization)
{
  using namespace pinocchio;

  // create model
  Model model;
  buildModels::manipulator(model);
  model.lowerPositionLimit.setConstant(-1.0);
  model.upperPositionLimit.setConstant(1.0);
  model.positionLimitMargin.setConstant(1.5);
  model.lowerDryFrictionLimit.setConstant(-1.0);
  model.upperDryFrictionLimit.setConstant(1.0);
  Data data(model);

  // setup data
  Eigen::VectorXd q0 = ::pinocchio::neutral(model);
  Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
  data.q_in = q0;
  aba(model, data, q0, v0, tau, Convention::WORLD);
  crba(model, data, q0, Convention::WORLD);

  // create constraints
  std::vector<ConstraintModel> constraint_models;
  std::vector<ConstraintData> constraint_datas;

  JointFrictionConstraintModel::JointIndexVector active_friction_idxs;
  JointFrictionConstraintModel::JointIndexVector active_limit_idxs;
  for (size_t i = 1; i < model.joints.size(); ++i)
  {
    const Model::JointModel & joint = model.joints[i];
    active_friction_idxs.push_back(joint.id());
    active_limit_idxs.push_back(joint.id());
  }
  JointFrictionConstraintModel joints_friction(model, active_friction_idxs);
  constraint_models.push_back(joints_friction);
  constraint_datas.push_back(joints_friction.createData());
  //
  JointLimitConstraintModel joints_limit(model, active_limit_idxs);
  constraint_models.push_back(joints_limit);
  constraint_datas.push_back(joints_limit.createData());

  for (size_t i = 0; i < constraint_models.size(); ++i)
  {
    ConstraintModel & cmodel = constraint_models[i];
    ConstraintData & cdata = constraint_datas[i];
    cmodel.calc(model, data, cdata);
  }

  // compute delassus
  ConstraintCholeskyDecomposition chol(model, data, constraint_models, constraint_datas);
  const double damping_val = 0.1234;
  chol.updateDamping(damping_val);
  chol.compute(model, data, constraint_models, constraint_datas);

  // check dense method
  DelassusOperatorDense delassus_operator_dense(chol.getDelassusOperatorCholeskyExpression());
  Eigen::MatrixXd damping_mat =
    damping_val
    * Eigen::MatrixXd::Identity(delassus_operator_dense.size(), delassus_operator_dense.size());
  BOOST_CHECK(delassus_operator_dense.getDamping().matrix().isApprox(damping_mat));

  // set random compliance
  Eigen::VectorXd compliance = Eigen::VectorXd::Random(delassus_operator_dense.size());
  compliance = compliance.cwiseAbs();
  delassus_operator_dense.updateCompliance(compliance);
  delassus_operator_dense.updateDecomposition();

  generic_test(
    delassus_operator_dense, TEST_SERIALIZATION_FOLDER "/DelassusOperatorDense",
    "DelassusOperatorDense");
}
