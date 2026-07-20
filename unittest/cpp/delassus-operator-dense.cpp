//
// Copyright (c) 2024-2026 INRIA
//

#define BOOST_TEST_MODULE delassus_operator_dense

#include <pinocchio/multibody/sample-models.hpp>
#include <pinocchio/constraints.hpp>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/delassus-operator.hpp>

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_memory_allocation)
{
  const Eigen::Index mat_size = 100;
  const Eigen::MatrixXd mat_ = Eigen::MatrixXd::Random(mat_size, mat_size);
  const Eigen::MatrixXd symmetric_mat = mat_.transpose() * mat_;

  BOOST_CHECK(isSymmetric(symmetric_mat));

  DelassusOperatorDense delassus(symmetric_mat);

  Eigen::VectorXd res(mat_size);
  const Eigen::VectorXd rhs = Eigen::VectorXd::Random(mat_size);
  res = delassus * rhs;
  BOOST_CHECK(res.isApprox((symmetric_mat * rhs).eval()));

  PowerIterationAlgoTpl<Eigen::VectorXd> power_iteration(mat_size);

  // Check memory allocations
  res = delassus * rhs;
  (delassus * rhs).evalTo(res);
  res.noalias() = symmetric_mat * rhs;
  res.noalias() = delassus * rhs;
  evalTo(symmetric_mat * rhs, res);
  power_iteration.run(delassus);
  power_iteration.run(symmetric_mat);
}

BOOST_AUTO_TEST_CASE(test_cholesky_expression_to_dense)
{
  // create model
  Model model;
  buildModels::manipulator(model);
  model.lowerPositionLimit.setConstant(-1.0);
  model.upperPositionLimit.setConstant(1.0);
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

  calc(model, data, constraint_models, constraint_datas);

  // compute delassus
  ConstraintCholeskyDecomposition chol(model, data, constraint_models, constraint_datas);
  chol.compute(model, data, constraint_models, constraint_datas, 1e-10);

  // check dense method
  DelassusOperatorDense delassus_operator_dense =
    chol.getDelassusOperatorCholeskyExpression().dense();
  Eigen::MatrixXd true_delassus_dense = chol.getDelassusOperatorCholeskyExpression().matrix();
  Eigen::VectorXd true_compliance = chol.getDelassusOperatorCholeskyExpression().getCompliance();
  auto true_damping = chol.getDelassusOperatorCholeskyExpression().getDamping();
  true_damping.subTo(true_delassus_dense);
  true_delassus_dense -= true_compliance.asDiagonal();
  DelassusOperatorDense true_delassus_operator_dense(true_delassus_dense);
  true_delassus_operator_dense.updateCompliance(true_compliance);
  true_delassus_operator_dense.updateDamping(true_damping);

  BOOST_CHECK(delassus_operator_dense == true_delassus_operator_dense);

  // check dense constructor from expression
  DelassusOperatorDense delassus_operator_dense2(chol.getDelassusOperatorCholeskyExpression());

  BOOST_CHECK(delassus_operator_dense2 == true_delassus_operator_dense);
}

BOOST_AUTO_TEST_CASE(delassus_from_dense_matrix_compliant)
{
  const Eigen::Index mat_size = 50;
  const Eigen::MatrixXd mat = Eigen::MatrixXd::Random(mat_size, mat_size);
  const Eigen::MatrixXd symmetric_mat = mat.transpose() * mat;
  const Eigen::VectorXd compliance = 1e-1 * Eigen::VectorXd::Ones(mat_size);
  const Eigen::MatrixXd compliance_matrix = compliance.asDiagonal();
  const Eigen::MatrixXd compliant_matrix = symmetric_mat + compliance_matrix;

  BOOST_CHECK(isSymmetric(symmetric_mat));
  BOOST_CHECK(isSymmetric(compliant_matrix));

  DelassusOperatorDense delassus(symmetric_mat);
  delassus.updateCompliance(compliance);
  BOOST_CHECK(delassus.getCompliance().isApprox(compliance));

  Eigen::VectorXd res(mat_size);
  const Eigen::VectorXd rhs = Eigen::VectorXd::Random(mat_size);

  // Checking matrix() method
  BOOST_CHECK(compliant_matrix.isApprox(delassus.matrix()));

  // Checking apply on the right
  delassus.applyOnTheRight(rhs, res);
  BOOST_CHECK(res.isApprox((compliant_matrix * rhs).eval()));

  // Checking solved
  Eigen::VectorXd damping_vec = 5e-3 * Eigen::VectorXd::Ones(mat_size);
  Eigen::MatrixXd damping = damping_vec.asDiagonal();
  delassus.updateDamping(damping_vec);
  BOOST_CHECK(delassus.getDamping().blocks()[0].map.isApprox(damping_vec));

  delassus.updateDecomposition();
  delassus.solve(rhs, res);
  const Eigen::MatrixXd compliant_matrix_inv = (compliant_matrix + damping).inverse();
  Eigen::VectorXd res_solve = compliant_matrix_inv * rhs;
  BOOST_CHECK(res.isApprox(res_solve));

  // Checking undampedMatrix() method
  BOOST_CHECK(compliant_matrix.isApprox(delassus.undampedMatrix()));

  // Checking solveInPlace
  delassus.solveInPlace(rhs);
  BOOST_CHECK(rhs.isApprox(res_solve));

  // Checking updateDamping
  const double new_damping = 1e-3;
  const Eigen::MatrixXd damped_compliant_matrix =
    compliant_matrix + new_damping * Eigen::MatrixXd::Identity(mat_size, mat_size);
  delassus.updateDamping(new_damping);
  BOOST_CHECK(delassus.getDamping().blocks()[0].map(0, 0) == new_damping);
  delassus.applyOnTheRight(rhs, res);
  Eigen::VectorXd res_apply = damped_compliant_matrix * rhs;
  BOOST_CHECK(res.isApprox(res_apply));

  // Checking updateCompliance
  const double new_compliance = 4e-3;
  const Eigen::MatrixXd new_compliance_matrix =
    Eigen::VectorXd::Constant(mat_size, new_compliance).asDiagonal();
  const Eigen::MatrixXd new_compliant_matrix = symmetric_mat + new_compliance_matrix;
  delassus.updateCompliance(new_compliance);
  BOOST_CHECK(
    delassus.getCompliance().isApprox(Eigen::VectorXd::Constant(mat_size, new_compliance)));
  const Eigen::MatrixXd new_damped_compliant_matrix =
    new_compliant_matrix + new_damping * Eigen::MatrixXd::Identity(mat_size, mat_size);
  delassus.updateDamping(new_damping);
  BOOST_CHECK(delassus.getDamping().blocks()[0].map(0, 0) == new_damping);
  delassus.applyOnTheRight(rhs, res);
  Eigen::VectorXd new_res_apply = new_damped_compliant_matrix * rhs;
  BOOST_CHECK(new_damped_compliant_matrix.isApprox(delassus.matrix()));
  BOOST_CHECK(res.isApprox(new_res_apply));
}

BOOST_AUTO_TEST_CASE(delassus_unsafe)
{
  const Eigen::Index mat_size = 50;
  const Eigen::MatrixXd mat = Eigen::MatrixXd::Random(mat_size, mat_size);
  const Eigen::MatrixXd symmetric_mat = mat.transpose() * mat;
  const Eigen::VectorXd damping = Eigen::VectorXd::Random(mat_size);

  DelassusOperatorDense delassus(symmetric_mat);
  delassus.unsafe().damping() = damping.asDiagonal();

  BOOST_CHECK(delassus.getDamping().diagonal() == damping);
}

BOOST_AUTO_TEST_CASE(test_copy)
{
  const Eigen::Index mat_size = 10;
  const Eigen::MatrixXd mat_ = Eigen::MatrixXd::Random(mat_size, mat_size);
  const Eigen::MatrixXd symmetric_mat = mat_.transpose() * mat_;
  const Eigen::VectorXd compliance = Eigen::VectorXd::LinSpaced(mat_size, 1e-4, 1e-3);

  DelassusOperatorDense delassus(symmetric_mat);
  delassus.updateCompliance(compliance);
  const Eigen::MatrixXd expected_matrix = delassus.matrix();
  const Eigen::VectorXd expected_compliance = delassus.getCompliance();

  // copy constructor: values match
  DelassusOperatorDense delassus_copy(delassus);
  BOOST_CHECK(delassus_copy.matrix().isApprox(expected_matrix));
  BOOST_CHECK(delassus_copy.getCompliance().isApprox(expected_compliance));

  // copy constructor: independence — modifying original does not affect copy
  const Eigen::MatrixXd new_mat_ = Eigen::MatrixXd::Random(mat_size, mat_size);
  const Eigen::MatrixXd new_symmetric_mat = new_mat_.transpose() * new_mat_;
  delassus.rebuild(new_symmetric_mat);
  delassus.updateCompliance(Eigen::VectorXd::Zero(mat_size));
  BOOST_CHECK(delassus_copy.matrix().isApprox(expected_matrix));
  BOOST_CHECK(delassus_copy.getCompliance().isApprox(expected_compliance));

  // copy assignment: values match
  DelassusOperatorDense delassus_assigned;
  delassus_assigned = delassus;
  BOOST_CHECK(delassus_assigned.matrix().isApprox(delassus.matrix()));
  BOOST_CHECK(delassus_assigned.getCompliance().isApprox(delassus.getCompliance()));

  // copy assignment: independence
  delassus.updateCompliance(compliance);
  BOOST_CHECK(!delassus_assigned.getCompliance().isApprox(compliance));
}
