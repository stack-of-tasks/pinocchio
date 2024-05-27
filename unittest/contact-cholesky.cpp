//
// Copyright (c) 2019-2022 INRIA
//

#include <iostream>

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/contact-cholesky.hxx"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

namespace pinocchio
{
  namespace cholesky
  {
    template<typename Scalar, int Options>
    struct ContactCholeskyDecompositionAccessorTpl
    : public ContactCholeskyDecompositionTpl<Scalar, Options>
    {
      typedef ContactCholeskyDecompositionTpl<Scalar, Options> Base;
      typedef typename Base::IndexVector IndexVector;
      typedef typename Base::BooleanVector BooleanVector;

      ContactCholeskyDecompositionAccessorTpl(const Base & other)
      : Base(other)
      {
      }

      const IndexVector & getParents_fromRow() const
      {
        return this->parents_fromRow;
      }
      const IndexVector & getLastChild() const
      {
        return this->last_child;
      }
      const IndexVector & getNvSubtree_fromRow() const
      {
        return this->nv_subtree_fromRow;
      }
      const std::vector<BooleanVector> & getJoint1_indexes() const
      {
        return this->joint1_indexes;
      }
      const std::vector<BooleanVector> & getJoint2_indexes() const
      {
        return this->joint2_indexes;
      }
    };

    typedef ContactCholeskyDecompositionAccessorTpl<double, 0> ContactCholeskyDecompositionAccessor;
  } // namespace cholesky
} // namespace pinocchio

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(contact_operator_equal)
{

  using namespace Eigen;
  using namespace pinocchio;
  using namespace cholesky;

  pinocchio::Model humanoid_model;
  pinocchio::buildModels::humanoidRandom(humanoid_model);
  Data humanoid_data(humanoid_model);

  pinocchio::Model manipulator_model;
  pinocchio::buildModels::manipulator(manipulator_model);
  Data manipulator_data(manipulator_model);

  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_empty;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas_empty;

  humanoid_model.lowerPositionLimit.head<3>().fill(-1.);
  humanoid_model.upperPositionLimit.head<3>().fill(1.);
  VectorXd humanoid_q = randomConfiguration(humanoid_model);
  crba(humanoid_model, humanoid_data, humanoid_q, Convention::WORLD);

  VectorXd manipulator_q = randomConfiguration(manipulator_model);
  crba(manipulator_model, manipulator_data, manipulator_q, Convention::WORLD);

  ContactCholeskyDecomposition humanoid_chol(humanoid_model), manipulator_chol(manipulator_model);
  humanoid_chol.compute(humanoid_model, humanoid_data, contact_models_empty, contact_datas_empty);
  manipulator_chol.compute(
    manipulator_model, manipulator_data, contact_models_empty, contact_datas_empty);

  BOOST_CHECK(humanoid_chol == humanoid_chol);
  BOOST_CHECK(humanoid_chol != manipulator_chol);
}

BOOST_AUTO_TEST_CASE(contact_cholesky_simple)
{
  using namespace Eigen;
  using namespace pinocchio;
  using namespace pinocchio::cholesky;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  crba(model, data_ref, q, Convention::WORLD);

  pinocchio::cholesky::decompose(model, data_ref);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.triangularView<Eigen::StrictlyUpper>().transpose();

  ContactCholeskyDecomposition contact_chol_decomposition;
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_empty;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas_empty;
  contact_chol_decomposition.allocate(model, contact_models_empty);

  BOOST_CHECK(contact_chol_decomposition.D.size() == model.nv);
  BOOST_CHECK(contact_chol_decomposition.Dinv.size() == model.nv);
  BOOST_CHECK(contact_chol_decomposition.U.rows() == model.nv);
  BOOST_CHECK(contact_chol_decomposition.U.cols() == model.nv);
  BOOST_CHECK(contact_chol_decomposition.size() == model.nv);
  BOOST_CHECK(contact_chol_decomposition.U.diagonal().isOnes());

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();

  contact_chol_decomposition.compute(model, data, contact_models_empty, contact_datas_empty);

  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  contact_chol_decomposition.getInverseMassMatrix(Minv_test);

  BOOST_CHECK(data.M.isApprox(data_ref.M));
  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));

  BOOST_CHECK(data_ref.D.isApprox(contact_chol_decomposition.D.tail(model.nv)));
  BOOST_CHECK(data_ref.Dinv.isApprox(contact_chol_decomposition.Dinv.tail(model.nv)));
  BOOST_CHECK(
    data_ref.U.isApprox(contact_chol_decomposition.U.bottomRightCorner(model.nv, model.nv)));

  ContactCholeskyDecompositionAccessor access(contact_chol_decomposition);
  for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    BOOST_CHECK(access.getParents_fromRow()[k] == data.parents_fromRow[(size_t)k]);
  }

  for (Eigen::DenseIndex k = 0; k < model.njoints; ++k)
  {
    BOOST_CHECK(access.getLastChild()[k] == data.lastChild[(size_t)k]);
  }

  for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    BOOST_CHECK(access.getNvSubtree_fromRow()[k] == data.nvSubtree_fromRow[(size_t)k]);
  }

  // Test basic operation
  VectorXd v_in(VectorXd::Random(model.nv));
  MatrixXd mat_in(MatrixXd::Random(contact_chol_decomposition.size(), 20));

  // Test Uv
  VectorXd Uv_op_res(v_in), Uv_op_ref(v_in);

  contact_chol_decomposition.Uv(Uv_op_res);
  pinocchio::cholesky::Uv(model, data_ref, Uv_op_ref);

  BOOST_CHECK(Uv_op_res.isApprox(Uv_op_ref));

  MatrixXd Uv_mat_op_res(mat_in), Uv_mat_op_ref(mat_in);

  contact_chol_decomposition.Uv(Uv_mat_op_res);
  pinocchio::cholesky::Uv(model, data_ref, Uv_mat_op_ref);

  BOOST_CHECK(Uv_mat_op_res.isApprox(Uv_mat_op_ref));

  // Test Utv
  VectorXd Utv_op_res(v_in), Utv_op_ref(v_in);

  contact_chol_decomposition.Utv(Utv_op_res);
  pinocchio::cholesky::Utv(model, data_ref, Utv_op_ref);

  BOOST_CHECK(Utv_op_res.isApprox(Utv_op_ref));

  MatrixXd Utv_mat_op_res(mat_in), Utv_mat_op_ref(mat_in);

  contact_chol_decomposition.Utv(Utv_mat_op_res);
  pinocchio::cholesky::Utv(model, data_ref, Utv_mat_op_ref);

  BOOST_CHECK(Utv_mat_op_res.isApprox(Utv_mat_op_ref));

  // Test Uiv
  VectorXd Uiv_op_res(v_in), Uiv_op_ref(v_in);

  contact_chol_decomposition.Uiv(Uiv_op_res);
  pinocchio::cholesky::Uiv(model, data_ref, Uiv_op_ref);

  BOOST_CHECK(Uiv_op_res.isApprox(Uiv_op_ref));

  MatrixXd Uiv_mat_op_res(mat_in), Uiv_mat_op_ref(mat_in);

  contact_chol_decomposition.Uiv(Uiv_mat_op_res);
  pinocchio::cholesky::Uiv(model, data_ref, Uiv_mat_op_ref);

  BOOST_CHECK(Uiv_mat_op_res.isApprox(Uiv_mat_op_ref));

  // Test Utiv
  VectorXd Utiv_op_res(v_in), Utiv_op_ref(v_in);

  contact_chol_decomposition.Utiv(Utiv_op_res);
  pinocchio::cholesky::Utiv(model, data_ref, Utiv_op_ref);

  BOOST_CHECK(Utiv_op_res.isApprox(Utiv_op_ref));

  MatrixXd Utiv_mat_op_res(mat_in), Utiv_mat_op_ref(mat_in);

  contact_chol_decomposition.Utiv(Utiv_mat_op_res);
  pinocchio::cholesky::Utiv(model, data_ref, Utiv_mat_op_ref);

  BOOST_CHECK(Utiv_mat_op_res.isApprox(Utiv_mat_op_ref));

  // SolveInPlace
  VectorXd sol(v_in);
  contact_chol_decomposition.solveInPlace(sol);

  VectorXd sol_ref(data.M.inverse() * v_in);

  BOOST_CHECK(sol.isApprox(sol_ref));

  MatrixXd sol_mat(mat_in);
  contact_chol_decomposition.solveInPlace(sol_mat);

  MatrixXd sol_mat_ref(data.M.inverse() * mat_in);

  BOOST_CHECK(sol_mat.isApprox(sol_mat_ref));

  // solve
  MatrixXd sol_copy_mat = contact_chol_decomposition.solve(mat_in);
  BOOST_CHECK(sol_copy_mat.isApprox(sol_mat));

  // inverse
  MatrixXd M_inv(model.nv, model.nv);
  contact_chol_decomposition.inverse(M_inv);

  MatrixXd M_inv_ref = data.M.inverse();
  BOOST_CHECK(M_inv.isApprox(M_inv_ref));

  // test retrieve Mass Matrix Cholesky Decomposition
  ContactCholeskyDecomposition mass_matrix_chol =
    contact_chol_decomposition.getMassMatrixChoeslkyDecomposition(model);

  // test Operational Space Inertia Matrix
  MatrixXd iosim = contact_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
  MatrixXd osim = contact_chol_decomposition.getOperationalSpaceInertiaMatrix();
  BOOST_CHECK(iosim.size() == 0);
  BOOST_CHECK(osim.size() == 0);

  BOOST_CHECK(mass_matrix_chol == contact_chol_decomposition);
  BOOST_CHECK(mass_matrix_chol.U.isApprox(data_ref.U));
  BOOST_CHECK(mass_matrix_chol.D.isApprox(data_ref.D));
  BOOST_CHECK(mass_matrix_chol.Dinv.isApprox(data_ref.Dinv));
}

BOOST_AUTO_TEST_CASE(contact_cholesky_contact6D_LOCAL)
{
  using namespace Eigen;
  using namespace pinocchio;
  using namespace pinocchio::cholesky;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D, model, model.getJointId(RF), LOCAL);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_6D, model, model.getJointId(LF), LOCAL);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));

  // Compute Mass Matrix
  crba(model, data_ref, q, Convention::WORLD);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.triangularView<Eigen::StrictlyUpper>().transpose();

  // Compute Cholesky decomposition
  pinocchio::cholesky::decompose(model, data_ref);

  // Compute Jacobians
  Data::Matrix6x J_RF(6, model.nv), J_LF(6, model.nv);
  J_RF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RF), LOCAL, J_RF);
  J_LF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LF), LOCAL, J_LF);

  const int constraint_dim = 12;
  const int total_dim = model.nv + constraint_dim;

  Data::MatrixXs H(total_dim, total_dim);
  H.setZero();
  H.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  H.middleRows<6>(0).rightCols(model.nv) = J_RF;
  H.middleRows<6>(6).rightCols(model.nv) = J_LF;

  H.triangularView<Eigen::StrictlyLower>() = H.triangularView<Eigen::StrictlyUpper>().transpose();

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  ContactCholeskyDecomposition contact_chol_decomposition;
  contact_chol_decomposition.allocate(model, contact_models);

  ContactCholeskyDecompositionAccessor access(contact_chol_decomposition);
  for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    if (data.parents_fromRow[(size_t)k] == -1)
      BOOST_CHECK(access.getParents_fromRow()[k + constraint_dim] == -1);
    else
      BOOST_CHECK(
        access.getParents_fromRow()[k + constraint_dim]
        == data.parents_fromRow[(size_t)k] + constraint_dim);
  }

  contact_chol_decomposition.compute(model, data, contact_models, contact_datas);

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();

  BOOST_CHECK(data_ref.D.isApprox(contact_chol_decomposition.D.tail(model.nv)));
  BOOST_CHECK(data_ref.Dinv.isApprox(contact_chol_decomposition.Dinv.tail(model.nv)));
  BOOST_CHECK(
    data_ref.U.isApprox(contact_chol_decomposition.U.bottomRightCorner(model.nv, model.nv)));

  Data::MatrixXs M_recomposed =
    contact_chol_decomposition.U.bottomRightCorner(model.nv, model.nv)
    * contact_chol_decomposition.D.tail(model.nv).asDiagonal()
    * contact_chol_decomposition.U.bottomRightCorner(model.nv, model.nv).transpose();
  BOOST_CHECK(M_recomposed.isApprox(data.M));

  Data::MatrixXs H_recomposed = contact_chol_decomposition.U
                                * contact_chol_decomposition.D.asDiagonal()
                                * contact_chol_decomposition.U.transpose();

  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv, model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_dim, model.nv)
                .isApprox(H.topRightCorner(constraint_dim, model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));

  // test Operational Space Inertia Matrix
  {
    MatrixXd JMinvJt = H.middleRows<12>(0).rightCols(model.nv) * data_ref.M.inverse()
                       * H.middleRows<12>(0).rightCols(model.nv).transpose();
    MatrixXd iosim = contact_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
    MatrixXd osim = contact_chol_decomposition.getOperationalSpaceInertiaMatrix();
    Eigen::MatrixXd JMinv_test(Eigen::MatrixXd::Zero(constraint_dim, model.nv));
    contact_chol_decomposition.getJMinv(JMinv_test);
    MatrixXd JMinv_ref = H.middleRows<12>(0).rightCols(model.nv) * data_ref.M.inverse();
    BOOST_CHECK(JMinv_ref.isApprox(JMinv_test));

    BOOST_CHECK(iosim.isApprox(JMinvJt));
    BOOST_CHECK(osim.isApprox(JMinvJt.inverse()));

    const MatrixXd rhs = MatrixXd::Random(12, 12);
    const MatrixXd res_delassus = contact_chol_decomposition.getDelassusCholeskyExpression() * rhs;
    const MatrixXd res_delassus_ref = iosim * rhs;

    BOOST_CHECK(res_delassus_ref.isApprox(res_delassus));

    const MatrixXd res_delassus_inverse =
      contact_chol_decomposition.getDelassusCholeskyExpression().solve(rhs);
    const MatrixXd res_delassus_inverse_ref = osim * rhs;

    BOOST_CHECK(res_delassus_inverse_ref.isApprox(res_delassus_inverse));
  }

  // test Mass matrix cholesky
  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  contact_chol_decomposition.getInverseMassMatrix(Minv_test);

  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));

  ContactCholeskyDecomposition contact_chol_decomposition_mu;
  contact_chol_decomposition_mu.allocate(model, contact_models);
  contact_chol_decomposition_mu.compute(model, data, contact_models, contact_datas, 0.);

  BOOST_CHECK(contact_chol_decomposition_mu.D.isApprox(contact_chol_decomposition.D));
  BOOST_CHECK(contact_chol_decomposition_mu.Dinv.isApprox(contact_chol_decomposition.Dinv));
  BOOST_CHECK(contact_chol_decomposition_mu.U.isApprox(contact_chol_decomposition.U));

  const double mu = 0.1;
  contact_chol_decomposition_mu.compute(model, data, contact_models, contact_datas, mu);
  Data::MatrixXs H_mu(H);
  H_mu.topLeftCorner(constraint_dim, constraint_dim).diagonal().fill(-mu);

  // test damped Operational Space Inertia Matrix
  {
    MatrixXd JMinvJt_mu = H_mu.middleRows<12>(0).rightCols(model.nv) * data_ref.M.inverse()
                            * H_mu.middleRows<12>(0).rightCols(model.nv).transpose()
                          + mu * MatrixXd::Identity(12, 12);
    MatrixXd iosim_mu = contact_chol_decomposition_mu.getInverseOperationalSpaceInertiaMatrix();
    MatrixXd osim_mu = contact_chol_decomposition_mu.getOperationalSpaceInertiaMatrix();

    BOOST_CHECK(iosim_mu.isApprox(JMinvJt_mu));
    BOOST_CHECK(osim_mu.isApprox(JMinvJt_mu.inverse()));

    const MatrixXd rhs = MatrixXd::Random(12, 1);
    const MatrixXd res = contact_chol_decomposition_mu.getDelassusCholeskyExpression() * rhs;
    const MatrixXd res_ref = iosim_mu * rhs;

    BOOST_CHECK(res_ref.isApprox(res));

    const MatrixXd res_no_mu =
      contact_chol_decomposition_mu.getDelassusCholeskyExpression() * rhs - mu * rhs;
    const MatrixXd res_no_mu_ref = contact_chol_decomposition.getDelassusCholeskyExpression() * rhs;

    BOOST_CHECK(res_no_mu.isApprox(res_no_mu_ref));
  }

  Data::MatrixXs H_recomposed_mu = contact_chol_decomposition_mu.U
                                   * contact_chol_decomposition_mu.D.asDiagonal()
                                   * contact_chol_decomposition_mu.U.transpose();

  BOOST_CHECK(H_recomposed_mu.isApprox(H_mu));

  // Test basic operation
  VectorXd v_in(VectorXd::Random(contact_chol_decomposition.size()));
  MatrixXd mat_in(MatrixXd::Random(contact_chol_decomposition.size(), 20));

  // Test Uv
  VectorXd Uv_op_res(v_in), Uv_op_ref(v_in);

  contact_chol_decomposition.Uv(Uv_op_res);
  Uv_op_ref.noalias() = contact_chol_decomposition.U * v_in;

  BOOST_CHECK(Uv_op_res.isApprox(Uv_op_ref));

  MatrixXd Uv_mat_op_res(mat_in), Uv_mat_op_ref(mat_in);

  contact_chol_decomposition.Uv(Uv_mat_op_res);
  Uv_mat_op_ref.noalias() = contact_chol_decomposition.U * mat_in;

  BOOST_CHECK(Uv_mat_op_res.isApprox(Uv_mat_op_ref));

  // Test Utv
  VectorXd Utv_op_res(v_in), Utv_op_ref(v_in);

  contact_chol_decomposition.Utv(Utv_op_res);
  Utv_op_ref.noalias() = contact_chol_decomposition.U.transpose() * v_in;

  BOOST_CHECK(Utv_op_res.isApprox(Utv_op_ref));

  MatrixXd Utv_mat_op_res(mat_in), Utv_mat_op_ref(mat_in);

  contact_chol_decomposition.Utv(Utv_mat_op_res);
  Utv_mat_op_ref.noalias() = contact_chol_decomposition.U.transpose() * mat_in;

  BOOST_CHECK(Utv_mat_op_res.isApprox(Utv_mat_op_ref));

  // Test Uiv
  VectorXd Uiv_op_res(v_in), Uiv_op_ref(v_in);

  contact_chol_decomposition.Uiv(Uiv_op_res);
  Uiv_op_ref.noalias() = contact_chol_decomposition.U.inverse() * v_in;

  BOOST_CHECK(Uiv_op_res.isApprox(Uiv_op_ref));

  MatrixXd Uiv_mat_op_res(mat_in), Uiv_mat_op_ref(mat_in);

  contact_chol_decomposition.Uiv(Uiv_mat_op_res);
  Uiv_mat_op_ref.noalias() = contact_chol_decomposition.U.inverse() * mat_in;

  BOOST_CHECK(Uiv_mat_op_res.isApprox(Uiv_mat_op_ref));

  // Test Utiv
  VectorXd Utiv_op_res(v_in), Utiv_op_ref(v_in);

  contact_chol_decomposition.Utiv(Utiv_op_res);
  Utiv_op_ref.noalias() = contact_chol_decomposition.U.inverse().transpose() * v_in;

  BOOST_CHECK(Utiv_op_res.isApprox(Utiv_op_ref));

  MatrixXd Utiv_mat_op_res(mat_in), Utiv_mat_op_ref(mat_in);

  contact_chol_decomposition.Utiv(Utiv_mat_op_res);
  Utiv_mat_op_ref.noalias() = contact_chol_decomposition.U.inverse().transpose() * mat_in;

  BOOST_CHECK(Utiv_mat_op_res.isApprox(Utiv_mat_op_ref));

  // SolveInPlace
  VectorXd sol(v_in);
  contact_chol_decomposition.solveInPlace(sol);

  VectorXd sol_ref(H.inverse() * v_in);

  BOOST_CHECK(sol.isApprox(sol_ref));

  MatrixXd sol_mat(mat_in), sol_mat_ref(mat_in);

  contact_chol_decomposition.solveInPlace(sol_mat);
  sol_mat_ref.noalias() = H.inverse() * mat_in;

  BOOST_CHECK(sol_mat.isApprox(sol_mat_ref));

  // solve
  MatrixXd sol_copy_mat = contact_chol_decomposition.solve(mat_in);
  BOOST_CHECK(sol_copy_mat.isApprox(sol_mat));

  // inverse
  MatrixXd H_inv(contact_chol_decomposition.size(), contact_chol_decomposition.size());
  contact_chol_decomposition.inverse(H_inv);

  MatrixXd H_inv_ref = H.inverse();
  BOOST_CHECK(H_inv.isApprox(H_inv_ref));

  // Check matrix
  MatrixXd mat1;
  contact_chol_decomposition.matrix(mat1);
  BOOST_CHECK(mat1.isApprox(H));

  MatrixXd mat2(constraint_dim + model.nv, constraint_dim + model.nv);
  contact_chol_decomposition.matrix(mat2.middleCols(0, constraint_dim + model.nv));
  BOOST_CHECK(mat2.isApprox(H));

  MatrixXd mat3 = contact_chol_decomposition.matrix();
  BOOST_CHECK(mat3.isApprox(H));
}

BOOST_AUTO_TEST_CASE(contact_cholesky_contact3D_6D_LOCAL)
{
  using namespace Eigen;
  using namespace pinocchio;
  using namespace pinocchio::cholesky;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  const std::string RA = "rarm6_joint";
  const std::string LA = "larm6_joint";

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D, model, model.getJointId(RF), LOCAL);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_3D, model, model.getJointId(LF), LOCAL);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));
  //  RigidConstraintModel ci_RA(CONTACT_3D,model.getJointId(RA),LOCAL);
  //  contact_models.push_back(ci_RA);
  //  contact_datas.push_back(RigidConstraintData(ci_RA));
  //  RigidConstraintModel ci_LA(CONTACT_3D,model.getJointId(LA),LOCAL_WORLD_ALIGNED);
  //  contact_models.push_back(ci_LA);
  //  contact_datas.push_back(RigidConstraintData(ci_LA));

  // Compute Mass Matrix
  crba(model, data_ref, q, Convention::WORLD);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.triangularView<Eigen::StrictlyUpper>().transpose();

  // Compute Cholesky decomposition
  pinocchio::cholesky::decompose(model, data_ref);

  // Compute Jacobians
  Data::Matrix6x J_RF(6, model.nv), J_LF(6, model.nv), J_RA(6, model.nv), J_LA(6, model.nv);
  J_RF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RF), LOCAL, J_RF);
  J_LF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LF), LOCAL, J_LF);
  J_RA.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RA), LOCAL, J_RA);
  J_LA.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LA), LOCAL_WORLD_ALIGNED, J_LA);

  const int constraint_dim = 9;
  const int total_dim = model.nv + constraint_dim;

  Data::MatrixXs H(total_dim, total_dim);
  H.setZero();
  H.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  H.middleRows<6>(0).rightCols(model.nv) = J_RF;
  H.middleRows<3>(6).rightCols(model.nv) = J_LF.middleRows<3>(Motion::LINEAR);
  //  H.middleRows<3>(9).rightCols(model.nv) = J_RA.middleRows<3>(Motion::LINEAR);
  //  H.middleRows<3>(12).rightCols(model.nv) = J_LA.middleRows<3>(Motion::LINEAR);

  H.triangularView<Eigen::StrictlyLower>() = H.triangularView<Eigen::StrictlyUpper>().transpose();

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  ContactCholeskyDecomposition contact_chol_decomposition;
  contact_chol_decomposition.allocate(model, contact_models);

  ContactCholeskyDecompositionAccessor access(contact_chol_decomposition);
  for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    if (data.parents_fromRow[(size_t)k] == -1)
      BOOST_CHECK(access.getParents_fromRow()[k + constraint_dim] == -1);
    else
      BOOST_CHECK(
        access.getParents_fromRow()[k + constraint_dim]
        == data.parents_fromRow[(size_t)k] + constraint_dim);
  }

  contact_chol_decomposition.compute(model, data, contact_models, contact_datas);

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();

  BOOST_CHECK(data_ref.D.isApprox(contact_chol_decomposition.D.tail(model.nv)));
  BOOST_CHECK(data_ref.Dinv.isApprox(contact_chol_decomposition.Dinv.tail(model.nv)));
  BOOST_CHECK(
    data_ref.U.isApprox(contact_chol_decomposition.U.bottomRightCorner(model.nv, model.nv)));

  Data::MatrixXs M_recomposed =
    contact_chol_decomposition.U.bottomRightCorner(model.nv, model.nv)
    * contact_chol_decomposition.D.tail(model.nv).asDiagonal()
    * contact_chol_decomposition.U.bottomRightCorner(model.nv, model.nv).transpose();
  BOOST_CHECK(M_recomposed.isApprox(data.M));

  Data::MatrixXs H_recomposed = contact_chol_decomposition.U
                                * contact_chol_decomposition.D.asDiagonal()
                                * contact_chol_decomposition.U.transpose();

  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv, model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_dim, model.nv)
                .isApprox(H.topRightCorner(constraint_dim, model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));

  // Test basic operation
  VectorXd v_in(VectorXd::Random(contact_chol_decomposition.size()));
  MatrixXd mat_in(MatrixXd::Random(contact_chol_decomposition.size(), 20));

  // Test Uv
  VectorXd Uv_op_res(v_in), Uv_op_ref(v_in);

  contact_chol_decomposition.Uv(Uv_op_res);
  Uv_op_ref.noalias() = contact_chol_decomposition.U * v_in;

  BOOST_CHECK(Uv_op_res.isApprox(Uv_op_ref));

  MatrixXd Uv_mat_op_res(mat_in), Uv_mat_op_ref(mat_in);

  contact_chol_decomposition.Uv(Uv_mat_op_res);
  Uv_mat_op_ref.noalias() = contact_chol_decomposition.U * mat_in;

  BOOST_CHECK(Uv_mat_op_res.isApprox(Uv_mat_op_ref));

  // Test Utv
  VectorXd Utv_op_res(v_in), Utv_op_ref(v_in);

  contact_chol_decomposition.Utv(Utv_op_res);
  Utv_op_ref.noalias() = contact_chol_decomposition.U.transpose() * v_in;

  BOOST_CHECK(Utv_op_res.isApprox(Utv_op_ref));

  MatrixXd Utv_mat_op_res(mat_in), Utv_mat_op_ref(mat_in);

  contact_chol_decomposition.Utv(Utv_mat_op_res);
  Utv_mat_op_ref.noalias() = contact_chol_decomposition.U.transpose() * mat_in;

  BOOST_CHECK(Utv_mat_op_res.isApprox(Utv_mat_op_ref));

  // Test Uiv
  VectorXd Uiv_op_res(v_in), Uiv_op_ref(v_in);

  contact_chol_decomposition.Uiv(Uiv_op_res);
  Uiv_op_ref.noalias() = contact_chol_decomposition.U.inverse() * v_in;

  BOOST_CHECK(Uiv_op_res.isApprox(Uiv_op_ref));

  MatrixXd Uiv_mat_op_res(mat_in), Uiv_mat_op_ref(mat_in);

  contact_chol_decomposition.Uiv(Uiv_mat_op_res);
  Uiv_mat_op_ref.noalias() = contact_chol_decomposition.U.inverse() * mat_in;

  BOOST_CHECK(Uiv_mat_op_res.isApprox(Uiv_mat_op_ref));

  // Test Utiv
  VectorXd Utiv_op_res(v_in), Utiv_op_ref(v_in);

  contact_chol_decomposition.Utiv(Utiv_op_res);
  Utiv_op_ref.noalias() = contact_chol_decomposition.U.inverse().transpose() * v_in;

  BOOST_CHECK(Utiv_op_res.isApprox(Utiv_op_ref));

  MatrixXd Utiv_mat_op_res(mat_in), Utiv_mat_op_ref(mat_in);

  contact_chol_decomposition.Utiv(Utiv_mat_op_res);
  Utiv_mat_op_ref.noalias() = contact_chol_decomposition.U.inverse().transpose() * mat_in;

  BOOST_CHECK(Utiv_mat_op_res.isApprox(Utiv_mat_op_ref));

  // SolveInPlace
  VectorXd sol(v_in);
  contact_chol_decomposition.solveInPlace(sol);

  VectorXd sol_ref(H.inverse() * v_in);

  BOOST_CHECK(sol.isApprox(sol_ref));

  MatrixXd sol_mat(mat_in), sol_mat_ref(mat_in);

  contact_chol_decomposition.solveInPlace(sol_mat);
  sol_mat_ref.noalias() = H.inverse() * mat_in;

  BOOST_CHECK(sol_mat.isApprox(sol_mat_ref));

  // solve
  MatrixXd sol_copy_mat = contact_chol_decomposition.solve(mat_in);
  BOOST_CHECK(sol_copy_mat.isApprox(sol_mat));

  // inverse
  MatrixXd H_inv(contact_chol_decomposition.size(), contact_chol_decomposition.size());
  contact_chol_decomposition.inverse(H_inv);

  MatrixXd H_inv_ref = H.inverse();
  BOOST_CHECK(H_inv.isApprox(H_inv_ref));

  // Check matrix
  MatrixXd mat1;
  contact_chol_decomposition.matrix(mat1);
  BOOST_CHECK(mat1.isApprox(H));

  MatrixXd mat2(constraint_dim + model.nv, constraint_dim + model.nv);
  contact_chol_decomposition.matrix(mat2.middleCols(0, constraint_dim + model.nv));
  BOOST_CHECK(mat2.isApprox(H));

  MatrixXd mat3 = contact_chol_decomposition.matrix();
  BOOST_CHECK(mat3.isApprox(H));

  // test Operational Space Inertia Matrix
  MatrixXd JMinvJt = H.middleRows<9>(0).rightCols(model.nv) * data_ref.M.inverse()
                     * H.middleRows<9>(0).rightCols(model.nv).transpose();
  MatrixXd iosim = contact_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
  MatrixXd osim = contact_chol_decomposition.getOperationalSpaceInertiaMatrix();

  BOOST_CHECK(iosim.isApprox(JMinvJt));
  BOOST_CHECK(osim.isApprox(JMinvJt.inverse()));

  // test Mass matrix cholesky
  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  contact_chol_decomposition.getInverseMassMatrix(Minv_test);

  Eigen::MatrixXd JMinv_test(Eigen::MatrixXd::Zero(9, model.nv));
  contact_chol_decomposition.getJMinv(JMinv_test);
  MatrixXd JMinv_ref = H.middleRows<9>(0).rightCols(model.nv) * data_ref.M.inverse();
  BOOST_CHECK(JMinv_ref.isApprox(JMinv_test));

  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));
}

BOOST_AUTO_TEST_CASE(contact_cholesky_contact6D_LOCAL_WORLD_ALIGNED)
{
  using namespace Eigen;
  using namespace pinocchio;
  using namespace pinocchio::cholesky;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D, model, model.getJointId(RF), LOCAL_WORLD_ALIGNED);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_6D, model, model.getJointId(LF), LOCAL);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));

  // Compute Mass Matrix
  crba(model, data_ref, q, Convention::WORLD);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.triangularView<Eigen::StrictlyUpper>().transpose();

  // Compute Cholesky decomposition
  pinocchio::cholesky::decompose(model, data_ref);

  // Compute Jacobians
  Data::Matrix6x J_RF(6, model.nv), J_LF(6, model.nv);
  J_RF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RF), ci_RF.reference_frame, J_RF);
  J_LF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LF), ci_LF.reference_frame, J_LF);

  const int constraint_dim = 12;
  const int total_dim = model.nv + constraint_dim;

  Data::MatrixXs H(total_dim, total_dim);
  H.setZero();
  H.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  H.middleRows<6>(0).rightCols(model.nv) = J_RF;
  H.middleRows<6>(6).rightCols(model.nv) = J_LF;

  H.triangularView<Eigen::StrictlyLower>() = H.triangularView<Eigen::StrictlyUpper>().transpose();

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  ContactCholeskyDecomposition contact_chol_decomposition;
  contact_chol_decomposition.allocate(model, contact_models);
  contact_chol_decomposition.compute(model, data, contact_models, contact_datas);

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();

  Data::MatrixXs H_recomposed = contact_chol_decomposition.matrix();

  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv, model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_dim, model.nv)
                .isApprox(H.topRightCorner(constraint_dim, model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));

  // inverse
  MatrixXd H_inv(contact_chol_decomposition.size(), contact_chol_decomposition.size());
  contact_chol_decomposition.inverse(H_inv);

  MatrixXd H_inv_ref = H_recomposed.inverse();
  BOOST_CHECK(H_inv.isApprox(H_inv_ref));

  // test Operational Space Inertia Matrix
  MatrixXd JMinvJt = H.middleRows<12>(0).rightCols(model.nv) * data_ref.M.inverse()
                     * H.middleRows<12>(0).rightCols(model.nv).transpose();
  MatrixXd iosim = contact_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
  MatrixXd osim = contact_chol_decomposition.getOperationalSpaceInertiaMatrix();

  BOOST_CHECK(iosim.isApprox(JMinvJt));
  BOOST_CHECK(osim.isApprox(JMinvJt.inverse()));

  // test Mass matrix cholesky
  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  contact_chol_decomposition.getInverseMassMatrix(Minv_test);

  Eigen::MatrixXd JMinv_test(Eigen::MatrixXd::Zero(12, model.nv));
  contact_chol_decomposition.getJMinv(JMinv_test);
  MatrixXd JMinv_ref = H.middleRows<12>(0).rightCols(model.nv) * data_ref.M.inverse();
  BOOST_CHECK(JMinv_ref.isApprox(JMinv_test));

  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));
}

BOOST_AUTO_TEST_CASE(contact_cholesky_contact6D_by_joint_2)
{
  using namespace Eigen;
  using namespace pinocchio;
  using namespace pinocchio::cholesky;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  const std::string RA = "rarm6_joint";
  const std::string LA = "larm6_joint";

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D, model, 0, model.getJointId(RF), LOCAL);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_6D, model, 0, model.getJointId(LF), LOCAL);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));
  RigidConstraintModel ci_RA(CONTACT_6D, model, 0, model.getJointId(RA), LOCAL_WORLD_ALIGNED);
  contact_models.push_back(ci_RA);
  contact_datas.push_back(RigidConstraintData(ci_RA));
  RigidConstraintModel ci_LA(CONTACT_6D, model, 0, model.getJointId(LA), LOCAL);
  contact_models.push_back(ci_LA);
  contact_datas.push_back(RigidConstraintData(ci_LA));

  // Compute Mass Matrix
  crba(model, data_ref, q, Convention::WORLD);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.triangularView<Eigen::StrictlyUpper>().transpose();

  // Compute Cholesky decomposition
  pinocchio::cholesky::decompose(model, data_ref);

  // Compute Jacobians
  Data::Matrix6x J_RF(6, model.nv), J_LF(6, model.nv), J_RA(6, model.nv), J_LA(6, model.nv);
  J_RF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RF), WORLD, J_RF);
  J_LF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LF), WORLD, J_LF);
  J_RA.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RA), WORLD, J_RA);
  J_LA.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LA), WORLD, J_LA);

  const int constraint_dim = 24;
  const int total_dim = model.nv + constraint_dim;

  const SE3 oMRA_wla = SE3(SE3::Matrix3::Identity(), ci_RA.joint1_placement.translation());

  const double mu = 0.1;
  Data::MatrixXs H(total_dim, total_dim);
  H.setZero();
  H.topLeftCorner(constraint_dim, constraint_dim).diagonal().fill(-mu);
  H.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  H.middleRows<6>(0).rightCols(model.nv) = -ci_RF.joint1_placement.toActionMatrixInverse() * J_RF;
  H.middleRows<6>(6).rightCols(model.nv) = -ci_LF.joint1_placement.toActionMatrixInverse() * J_LF;
  H.middleRows<6>(12).rightCols(model.nv) = -oMRA_wla.toActionMatrixInverse() * J_RA;
  H.middleRows<6>(18).rightCols(model.nv) = -ci_LA.joint1_placement.toActionMatrixInverse() * J_LA;

  H.triangularView<Eigen::StrictlyLower>() = H.triangularView<Eigen::StrictlyUpper>().transpose();

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  ContactCholeskyDecomposition contact_chol_decomposition;
  contact_chol_decomposition.allocate(model, contact_models);
  contact_chol_decomposition.compute(model, data, contact_models, contact_datas, mu);

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();

  Data::MatrixXs H_recomposed = contact_chol_decomposition.matrix();

  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv, model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_dim, model.nv)
                .isApprox(H.topRightCorner(constraint_dim, model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));

  VectorXd v_in(VectorXd::Random(contact_chol_decomposition.size()));
  MatrixXd mat_in(MatrixXd::Random(contact_chol_decomposition.size(), 20));

  // Test Uv
  VectorXd Uv_op_res(v_in), Uv_op_ref(v_in);

  contact_chol_decomposition.Uv(Uv_op_res);
  Uv_op_ref.noalias() = contact_chol_decomposition.U * v_in;

  BOOST_CHECK(Uv_op_res.isApprox(Uv_op_ref));

  MatrixXd Uv_mat_op_res(mat_in), Uv_mat_op_ref(mat_in);

  contact_chol_decomposition.Uv(Uv_mat_op_res);
  Uv_mat_op_ref.noalias() = contact_chol_decomposition.U * mat_in;

  BOOST_CHECK(Uv_mat_op_res.isApprox(Uv_mat_op_ref));

  // Test Utv
  VectorXd Utv_op_res(v_in), Utv_op_ref(v_in);

  contact_chol_decomposition.Utv(Utv_op_res);
  Utv_op_ref.noalias() = contact_chol_decomposition.U.transpose() * v_in;

  BOOST_CHECK(Utv_op_res.isApprox(Utv_op_ref));

  MatrixXd Utv_mat_op_res(mat_in), Utv_mat_op_ref(mat_in);

  contact_chol_decomposition.Utv(Utv_mat_op_res);
  Utv_mat_op_ref.noalias() = contact_chol_decomposition.U.transpose() * mat_in;

  BOOST_CHECK(Utv_mat_op_res.isApprox(Utv_mat_op_ref));

  // Test Uiv
  VectorXd Uiv_op_res(v_in), Uiv_op_ref(v_in);

  contact_chol_decomposition.Uiv(Uiv_op_res);
  Uiv_op_ref.noalias() = contact_chol_decomposition.U.inverse() * v_in;

  BOOST_CHECK(Uiv_op_res.isApprox(Uiv_op_ref));

  MatrixXd Uiv_mat_op_res(mat_in), Uiv_mat_op_ref(mat_in);

  contact_chol_decomposition.Uiv(Uiv_mat_op_res);
  Uiv_mat_op_ref.noalias() = contact_chol_decomposition.U.inverse() * mat_in;

  BOOST_CHECK(Uiv_mat_op_res.isApprox(Uiv_mat_op_ref));

  // Test Utiv
  VectorXd Utiv_op_res(v_in), Utiv_op_ref(v_in);

  contact_chol_decomposition.Utiv(Utiv_op_res);
  Utiv_op_ref.noalias() = contact_chol_decomposition.U.inverse().transpose() * v_in;

  BOOST_CHECK(Utiv_op_res.isApprox(Utiv_op_ref));

  MatrixXd Utiv_mat_op_res(mat_in), Utiv_mat_op_ref(mat_in);

  contact_chol_decomposition.Utiv(Utiv_mat_op_res);
  Utiv_mat_op_ref.noalias() = contact_chol_decomposition.U.inverse().transpose() * mat_in;

  BOOST_CHECK(Utiv_mat_op_res.isApprox(Utiv_mat_op_ref));

  // inverse
  MatrixXd H_inv(contact_chol_decomposition.size(), contact_chol_decomposition.size());
  contact_chol_decomposition.inverse(H_inv);

  MatrixXd H_inv2 = contact_chol_decomposition.U.transpose().inverse()
                    * contact_chol_decomposition.Dinv.asDiagonal()
                    * contact_chol_decomposition.U.inverse();

  MatrixXd H_inv3 =
    MatrixXd::Identity(contact_chol_decomposition.size(), contact_chol_decomposition.size());
  contact_chol_decomposition.solveInPlace(H_inv3);

  MatrixXd H_inv_ref = H_recomposed.inverse();
  BOOST_CHECK(H_inv.topLeftCorner(constraint_dim, constraint_dim)
                .isApprox(H_inv_ref.topLeftCorner(constraint_dim, constraint_dim)));
  BOOST_CHECK(H_inv.bottomRightCorner(model.nv, model.nv)
                .isApprox(H_inv_ref.bottomRightCorner(model.nv, model.nv)));
  BOOST_CHECK(H_inv.topRightCorner(constraint_dim, model.nv)
                .isApprox(H_inv_ref.topRightCorner(constraint_dim, model.nv)));

  BOOST_CHECK(H_inv_ref.isApprox(H_inv));
  BOOST_CHECK(H_inv_ref.isApprox(H_inv2));
  BOOST_CHECK(H_inv_ref.isApprox(H_inv3));
  const VectorXd ei = VectorXd::Unit(contact_chol_decomposition.size(), constraint_dim);
  VectorXd ei_inv = ei;
  contact_chol_decomposition.solveInPlace(ei_inv);
  VectorXd ei_inv2 = ei;
  contact_chol_decomposition.Uiv(ei_inv2);
  ei_inv2 = contact_chol_decomposition.Dinv.asDiagonal() * ei_inv2;
  contact_chol_decomposition.Utiv(ei_inv2);

  BOOST_CHECK(ei_inv.isApprox(H_inv_ref * ei));
  BOOST_CHECK(ei_inv2.isApprox(H_inv_ref * ei));
  BOOST_CHECK(ei_inv.isApprox(H_inv * ei));
  BOOST_CHECK(ei_inv2.isApprox(H_inv * ei));

  // test Operational Space Inertia Matrix
  MatrixXd JMinvJt = H.middleRows<24>(0).rightCols(model.nv) * data_ref.M.inverse()
                       * H.middleRows<24>(0).rightCols(model.nv).transpose()
                     + mu * Eigen::MatrixXd::Identity(24, 24);
  MatrixXd iosim = contact_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
  MatrixXd osim = contact_chol_decomposition.getOperationalSpaceInertiaMatrix();

  BOOST_CHECK(iosim.isApprox(JMinvJt));
  BOOST_CHECK(osim.isApprox(JMinvJt.inverse()));

  // test Mass matrix cholesky
  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  contact_chol_decomposition.getInverseMassMatrix(Minv_test);

  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));
  Eigen::MatrixXd JMinv_test(Eigen::MatrixXd::Zero(24, model.nv));
  contact_chol_decomposition.getJMinv(JMinv_test);
  MatrixXd JMinv_ref = H.middleRows<24>(0).rightCols(model.nv) * data_ref.M.inverse();
  BOOST_CHECK(JMinv_ref.isApprox(JMinv_test));
}

BOOST_AUTO_TEST_CASE(contact_cholesky_contact3D_6D_WORLD_by_joint_2)
{
  using namespace Eigen;
  using namespace pinocchio;
  using namespace pinocchio::cholesky;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  const std::string RA = "rarm6_joint";
  const std::string LA = "larm6_joint";

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D, model, 0, model.getJointId(RF), LOCAL);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_3D, model, 0, model.getJointId(LF), LOCAL);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));
  RigidConstraintModel ci_RA(CONTACT_3D, model, 0, model.getJointId(RA), LOCAL);
  contact_models.push_back(ci_RA);
  contact_datas.push_back(RigidConstraintData(ci_RA));
  RigidConstraintModel ci_LA(CONTACT_3D, model, 0, model.getJointId(LA), LOCAL_WORLD_ALIGNED);
  contact_models.push_back(ci_LA);
  contact_datas.push_back(RigidConstraintData(ci_LA));

  // Compute Mass Matrix
  crba(model, data_ref, q, Convention::WORLD);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.triangularView<Eigen::StrictlyUpper>().transpose();

  // Compute Cholesky decomposition
  pinocchio::cholesky::decompose(model, data_ref);

  // Compute Jacobians
  Data::Matrix6x J_RF(6, model.nv), J_LF(6, model.nv), J_RA(6, model.nv), J_LA(6, model.nv);
  J_RF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RF), WORLD, J_RF);
  J_LF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LF), LOCAL, J_LF);
  J_RA.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RA), LOCAL, J_RA);
  J_LA.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LA), LOCAL_WORLD_ALIGNED, J_LA);

  const int constraint_dim = 15;
  const int total_dim = model.nv + constraint_dim;

  Data::MatrixXs H(total_dim, total_dim);
  H.setZero();
  H.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  H.middleRows<6>(0).rightCols(model.nv) = -ci_RF.joint1_placement.toActionMatrix() * J_RF;
  H.middleRows<3>(6).rightCols(model.nv) =
    -data_ref.oMi[model.getJointId(LF)].rotation() * J_LF.middleRows<3>(Motion::LINEAR);
  H.middleRows<3>(9).rightCols(model.nv) =
    -data_ref.oMi[model.getJointId(RA)].rotation() * J_RA.middleRows<3>(Motion::LINEAR);
  H.middleRows<3>(12).rightCols(model.nv) = -J_LA.middleRows<3>(Motion::LINEAR);

  H.triangularView<Eigen::StrictlyLower>() = H.triangularView<Eigen::StrictlyUpper>().transpose();

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  ContactCholeskyDecomposition contact_chol_decomposition;
  contact_chol_decomposition.allocate(model, contact_models);

  contact_chol_decomposition.compute(model, data, contact_models, contact_datas);

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();

  BOOST_CHECK(data_ref.D.isApprox(contact_chol_decomposition.D.tail(model.nv)));
  BOOST_CHECK(data_ref.Dinv.isApprox(contact_chol_decomposition.Dinv.tail(model.nv)));
  BOOST_CHECK(
    data_ref.U.isApprox(contact_chol_decomposition.U.bottomRightCorner(model.nv, model.nv)));

  Data::MatrixXs M_recomposed =
    contact_chol_decomposition.U.bottomRightCorner(model.nv, model.nv)
    * contact_chol_decomposition.D.tail(model.nv).asDiagonal()
    * contact_chol_decomposition.U.bottomRightCorner(model.nv, model.nv).transpose();
  BOOST_CHECK(M_recomposed.isApprox(data.M));

  Data::MatrixXs H_recomposed = contact_chol_decomposition.U
                                * contact_chol_decomposition.D.asDiagonal()
                                * contact_chol_decomposition.U.transpose();

  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv, model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_dim, model.nv)
                .isApprox(H.topRightCorner(constraint_dim, model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));

  // test Operational Space Inertia Matrix
  MatrixXd JMinvJt = H.middleRows<15>(0).rightCols(model.nv) * data_ref.M.inverse()
                     * H.middleRows<15>(0).rightCols(model.nv).transpose();
  MatrixXd iosim = contact_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
  MatrixXd osim = contact_chol_decomposition.getOperationalSpaceInertiaMatrix();

  BOOST_CHECK(iosim.isApprox(JMinvJt));
  BOOST_CHECK(osim.isApprox(JMinvJt.inverse()));

  // test Mass matrix cholesky
  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  contact_chol_decomposition.getInverseMassMatrix(Minv_test);

  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));
  Eigen::MatrixXd JMinv_test(Eigen::MatrixXd::Zero(15, model.nv));
  contact_chol_decomposition.getJMinv(JMinv_test);
  MatrixXd JMinv_ref = H.middleRows<15>(0).rightCols(model.nv) * data_ref.M.inverse();
  BOOST_CHECK(JMinv_ref.isApprox(JMinv_test));
}

BOOST_AUTO_TEST_CASE(loop_contact_cholesky_contact6D)
{
  using namespace Eigen;
  using namespace pinocchio;
  using namespace pinocchio::cholesky;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  const std::string RA = "rarm6_joint";
  const std::string LA = "larm6_joint";

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel loop_RF_LF_local(
    CONTACT_6D, model, model.getJointId(RF), model.getJointId(LF), LOCAL);
  RigidConstraintModel loop_RA_LA_lwa(
    CONTACT_6D, model, model.getJointId(RA), model.getJointId(LA), LOCAL_WORLD_ALIGNED);

  loop_RF_LF_local.joint1_placement.setRandom();
  loop_RF_LF_local.joint2_placement.setRandom();

  loop_RA_LA_lwa.joint1_placement.setRandom();
  loop_RA_LA_lwa.joint2_placement.setRandom();

  contact_models.push_back(loop_RF_LF_local);
  contact_datas.push_back(RigidConstraintData(loop_RF_LF_local));

  contact_models.push_back(loop_RA_LA_lwa);
  contact_datas.push_back(RigidConstraintData(loop_RA_LA_lwa));

  // Compute Mass Matrix
  crba(model, data_ref, q, Convention::WORLD);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.triangularView<Eigen::StrictlyUpper>().transpose();

  // Compute Cholesky decomposition
  const double mu = 0.1;
  pinocchio::cholesky::decompose(model, data_ref);

  // Compute Jacobians
  Data::Matrix6x J_RF(6, model.nv), J_LF(6, model.nv), J_RA_local(6, model.nv),
    J_LA_local(6, model.nv);
  J_RF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RF), LOCAL, J_RF);
  Data::Matrix6x J_RF_local = loop_RF_LF_local.joint1_placement.toActionMatrixInverse() * J_RF;
  J_LF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LF), LOCAL, J_LF);
  Data::Matrix6x J_LF_local = loop_RF_LF_local.joint2_placement.toActionMatrixInverse() * J_LF;
  J_RA_local.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RA), LOCAL, J_RA_local);
  J_RA_local = loop_RA_LA_lwa.joint1_placement.toActionMatrixInverse() * J_RA_local;
  J_LA_local.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LA), LOCAL, J_LA_local);
  J_LA_local = loop_RA_LA_lwa.joint2_placement.toActionMatrixInverse() * J_LA_local;

  Data::Matrix6x J_RF_world(6, model.nv), J_LF_world(6, model.nv);
  J_RF_world.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RF), WORLD, J_RF_world);
  J_LF_world.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LF), WORLD, J_LF_world);

  forwardKinematics(model, data_ref, q);
  const SE3 oM1_loop1 =
    data_ref.oMi[loop_RF_LF_local.joint1_id] * loop_RF_LF_local.joint1_placement;
  const SE3 oM2_loop1 =
    data_ref.oMi[loop_RF_LF_local.joint2_id] * loop_RF_LF_local.joint2_placement;
  const SE3 _1M2_loop1 = oM1_loop1.inverse() * oM2_loop1;
  const SE3 oM1_loop2 = data_ref.oMi[loop_RA_LA_lwa.joint1_id] * loop_RA_LA_lwa.joint1_placement;
  const SE3 oM2_loop2 = data_ref.oMi[loop_RA_LA_lwa.joint2_id] * loop_RA_LA_lwa.joint2_placement;
  const SE3 _1M2_loop2 = oM1_loop2.inverse() * oM2_loop2;

  const int constraint_dim = 12;
  const int total_dim = model.nv + constraint_dim;

  Data::MatrixXs H(total_dim, total_dim);
  H.setZero();
  H.topLeftCorner(constraint_dim, constraint_dim).diagonal().fill(-mu);
  H.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  H.middleRows<6>(0).rightCols(model.nv) = J_RF_local - _1M2_loop1.toActionMatrix() * J_LF_local;
  const SE3 oM1_loop2_lwa = SE3(oM1_loop2.rotation(), SE3::Vector3::Zero());
  H.middleRows<6>(6).rightCols(model.nv) =
    oM1_loop2_lwa.toActionMatrix() * J_RA_local
    - (oM1_loop2_lwa.toActionMatrix() * _1M2_loop2.toActionMatrix()) * J_LA_local;

  H.triangularView<Eigen::StrictlyLower>() = H.triangularView<Eigen::StrictlyUpper>().transpose();

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  ContactCholeskyDecomposition contact_chol_decomposition;
  contact_chol_decomposition.allocate(model, contact_models);
  contact_chol_decomposition.compute(model, data, contact_models, contact_datas, mu);

  Data::MatrixXs H_recomposed = contact_chol_decomposition.matrix();

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();
  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv, model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_dim, model.nv)
                .isApprox(H.topRightCorner(constraint_dim, model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));

  // inverse
  MatrixXd H_inv(contact_chol_decomposition.size(), contact_chol_decomposition.size());
  contact_chol_decomposition.inverse(H_inv);

  MatrixXd H_inv_ref = H_recomposed.inverse();

  BOOST_CHECK(H_inv_ref.isApprox(H_inv));

  // test Operational Space Inertia Matrix
  MatrixXd JMinvJt = H.middleRows<12>(0).rightCols(model.nv) * data_ref.M.inverse()
                       * H.middleRows<12>(0).rightCols(model.nv).transpose()
                     + mu * Eigen::MatrixXd::Identity(12, 12);
  MatrixXd iosim = contact_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
  MatrixXd osim = contact_chol_decomposition.getOperationalSpaceInertiaMatrix();

  BOOST_CHECK(iosim.isApprox(JMinvJt));
  BOOST_CHECK(osim.isApprox(JMinvJt.inverse()));

  // test Mass matrix cholesky
  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  contact_chol_decomposition.getInverseMassMatrix(Minv_test);

  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));
  Eigen::MatrixXd JMinv_test(Eigen::MatrixXd::Zero(12, model.nv));
  contact_chol_decomposition.getJMinv(JMinv_test);
  MatrixXd JMinv_ref = H.middleRows<12>(0).rightCols(model.nv) * data_ref.M.inverse();
  BOOST_CHECK(JMinv_ref.isApprox(JMinv_test));
}

BOOST_AUTO_TEST_CASE(loop_contact_cholesky_contact_3d)
{
  using namespace Eigen;
  using namespace pinocchio;
  using namespace pinocchio::cholesky;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  const std::string RA = "rarm6_joint";
  const std::string LA = "larm6_joint";

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel loop_RF_LF_local(
    CONTACT_3D, model, model.getJointId(RF), model.getJointId(LF), LOCAL);
  RigidConstraintModel loop_RA_LA_lwa(
    CONTACT_3D, model, model.getJointId(RA), model.getJointId(LA), LOCAL_WORLD_ALIGNED);

  loop_RF_LF_local.joint1_placement.setRandom();
  loop_RF_LF_local.joint2_placement.setRandom();

  loop_RA_LA_lwa.joint1_placement.setRandom();
  loop_RA_LA_lwa.joint2_placement.setRandom();

  contact_models.push_back(loop_RF_LF_local);
  contact_datas.push_back(RigidConstraintData(loop_RF_LF_local));

  contact_models.push_back(loop_RA_LA_lwa);
  contact_datas.push_back(RigidConstraintData(loop_RA_LA_lwa));

  // Compute Mass Matrix
  crba(model, data_ref, q, Convention::WORLD);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.triangularView<Eigen::StrictlyUpper>().transpose();

  // Compute Cholesky decomposition
  const double mu = 0.1;
  pinocchio::cholesky::decompose(model, data_ref);

  // Compute Jacobians
  Data::Matrix6x J_RF(6, model.nv), J_LF(6, model.nv), J_RA_local(6, model.nv),
    J_LA_local(6, model.nv);
  J_RF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RF), LOCAL, J_RF);
  const Data::Matrix6x J_RF_local =
    loop_RF_LF_local.joint1_placement.toActionMatrixInverse() * J_RF;
  J_LF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LF), LOCAL, J_LF);
  const Data::Matrix6x J_LF_local =
    loop_RF_LF_local.joint2_placement.toActionMatrixInverse() * J_LF;
  J_RA_local.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RA), LOCAL, J_RA_local);
  J_RA_local = loop_RA_LA_lwa.joint1_placement.toActionMatrixInverse() * J_RA_local;
  J_LA_local.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LA), LOCAL, J_LA_local);
  J_LA_local = loop_RA_LA_lwa.joint2_placement.toActionMatrixInverse() * J_LA_local;

  Data::Matrix6x J_RF_world(6, model.nv), J_LF_world(6, model.nv);
  J_RF_world.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RF), WORLD, J_RF_world);
  J_LF_world.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LF), WORLD, J_LF_world);

  forwardKinematics(model, data_ref, q);
  const SE3 oM1_loop1 =
    data_ref.oMi[loop_RF_LF_local.joint1_id] * loop_RF_LF_local.joint1_placement;
  const SE3 oM2_loop1 =
    data_ref.oMi[loop_RF_LF_local.joint2_id] * loop_RF_LF_local.joint2_placement;
  const SE3 _1M2_loop1 = oM1_loop1.inverse() * oM2_loop1;
  const SE3 oM1_loop2 = data_ref.oMi[loop_RA_LA_lwa.joint1_id] * loop_RA_LA_lwa.joint1_placement;
  const SE3 oM2_loop2 = data_ref.oMi[loop_RA_LA_lwa.joint2_id] * loop_RA_LA_lwa.joint2_placement;
  const SE3 _1M2_loop2 = oM1_loop2.inverse() * oM2_loop2;

  const int constraint_dim = 6;
  const int total_dim = model.nv + constraint_dim;

  Data::MatrixXs H(total_dim, total_dim);
  H.setZero();
  H.topLeftCorner(constraint_dim, constraint_dim).diagonal().fill(-mu);
  H.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  H.middleRows<3>(0).rightCols(model.nv) =
    J_RF_local.middleRows<3>(Motion::LINEAR)
    - _1M2_loop1.rotation() * J_LF_local.middleRows<3>(Motion::LINEAR);
  const SE3 oM1_loop2_lwa = SE3(oM1_loop2.rotation(), SE3::Vector3::Zero());
  const SE3 oM2_loop2_lwa = SE3(oM2_loop2.rotation(), SE3::Vector3::Zero());
  H.middleRows<3>(3).rightCols(model.nv) =
    (oM1_loop2_lwa.toActionMatrix() * J_RA_local - (oM2_loop2_lwa.toActionMatrix()) * J_LA_local)
      .middleRows<3>(Motion::LINEAR);

  H.triangularView<Eigen::StrictlyLower>() = H.triangularView<Eigen::StrictlyUpper>().transpose();

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  ContactCholeskyDecomposition contact_chol_decomposition;
  contact_chol_decomposition.allocate(model, contact_models);
  contact_chol_decomposition.compute(model, data, contact_models, contact_datas, mu);
  BOOST_CHECK(contact_datas[0].c1Mc2.isApprox(_1M2_loop1));
  BOOST_CHECK(contact_datas[1].c1Mc2.isApprox(_1M2_loop2));

  Data::MatrixXs H_recomposed = contact_chol_decomposition.matrix();

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();
  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv, model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_dim, model.nv)
                .isApprox(H.topRightCorner(constraint_dim, model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));

  // std::cout << "H_recomposed.topRightCorner(constraint_dim,model.nv):\n" <<
  // H_recomposed.topRightCorner(constraint_dim,model.nv) << std::endl; std::cout <<
  // "H.topRightCorner(constraint_dim,model.nv):\n" << H  .topRightCorner(constraint_dim,model.nv)
  // << std::endl;

  // inverse
  MatrixXd H_inv(contact_chol_decomposition.size(), contact_chol_decomposition.size());
  contact_chol_decomposition.inverse(H_inv);

  MatrixXd H_inv_ref = H_recomposed.inverse();

  BOOST_CHECK(H_inv_ref.isApprox(H_inv));

  // test Operational Space Inertia Matrix
  MatrixXd JMinvJt = H.middleRows<6>(0).rightCols(model.nv) * data_ref.M.inverse()
                       * H.middleRows<6>(0).rightCols(model.nv).transpose()
                     + mu * Eigen::MatrixXd::Identity(6, 6);
  MatrixXd iosim = contact_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
  MatrixXd osim = contact_chol_decomposition.getOperationalSpaceInertiaMatrix();

  BOOST_CHECK(iosim.isApprox(JMinvJt));
  BOOST_CHECK(osim.isApprox(JMinvJt.inverse()));

  // test Mass matrix cholesky
  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  contact_chol_decomposition.getInverseMassMatrix(Minv_test);

  Eigen::MatrixXd JMinv_test(Eigen::MatrixXd::Zero(6, model.nv));
  contact_chol_decomposition.getJMinv(JMinv_test);
  MatrixXd JMinv_ref = H.middleRows<6>(0).rightCols(model.nv) * data_ref.M.inverse();
  BOOST_CHECK(JMinv_ref.isApprox(JMinv_test));
  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));
}

BOOST_AUTO_TEST_CASE(contact_cholesky_updateDamping)
{
  using namespace Eigen;
  using namespace pinocchio;
  using namespace pinocchio::cholesky;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D, model, model.getJointId(RF), LOCAL);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_6D, model, model.getJointId(LF), LOCAL);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));

  Data data(model);
  crba(model, data, q, Convention::WORLD);

  const double mu1 = 1e-2, mu2 = 1e-10;

  {
    ContactCholeskyDecomposition contact_chol_decomposition;
    contact_chol_decomposition.allocate(model, contact_models);
    contact_chol_decomposition.compute(model, data, contact_models, contact_datas, mu1);
    contact_chol_decomposition.updateDamping(mu2);

    ContactCholeskyDecomposition contact_chol_decomposition_ref;
    contact_chol_decomposition_ref.allocate(model, contact_models);
    contact_chol_decomposition_ref.compute(model, data, contact_models, contact_datas, mu2);

    BOOST_CHECK(contact_chol_decomposition.D.isApprox(contact_chol_decomposition_ref.D));
    BOOST_CHECK(contact_chol_decomposition.Dinv.isApprox(contact_chol_decomposition_ref.Dinv));
    BOOST_CHECK(contact_chol_decomposition.U.isApprox(contact_chol_decomposition_ref.U));
  }

  {
    ContactCholeskyDecomposition contact_chol_decomposition;
    contact_chol_decomposition.allocate(model, contact_models);
    contact_chol_decomposition.compute(model, data, contact_models, contact_datas, mu1);
    contact_chol_decomposition.getDelassusCholeskyExpression().updateDamping(mu2);

    ContactCholeskyDecomposition contact_chol_decomposition_ref;
    contact_chol_decomposition_ref.allocate(model, contact_models);
    contact_chol_decomposition_ref.compute(model, data, contact_models, contact_datas, mu2);

    BOOST_CHECK(contact_chol_decomposition.D.isApprox(contact_chol_decomposition_ref.D));
    BOOST_CHECK(contact_chol_decomposition.Dinv.isApprox(contact_chol_decomposition_ref.Dinv));
    BOOST_CHECK(contact_chol_decomposition.U.isApprox(contact_chol_decomposition_ref.U));
  }
}

BOOST_AUTO_TEST_SUITE_END()
