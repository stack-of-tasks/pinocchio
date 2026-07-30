//
// Copyright (c) 2019-2025 INRIA
//

#define BOOST_TEST_MODULE constraint_cholesky

#include "pinocchio/math.hpp"
#include "pinocchio/constraints.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/constraint-cholesky.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

namespace pinocchio
{

  template<typename _Matrix>
  struct UDUt
  {
    typedef _Matrix Matrix;
    typedef typename Matrix::Scalar Scalar;
    static constexpr int Options = Matrix::Options;
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix) PlainMatrix;
    typedef typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Matrix) RowMatrix;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> Vector;

    explicit UDUt(const Matrix & mat)
    : U(mat.rows(), mat.cols())
    , D(mat.rows())
    {
      assert(mat.rows() == mat.cols());
      U.setZero();
      U.diagonal().setOnes();
      compute(mat);
    }

    PlainMatrix matrix() const
    {
      return U * D.asDiagonal() * U.transpose();
    }

    template<typename MatrixDerived>
    void compute(const Eigen::MatrixBase<MatrixDerived> & mat)
    {
      assert(mat.rows() == mat.cols());
      U.template triangularView<Eigen::Upper>() = mat.template triangularView<Eigen::Upper>();
      for (Eigen::Index k = mat.rows() - 1; k >= 0; --k)
      {
        for (Eigen::Index i = k - 1; i >= 0; --i)
        {
          const Scalar a = U(i, k) / U(k, k);
          for (Eigen::Index j = i; j >= 0; --j)
          {
            U(j, i) -= U(j, k) * a;
          }
          U(i, k) = a;
        }
      }
      D = U.diagonal();
      U.diagonal().setOnes();
    }

    RowMatrix U;
    Vector D;
  };

  namespace cholesky
  {
    template<typename Scalar, int Options>
    struct ConstraintCholeskyDecompositionAccessorTpl
    : public ConstraintCholeskyDecompositionTpl<Scalar, Options>
    {
      typedef ConstraintCholeskyDecompositionTpl<Scalar, Options> Base;
      using typename Base::BooleanVector;
      using typename Base::EigenIndexVector;

      ConstraintCholeskyDecompositionAccessorTpl(const Base & other)
      : Base(other)
      {
      }

      const EigenIndexVector & getParents_fromRow() const
      {
        return this->parents_fromRow;
      }
      const EigenIndexVector & getNvSubtree_fromRow() const
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

    typedef ConstraintCholeskyDecompositionAccessorTpl<double, 0>
      ConstraintCholeskyDecompositionAccessor;
  } // namespace cholesky
} // namespace pinocchio

BOOST_AUTO_TEST_CASE(UDUt_solver)
{
  const Eigen::Index size = 100;
  Eigen::MatrixXd mat = Eigen::MatrixXd::Random(size, size);
  mat = mat * mat.transpose();

  pinocchio::UDUt<Eigen::MatrixXd> udut(mat);
  BOOST_CHECK(udut.matrix().isApprox(mat));
}

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

  const std::vector<RigidConstraintModel> contact_models_empty;
  std::vector<RigidConstraintData> contact_datas_empty;

  humanoid_model.lowerPositionLimit.head<3>().fill(-1.);
  humanoid_model.upperPositionLimit.head<3>().fill(1.);
  VectorXd humanoid_q = randomConfiguration(humanoid_model);
  crba(humanoid_model, humanoid_data, humanoid_q, Convention::WORLD);

  VectorXd manipulator_q = randomConfiguration(manipulator_model);
  crba(manipulator_model, manipulator_data, manipulator_q, Convention::WORLD);

  ConstraintCholeskyDecomposition humanoid_chol(humanoid_model, humanoid_data),
    manipulator_chol(manipulator_model, manipulator_data);
  humanoid_chol.compute(humanoid_model, humanoid_data, contact_models_empty, contact_datas_empty);
  manipulator_chol.compute(
    manipulator_model, manipulator_data, contact_models_empty, contact_datas_empty);

  BOOST_CHECK(humanoid_chol == humanoid_chol);
  BOOST_CHECK(humanoid_chol != manipulator_chol);
}

BOOST_AUTO_TEST_CASE(constraint_cholesky_simple)
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

  ConstraintCholeskyDecomposition constraint_chol_decomposition;
  const std::vector<RigidConstraintModel> contact_models_empty;
  std::vector<RigidConstraintData> contact_datas_empty;
  constraint_chol_decomposition.rebuild(model, data_ref, contact_models_empty, contact_datas_empty);

  BOOST_CHECK(constraint_chol_decomposition.D.size() == model.nv);
  BOOST_CHECK(constraint_chol_decomposition.Dinv.size() == model.nv);
  BOOST_CHECK(constraint_chol_decomposition.U.rows() == model.nv);
  BOOST_CHECK(constraint_chol_decomposition.U.cols() == model.nv);
  BOOST_CHECK(constraint_chol_decomposition.size() == model.nv);
  BOOST_CHECK(constraint_chol_decomposition.U.diagonal().isOnes());

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();

  constraint_chol_decomposition.compute(model, data, contact_models_empty, contact_datas_empty);

  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  constraint_chol_decomposition.getInverseMassMatrix(Minv_test);

  BOOST_CHECK(data.M.isApprox(data_ref.M));
  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));

  BOOST_CHECK(data_ref.D.isApprox(constraint_chol_decomposition.D.tail(model.nv)));
  BOOST_CHECK(data_ref.Dinv.isApprox(constraint_chol_decomposition.Dinv.tail(model.nv)));
  BOOST_CHECK(
    data_ref.U.isApprox(constraint_chol_decomposition.U.bottomRightCorner(model.nv, model.nv)));

  ConstraintCholeskyDecompositionAccessor access(constraint_chol_decomposition);
  for (Eigen::Index k = 0; k < model.nv; ++k)
  {
    BOOST_CHECK(access.getParents_fromRow()[k] == data.parents_fromRow[(size_t)k]);
  }

  for (Eigen::Index k = 0; k < model.nv; ++k)
  {
    BOOST_CHECK(access.getNvSubtree_fromRow()[k] == data.nvSubtree_fromRow[(size_t)k]);
  }

  // Test basic operation
  VectorXd v_in(VectorXd::Random(model.nv));
  MatrixXd mat_in(MatrixXd::Random(constraint_chol_decomposition.size(), 20));

  // Test Uv
  VectorXd Uv_op_res(v_in), Uv_op_ref(v_in);

  constraint_chol_decomposition.Uv(Uv_op_res);
  pinocchio::cholesky::Uv(model, data_ref, Uv_op_ref);

  BOOST_CHECK(Uv_op_res.isApprox(Uv_op_ref));

  MatrixXd Uv_mat_op_res(mat_in), Uv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Uv(Uv_mat_op_res);
  pinocchio::cholesky::Uv(model, data_ref, Uv_mat_op_ref);

  BOOST_CHECK(Uv_mat_op_res.isApprox(Uv_mat_op_ref));

  // Test Utv
  VectorXd Utv_op_res(v_in), Utv_op_ref(v_in);

  constraint_chol_decomposition.Utv(Utv_op_res);
  pinocchio::cholesky::Utv(model, data_ref, Utv_op_ref);

  BOOST_CHECK(Utv_op_res.isApprox(Utv_op_ref));

  MatrixXd Utv_mat_op_res(mat_in), Utv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Utv(Utv_mat_op_res);
  pinocchio::cholesky::Utv(model, data_ref, Utv_mat_op_ref);

  BOOST_CHECK(Utv_mat_op_res.isApprox(Utv_mat_op_ref));

  // Test Uiv
  VectorXd Uiv_op_res(v_in), Uiv_op_ref(v_in);

  constraint_chol_decomposition.Uiv(Uiv_op_res);
  pinocchio::cholesky::Uiv(model, data_ref, Uiv_op_ref);

  BOOST_CHECK(Uiv_op_res.isApprox(Uiv_op_ref));

  MatrixXd Uiv_mat_op_res(mat_in), Uiv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Uiv(Uiv_mat_op_res);
  pinocchio::cholesky::Uiv(model, data_ref, Uiv_mat_op_ref);

  BOOST_CHECK(Uiv_mat_op_res.isApprox(Uiv_mat_op_ref));

  // Test Utiv
  VectorXd Utiv_op_res(v_in), Utiv_op_ref(v_in);

  constraint_chol_decomposition.Utiv(Utiv_op_res);
  pinocchio::cholesky::Utiv(model, data_ref, Utiv_op_ref);

  BOOST_CHECK(Utiv_op_res.isApprox(Utiv_op_ref));

  MatrixXd Utiv_mat_op_res(mat_in), Utiv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Utiv(Utiv_mat_op_res);
  pinocchio::cholesky::Utiv(model, data_ref, Utiv_mat_op_ref);

  BOOST_CHECK(Utiv_mat_op_res.isApprox(Utiv_mat_op_ref));

  // SolveInPlace
  VectorXd sol(v_in);
  constraint_chol_decomposition.solveInPlace(sol);

  VectorXd sol_ref(data.M.inverse() * v_in);

  BOOST_CHECK(sol.isApprox(sol_ref));

  MatrixXd sol_mat(mat_in);
  constraint_chol_decomposition.solveInPlace(sol_mat);

  MatrixXd sol_mat_ref(data.M.inverse() * mat_in);

  BOOST_CHECK(sol_mat.isApprox(sol_mat_ref));

  // solve
  MatrixXd sol_copy_mat = constraint_chol_decomposition.solve(mat_in);
  BOOST_CHECK(sol_copy_mat.isApprox(sol_mat));

  // inverse
  MatrixXd M_inv(model.nv, model.nv);
  constraint_chol_decomposition.inverse(M_inv);

  MatrixXd M_inv_ref = data.M.inverse();
  BOOST_CHECK(M_inv.isApprox(M_inv_ref));

  // test retrieve Mass Matrix Cholesky Decomposition
  ConstraintCholeskyDecomposition mass_matrix_chol =
    constraint_chol_decomposition.getMassMatrixChoeslkyDecomposition(model, data);

  // test Operational Space Inertia Matrix
  MatrixXd iosim = constraint_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
  MatrixXd osim = constraint_chol_decomposition.getOperationalSpaceInertiaMatrix();
  BOOST_CHECK(iosim.size() == 0);
  BOOST_CHECK(osim.size() == 0);

  BOOST_CHECK(mass_matrix_chol.U.isApprox(data_ref.U));
  BOOST_CHECK(mass_matrix_chol.U == constraint_chol_decomposition.U);
  BOOST_CHECK(mass_matrix_chol.D.isApprox(data_ref.D));
  BOOST_CHECK(mass_matrix_chol.D == constraint_chol_decomposition.D);
  BOOST_CHECK(mass_matrix_chol.Dinv.isApprox(data_ref.Dinv));
  BOOST_CHECK(mass_matrix_chol.Dinv == constraint_chol_decomposition.Dinv);
}

BOOST_AUTO_TEST_CASE(constraint_cholesky_contact6D_LOCAL)
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

  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D, model, model.getJointId(RF), LOCAL);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_6D, model, model.getJointId(LF), LOCAL);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));

  // Compute constraint data requirements
  computeJointJacobians(model, data_ref, q);
  data_ref.q_in = q;
  calc(model, data_ref, contact_models, contact_datas);

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

  const int constraint_size = 12;
  const int total_dim = model.nv + constraint_size;

  Data::MatrixXs H(total_dim, total_dim);
  H.setZero();
  H.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  H.middleRows<6>(0).rightCols(model.nv) = J_RF;
  H.middleRows<6>(6).rightCols(model.nv) = J_LF;

  H.triangularView<Eigen::StrictlyLower>() = H.triangularView<Eigen::StrictlyUpper>().transpose();

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  ConstraintCholeskyDecomposition constraint_chol_decomposition;
  constraint_chol_decomposition.rebuild(model, data_ref, contact_models, contact_datas);

  ConstraintCholeskyDecompositionAccessor access(constraint_chol_decomposition);
  for (Eigen::Index k = 0; k < model.nv; ++k)
  {
    if (data.parents_fromRow[(size_t)k] == -1)
      BOOST_CHECK(access.getParents_fromRow()[k + constraint_size] == -1);
    else
      BOOST_CHECK(
        access.getParents_fromRow()[k + constraint_size]
        == data.parents_fromRow[(size_t)k] + constraint_size);
  }

  constraint_chol_decomposition.compute(model, data, contact_models, contact_datas);

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();

  BOOST_CHECK(data_ref.D.isApprox(constraint_chol_decomposition.D.tail(model.nv)));
  BOOST_CHECK(data_ref.Dinv.isApprox(constraint_chol_decomposition.Dinv.tail(model.nv)));
  BOOST_CHECK(
    data_ref.U.isApprox(constraint_chol_decomposition.U.bottomRightCorner(model.nv, model.nv)));

  Data::MatrixXs M_recomposed =
    constraint_chol_decomposition.U.bottomRightCorner(model.nv, model.nv)
    * constraint_chol_decomposition.D.tail(model.nv).asDiagonal()
    * constraint_chol_decomposition.U.bottomRightCorner(model.nv, model.nv).transpose();
  BOOST_CHECK(M_recomposed.isApprox(data.M));

  Data::MatrixXs H_recomposed = constraint_chol_decomposition.U
                                * constraint_chol_decomposition.D.asDiagonal()
                                * constraint_chol_decomposition.U.transpose();

  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv, model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_size, model.nv)
                .isApprox(H.topRightCorner(constraint_size, model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));

  // test Operational Space Inertia Matrix
  {
    MatrixXd JMinvJt = H.middleRows<12>(0).rightCols(model.nv) * data_ref.M.inverse()
                       * H.middleRows<12>(0).rightCols(model.nv).transpose();
    MatrixXd iosim = constraint_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
    MatrixXd osim = constraint_chol_decomposition.getOperationalSpaceInertiaMatrix();
    Eigen::MatrixXd JMinv_test(Eigen::MatrixXd::Zero(constraint_size, model.nv));
    constraint_chol_decomposition.getJMinv(JMinv_test);
    MatrixXd JMinv_ref = H.middleRows<12>(0).rightCols(model.nv) * data_ref.M.inverse();
    BOOST_CHECK(JMinv_ref.isApprox(JMinv_test));

    BOOST_CHECK(iosim.isApprox(JMinvJt));
    BOOST_CHECK(osim.isApprox(JMinvJt.inverse()));

    const MatrixXd rhs = MatrixXd::Random(12, 12);
    BOOST_CHECK(constraint_chol_decomposition.getDelassusOperatorCholeskyExpression()
                  .getDamping()
                  .matrix()
                  .isZero());
    const MatrixXd res_delassus =
      constraint_chol_decomposition.getDelassusOperatorCholeskyExpression() * rhs;
    const MatrixXd res_delassus_ref = iosim * rhs;

    BOOST_CHECK(res_delassus_ref.isApprox(res_delassus));

    const MatrixXd res_delassus_inverse =
      constraint_chol_decomposition.getDelassusOperatorCholeskyExpression().solve(rhs);
    const MatrixXd res_delassus_inverse_ref = osim * rhs;

    BOOST_CHECK(res_delassus_inverse_ref.isApprox(res_delassus_inverse));
  }

  // test Mass matrix cholesky
  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  constraint_chol_decomposition.getInverseMassMatrix(Minv_test);

  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));

  ConstraintCholeskyDecomposition constraint_chol_decomposition_mu;
  constraint_chol_decomposition_mu.rebuild(model, data, contact_models, contact_datas);
  constraint_chol_decomposition_mu.compute(model, data, contact_models, contact_datas);

  BOOST_CHECK(constraint_chol_decomposition_mu.D.isApprox(constraint_chol_decomposition.D));
  BOOST_CHECK(constraint_chol_decomposition_mu.Dinv.isApprox(constraint_chol_decomposition.Dinv));
  BOOST_CHECK(constraint_chol_decomposition_mu.U.isApprox(constraint_chol_decomposition.U));

  const double mu = 0.1;
  constraint_chol_decomposition_mu.updateDamping(mu);
  constraint_chol_decomposition_mu.compute(model, data, contact_models, contact_datas);
  Data::MatrixXs H_mu(H);
  H_mu.topLeftCorner(constraint_size, constraint_size).diagonal().fill(-mu);

  // test damped Operational Space Inertia Matrix
  {
    MatrixXd JMinvJt_mu = H_mu.middleRows<12>(0).rightCols(model.nv) * data_ref.M.inverse()
                            * H_mu.middleRows<12>(0).rightCols(model.nv).transpose()
                          + mu * MatrixXd::Identity(12, 12);
    MatrixXd iosim_mu = constraint_chol_decomposition_mu.getInverseOperationalSpaceInertiaMatrix();
    MatrixXd osim_mu = constraint_chol_decomposition_mu.getOperationalSpaceInertiaMatrix();

    BOOST_CHECK(iosim_mu.isApprox(JMinvJt_mu));
    BOOST_CHECK(osim_mu.isApprox(JMinvJt_mu.inverse()));

    const MatrixXd rhs = MatrixXd::Random(12, 1);
    const MatrixXd res =
      constraint_chol_decomposition_mu.getDelassusOperatorCholeskyExpression() * rhs;
    const MatrixXd res_ref = iosim_mu * rhs;

    BOOST_CHECK(res_ref.isApprox(res));

    const MatrixXd res_no_mu =
      constraint_chol_decomposition_mu.getDelassusOperatorCholeskyExpression() * rhs - mu * rhs;
    const MatrixXd res_no_mu_ref =
      constraint_chol_decomposition.getDelassusOperatorCholeskyExpression() * rhs;

    BOOST_CHECK(res_no_mu.isApprox(res_no_mu_ref));
  }

  Data::MatrixXs H_recomposed_mu = constraint_chol_decomposition_mu.U
                                   * constraint_chol_decomposition_mu.D.asDiagonal()
                                   * constraint_chol_decomposition_mu.U.transpose();

  BOOST_CHECK(H_recomposed_mu.isApprox(H_mu));

  // Test basic operation
  VectorXd v_in(VectorXd::Random(constraint_chol_decomposition.size()));
  MatrixXd mat_in(MatrixXd::Random(constraint_chol_decomposition.size(), 20));

  // Test Uv
  VectorXd Uv_op_res(v_in), Uv_op_ref(v_in);

  constraint_chol_decomposition.Uv(Uv_op_res);
  Uv_op_ref.noalias() = constraint_chol_decomposition.U * v_in;

  BOOST_CHECK(Uv_op_res.isApprox(Uv_op_ref));

  MatrixXd Uv_mat_op_res(mat_in), Uv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Uv(Uv_mat_op_res);
  Uv_mat_op_ref.noalias() = constraint_chol_decomposition.U * mat_in;

  BOOST_CHECK(Uv_mat_op_res.isApprox(Uv_mat_op_ref));

  // Test Utv
  VectorXd Utv_op_res(v_in), Utv_op_ref(v_in);

  constraint_chol_decomposition.Utv(Utv_op_res);
  Utv_op_ref.noalias() = constraint_chol_decomposition.U.transpose() * v_in;

  BOOST_CHECK(Utv_op_res.isApprox(Utv_op_ref));

  MatrixXd Utv_mat_op_res(mat_in), Utv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Utv(Utv_mat_op_res);
  Utv_mat_op_ref.noalias() = constraint_chol_decomposition.U.transpose() * mat_in;

  BOOST_CHECK(Utv_mat_op_res.isApprox(Utv_mat_op_ref));

  // Test Uiv
  VectorXd Uiv_op_res(v_in), Uiv_op_ref(v_in);

  constraint_chol_decomposition.Uiv(Uiv_op_res);
  Uiv_op_ref.noalias() = constraint_chol_decomposition.U.inverse() * v_in;

  BOOST_CHECK(Uiv_op_res.isApprox(Uiv_op_ref));

  MatrixXd Uiv_mat_op_res(mat_in), Uiv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Uiv(Uiv_mat_op_res);
  Uiv_mat_op_ref.noalias() = constraint_chol_decomposition.U.inverse() * mat_in;

  BOOST_CHECK(Uiv_mat_op_res.isApprox(Uiv_mat_op_ref));

  // Test Utiv
  VectorXd Utiv_op_res(v_in), Utiv_op_ref(v_in);

  constraint_chol_decomposition.Utiv(Utiv_op_res);
  Utiv_op_ref.noalias() = constraint_chol_decomposition.U.inverse().transpose() * v_in;

  BOOST_CHECK(Utiv_op_res.isApprox(Utiv_op_ref));

  MatrixXd Utiv_mat_op_res(mat_in), Utiv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Utiv(Utiv_mat_op_res);
  Utiv_mat_op_ref.noalias() = constraint_chol_decomposition.U.inverse().transpose() * mat_in;

  BOOST_CHECK(Utiv_mat_op_res.isApprox(Utiv_mat_op_ref));

  // SolveInPlace
  VectorXd sol(v_in);
  constraint_chol_decomposition.solveInPlace(sol);

  VectorXd sol_ref(H.inverse() * v_in);

  BOOST_CHECK(sol.isApprox(sol_ref));

  MatrixXd sol_mat(mat_in), sol_mat_ref(mat_in);

  constraint_chol_decomposition.solveInPlace(sol_mat);
  sol_mat_ref.noalias() = H.inverse() * mat_in;

  BOOST_CHECK(sol_mat.isApprox(sol_mat_ref));

  // solve
  MatrixXd sol_copy_mat = constraint_chol_decomposition.solve(mat_in);
  BOOST_CHECK(sol_copy_mat.isApprox(sol_mat));

  // inverse
  MatrixXd H_inv(constraint_chol_decomposition.size(), constraint_chol_decomposition.size());
  constraint_chol_decomposition.inverse(H_inv);

  MatrixXd H_inv_ref = H.inverse();
  BOOST_CHECK(H_inv.isApprox(H_inv_ref));

  // Check matrix
  MatrixXd mat1;
  constraint_chol_decomposition.matrix(mat1);
  BOOST_CHECK(mat1.isApprox(H));

  MatrixXd mat2(constraint_size + model.nv, constraint_size + model.nv);
  constraint_chol_decomposition.matrix(mat2.middleCols(0, constraint_size + model.nv));
  BOOST_CHECK(mat2.isApprox(H));

  MatrixXd mat3 = constraint_chol_decomposition.matrix();
  BOOST_CHECK(mat3.isApprox(H));

  // Check memory allocation
  {
    const auto delassus_chol =
      constraint_chol_decomposition.getDelassusOperatorCholeskyExpression();
    PowerIterationAlgoTpl<Eigen::VectorXd> power_iteration(delassus_chol.rows());
    const Eigen::VectorXd rhs = Eigen::VectorXd::Random(delassus_chol.rows());
    Eigen::VectorXd res = Eigen::VectorXd::Random(delassus_chol.rows());
    res.noalias() = delassus_chol * rhs;
    power_iteration.run(delassus_chol);
  }
}

BOOST_AUTO_TEST_CASE(constraint_cholesky_contact3D_6D_LOCAL)
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

  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;
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
  computeJointJacobians(model, data_ref, q);
  data_ref.q_in = q;
  calc(model, data_ref, contact_models, contact_datas);

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

  const int constraint_size = 9;
  const int total_dim = model.nv + constraint_size;

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
  ConstraintCholeskyDecomposition constraint_chol_decomposition;
  constraint_chol_decomposition.rebuild(model, data, contact_models, contact_datas);

  ConstraintCholeskyDecompositionAccessor access(constraint_chol_decomposition);
  for (Eigen::Index k = 0; k < model.nv; ++k)
  {
    if (data.parents_fromRow[(size_t)k] == -1)
      BOOST_CHECK(access.getParents_fromRow()[k + constraint_size] == -1);
    else
      BOOST_CHECK(
        access.getParents_fromRow()[k + constraint_size]
        == data.parents_fromRow[(size_t)k] + constraint_size);
  }

  constraint_chol_decomposition.compute(model, data, contact_models, contact_datas);

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();

  BOOST_CHECK(data_ref.D.isApprox(constraint_chol_decomposition.D.tail(model.nv)));
  BOOST_CHECK(data_ref.Dinv.isApprox(constraint_chol_decomposition.Dinv.tail(model.nv)));
  BOOST_CHECK(
    data_ref.U.isApprox(constraint_chol_decomposition.U.bottomRightCorner(model.nv, model.nv)));

  Data::MatrixXs M_recomposed =
    constraint_chol_decomposition.U.bottomRightCorner(model.nv, model.nv)
    * constraint_chol_decomposition.D.tail(model.nv).asDiagonal()
    * constraint_chol_decomposition.U.bottomRightCorner(model.nv, model.nv).transpose();
  BOOST_CHECK(M_recomposed.isApprox(data.M));

  Data::MatrixXs H_recomposed = constraint_chol_decomposition.U
                                * constraint_chol_decomposition.D.asDiagonal()
                                * constraint_chol_decomposition.U.transpose();

  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv, model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_size, model.nv)
                .isApprox(H.topRightCorner(constraint_size, model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));

  // Test basic operation
  VectorXd v_in(VectorXd::Random(constraint_chol_decomposition.size()));
  MatrixXd mat_in(MatrixXd::Random(constraint_chol_decomposition.size(), 20));

  // Test Uv
  VectorXd Uv_op_res(v_in), Uv_op_ref(v_in);

  constraint_chol_decomposition.Uv(Uv_op_res);
  Uv_op_ref.noalias() = constraint_chol_decomposition.U * v_in;

  BOOST_CHECK(Uv_op_res.isApprox(Uv_op_ref));

  MatrixXd Uv_mat_op_res(mat_in), Uv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Uv(Uv_mat_op_res);
  Uv_mat_op_ref.noalias() = constraint_chol_decomposition.U * mat_in;

  BOOST_CHECK(Uv_mat_op_res.isApprox(Uv_mat_op_ref));

  // Test Utv
  VectorXd Utv_op_res(v_in), Utv_op_ref(v_in);

  constraint_chol_decomposition.Utv(Utv_op_res);
  Utv_op_ref.noalias() = constraint_chol_decomposition.U.transpose() * v_in;

  BOOST_CHECK(Utv_op_res.isApprox(Utv_op_ref));

  MatrixXd Utv_mat_op_res(mat_in), Utv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Utv(Utv_mat_op_res);
  Utv_mat_op_ref.noalias() = constraint_chol_decomposition.U.transpose() * mat_in;

  BOOST_CHECK(Utv_mat_op_res.isApprox(Utv_mat_op_ref));

  // Test Uiv
  VectorXd Uiv_op_res(v_in), Uiv_op_ref(v_in);

  constraint_chol_decomposition.Uiv(Uiv_op_res);
  Uiv_op_ref.noalias() = constraint_chol_decomposition.U.inverse() * v_in;

  BOOST_CHECK(Uiv_op_res.isApprox(Uiv_op_ref));

  MatrixXd Uiv_mat_op_res(mat_in), Uiv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Uiv(Uiv_mat_op_res);
  Uiv_mat_op_ref.noalias() = constraint_chol_decomposition.U.inverse() * mat_in;

  BOOST_CHECK(Uiv_mat_op_res.isApprox(Uiv_mat_op_ref));

  // Test Utiv
  VectorXd Utiv_op_res(v_in), Utiv_op_ref(v_in);

  constraint_chol_decomposition.Utiv(Utiv_op_res);
  Utiv_op_ref.noalias() = constraint_chol_decomposition.U.inverse().transpose() * v_in;

  BOOST_CHECK(Utiv_op_res.isApprox(Utiv_op_ref));

  MatrixXd Utiv_mat_op_res(mat_in), Utiv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Utiv(Utiv_mat_op_res);
  Utiv_mat_op_ref.noalias() = constraint_chol_decomposition.U.inverse().transpose() * mat_in;

  BOOST_CHECK(Utiv_mat_op_res.isApprox(Utiv_mat_op_ref));

  // SolveInPlace
  VectorXd sol(v_in);
  constraint_chol_decomposition.solveInPlace(sol);

  VectorXd sol_ref(H.inverse() * v_in);

  BOOST_CHECK(sol.isApprox(sol_ref));

  MatrixXd sol_mat(mat_in), sol_mat_ref(mat_in);

  constraint_chol_decomposition.solveInPlace(sol_mat);
  sol_mat_ref.noalias() = H.inverse() * mat_in;

  BOOST_CHECK(sol_mat.isApprox(sol_mat_ref));

  // solve
  MatrixXd sol_copy_mat = constraint_chol_decomposition.solve(mat_in);
  BOOST_CHECK(sol_copy_mat.isApprox(sol_mat));

  // inverse
  MatrixXd H_inv(constraint_chol_decomposition.size(), constraint_chol_decomposition.size());
  constraint_chol_decomposition.inverse(H_inv);

  MatrixXd H_inv_ref = H.inverse();
  BOOST_CHECK(H_inv.isApprox(H_inv_ref));

  // Check matrix
  MatrixXd mat1;
  constraint_chol_decomposition.matrix(mat1);
  BOOST_CHECK(mat1.isApprox(H));

  MatrixXd mat2(constraint_size + model.nv, constraint_size + model.nv);
  constraint_chol_decomposition.matrix(mat2.middleCols(0, constraint_size + model.nv));
  BOOST_CHECK(mat2.isApprox(H));

  MatrixXd mat3 = constraint_chol_decomposition.matrix();
  BOOST_CHECK(mat3.isApprox(H));

  // test Operational Space Inertia Matrix
  MatrixXd JMinvJt = H.middleRows<9>(0).rightCols(model.nv) * data_ref.M.inverse()
                     * H.middleRows<9>(0).rightCols(model.nv).transpose();
  MatrixXd iosim = constraint_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
  MatrixXd osim = constraint_chol_decomposition.getOperationalSpaceInertiaMatrix();

  BOOST_CHECK(iosim.isApprox(JMinvJt));
  BOOST_CHECK(osim.isApprox(JMinvJt.inverse()));

  // test Mass matrix cholesky
  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  constraint_chol_decomposition.getInverseMassMatrix(Minv_test);

  Eigen::MatrixXd JMinv_test(Eigen::MatrixXd::Zero(9, model.nv));
  constraint_chol_decomposition.getJMinv(JMinv_test);
  MatrixXd JMinv_ref = H.middleRows<9>(0).rightCols(model.nv) * data_ref.M.inverse();
  BOOST_CHECK(JMinv_ref.isApprox(JMinv_test));

  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));
}

BOOST_AUTO_TEST_CASE(constraint_cholesky_contact6D_LOCAL_WORLD_ALIGNED)
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

  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D, model, model.getJointId(RF), LOCAL_WORLD_ALIGNED);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_6D, model, model.getJointId(LF), LOCAL);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));

  computeJointJacobians(model, data_ref, q);
  data_ref.q_in = q;
  calc(model, data_ref, contact_models, contact_datas);

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

  const int constraint_size = 12;
  const int total_dim = model.nv + constraint_size;

  Data::MatrixXs H(total_dim, total_dim);
  H.setZero();
  H.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  H.middleRows<6>(0).rightCols(model.nv) = J_RF;
  H.middleRows<6>(6).rightCols(model.nv) = J_LF;

  H.triangularView<Eigen::StrictlyLower>() = H.triangularView<Eigen::StrictlyUpper>().transpose();

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  ConstraintCholeskyDecomposition constraint_chol_decomposition;
  constraint_chol_decomposition.rebuild(model, data, contact_models, contact_datas);
  constraint_chol_decomposition.compute(model, data, contact_models, contact_datas);

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();

  Data::MatrixXs H_recomposed = constraint_chol_decomposition.matrix();

  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv, model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_size, model.nv)
                .isApprox(H.topRightCorner(constraint_size, model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));

  // inverse
  MatrixXd H_inv(constraint_chol_decomposition.size(), constraint_chol_decomposition.size());
  constraint_chol_decomposition.inverse(H_inv);

  MatrixXd H_inv_ref = H_recomposed.inverse();
  BOOST_CHECK(H_inv.isApprox(H_inv_ref));

  // test Operational Space Inertia Matrix
  MatrixXd JMinvJt = H.middleRows<12>(0).rightCols(model.nv) * data_ref.M.inverse()
                     * H.middleRows<12>(0).rightCols(model.nv).transpose();
  MatrixXd iosim = constraint_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
  MatrixXd osim = constraint_chol_decomposition.getOperationalSpaceInertiaMatrix();

  BOOST_CHECK(iosim.isApprox(JMinvJt));
  BOOST_CHECK(osim.isApprox(JMinvJt.inverse()));

  // test Mass matrix cholesky
  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  constraint_chol_decomposition.getInverseMassMatrix(Minv_test);

  Eigen::MatrixXd JMinv_test(Eigen::MatrixXd::Zero(12, model.nv));
  constraint_chol_decomposition.getJMinv(JMinv_test);
  MatrixXd JMinv_ref = H.middleRows<12>(0).rightCols(model.nv) * data_ref.M.inverse();
  BOOST_CHECK(JMinv_ref.isApprox(JMinv_test));

  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));
}

BOOST_AUTO_TEST_CASE(constraint_cholesky_contact6D_by_joint_2)
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

  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;
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

  computeJointJacobians(model, data_ref, q);
  data_ref.q_in = q;
  calc(model, data_ref, contact_models, contact_datas);

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

  const int constraint_size = 24;
  const int total_dim = model.nv + constraint_size;

  const SE3 oMRA_wla = SE3(SE3::Matrix3::Identity(), ci_RA.joint1_placement.translation());

  const double mu = 0.1;
  Data::MatrixXs H(total_dim, total_dim);
  H.setZero();
  H.topLeftCorner(constraint_size, constraint_size).diagonal().fill(-mu);
  H.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  H.middleRows<6>(0).rightCols(model.nv) = -ci_RF.joint1_placement.toActionMatrixInverse() * J_RF;
  H.middleRows<6>(6).rightCols(model.nv) = -ci_LF.joint1_placement.toActionMatrixInverse() * J_LF;
  H.middleRows<6>(12).rightCols(model.nv) = -oMRA_wla.toActionMatrixInverse() * J_RA;
  H.middleRows<6>(18).rightCols(model.nv) = -ci_LA.joint1_placement.toActionMatrixInverse() * J_LA;

  H.triangularView<Eigen::StrictlyLower>() = H.triangularView<Eigen::StrictlyUpper>().transpose();

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  ConstraintCholeskyDecomposition constraint_chol_decomposition;
  constraint_chol_decomposition.rebuild(model, data, contact_models, contact_datas);
  constraint_chol_decomposition.compute(model, data, contact_models, contact_datas, mu);

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();

  Data::MatrixXs H_recomposed = constraint_chol_decomposition.matrix();

  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv, model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_size, model.nv)
                .isApprox(H.topRightCorner(constraint_size, model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));

  VectorXd v_in(VectorXd::Random(constraint_chol_decomposition.size()));
  MatrixXd mat_in(MatrixXd::Random(constraint_chol_decomposition.size(), 20));

  // Test Uv
  VectorXd Uv_op_res(v_in), Uv_op_ref(v_in);

  constraint_chol_decomposition.Uv(Uv_op_res);
  Uv_op_ref.noalias() = constraint_chol_decomposition.U * v_in;

  BOOST_CHECK(Uv_op_res.isApprox(Uv_op_ref));

  MatrixXd Uv_mat_op_res(mat_in), Uv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Uv(Uv_mat_op_res);
  Uv_mat_op_ref.noalias() = constraint_chol_decomposition.U * mat_in;

  BOOST_CHECK(Uv_mat_op_res.isApprox(Uv_mat_op_ref));

  // Test Utv
  VectorXd Utv_op_res(v_in), Utv_op_ref(v_in);

  constraint_chol_decomposition.Utv(Utv_op_res);
  Utv_op_ref.noalias() = constraint_chol_decomposition.U.transpose() * v_in;

  BOOST_CHECK(Utv_op_res.isApprox(Utv_op_ref));

  MatrixXd Utv_mat_op_res(mat_in), Utv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Utv(Utv_mat_op_res);
  Utv_mat_op_ref.noalias() = constraint_chol_decomposition.U.transpose() * mat_in;

  BOOST_CHECK(Utv_mat_op_res.isApprox(Utv_mat_op_ref));

  // Test Uiv
  VectorXd Uiv_op_res(v_in), Uiv_op_ref(v_in);

  constraint_chol_decomposition.Uiv(Uiv_op_res);
  Uiv_op_ref.noalias() = constraint_chol_decomposition.U.inverse() * v_in;

  BOOST_CHECK(Uiv_op_res.isApprox(Uiv_op_ref));

  MatrixXd Uiv_mat_op_res(mat_in), Uiv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Uiv(Uiv_mat_op_res);
  Uiv_mat_op_ref.noalias() = constraint_chol_decomposition.U.inverse() * mat_in;

  BOOST_CHECK(Uiv_mat_op_res.isApprox(Uiv_mat_op_ref));

  // Test Utiv
  VectorXd Utiv_op_res(v_in), Utiv_op_ref(v_in);

  constraint_chol_decomposition.Utiv(Utiv_op_res);
  Utiv_op_ref.noalias() = constraint_chol_decomposition.U.inverse().transpose() * v_in;

  BOOST_CHECK(Utiv_op_res.isApprox(Utiv_op_ref));

  MatrixXd Utiv_mat_op_res(mat_in), Utiv_mat_op_ref(mat_in);

  constraint_chol_decomposition.Utiv(Utiv_mat_op_res);
  Utiv_mat_op_ref.noalias() = constraint_chol_decomposition.U.inverse().transpose() * mat_in;

  BOOST_CHECK(Utiv_mat_op_res.isApprox(Utiv_mat_op_ref));

  // inverse
  MatrixXd H_inv(constraint_chol_decomposition.size(), constraint_chol_decomposition.size());
  constraint_chol_decomposition.inverse(H_inv);

  MatrixXd H_inv2 = constraint_chol_decomposition.U.transpose().inverse()
                    * constraint_chol_decomposition.Dinv.asDiagonal()
                    * constraint_chol_decomposition.U.inverse();

  MatrixXd H_inv3 =
    MatrixXd::Identity(constraint_chol_decomposition.size(), constraint_chol_decomposition.size());
  constraint_chol_decomposition.solveInPlace(H_inv3);

  MatrixXd H_inv_ref = H_recomposed.inverse();
  BOOST_CHECK(H_inv.topLeftCorner(constraint_size, constraint_size)
                .isApprox(H_inv_ref.topLeftCorner(constraint_size, constraint_size)));
  BOOST_CHECK(H_inv.bottomRightCorner(model.nv, model.nv)
                .isApprox(H_inv_ref.bottomRightCorner(model.nv, model.nv)));
  BOOST_CHECK(H_inv.topRightCorner(constraint_size, model.nv)
                .isApprox(H_inv_ref.topRightCorner(constraint_size, model.nv)));

  BOOST_CHECK(H_inv_ref.isApprox(H_inv));
  BOOST_CHECK(H_inv_ref.isApprox(H_inv2));
  BOOST_CHECK(H_inv_ref.isApprox(H_inv3));
  const VectorXd ei = VectorXd::Unit(constraint_chol_decomposition.size(), constraint_size);
  VectorXd ei_inv = ei;
  constraint_chol_decomposition.solveInPlace(ei_inv);
  VectorXd ei_inv2 = ei;
  constraint_chol_decomposition.Uiv(ei_inv2);
  ei_inv2 = constraint_chol_decomposition.Dinv.asDiagonal() * ei_inv2;
  constraint_chol_decomposition.Utiv(ei_inv2);

  BOOST_CHECK(ei_inv.isApprox(H_inv_ref * ei));
  BOOST_CHECK(ei_inv2.isApprox(H_inv_ref * ei));
  BOOST_CHECK(ei_inv.isApprox(H_inv * ei));
  BOOST_CHECK(ei_inv2.isApprox(H_inv * ei));

  // test Operational Space Inertia Matrix
  MatrixXd JMinvJt = H.middleRows<24>(0).rightCols(model.nv) * data_ref.M.inverse()
                       * H.middleRows<24>(0).rightCols(model.nv).transpose()
                     + mu * Eigen::MatrixXd::Identity(24, 24);
  MatrixXd iosim = constraint_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
  MatrixXd osim = constraint_chol_decomposition.getOperationalSpaceInertiaMatrix();

  BOOST_CHECK(iosim.isApprox(JMinvJt));
  BOOST_CHECK(osim.isApprox(JMinvJt.inverse()));

  // test Mass matrix cholesky
  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  constraint_chol_decomposition.getInverseMassMatrix(Minv_test);

  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));
  Eigen::MatrixXd JMinv_test(Eigen::MatrixXd::Zero(24, model.nv));
  constraint_chol_decomposition.getJMinv(JMinv_test);
  MatrixXd JMinv_ref = H.middleRows<24>(0).rightCols(model.nv) * data_ref.M.inverse();
  BOOST_CHECK(JMinv_ref.isApprox(JMinv_test));
}

BOOST_AUTO_TEST_CASE(constraint_cholesky_contact3D_6D_WORLD_by_joint_2)
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

  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;
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

  computeJointJacobians(model, data_ref, q);
  data_ref.q_in = q;
  calc(model, data_ref, contact_models, contact_datas);

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

  const int constraint_size = 15;
  const int total_dim = model.nv + constraint_size;

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
  ConstraintCholeskyDecomposition constraint_chol_decomposition;
  constraint_chol_decomposition.rebuild(model, data, contact_models, contact_datas);
  constraint_chol_decomposition.compute(model, data, contact_models, contact_datas);

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();

  BOOST_CHECK(data_ref.D.isApprox(constraint_chol_decomposition.D.tail(model.nv)));
  BOOST_CHECK(data_ref.Dinv.isApprox(constraint_chol_decomposition.Dinv.tail(model.nv)));
  BOOST_CHECK(
    data_ref.U.isApprox(constraint_chol_decomposition.U.bottomRightCorner(model.nv, model.nv)));

  Data::MatrixXs M_recomposed =
    constraint_chol_decomposition.U.bottomRightCorner(model.nv, model.nv)
    * constraint_chol_decomposition.D.tail(model.nv).asDiagonal()
    * constraint_chol_decomposition.U.bottomRightCorner(model.nv, model.nv).transpose();
  BOOST_CHECK(M_recomposed.isApprox(data.M));

  Data::MatrixXs H_recomposed = constraint_chol_decomposition.U
                                * constraint_chol_decomposition.D.asDiagonal()
                                * constraint_chol_decomposition.U.transpose();

  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv, model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_size, model.nv)
                .isApprox(H.topRightCorner(constraint_size, model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));

  // test Operational Space Inertia Matrix
  MatrixXd JMinvJt = H.middleRows<15>(0).rightCols(model.nv) * data_ref.M.inverse()
                     * H.middleRows<15>(0).rightCols(model.nv).transpose();
  MatrixXd iosim = constraint_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
  MatrixXd osim = constraint_chol_decomposition.getOperationalSpaceInertiaMatrix();

  BOOST_CHECK(iosim.isApprox(JMinvJt));
  BOOST_CHECK(osim.isApprox(JMinvJt.inverse()));

  // test Mass matrix cholesky
  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  constraint_chol_decomposition.getInverseMassMatrix(Minv_test);

  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));
  Eigen::MatrixXd JMinv_test(Eigen::MatrixXd::Zero(15, model.nv));
  constraint_chol_decomposition.getJMinv(JMinv_test);
  MatrixXd JMinv_ref = H.middleRows<15>(0).rightCols(model.nv) * data_ref.M.inverse();
  BOOST_CHECK(JMinv_ref.isApprox(JMinv_test));
}

BOOST_AUTO_TEST_CASE(loop_constraint_cholesky_contact6D)
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

  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;
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

  computeJointJacobians(model, data_ref, q);
  data_ref.q_in = q;
  calc(model, data_ref, contact_models, contact_datas);

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

  const int constraint_size = 12;
  const int total_dim = model.nv + constraint_size;

  Data::MatrixXs H(total_dim, total_dim);
  H.setZero();
  H.topLeftCorner(constraint_size, constraint_size).diagonal().fill(-mu);
  H.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  H.middleRows<6>(0).rightCols(model.nv) = J_RF_local - _1M2_loop1.toActionMatrix() * J_LF_local;
  const SE3 oM1_loop2_lwa = SE3(oM1_loop2.rotation(), SE3::Vector3::Zero());
  H.middleRows<6>(6).rightCols(model.nv) =
    oM1_loop2_lwa.toActionMatrix() * J_RA_local
    - (oM1_loop2_lwa.toActionMatrix() * _1M2_loop2.toActionMatrix()) * J_LA_local;

  H.triangularView<Eigen::StrictlyLower>() = H.triangularView<Eigen::StrictlyUpper>().transpose();

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  ConstraintCholeskyDecomposition constraint_chol_decomposition;
  constraint_chol_decomposition.rebuild(model, data, contact_models, contact_datas);
  constraint_chol_decomposition.compute(model, data, contact_models, contact_datas, mu);

  Data::MatrixXs H_recomposed = constraint_chol_decomposition.matrix();

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();
  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv, model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_size, model.nv)
                .isApprox(H.topRightCorner(constraint_size, model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));

  // inverse
  MatrixXd H_inv(constraint_chol_decomposition.size(), constraint_chol_decomposition.size());
  constraint_chol_decomposition.inverse(H_inv);

  MatrixXd H_inv_ref = H_recomposed.inverse();

  BOOST_CHECK(H_inv_ref.isApprox(H_inv));

  // test Operational Space Inertia Matrix
  MatrixXd JMinvJt = H.middleRows<12>(0).rightCols(model.nv) * data_ref.M.inverse()
                       * H.middleRows<12>(0).rightCols(model.nv).transpose()
                     + mu * Eigen::MatrixXd::Identity(12, 12);
  MatrixXd iosim = constraint_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
  MatrixXd osim = constraint_chol_decomposition.getOperationalSpaceInertiaMatrix();

  BOOST_CHECK(iosim.isApprox(JMinvJt));
  BOOST_CHECK(osim.isApprox(JMinvJt.inverse()));

  // test Mass matrix cholesky
  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  constraint_chol_decomposition.getInverseMassMatrix(Minv_test);

  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));
  Eigen::MatrixXd JMinv_test(Eigen::MatrixXd::Zero(12, model.nv));
  constraint_chol_decomposition.getJMinv(JMinv_test);
  MatrixXd JMinv_ref = H.middleRows<12>(0).rightCols(model.nv) * data_ref.M.inverse();
  BOOST_CHECK(JMinv_ref.isApprox(JMinv_test));
}

BOOST_AUTO_TEST_CASE(loop_constraint_cholesky_contact_3d)
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

  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;
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

  computeJointJacobians(model, data_ref, q);
  data_ref.q_in = q;
  calc(model, data_ref, contact_models, contact_datas);

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

  const int constraint_size = 6;
  const int total_dim = model.nv + constraint_size;

  Data::MatrixXs H(total_dim, total_dim);
  H.setZero();
  H.topLeftCorner(constraint_size, constraint_size).diagonal().fill(-mu);
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
  ConstraintCholeskyDecomposition constraint_chol_decomposition;
  constraint_chol_decomposition.rebuild(model, data, contact_models, contact_datas);
  constraint_chol_decomposition.compute(model, data, contact_models, contact_datas, mu);
  BOOST_CHECK(contact_datas[0].c1Mc2.isApprox(_1M2_loop1));
  BOOST_CHECK(contact_datas[1].c1Mc2.isApprox(_1M2_loop2));

  Data::MatrixXs H_recomposed = constraint_chol_decomposition.matrix();

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();
  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv, model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_size, model.nv)
                .isApprox(H.topRightCorner(constraint_size, model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));

  // std::cout << "H_recomposed.topRightCorner(constraint_size,model.nv):\n" <<
  // H_recomposed.topRightCorner(constraint_size,model.nv) << std::endl; std::cout <<
  // "H.topRightCorner(constraint_size,model.nv):\n" << H  .topRightCorner(constraint_size,model.nv)
  // << std::endl;

  // inverse
  MatrixXd H_inv(constraint_chol_decomposition.size(), constraint_chol_decomposition.size());
  constraint_chol_decomposition.inverse(H_inv);

  MatrixXd H_inv_ref = H_recomposed.inverse();

  BOOST_CHECK(H_inv_ref.isApprox(H_inv));

  // test Operational Space Inertia Matrix
  MatrixXd JMinvJt = H.middleRows<6>(0).rightCols(model.nv) * data_ref.M.inverse()
                       * H.middleRows<6>(0).rightCols(model.nv).transpose()
                     + mu * Eigen::MatrixXd::Identity(6, 6);
  MatrixXd iosim = constraint_chol_decomposition.getInverseOperationalSpaceInertiaMatrix();
  MatrixXd osim = constraint_chol_decomposition.getOperationalSpaceInertiaMatrix();

  BOOST_CHECK(iosim.isApprox(JMinvJt));
  BOOST_CHECK(osim.isApprox(JMinvJt.inverse()));

  // test Mass matrix cholesky
  data_ref.Minv = data_ref.M.inverse();
  Eigen::MatrixXd Minv_test(Eigen::MatrixXd::Zero(model.nv, model.nv));
  constraint_chol_decomposition.getInverseMassMatrix(Minv_test);

  Eigen::MatrixXd JMinv_test(Eigen::MatrixXd::Zero(6, model.nv));
  constraint_chol_decomposition.getJMinv(JMinv_test);
  MatrixXd JMinv_ref = H.middleRows<6>(0).rightCols(model.nv) * data_ref.M.inverse();
  BOOST_CHECK(JMinv_ref.isApprox(JMinv_test));
  BOOST_CHECK(Minv_test.isApprox(data_ref.Minv));
}

BOOST_AUTO_TEST_CASE(constraint_cholesky_updateDamping)
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

  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D, model, model.getJointId(RF), LOCAL);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_6D, model, model.getJointId(LF), LOCAL);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));

  computeJointJacobians(model, data_ref, q);
  data_ref.q_in = q;
  calc(model, data_ref, contact_models, contact_datas);

  Data data(model);
  crba(model, data, q, Convention::WORLD);

  const double mu1 = 1e-2, mu2 = 1e-10;

  {
    ConstraintCholeskyDecomposition constraint_chol_decomposition;
    constraint_chol_decomposition.rebuild(model, data, contact_models, contact_datas);

    constraint_chol_decomposition.compute(model, data, contact_models, contact_datas, mu1);
    BOOST_CHECK(constraint_chol_decomposition.getDamping().diagonal().isConstant(mu1));

    constraint_chol_decomposition.updateDamping(mu2);
    BOOST_CHECK(constraint_chol_decomposition.getDamping().diagonal().isConstant(mu2));
    constraint_chol_decomposition.computeDelassusCholeskyDecomposition();

    ConstraintCholeskyDecomposition constraint_chol_decomposition_ref;
    constraint_chol_decomposition_ref.rebuild(model, data, contact_models, contact_datas);
    constraint_chol_decomposition_ref.compute(model, data, contact_models, contact_datas, mu2);

    BOOST_CHECK(constraint_chol_decomposition.D.isApprox(constraint_chol_decomposition_ref.D));
    BOOST_CHECK(
      constraint_chol_decomposition.Dinv.isApprox(constraint_chol_decomposition_ref.Dinv));
    BOOST_CHECK(constraint_chol_decomposition.U.isApprox(constraint_chol_decomposition_ref.U));
  }

  {
    ConstraintCholeskyDecomposition constraint_chol_decomposition;
    constraint_chol_decomposition.rebuild(model, data, contact_models, contact_datas);
    constraint_chol_decomposition.compute(model, data, contact_models, contact_datas, mu1);
    constraint_chol_decomposition.getDelassusOperatorCholeskyExpression().updateDamping(mu2);
    constraint_chol_decomposition.getDelassusOperatorCholeskyExpression().updateDecomposition();

    ConstraintCholeskyDecomposition constraint_chol_decomposition_ref;
    constraint_chol_decomposition_ref.rebuild(model, data, contact_models, contact_datas);
    constraint_chol_decomposition_ref.compute(model, data, contact_models, contact_datas, mu2);

    BOOST_CHECK(constraint_chol_decomposition.D.isApprox(constraint_chol_decomposition_ref.D));
    BOOST_CHECK(
      constraint_chol_decomposition.Dinv.isApprox(constraint_chol_decomposition_ref.Dinv));
    BOOST_CHECK(constraint_chol_decomposition.U.isApprox(constraint_chol_decomposition_ref.U));
  }
}

BOOST_AUTO_TEST_CASE(constraint_cholesky_joint_friction_constraint)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);

  const Eigen::VectorXd q = neutral(model);

  const std::string RF_name = "rleg6_joint";
  const JointIndex RF_id = model.getJointId(RF_name);

  const Model::IndexVector & RF_support = model.supports[RF_id];
  const Model::IndexVector active_joint_ids(RF_support.begin() + 1, RF_support.end());

  JointFrictionConstraintModel constraint_model(model, active_joint_ids);
  JointFrictionConstraintData constraint_data(constraint_model);

  std::vector<JointFrictionConstraintModel> constraint_models;
  constraint_models.push_back(constraint_model);
  std::vector<JointFrictionConstraintData> constraint_datas;
  constraint_datas.push_back(constraint_data);

  // Build Cholesky decomposition
  Data data(model);
  crba(model, data, q, Convention::WORLD);
  computeJointJacobians(model, data, q);
  calc(model, data, constraint_models, constraint_datas);

  ConstraintCholeskyDecomposition constraint_chol_decomposition;
  constraint_chol_decomposition.rebuild(model, data, constraint_models, constraint_datas);

  // Compute decompositions
  const double mu = 1e-10;
  constraint_chol_decomposition.compute(model, data, constraint_models, constraint_datas, mu);

  const int constraint_size = constraint_model.residualSize();
  const int total_dim = model.nv + constraint_size;
  Data::MatrixXs H(total_dim, total_dim);
  H.setZero();
  H.topLeftCorner(constraint_size, constraint_size).diagonal().fill(-mu);
  H.bottomRightCorner(model.nv, model.nv) = data.M;
  constraint_model.jacobian(
    model, data, constraint_data, H.middleRows(0, constraint_size).rightCols(model.nv));

  H.triangularView<Eigen::StrictlyLower>() = H.triangularView<Eigen::StrictlyUpper>().transpose();

  BOOST_CHECK(constraint_chol_decomposition.matrix().isApprox(H));
  BOOST_CHECK(constraint_chol_decomposition.matrix().middleRows(6, 6).rightCols(model.nv).isApprox(
    H.middleRows(6, 6).rightCols(model.nv)));

  UDUt<Data::MatrixXs> udut(H);

  BOOST_CHECK(udut.D.isApprox(constraint_chol_decomposition.D));
  BOOST_CHECK(udut.U.isApprox(constraint_chol_decomposition.U));
}

BOOST_AUTO_TEST_CASE(constraint_cholesky_model_generic)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  const std::string RF_name = "rleg6_joint";
  const std::string LF_name = "lleg6_joint";

  std::vector<ConstraintModel> constraint_models;
  std::vector<ConstraintData> constraint_datas;

  std::vector<PointAnchorConstraintModel> rigid_constraint_models;
  std::vector<PointAnchorConstraintData> rigid_constraint_datas;

  PointAnchorConstraintModel ci_RF(
    model, 0, SE3::Identity(), model.getJointId(RF_name), SE3::Identity());
  constraint_models.push_back(ci_RF);
  rigid_constraint_models.push_back(ci_RF);
  constraint_datas.push_back(PointAnchorConstraintData(ci_RF));
  rigid_constraint_datas.push_back(PointAnchorConstraintData(ci_RF));
  PointAnchorConstraintModel ci_LF(
    model, 0, SE3::Identity(), model.getJointId(LF_name), SE3::Identity());
  constraint_models.push_back(ci_LF);
  rigid_constraint_models.push_back(ci_LF);
  constraint_datas.push_back(PointAnchorConstraintData(ci_LF));
  rigid_constraint_datas.push_back(PointAnchorConstraintData(ci_LF));

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  computeJointJacobians(model, data, q);
  data.q_in = q;
  calc(model, data, constraint_models, constraint_datas);
  calc(model, data, rigid_constraint_models, rigid_constraint_datas);

  ConstraintCholeskyDecomposition constraint_chol_decomposition, constraint_chol_decomposition_ref;
  constraint_chol_decomposition.rebuild(model, data, constraint_models, constraint_datas);
  constraint_chol_decomposition_ref.rebuild(
    model, data, rigid_constraint_models, rigid_constraint_datas);

  const double mu = 1e-10;
  constraint_chol_decomposition.compute(model, data, constraint_models, constraint_datas, mu);
  constraint_chol_decomposition_ref.compute(
    model, data, rigid_constraint_models, rigid_constraint_datas, mu);

  BOOST_CHECK(constraint_chol_decomposition.D.isApprox(constraint_chol_decomposition_ref.D));
  BOOST_CHECK(constraint_chol_decomposition.Dinv.isApprox(constraint_chol_decomposition_ref.Dinv));
  BOOST_CHECK(constraint_chol_decomposition.U.isApprox(constraint_chol_decomposition_ref.U));

  // Test sizeInBytes
  {
    BOOST_CHECK(constraint_chol_decomposition.sizeInBytes() > 0);
    BOOST_CHECK(
      constraint_chol_decomposition.sizeInBytes()
      > constraint_chol_decomposition.getDamping().sizeInBytes());
    BOOST_CHECK(
      constraint_chol_decomposition.sizeInBytes()
      == constraint_chol_decomposition_ref.sizeInBytes());
  }
}

BOOST_AUTO_TEST_CASE(constraint_cholesky_dynamic_size)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  // First, test not activable limits on joints
  {
    model.lowerPositionLimit.setConstant(-std::numeric_limits<double>::max());
    model.upperPositionLimit.setConstant(std::numeric_limits<double>::max());
    VectorXd q = pinocchio::neutral(model);
    data.q_in = q;

    std::vector<ConstraintModel> constraint_models;
    std::vector<ConstraintData> constraint_datas;

    JointLimitConstraintModel::JointIndexVector joint_indices;
    for (int i = 1; i < model.njoints; ++i)
    {
      joint_indices.push_back((Model::JointIndex)i);
    }
    model.positionLimitMargin.setZero();
    JointLimitConstraintModel joint_limit_constraint_model(model, joint_indices);
    constraint_models.push_back(joint_limit_constraint_model);
    // No activable joint limits because min/max position limit is infinity
    BOOST_CHECK(constraint_models[0].residualSize(MaximalSelection()) == 0);
    BOOST_CHECK(constraint_models[0].residualSize(CurrentSelection()) == 0);

    const std::string RF_name = "rleg6_joint";
    const std::string LF_name = "lleg6_joint";

    PointAnchorConstraintModel ci_RF(
      model, 0, SE3::Identity(), model.getJointId(RF_name), SE3::Identity());
    constraint_models.push_back(ci_RF);
    PointAnchorConstraintModel ci_LF(
      model, 0, SE3::Identity(), model.getJointId(LF_name), SE3::Identity());
    constraint_models.push_back(ci_LF);

    for (const auto & cm : constraint_models)
      constraint_datas.push_back(cm.createData());

    crba(model, data, q, Convention::WORLD);
    computeJointJacobians(model, data, q);
    data.q_in = q;
    calc(model, data, constraint_models, constraint_datas);

    // No active joint limits
    BOOST_CHECK(constraint_models[0].residualSize() == 0);
    // Size of point anchor constraints should be 3
    BOOST_CHECK(constraint_models[1].residualSize() == 3);
    BOOST_CHECK(constraint_models[2].residualSize() == 3);

    ConstraintCholeskyDecomposition constraint_chol_decomposition,
      constraint_chol_decomposition_ref;
    constraint_chol_decomposition.rebuild(model, data, constraint_models, constraint_datas);

    const double mu = 1e-10;
    constraint_chol_decomposition.compute(model, data, constraint_models, constraint_datas, mu);

    // Only point anchor constraints should be active
    BOOST_CHECK(constraint_chol_decomposition.constraintDim() == 6);

    // BOOST_CHECK(constraint_chol_decomposition.D.isApprox(constraint_chol_decomposition_ref.D));
    // BOOST_CHECK(constraint_chol_decomposition.Dinv.isApprox(constraint_chol_decomposition_ref.Dinv));
    // BOOST_CHECK(constraint_chol_decomposition.U.isApprox(constraint_chol_decomposition_ref.U));
  }

  // Second, test activable but unactive limits on joints
  {
    model.lowerPositionLimit.setConstant(-1e4);
    model.upperPositionLimit.setConstant(1e4);
    VectorXd q = pinocchio::neutral(model);
    data.q_in = q;

    std::vector<ConstraintModel> constraint_models;
    std::vector<ConstraintData> constraint_datas;

    JointLimitConstraintModel::JointIndexVector joint_indices;
    for (int i = 1; i < model.njoints; ++i)
    {
      joint_indices.push_back((Model::JointIndex)i);
    }
    model.positionLimitMargin.setZero();
    JointLimitConstraintModel joint_limit_constraint_model(model, joint_indices);
    constraint_models.emplace_back(joint_limit_constraint_model);
    // Activable joint limits (only the rotation part of the freeflyer is not activable)
    BOOST_CHECK(constraint_models[0].residualSize(MaximalSelection()) == 2 * (model.nv - 3));
    BOOST_CHECK(constraint_models[0].residualSize(CurrentSelection()) == 2 * (model.nv - 3));

    const std::string RF_name = "rleg6_joint";
    const std::string LF_name = "lleg6_joint";

    PointAnchorConstraintModel ci_RF(
      model, 0, SE3::Identity(), model.getJointId(RF_name), SE3::Identity());
    constraint_models.push_back(ci_RF);
    PointAnchorConstraintModel ci_LF(
      model, 0, SE3::Identity(), model.getJointId(LF_name), SE3::Identity());
    constraint_models.push_back(ci_LF);

    for (const auto & cm : constraint_models)
      constraint_datas.push_back(cm.createData());

    crba(model, data, q, Convention::WORLD);
    computeJointJacobians(model, data, q);
    data.q_in = q;
    calc(model, data, constraint_models, constraint_datas);
    BOOST_CHECK(constraint_models[0].residualSize() == 2 * (model.nv - 3));

    // Now only active the one near limit
    joint_limit_constraint_model.makeSelectionFilteredByLimitProximity(q);
    constraint_models[0] = ConstraintModel(joint_limit_constraint_model);
    calc(model, data, constraint_models, constraint_datas);

    BOOST_CHECK(constraint_models[0].residualSize() == 0);
    BOOST_CHECK(constraint_models[0].residualSize(MaximalSelection()) == 2 * (model.nv - 3));

    ConstraintCholeskyDecomposition constraint_chol_decomposition,
      constraint_chol_decomposition_ref;
    constraint_chol_decomposition.rebuild(model, data, constraint_models, constraint_datas);

    const double mu = 1e-10;
    constraint_chol_decomposition.compute(model, data, constraint_models, constraint_datas, mu);

    // Only point anchor constraints should be active
    BOOST_CHECK(constraint_chol_decomposition.constraintDim() == 6);

    // BOOST_CHECK(constraint_chol_decomposition.D.isApprox(constraint_chol_decomposition_ref.D));
    // BOOST_CHECK(constraint_chol_decomposition.Dinv.isApprox(constraint_chol_decomposition_ref.Dinv));
    // BOOST_CHECK(constraint_chol_decomposition.U.isApprox(constraint_chol_decomposition_ref.U));
  }

  // Third, test only active lower limits on joints
  {
    model.lowerPositionLimit.setConstant(0);
    model.upperPositionLimit.setConstant(1e4);
    VectorXd q = pinocchio::neutral(model);
    data.q_in = q;

    std::vector<ConstraintModel> constraint_models;
    std::vector<ConstraintData> constraint_datas;

    JointLimitConstraintModel::JointIndexVector joint_indices;
    for (int i = 1; i < model.njoints; ++i)
    {
      joint_indices.push_back((Model::JointIndex)i);
    }
    model.positionLimitMargin.setZero();
    JointLimitConstraintModel joint_limit_constraint_model(model, joint_indices);
    BOOST_CHECK(
      joint_limit_constraint_model.residualSize(MaximalSelection()) == 2 * (model.nv - 3));
    BOOST_CHECK(
      joint_limit_constraint_model.residualSize(CurrentSelection()) == 2 * (model.nv - 3));
    BOOST_CHECK(joint_limit_constraint_model.residualSize() == 2 * (model.nv - 3));
    joint_limit_constraint_model.makeSelectionFilteredByLimitProximity(q);
    BOOST_CHECK(
      joint_limit_constraint_model.residualSize(MaximalSelection()) == 2 * (model.nv - 3));
    BOOST_CHECK(joint_limit_constraint_model.residualSize(CurrentSelection()) == model.nv - 3);
    BOOST_CHECK(joint_limit_constraint_model.residualSize() == model.nv - 3);
    constraint_models.push_back(joint_limit_constraint_model);
    BOOST_CHECK(constraint_models[0].residualSize(MaximalSelection()) == 2 * (model.nv - 3));
    BOOST_CHECK(constraint_models[0].residualSize(CurrentSelection()) == (model.nv - 3));
    BOOST_CHECK(constraint_models[0].residualSize() == (model.nv - 3));

    // Activable joint limits (only the rotation part of the freeflyer is not activable)

    const std::string RF_name = "rleg6_joint";
    const std::string LF_name = "lleg6_joint";

    PointAnchorConstraintModel ci_RF(
      model, 0, SE3::Identity(), model.getJointId(RF_name), SE3::Identity());
    constraint_models.push_back(ci_RF);
    PointAnchorConstraintModel ci_LF(
      model, 0, SE3::Identity(), model.getJointId(LF_name), SE3::Identity());
    constraint_models.push_back(ci_LF);

    for (const auto & cm : constraint_models)
      constraint_datas.push_back(cm.createData());

    crba(model, data, q, Convention::WORLD);
    computeJointJacobians(model, data, q);
    data.q_in = q;
    calc(model, data, constraint_models, constraint_datas);

    ConstraintCholeskyDecomposition constraint_chol_decomposition,
      constraint_chol_decomposition_ref;
    constraint_chol_decomposition.rebuild(model, data, constraint_models, constraint_datas);

    const double mu = 1e-10;
    constraint_chol_decomposition.compute(model, data, constraint_models, constraint_datas, mu);

    // Bilateral constraints and lower limits should be active (except for the rotation part of the
    // freeflyer)
    BOOST_CHECK(constraint_chol_decomposition.constraintDim() == 6 + (model.nv - 3));

    // BOOST_CHECK(constraint_chol_decomposition.D.isApprox(constraint_chol_decomposition_ref.D));
    // BOOST_CHECK(constraint_chol_decomposition.Dinv.isApprox(constraint_chol_decomposition_ref.Dinv));
    // BOOST_CHECK(constraint_chol_decomposition.U.isApprox(constraint_chol_decomposition_ref.U));
  }
}

BOOST_AUTO_TEST_CASE(constraint_cholesky_model_with_compliance)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  const std::string RF_name = "rleg6_joint";
  const std::string LF_name = "lleg6_joint";

  std::vector<ConstraintModel> constraint_models;
  std::vector<ConstraintData> constraint_datas;

  PointAnchorConstraintModel ci_RF(
    model, 0, SE3::Identity(), model.getJointId(RF_name), SE3::Identity());
  ci_RF.setCompliance(Eigen::VectorXd::Constant(3, 6e-4));
  constraint_models.push_back(ci_RF);
  constraint_datas.push_back(PointAnchorConstraintData(ci_RF));
  PointAnchorConstraintModel ci_LF(
    model, 0, SE3::Identity(), model.getJointId(LF_name), SE3::Identity());
  ci_LF.setCompliance(Eigen::VectorXd::Constant(3, 7e-4));
  constraint_models.push_back(ci_LF);
  constraint_datas.push_back(PointAnchorConstraintData(ci_LF));

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  computeJointJacobians(model, data, q);
  data.q_in = q;
  calc(model, data, constraint_models, constraint_datas);

  ConstraintCholeskyDecomposition constraint_chol_decomposition, constraint_chol_decomposition_ref;
  constraint_chol_decomposition.rebuild(model, data, constraint_models, constraint_datas);

  const double mu = 1e-10;
  constraint_chol_decomposition.compute(model, data, constraint_models, constraint_datas, mu);

  auto delassus_chol = constraint_chol_decomposition.getDelassusOperatorCholeskyExpression();

  Eigen::VectorXd compliance_vec(6);
  compliance_vec.head(3) = Eigen::VectorXd::Constant(3, 6e-4);
  compliance_vec.tail(3) = Eigen::VectorXd::Constant(3, 7e-4);

  BOOST_CHECK(delassus_chol.getDamping().matrix().isApprox(mu * Eigen::MatrixXd::Identity(6, 6)));
  BOOST_CHECK(delassus_chol.getCompliance().isApprox(compliance_vec));

  const double new_mu = 1e-3;
  delassus_chol.updateDamping(new_mu);
  const double new_compliance = 1e1;
  delassus_chol.updateCompliance(new_compliance);

  BOOST_CHECK(
    delassus_chol.getDamping().matrix().isApprox(new_mu * Eigen::MatrixXd::Identity(6, 6)));
  BOOST_CHECK(delassus_chol.getCompliance().isApprox(Eigen::VectorXd::Constant(6, new_compliance)));
}

BOOST_AUTO_TEST_CASE(constraint_cholesky_check_resize)
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

  typedef std::vector<RigidConstraintModel> RigidConstraintModelStdVector;
  typedef std::vector<RigidConstraintData> RigidConstraintDataStdVector;

  RigidConstraintModelStdVector contact_models;
  RigidConstraintDataStdVector contact_datas;

  {
    RigidConstraintModel ci_RF(CONTACT_6D, model, model.getJointId(RF), LOCAL);
    contact_models.push_back(ci_RF);
    contact_datas.push_back(RigidConstraintData(ci_RF));
    RigidConstraintModel ci_LF(CONTACT_6D, model, model.getJointId(LF), LOCAL);
    contact_models.push_back(ci_LF);
    contact_datas.push_back(RigidConstraintData(ci_LF));
  }

  Data data(model);
  crba(model, data, q, Convention::WORLD);
  computeJointJacobians(model, data, q);
  data.q_in = q;
  calc(model, data, contact_models, contact_datas);
  ConstraintCholeskyDecomposition constraint_chol_decomposition;
  constraint_chol_decomposition.rebuild(model, data, contact_models, contact_datas);
  constraint_chol_decomposition.compute(model, data, contact_models, contact_datas);

  // Check copy constructor
  {
    ConstraintCholeskyDecomposition constraint_chol_decomposition_copy(
      constraint_chol_decomposition);
    BOOST_CHECK(constraint_chol_decomposition_copy.U == constraint_chol_decomposition.U);
    BOOST_CHECK(
      constraint_chol_decomposition_copy.U.data() != constraint_chol_decomposition.U.data());
    BOOST_CHECK(constraint_chol_decomposition_copy.D == constraint_chol_decomposition.D);
    BOOST_CHECK(
      constraint_chol_decomposition_copy.D.data() != constraint_chol_decomposition.D.data());
    BOOST_CHECK(constraint_chol_decomposition_copy.Dinv == constraint_chol_decomposition.Dinv);
    BOOST_CHECK(
      constraint_chol_decomposition_copy.Dinv.data() != constraint_chol_decomposition.Dinv.data());
  }

  // Test resize
  {
    ConstraintCholeskyDecomposition constraint_chol_decomposition_empty(model, data);
    {
      RigidConstraintModelStdVector contact_models_empty;
      RigidConstraintDataStdVector contact_datas_empty;
      constraint_chol_decomposition_empty.compute(
        model, data, contact_models_empty, contact_datas_empty);

      BOOST_CHECK(
        constraint_chol_decomposition_empty.U.bottomRightCorner(model.nv, model.nv)
        == constraint_chol_decomposition.U.bottomRightCorner(model.nv, model.nv));
      BOOST_CHECK(
        constraint_chol_decomposition_empty.D.tail(model.nv)
        == constraint_chol_decomposition.D.tail(model.nv));
      BOOST_CHECK(
        constraint_chol_decomposition_empty.Dinv.tail(model.nv)
        == constraint_chol_decomposition.Dinv.tail(model.nv));
    }

    constraint_chol_decomposition_empty.rebuild(model, data, contact_models, contact_datas);
    {
      RigidConstraintDataStdVector contact_datas;
      for (const auto & cmodel : contact_models)
        contact_datas.push_back(cmodel.createData());
      calc(model, data, contact_models, contact_datas);

      constraint_chol_decomposition_empty.compute(model, data, contact_models, contact_datas);
      BOOST_CHECK(constraint_chol_decomposition_empty.U == constraint_chol_decomposition.U);
      BOOST_CHECK(constraint_chol_decomposition_empty.D == constraint_chol_decomposition.D);
      BOOST_CHECK(constraint_chol_decomposition_empty.Dinv == constraint_chol_decomposition.Dinv);
    }

    constraint_chol_decomposition_empty.rebuild(
      model, data, RigidConstraintModelStdVector(), RigidConstraintDataStdVector());
    {
      RigidConstraintModelStdVector contact_models_empty;
      RigidConstraintDataStdVector contact_datas_empty;
      constraint_chol_decomposition_empty.U.triangularView<Eigen::StrictlyUpper>().setZero();
      constraint_chol_decomposition_empty.D.setZero();
      constraint_chol_decomposition_empty.Dinv.setZero();
      constraint_chol_decomposition_empty.compute(
        model, data, contact_models_empty, contact_datas_empty);

      BOOST_CHECK(
        constraint_chol_decomposition_empty.U.bottomRightCorner(model.nv, model.nv)
        == constraint_chol_decomposition.U.bottomRightCorner(model.nv, model.nv));
      BOOST_CHECK(
        constraint_chol_decomposition_empty.D.tail(model.nv)
        == constraint_chol_decomposition.D.tail(model.nv));
      BOOST_CHECK(
        constraint_chol_decomposition_empty.Dinv.tail(model.nv)
        == constraint_chol_decomposition.Dinv.tail(model.nv));
    }
  }
}
