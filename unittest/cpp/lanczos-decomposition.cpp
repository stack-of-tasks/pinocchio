//
// Copyright (c) 2024-2025 INRIA
//

#define BOOST_TEST_MODULE lanczos_decomposition

#include "pinocchio/math.hpp"
#include "pinocchio/constraints.hpp"

#include "pinocchio/algorithm/delassus-operator.hpp"
#include "pinocchio/algorithm/constraint-cholesky.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/diagonal-preconditioner.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

int line;
#define SET_LINE line = __LINE__ + 1
#define PINOCCHIO_CHECK(cond) BOOST_CHECK_MESSAGE(cond, "from line " << line << ": " #cond)
#define PINOCCHIO_CHECK_EQUAL(a, b)                                                                \
  BOOST_CHECK_MESSAGE(                                                                             \
    (a) == (b), "from line " << line << ": " #a "[" << (a) << "] != " #b "[" << (b) << "].")

BOOST_AUTO_TEST_CASE(test_basic_constructor)
{
  const Eigen::Index mat_size = 20;
  const Eigen::Index decomposition_size = 10;

  typedef LanczosDecompositionTpl<Eigen::MatrixXd> LanczosDecomposition;
  LanczosDecomposition lanczos_decomposition(mat_size, decomposition_size);

  BOOST_CHECK(lanczos_decomposition.size() == mat_size);
  BOOST_CHECK(lanczos_decomposition.decompositionSize() == decomposition_size);
}

BOOST_AUTO_TEST_CASE(test_identity)
{
  const Eigen::Index mat_size = 20;
  const Eigen::MatrixXd identity_matrix = Eigen::MatrixXd::Identity(mat_size, mat_size);

  typedef LanczosDecompositionTpl<Eigen::MatrixXd> LanczosDecomposition;
  LanczosDecomposition lanczos_decomposition(identity_matrix, 10);

  const auto residual = lanczos_decomposition.computeDecompositionResidual(identity_matrix);
  BOOST_CHECK(residual.isZero());

  BOOST_CHECK((lanczos_decomposition.Qs().transpose() * lanczos_decomposition.Qs()).isIdentity());
}

BOOST_AUTO_TEST_CASE(test_diagonal_matrix)
{
  const Eigen::Index mat_size = 20;
  const Eigen::VectorXd diagonal_terms = Eigen::VectorXd::LinSpaced(mat_size, 0.0, mat_size - 1);
  const Eigen::MatrixXd diagonal_matrix = diagonal_terms.asDiagonal();

  typedef LanczosDecompositionTpl<Eigen::MatrixXd> LanczosDecomposition;
  LanczosDecomposition lanczos_decomposition(diagonal_matrix, 10);

  const auto residual = lanczos_decomposition.computeDecompositionResidual(diagonal_matrix);
  BOOST_CHECK(residual.isZero());

  for (Eigen::Index col_id = 0; col_id < lanczos_decomposition.Ts().cols(); ++col_id)
  {
    BOOST_CHECK(math::fabs(lanczos_decomposition.Qs().col(col_id).norm() - 1.) <= 1e-12);
  }

  BOOST_CHECK((lanczos_decomposition.Qs().transpose() * lanczos_decomposition.Qs()).isIdentity());
}

void checkDecomposition(
  const LanczosDecompositionTpl<Eigen::MatrixXd> & lanczos_decomposition,
  const Eigen::MatrixXd & matrix)
{
  const auto residual = lanczos_decomposition.computeDecompositionResidual(matrix);

  const auto & Ts = lanczos_decomposition.Ts();
  const auto & Qs = lanczos_decomposition.Qs();
  const Eigen::MatrixXd residual_bis = matrix * Qs - Qs * Ts.matrix();
  const Eigen::MatrixXd residual_tierce = Qs.transpose() * matrix * Qs - Ts.matrix();
  const Eigen::MatrixXd residual_fourth = matrix - Qs * Ts.matrix() * Qs.transpose();
  PINOCCHIO_CHECK(residual.isZero(1e-10));

  const auto size = lanczos_decomposition.decompositionSize();

  for (Eigen::Index col_id = 0; col_id < size; ++col_id)
  {
    PINOCCHIO_CHECK(math::fabs(lanczos_decomposition.Qs().col(col_id).norm() - 1.) <= 1e-12);
  }

  PINOCCHIO_CHECK((Qs.transpose() * Qs).isIdentity());
}

BOOST_AUTO_TEST_CASE(test_random)
{
  typedef LanczosDecompositionTpl<Eigen::MatrixXd> LanczosDecomposition;
  const Eigen::Index mat_size = 20;

  for (int it = 0; it < 1000; ++it)
  {
    const Eigen::MatrixXd A = Eigen::MatrixXd::Random(mat_size, mat_size);
    const Eigen::MatrixXd matrix = A.transpose() * A;

    LanczosDecomposition lanczos_decomposition(matrix, 10);

    SET_LINE;
    checkDecomposition(lanczos_decomposition, matrix);
  }
}

BOOST_AUTO_TEST_CASE(test_low_rank)
{
  typedef LanczosDecompositionTpl<Eigen::MatrixXd> LanczosDecomposition;
  const Eigen::Index mat_size = 20;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(mat_size, mat_size);
  A.row(mat_size - 1).setZero();
  A.col(mat_size - 1).setZero();

  LanczosDecomposition lanczos_decomposition_10(A, 10);
  SET_LINE;
  checkDecomposition(lanczos_decomposition_10, A);
}

BOOST_AUTO_TEST_CASE(test_delassus)
{
  typedef LanczosDecompositionTpl<Eigen::MatrixXd> LanczosDecomposition;
  const Eigen::Index mat_size = 20;

  for (int it = 0; it < 1000; ++it)
  {
    const Eigen::MatrixXd A = Eigen::MatrixXd::Random(mat_size, mat_size);
    const Eigen::MatrixXd matrix = A.transpose() * A;

    DelassusOperatorDense delassus(matrix);

    LanczosDecomposition lanczos_decomposition(delassus, 10);

    SET_LINE;
    checkDecomposition(lanczos_decomposition, matrix);
  }
}

void buildStackOfCubeModel(
  std::vector<double> masses,
  ::pinocchio::Model & model,
  std::vector<PointContactConstraintModel> & constraint_models)
{
  const SE3::Vector3 box_dims = SE3::Vector3::Ones();
  const int n_cubes = (int)masses.size();

  for (int i = 0; i < n_cubes; i++)
  {
    const double box_mass = masses[(std::size_t)i];
    const Inertia box_inertia = Inertia::FromBox(box_mass, box_dims[0], box_dims[1], box_dims[2]);
    JointIndex joint_id =
      model.addJoint(0, JointModelFreeFlyer(), SE3::Identity(), "free_flyer_" + std::to_string(i));
    model.appendBodyToJoint(joint_id, box_inertia);
  }

  const double friction_value = 0.4;
  for (int i = 0; i < n_cubes; i++)
  {
    const SE3 local_placement_box_1(
      SE3::Matrix3::Identity(), 0.5 * SE3::Vector3(box_dims[0], box_dims[1], box_dims[2]));
    const SE3 local_placement_box_2(
      SE3::Matrix3::Identity(), 0.5 * SE3::Vector3(box_dims[0], box_dims[1], -box_dims[2]));
    SE3::Matrix3 rot = SE3::Matrix3::Identity();
    for (int j = 0; j < 4; ++j)
    {
      const SE3 local_placement_1(
        SE3::Matrix3::Identity(), rot * local_placement_box_1.translation());
      const SE3 local_placement_2(
        SE3::Matrix3::Identity(), rot * local_placement_box_2.translation());
      PointContactConstraintModel cm(
        model, (JointIndex)i, local_placement_1, (JointIndex)i + 1, local_placement_2);
      cm.setFriction(friction_value);
      constraint_models.push_back(cm);
      rot = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix() * rot;
    }
  }
}

BOOST_AUTO_TEST_CASE(test_delassus_cube)
{
  typedef LanczosDecompositionTpl<Eigen::MatrixXd> LanczosDecomposition;
  ::pinocchio::Model model;
  typedef PointContactConstraintModel ConstraintModel;
  std::vector<ConstraintModel> constraint_models;

  const double box_mass = 10;
  std::vector<double> masses = {box_mass};

  buildStackOfCubeModel(masses, model, constraint_models);

  BOOST_CHECK(model.check(model.createData()));

  Data data(model);

  Eigen::VectorXd q0 = neutral(model);

  crba(model, data, q0, Convention::WORLD);
  auto constraint_datas = createData(constraint_models);
  calc(model, data, constraint_models, constraint_datas);

  ConstraintCholeskyDecomposition chol(model, data, constraint_models, constraint_datas);
  chol.compute(model, data, constraint_models, constraint_datas, 1e-10);

  const Eigen::MatrixXd delassus_matrix_plain =
    chol.getDelassusOperatorCholeskyExpression().matrix(true);
  auto G_expression = chol.getDelassusOperatorCholeskyExpression();

  BOOST_CHECK(delassus_matrix_plain.isApprox(delassus_matrix_plain.transpose(), 0));

  //  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(delassus_matrix_plain);
  //  std::cout << "The eigenvalues of delassus_matrix_plain are:\n" <<
  //  eigensolver.eigenvalues().transpose() << std::endl;

  for (int decomposition_size = 3; decomposition_size <= delassus_matrix_plain.rows();
       ++decomposition_size)
  {
    LanczosDecomposition lanczos_decomposition(G_expression, decomposition_size);
    SET_LINE;
    checkDecomposition(lanczos_decomposition, delassus_matrix_plain);
  }

  // {
  //   LanczosDecomposition lanczos_decomposition(G_expression, 7);
  //   SET_LINE;
  //   checkDecomposition(lanczos_decomposition, delassus_matrix_plain);
  // }

  Eigen::VectorXd mean_inertia =
    Eigen::VectorXd::Constant(delassus_matrix_plain.rows(), data.M.diagonal().trace());
  mean_inertia /= (double)delassus_matrix_plain.rows();
  mean_inertia = mean_inertia.array().sqrt();
  typedef DiagonalPreconditionerTpl<Eigen::VectorXd> Preconditionner;
  Preconditionner diag_preconditioner(mean_inertia);
  DelassusOperatorPreconditionedTpl<
    DelassusOperatorCholeskyExpressionTpl<ConstraintCholeskyDecomposition>, Preconditionner>
    delassus_preconditioned(G_expression, diag_preconditioner);

  const Eigen::MatrixXd delassus_preconditioned_matrix_plain = delassus_preconditioned.matrix(true);

  for (int decomposition_size = 3; decomposition_size <= delassus_matrix_plain.rows();
       ++decomposition_size)
  {
    LanczosDecomposition lanczos_decomposition(delassus_preconditioned, decomposition_size);
    SET_LINE;
    checkDecomposition(lanczos_decomposition, delassus_preconditioned_matrix_plain);
  }
}

BOOST_AUTO_TEST_CASE(test_delassus_light_cube)
{
  typedef LanczosDecompositionTpl<Eigen::MatrixXd> LanczosDecomposition;
  ::pinocchio::Model model;
  typedef PointContactConstraintModel ConstraintModel;
  std::vector<ConstraintModel> constraint_models;

  const double box_mass = 1e-3;
  std::vector<double> masses = {box_mass};

  buildStackOfCubeModel(masses, model, constraint_models);

  BOOST_CHECK(model.check(model.createData()));

  Data data(model);

  Eigen::VectorXd q0 = neutral(model);

  crba(model, data, q0, Convention::WORLD);
  auto constraint_datas = createData(constraint_models);

  calc(model, data, constraint_models, constraint_datas);
  ConstraintCholeskyDecomposition chol(model, data, constraint_models, constraint_datas);
  chol.compute(model, data, constraint_models, constraint_datas, 1e-10);

  const Eigen::MatrixXd delassus_matrix_plain =
    chol.getDelassusOperatorCholeskyExpression().matrix(true);
  auto G_expression = chol.getDelassusOperatorCholeskyExpression();

  BOOST_CHECK(delassus_matrix_plain.isApprox(delassus_matrix_plain.transpose(), 0));

  for (int decomposition_size = 3; decomposition_size <= 6; ++decomposition_size)
  {
    LanczosDecomposition lanczos_decomposition(G_expression, decomposition_size);
    SET_LINE;
    checkDecomposition(lanczos_decomposition, delassus_matrix_plain);
  }
}

BOOST_AUTO_TEST_CASE(test_delassus_preconditioned)
{
  typedef LanczosDecompositionTpl<Eigen::MatrixXd> LanczosDecomposition;
  const Eigen::Index mat_size = 20;

  for (int it = 0; it < 1000; ++it)
  {
    const Eigen::MatrixXd A = Eigen::MatrixXd::Random(mat_size, mat_size);
    const Eigen::MatrixXd matrix = A.transpose() * A;

    Eigen::VectorXd diag_vec = Eigen::VectorXd::Constant(mat_size, 1e-6);
    Eigen::MatrixXd preconditioner_diag = diag_vec.asDiagonal();
    const Eigen::MatrixXd matrix_preconditioned =
      preconditioner_diag * matrix * preconditioner_diag;

    DelassusOperatorDense delassus(matrix);
    typedef DiagonalPreconditionerTpl<Eigen::VectorXd> Preconditionner;
    Preconditionner diag_preconditioner(diag_vec);
    DelassusOperatorPreconditionedTpl<DelassusOperatorDense, Preconditionner>
      delassus_preconditioned(delassus, diag_preconditioner);

    LanczosDecomposition lanczos_decomposition(delassus_preconditioned, 10);

    SET_LINE;
    checkDecomposition(lanczos_decomposition, matrix_preconditioned);
  }
}
