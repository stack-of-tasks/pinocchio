//
// Copyright (c) 2024-2025 INRIA
//

#define BOOST_TEST_MODULE contact_inverse_dynamics

#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/constraints.hpp"

#include "pinocchio/algorithm/contact-inverse-dynamics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/algorithm/solvers/constraint-solver-utils.hpp"

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

typedef std::vector<PointContactConstraintModel> PointContactConstraintModelVector;
typedef std::vector<PointContactConstraintData> PointContactConstraintDataVector;

void init(Model & model, PointContactConstraintModelVector & constraint_models)
{
  pinocchio::buildModels::humanoidRandom(model, true);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  const std::string RF = "rleg6_joint";
  PointContactConstraintModel ci_RF(model, model.getJointId(RF));
  ci_RF.setFriction(0.4);
  constraint_models.push_back(ci_RF);

  const std::string LF = "lleg6_joint";
  PointContactConstraintModel ci_LF(model, model.getJointId(LF));
  ci_LF.setFriction(0.4);
  constraint_models.push_back(ci_LF);
}

PointContactConstraintDataVector
createData(const PointContactConstraintModelVector & constraint_models)
{
  PointContactConstraintDataVector constraint_datas;

  for (const auto & cmodel : constraint_models)
  {
    constraint_datas.push_back(cmodel.createData());
  }

  return constraint_datas;
}

template<typename VectorLike>
typename PINOCCHIO_EIGEN_PLAIN_TYPE(VectorLike) abs(const Eigen::MatrixBase<VectorLike> & vec)
{
  return Eigen::abs(vec.array()).matrix();
}

template<typename VectorLike>
void makeIsotropic(
  PointContactConstraintModelVector & constraint_models, const Eigen::MatrixBase<VectorLike> & vec_)
{
  auto & vec = vec_.const_cast_derived();

  Eigen::Index row_id = 0;
  for (const auto & cmodel : constraint_models)
  {
    const auto csize = cmodel.residualSize();

    auto vec_seg = vec.segment(row_id, csize);
    vec_seg[1] = vec_seg[0];

    row_id += csize;
  }
}

BOOST_AUTO_TEST_CASE(test_contact_inverse_dynamics_3D)
{
#ifdef NDEBUG
  const int num_tests = int(1e4);
#else
  const int num_tests = int(1e2);
#endif

  Model model;
  PointContactConstraintModelVector constraint_models;

  init(model, constraint_models);
  auto constraint_datas = createData(constraint_models);

  const double mu_prox = 1e-4;
  Eigen::VectorXd q = randomConfiguration(model);
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

  Eigen::Index constraint_size = 0;
  for (const auto & cmodel : constraint_models)
  {
    // We know that point contact is of constant size
    constraint_size += cmodel.residualSize();
  }

  BOOST_CHECK(constraint_size > 0);

  for (int n = 0; n < num_tests; ++n)
  {
    Eigen::VectorXd R = abs(Eigen::VectorXd::Random(constraint_size))
                        + Eigen::VectorXd::Constant(constraint_size, 1e-10);
    makeIsotropic(constraint_models, R);

    Eigen::Index constraint_index = 0;
    for (auto & cmodel : constraint_models)
    {
      int csize = cmodel.residualSize();
      cmodel.setCompliance(R.segment(constraint_index, csize));
      constraint_index += csize;
    }

    ProximalSettings prox_settings(1e-12, 1e-12, /*mu = */ 0, 100);

    const Eigen::VectorXd x_positive = abs(Eigen::VectorXd::Random(constraint_size));
    const Eigen::VectorXd x_in_cone = Eigen::VectorXd::Zero(constraint_size);

    internal::computeConstraintSetProjection(
      constraint_models, constraint_datas, x_positive, x_in_cone);

    const Eigen::VectorXd constraint_velocity_ref = -(R.asDiagonal() * x_in_cone).eval();
    const Eigen::VectorXd sigma_ref = (constraint_velocity_ref + R.asDiagonal() * x_in_cone);
    BOOST_CHECK(sigma_ref.isZero());

    Eigen::VectorXd x_sol = Eigen::VectorXd::Zero(constraint_size);

    bool has_converged = computeInverseDynamicsConstraintForces(
      constraint_models, constraint_datas, constraint_velocity_ref, x_sol, prox_settings,
      /*solve_ncp = */ false);
    BOOST_CHECK(has_converged);

    Eigen::VectorXd sigma = constraint_velocity_ref + R.asDiagonal() * x_sol;

    Eigen::VectorXd sigma_correction(sigma);
    internal::computeDeSaxeCorrection(constraint_models, constraint_datas, sigma, sigma_correction);
    sigma += sigma_correction;

    BOOST_CHECK(sigma.isZero(1e-8));
    Eigen::VectorXd sigma_projected(sigma);
    internal::computeDualConstraintSetProjection(
      constraint_models, constraint_datas, sigma, sigma_projected);
    BOOST_CHECK((sigma_projected - sigma).lpNorm<Eigen::Infinity>() <= 1e-10);
  }

  // test with mu_prox > 0
  for (int n = 0; n < num_tests; ++n)
  {
    for (auto & cmodel : constraint_models)
    {
      cmodel.setCompliance(Eigen::VectorXd::Zero(cmodel.residualSize()));
    }

    ProximalSettings prox_settings(1e-12, 1e-12, mu_prox, 200);
    auto constraint_datas = createData(constraint_models);

    const Eigen::VectorXd constraint_velocity = Eigen::VectorXd::Random(constraint_size);
    Eigen::VectorXd constraint_velocity_projected(constraint_velocity);
    internal::computeDualConstraintSetProjection(
      constraint_models, constraint_datas, constraint_velocity, constraint_velocity_projected);

    Eigen::VectorXd x_sol = Eigen::VectorXd::Zero(constraint_size);
    bool has_converged = computeInverseDynamicsConstraintForces(
      constraint_models, constraint_datas, constraint_velocity_projected, x_sol, prox_settings,
      /*solve_ncp = */ false);
    BOOST_CHECK(has_converged);

    Eigen::VectorXd x_sol_projected(x_sol);
    internal::computeConstraintSetProjection(
      constraint_models, constraint_datas, x_sol, x_sol_projected);
    BOOST_CHECK((x_sol_projected - x_sol).lpNorm<Eigen::Infinity>() <= 1e-10);

    BOOST_CHECK(std::abs(constraint_velocity_projected.dot(x_sol)) <= 1e-10);
  }
}
