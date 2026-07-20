//
// Copyright (c) 2025 INRIA
//

#define BOOST_TEST_MODULE dynamic_allocations

#include "pinocchio/spatial.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/constraints.hpp"

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/center-of-mass-derivatives.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/centroidal-derivatives.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/energy.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/constraint-cholesky.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/impulse-dynamics.hpp"
#include "pinocchio/algorithm/impulse-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/regressor.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

#if !(defined(__has_feature) && __has_feature(realtime_sanitizer))
  #error "rtsan not enabled. Please enable rtsan with -fsanitize=realtime"
#endif

bool hasMimicJoints(const Model & model)
{
  return !model.mimicked_joints.empty();
}

bool hasCompositeJoints(const Model & model)
{
  for (JointIndex jidx = 0; jidx < static_cast<JointIndex>(model.njoints); ++jidx)
  {
    if (model.joints[jidx].shortname() == "JointModelComposite")
      return true;
  }
  return false;
}

void runKinematicsTest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    // Forward kinematics (position only)
    forwardKinematics(model, data, q);

    // Forward kinematics (position + velocity)
    forwardKinematics(model, data, q, v);

    // Forward kinematics (position + velocity + acceleration)
    forwardKinematics(model, data, q, v, a);

    // Update global placements
    updateGlobalPlacements(model, data);
  }();
}

void runJacobianTest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    // Compute joint Jacobians
    computeJointJacobians(model, data, q);
  }();

  // Get specific joint Jacobian
  const Data::Matrix6x J = Data::Matrix6x::Zero(6, model.nv);
  const JointIndex joint_id = static_cast<JointIndex>(model.njoints - 1);

  [&]() noexcept [[clang::nonblocking]] {
    getJointJacobian(model, data, joint_id, LOCAL, J);
    getJointJacobian(model, data, joint_id, WORLD, J);
    getJointJacobian(model, data, joint_id, LOCAL_WORLD_ALIGNED, J);

    // Compute Jacobian time variation
    computeJointJacobiansTimeVariation(model, data, q, v);
  }();
}

void runNonLinearEffectsTest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    // Non-linear effects
    nonLinearEffects(model, data, q, v);
  }();
}

void runRNEATest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    // RNEA (Recursive Newton-Euler Algorithm)
    rnea(model, data, q, v, a);
  }();
}

void runCRBATest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);

  [&]() noexcept [[clang::nonblocking]] {
    // CRBA (Composite Rigid Body Algorithm)
    crba(model, data, q);
    crba(model, data, q, Convention::WORLD);
    crba(model, data, q, Convention::LOCAL);
  }();
}

void runABATest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    // ABA (Articulated Body Algorithm)
    aba(model, data, q, v, tau);
  }();
}

void runDerivativesTest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    // Compute forward kinematics derivatives
    computeForwardKinematicsDerivatives(model, data, q, v, a);
  }();

  // RNEA derivatives
  const Data::MatrixXs rnea_partial_dq = Data::MatrixXs::Zero(model.nv, model.nv);
  const Data::MatrixXs rnea_partial_dv = Data::MatrixXs::Zero(model.nv, model.nv);
  const Data::MatrixXs rnea_partial_da = Data::MatrixXs::Zero(model.nv, model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    computeRNEADerivatives(model, data, q, v, a, rnea_partial_dq, rnea_partial_dv, rnea_partial_da);
  }();

  // ABA derivatives
  const Data::MatrixXs aba_partial_dq = Data::MatrixXs::Zero(model.nv, model.nv);
  const Data::MatrixXs aba_partial_dv = Data::MatrixXs::Zero(model.nv, model.nv);
  const Data::MatrixXs aba_partial_dtau = Data::MatrixXs::Zero(model.nv, model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    computeABADerivatives(model, data, q, v, tau, aba_partial_dq, aba_partial_dv, aba_partial_dtau);
  }();
}

void runCenterOfMassTest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    // Compute center of mass position
    centerOfMass(model, data, q);

    // Compute center of mass velocity
    centerOfMass(model, data, q, v);

    // Compute center of mass acceleration
    centerOfMass(model, data, q, v, a);

    // Jacobian of center of mass
    jacobianCenterOfMass(model, data, q);
  }();
}

void runCenterOfMassDerivativesTest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Data::Matrix3x vcom_partial_dq = Data::Matrix3x::Zero(3, model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    // Center of mass derivatives
    getCenterOfMassVelocityDerivatives(model, data, vcom_partial_dq);
  }();
}

void runCentroidalDynamicsTest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);
  Data::Matrix6x dh_dq = Data::Matrix6x::Zero(6, model.nv);
  Data::Matrix6x dhdot_dq = Data::Matrix6x::Zero(6, model.nv);
  Data::Matrix6x dhdot_dv = Data::Matrix6x::Zero(6, model.nv);
  Data::Matrix6x dhdot_da = Data::Matrix6x::Zero(6, model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    // Compute centroidal momentum
    computeCentroidalMomentum(model, data, q, v);

    // Compute centroidal momentum time variation
    computeCentroidalMomentumTimeVariation(model, data, q, v, a);

    // Centroidal momentum Jacobian
    ccrba(model, data, q, v);

    // Centroidal derivatives
    computeCentroidalDynamicsDerivatives(model, data, q, v, a, dh_dq, dhdot_dq, dhdot_dv, dhdot_da);
  }();
}

void runFrameAlgorithmsTest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);

  [&]() noexcept [[clang::nonblocking]] {
    // Update frame placements
    updateFramePlacements(model, data);

    // Forward kinematics for frames
    framesForwardKinematics(model, data, q);
  }();

  const Data::Matrix6x frame_J = Data::Matrix6x::Zero(6, model.nv);
  Data::Matrix6x v_partial_dq = Data::Matrix6x::Zero(6, model.nv);
  Data::Matrix6x v_partial_dv = Data::Matrix6x::Zero(6, model.nv);
  Data::Matrix6x a_partial_dq = Data::Matrix6x::Zero(6, model.nv);
  Data::Matrix6x a_partial_dv = Data::Matrix6x::Zero(6, model.nv);
  Data::Matrix6x a_partial_da = Data::Matrix6x::Zero(6, model.nv);

  for (FrameIndex frame_idx = 0; frame_idx < static_cast<FrameIndex>(model.nframes); ++frame_idx)
  {
    [&]() noexcept [[clang::nonblocking]] {
      getFrameJacobian(model, data, frame_idx, LOCAL, frame_J);
      getFrameJacobian(model, data, frame_idx, WORLD, frame_J);
      getFrameJacobian(model, data, frame_idx, LOCAL_WORLD_ALIGNED, frame_J);

      // Compute frame Jacobian
      computeFrameJacobian(model, data, q, frame_idx, LOCAL, frame_J);
      computeFrameJacobian(model, data, q, frame_idx, WORLD, frame_J);

      // Frame Jacobian time variation
      getFrameJacobianTimeVariation(model, data, frame_idx, LOCAL, frame_J);
      getFrameJacobianTimeVariation(model, data, frame_idx, WORLD, frame_J);

      // Frame velocity
      getFrameVelocity(model, data, frame_idx, LOCAL);
      getFrameVelocity(model, data, frame_idx, WORLD);
      getFrameVelocity(model, data, frame_idx, LOCAL_WORLD_ALIGNED);

      // Frame acceleration
      getFrameAcceleration(model, data, frame_idx, LOCAL);
      getFrameAcceleration(model, data, frame_idx, WORLD);
      getFrameAcceleration(model, data, frame_idx, LOCAL_WORLD_ALIGNED);

      // Frame classical acceleration
      getFrameClassicalAcceleration(model, data, frame_idx, LOCAL);
      getFrameClassicalAcceleration(model, data, frame_idx, WORLD);
      getFrameClassicalAcceleration(model, data, frame_idx, LOCAL_WORLD_ALIGNED);
    }();
    // Frame derivatives
    if (!hasMimicJoints(model))
    {
      [&]() noexcept [[clang::nonblocking]] {
        getFrameVelocityDerivatives(model, data, frame_idx, LOCAL, v_partial_dq, v_partial_dv);
        getFrameAccelerationDerivatives(
          model, data, frame_idx, LOCAL, v_partial_dq, a_partial_dq, a_partial_dv, a_partial_da);
      }();
    }
  }
}

void runComputeAllTermsTest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);

  [&]() noexcept [[clang::nonblocking]] { computeAllTerms(model, data, q, v); }();
}

void runEnergyTest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    // Compute kinetic energy
    computeKineticEnergy(model, data, q, v);

    // Compute potential energy
    computePotentialEnergy(model, data, q);
  }();
}

void runComputeGeneralizedGravityTest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);

  [&]() noexcept [[clang::nonblocking]] {
    // Compute generalized gravity
    computeGeneralizedGravity(model, data, q);
  }();
}

void runCholeskyTest(const Model & model, Data & data)
{
  Eigen::VectorXd v_chol = Eigen::VectorXd::Random(model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    cholesky::decompose(model, data);
    cholesky::solve(model, data, v_chol);
  }();

  Data::MatrixXs M_inv = Data::MatrixXs::Zero(model.nv, model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    // Compute inverse of mass matrix using Cholesky
    cholesky::computeMinv(model, data, M_inv);
  }();
}

void runJointConfigurationOperationsTest(const Model & model)
{
  Eigen::VectorXd q = randomConfiguration(model);
  Eigen::VectorXd dq = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd q_neutral = neutral(model);
  Eigen::VectorXd q_integrated(model.nq);
  Eigen::VectorXd q_interp(model.nq);

  [&]() noexcept [[clang::nonblocking]] {
    normalize(model, q);
    difference(model, q, q_neutral, dq);
    integrate(model, q, v, q_integrated);
    interpolate(model, q, q_neutral, 0.5, q_interp);
    distance(model, q, q_neutral);
    isNormalized(model, q);
    isSameConfiguration(model, q, q, 1e-12);
  }();
}

void runJointTorqueRegressorTest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);

  [&]() noexcept [[clang::nonblocking]] {
    // Compute joint torque regressor
    computeJointTorqueRegressor(model, data, q, v, a);
  }();
}

void runContactDynamicsTest(const Model & model, Data & data)
{
  const Eigen::VectorXd q = randomConfiguration(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);

  // ============ CONTACT DYNAMICS ============
  // Create contacts
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;

  if (model.nframes > 1)
  {
    size_t frame_idx = static_cast<size_t>(model.nframes - 1);
    JointIndex contact_joint = model.frames[frame_idx].parentJoint;
    RigidConstraintModel contact_model_6d(CONTACT_6D, model, contact_joint, LOCAL);
    contact_models.push_back(contact_model_6d);
    contact_data.push_back(RigidConstraintData(contact_model_6d));

    // Initialize contact data
    initConstraintDynamics(model, data, contact_models, contact_data);

    [&]() noexcept [[clang::nonblocking]] {
      // Constrained forward dynamics
      constraintDynamics(model, data, q, v, tau, contact_models, contact_data);
    }();

    // Contact Cholesky
    ConstraintCholeskyDecomposition constraint_chol;
    constraint_chol.rebuild(model, data, contact_models, contact_data);

    [&]() noexcept [[clang::nonblocking]] {
      crba(model, data, q, Convention::WORLD);
      constraint_chol.compute(model, data, contact_models, contact_data);
    }();

    // Constrained dynamics derivatives
    Data::MatrixXs ddq_dq = Data::MatrixXs::Zero(model.nv, model.nv);
    Data::MatrixXs ddq_dv = Data::MatrixXs::Zero(model.nv, model.nv);
    Data::MatrixXs ddq_dtau = Data::MatrixXs::Zero(model.nv, model.nv);
    const int constraint_dim = contact_models[0].residualSize();
    Data::MatrixXs lambda_dq = Data::MatrixXs::Zero(constraint_dim, model.nv);
    Data::MatrixXs lambda_dv = Data::MatrixXs::Zero(constraint_dim, model.nv);
    Data::MatrixXs lambda_dtau = Data::MatrixXs::Zero(constraint_dim, model.nv);

    [&]() noexcept [[clang::nonblocking]] {
      computeConstraintDynamicsDerivatives(
        model, data, contact_models, contact_data, ddq_dq, ddq_dv, ddq_dtau, lambda_dq, lambda_dv,
        lambda_dtau);
    }();

    // Impulse dynamics
    Eigen::VectorXd v_before = v;
    const double r_coeff = 0.0;
    ProximalSettings prox_settings(1e-12, 0., 1);

    [&]() noexcept [[clang::nonblocking]] {
      impulseDynamics(
        model, data, q, v_before, contact_models, contact_data, r_coeff, prox_settings);
    }();

    // Impulse dynamics derivatives
    Data::MatrixXs ddv_dq = Data::MatrixXs::Zero(model.nv, model.nv);
    Data::MatrixXs ddv_dvbefore = Data::MatrixXs::Zero(model.nv, model.nv);
    Data::MatrixXs impulse_dq = Data::MatrixXs::Zero(constraint_dim, model.nv);
    Data::MatrixXs impulse_dv = Data::MatrixXs::Zero(constraint_dim, model.nv);

    [&]() noexcept [[clang::nonblocking]] {
      computeImpulseDynamicsDerivatives(
        model, data, contact_models, contact_data, r_coeff, prox_settings);
    }();
  }
}

void runDynamicAllocationsTest(const Model & model)
{
  Data data(model);
  runKinematicsTest(model, data);
  runJointConfigurationOperationsTest(model);

  if (hasCompositeJoints(model))
  {
    // Joint Composite currently performs dynamic allocations
    return;
  }

  runJacobianTest(model, data);
  runNonLinearEffectsTest(model, data);
  runFrameAlgorithmsTest(model, data);
  runRNEATest(model, data);
  runCRBATest(model, data);
  runCenterOfMassTest(model, data);
  runComputeGeneralizedGravityTest(model, data);

  if (hasMimicJoints(model))
  {
    // Mimic joints currently perform dynamic allocations
    return;
  }

  runJointTorqueRegressorTest(model, data);
  runABATest(model, data);
  runCenterOfMassDerivativesTest(model, data);
  runDerivativesTest(model, data);
  runCentroidalDynamicsTest(model, data);
  runComputeAllTermsTest(model, data);
  runEnergyTest(model, data);
  runCholeskyTest(model, data);
  // Those tests are disabled for now as they trigger dynamic allocations
  // runContactDynamicsTest(model, data);
}

BOOST_AUTO_TEST_CASE(dynamic_allocations_humanoid_random_free_floating)
{
  // Test with humanoid random model (Free floating)
  Model model;
  const bool using_free_floating = true;
  const bool using_mimic = false;
  buildModels::humanoidRandom(model, using_free_floating, using_mimic);
  BOOST_CHECK(hasMimicJoints(model) == false);
  BOOST_CHECK(hasCompositeJoints(model) == false);
  runDynamicAllocationsTest(model);
}

BOOST_AUTO_TEST_CASE(dynamic_allocations_humanoid_random_composite)
{
  // Test with humanoid random model (Composite)
  Model model;
  const bool using_free_floating = false;
  const bool using_mimic = false;
  buildModels::humanoidRandom(model, using_free_floating, using_mimic);
  BOOST_CHECK(hasMimicJoints(model) == false);
  BOOST_CHECK(hasCompositeJoints(model) == true);
  runDynamicAllocationsTest(model);
}

BOOST_AUTO_TEST_CASE(dynamic_allocations_manipulator)
{
  // Test with manipulator
  Model model;
  const bool using_mimic = false;
  buildModels::manipulator(model, using_mimic);
  BOOST_CHECK(hasMimicJoints(model) == false);
  BOOST_CHECK(hasCompositeJoints(model) == false);
  runDynamicAllocationsTest(model);
}

BOOST_AUTO_TEST_CASE(dynamic_allocations_manipulator_mimic)
{
  // Test with manipulator (Mimic)
  Model model;
  const bool using_mimic = true;
  buildModels::manipulator(model, using_mimic);
  BOOST_CHECK(hasMimicJoints(model) == true);
  BOOST_CHECK(hasCompositeJoints(model) == false);
  runDynamicAllocationsTest(model);
}

BOOST_AUTO_TEST_CASE(dynamic_allocations_humanoid_free_floating)
{
  // Test with humanoid (Free floating)
  Model model;
  const bool using_free_floating = true;
  buildModels::humanoid(model, using_free_floating);
  BOOST_CHECK(hasMimicJoints(model) == false);
  BOOST_CHECK(hasCompositeJoints(model) == false);
  runDynamicAllocationsTest(model);
}

BOOST_AUTO_TEST_CASE(dynamic_allocations_humanoid_composite)
{
  // Test with humanoid (Composite)
  Model model;
  const bool using_free_floating = false;
  buildModels::humanoid(model, using_free_floating);
  BOOST_CHECK(hasMimicJoints(model) == false);
  BOOST_CHECK(hasCompositeJoints(model) == true);
  runDynamicAllocationsTest(model);
}

BOOST_AUTO_TEST_CASE(dynamic_allocations_spatial_operations)
{
  [&]() noexcept [[clang::nonblocking]] {
    // Classic acceleration
    SE3 M = SE3::Random();
    Motion v_spatial = Motion::Random();
    Motion a_spatial = Motion::Random();
    classicAcceleration(v_spatial, a_spatial);

    // Exponential/logarithm maps
    SE3::Vector3 w = SE3::Vector3::Random();
    exp3(w);
    log3(SE3::Random().rotation());

    Motion::Vector6 nu = Motion::Vector6::Random();
    exp6(nu);
    log6(M);
  }();
}
