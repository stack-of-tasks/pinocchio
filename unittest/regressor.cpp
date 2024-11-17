//
// Copyright (c) 2018-2024 CNRS INRIA
//

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/regressor.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <iostream>
#include "pinocchio/algorithm/kinematics.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_kinematic_regressor_joint)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  buildModels::humanoidRandom(model);

  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill(1.);

  //  const std::string joint_name = "larm5_joint";
  //  const JointIndex joint_id = model.getJointId(joint_name);

  const VectorXd q = randomConfiguration(model);

  forwardKinematics(model, data, q);

  const double eps = 1e-8;
  for (JointIndex joint_id = 1; joint_id < (JointIndex)model.njoints; ++joint_id)
  {
    Data::Matrix6x kinematic_regressor_L(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));
    Data::Matrix6x kinematic_regressor_LWA(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));
    Data::Matrix6x kinematic_regressor_W(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));

    Data::Matrix6x kinematic_regressor_L_fd(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));
    Data::Matrix6x kinematic_regressor_LWA_fd(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));
    Data::Matrix6x kinematic_regressor_W_fd(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));

    computeJointKinematicRegressor(model, data, joint_id, LOCAL, kinematic_regressor_L);
    computeJointKinematicRegressor(
      model, data, joint_id, LOCAL_WORLD_ALIGNED, kinematic_regressor_LWA);
    computeJointKinematicRegressor(model, data, joint_id, WORLD, kinematic_regressor_W);

    Model model_plus = model;
    Data data_plus(model_plus);
    const SE3 & oMi = data.oMi[joint_id];
    const SE3 Mi_LWA = SE3(oMi.rotation(), SE3::Vector3::Zero());
    const SE3 & oMi_plus = data_plus.oMi[joint_id];
    for (int i = 1; i < model.njoints; ++i)
    {
      Motion::Vector6 v = Motion::Vector6::Zero();
      const SE3 & M_placement = model.jointPlacements[(JointIndex)i];
      SE3 & M_placement_plus = model_plus.jointPlacements[(JointIndex)i];
      for (Eigen::DenseIndex k = 0; k < 6; ++k)
      {
        v[k] = eps;
        M_placement_plus = M_placement * exp6(Motion(v));

        forwardKinematics(model_plus, data_plus, q);

        const Motion diff_L = log6(oMi.actInv(oMi_plus));
        kinematic_regressor_L_fd.middleCols<6>(6 * (i - 1)).col(k) = diff_L.toVector() / eps;
        const Motion diff_LWA = Mi_LWA.act(diff_L);
        kinematic_regressor_LWA_fd.middleCols<6>(6 * (i - 1)).col(k) = diff_LWA.toVector() / eps;
        const Motion diff_W = oMi.act(diff_L);
        kinematic_regressor_W_fd.middleCols<6>(6 * (i - 1)).col(k) = diff_W.toVector() / eps;
        v[k] = 0.;
      }

      M_placement_plus = M_placement;
    }

    BOOST_CHECK(kinematic_regressor_L.isApprox(kinematic_regressor_L_fd, sqrt(eps)));
    BOOST_CHECK(kinematic_regressor_LWA.isApprox(kinematic_regressor_LWA_fd, sqrt(eps)));
    BOOST_CHECK(kinematic_regressor_W.isApprox(kinematic_regressor_W_fd, sqrt(eps)));
  }
}

BOOST_AUTO_TEST_CASE(test_kinematic_regressor_joint_placement)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  buildModels::humanoidRandom(model);

  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill(1.);

  const VectorXd q = randomConfiguration(model);

  forwardKinematics(model, data, q);
  forwardKinematics(model, data_ref, q);

  for (JointIndex joint_id = 1; joint_id < (JointIndex)model.njoints; ++joint_id)
  {
    Data::Matrix6x kinematic_regressor_L(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));
    Data::Matrix6x kinematic_regressor_LWA(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));
    Data::Matrix6x kinematic_regressor_W(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));

    computeJointKinematicRegressor(
      model, data, joint_id, LOCAL, SE3::Identity(), kinematic_regressor_L);
    computeJointKinematicRegressor(
      model, data, joint_id, LOCAL_WORLD_ALIGNED, SE3::Identity(), kinematic_regressor_LWA);
    computeJointKinematicRegressor(
      model, data, joint_id, WORLD, SE3::Identity(), kinematic_regressor_W);

    Data::Matrix6x kinematic_regressor_L_ref(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));
    Data::Matrix6x kinematic_regressor_LWA_ref(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));
    Data::Matrix6x kinematic_regressor_W_ref(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));

    computeJointKinematicRegressor(model, data_ref, joint_id, LOCAL, kinematic_regressor_L_ref);
    computeJointKinematicRegressor(
      model, data_ref, joint_id, LOCAL_WORLD_ALIGNED, kinematic_regressor_LWA_ref);
    computeJointKinematicRegressor(model, data_ref, joint_id, WORLD, kinematic_regressor_W_ref);

    BOOST_CHECK(kinematic_regressor_L.isApprox(kinematic_regressor_L_ref));
    BOOST_CHECK(kinematic_regressor_LWA.isApprox(kinematic_regressor_LWA_ref));
    BOOST_CHECK(kinematic_regressor_W.isApprox(kinematic_regressor_W_ref));
  }
}

BOOST_AUTO_TEST_CASE(test_kinematic_regressor_frame)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  buildModels::humanoidRandom(model);

  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill(1.);

  const std::string joint_name = "larm5_joint";
  const JointIndex joint_id = model.getJointId(joint_name);
  model.addBodyFrame("test_body", joint_id, SE3::Random(), -1);

  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);

  const VectorXd q = randomConfiguration(model);

  forwardKinematics(model, data, q);
  updateFramePlacements(model, data);
  forwardKinematics(model, data_ref, q);

  const double eps = 1e-8;
  for (FrameIndex frame_id = 1; frame_id < (FrameIndex)model.nframes; ++frame_id)
  {
    const Frame & frame = model.frames[frame_id];

    Data::Matrix6x kinematic_regressor_L(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));
    Data::Matrix6x kinematic_regressor_LWA(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));
    Data::Matrix6x kinematic_regressor_W(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));

    computeFrameKinematicRegressor(model, data, frame_id, LOCAL, kinematic_regressor_L);
    computeFrameKinematicRegressor(
      model, data, frame_id, LOCAL_WORLD_ALIGNED, kinematic_regressor_LWA);
    computeFrameKinematicRegressor(model, data, frame_id, WORLD, kinematic_regressor_W);

    Data::Matrix6x kinematic_regressor_L_ref(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));
    Data::Matrix6x kinematic_regressor_LWA_ref(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));
    Data::Matrix6x kinematic_regressor_W_ref(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));

    computeJointKinematicRegressor(
      model, data_ref, frame.parentJoint, LOCAL, frame.placement, kinematic_regressor_L_ref);
    computeJointKinematicRegressor(
      model, data_ref, frame.parentJoint, LOCAL_WORLD_ALIGNED, frame.placement,
      kinematic_regressor_LWA_ref);
    computeJointKinematicRegressor(
      model, data_ref, frame.parentJoint, WORLD, frame.placement, kinematic_regressor_W_ref);

    BOOST_CHECK(kinematic_regressor_L.isApprox(kinematic_regressor_L_ref));
    BOOST_CHECK(kinematic_regressor_LWA.isApprox(kinematic_regressor_LWA_ref));
    BOOST_CHECK(kinematic_regressor_W.isApprox(kinematic_regressor_W_ref));

    Data::Matrix6x kinematic_regressor_L_fd(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));
    Data::Matrix6x kinematic_regressor_LWA_fd(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));
    Data::Matrix6x kinematic_regressor_W_fd(Data::Matrix6x::Zero(6, 6 * (model.njoints - 1)));

    Model model_plus = model;
    Data data_plus(model_plus);
    const SE3 & oMf = data.oMf[frame_id];
    const SE3 Mf_LWA = SE3(oMf.rotation(), SE3::Vector3::Zero());
    const SE3 & oMf_plus = data_plus.oMf[frame_id];
    for (int i = 1; i < model.njoints; ++i)
    {
      Motion::Vector6 v = Motion::Vector6::Zero();
      const SE3 & M_placement = model.jointPlacements[(JointIndex)i];
      SE3 & M_placement_plus = model_plus.jointPlacements[(JointIndex)i];
      for (Eigen::DenseIndex k = 0; k < 6; ++k)
      {
        v[k] = eps;
        M_placement_plus = M_placement * exp6(Motion(v));

        forwardKinematics(model_plus, data_plus, q);
        updateFramePlacements(model_plus, data_plus);

        const Motion diff_L = log6(oMf.actInv(oMf_plus));
        kinematic_regressor_L_fd.middleCols<6>(6 * (i - 1)).col(k) = diff_L.toVector() / eps;
        const Motion diff_LWA = Mf_LWA.act(diff_L);
        kinematic_regressor_LWA_fd.middleCols<6>(6 * (i - 1)).col(k) = diff_LWA.toVector() / eps;
        const Motion diff_W = oMf.act(diff_L);
        kinematic_regressor_W_fd.middleCols<6>(6 * (i - 1)).col(k) = diff_W.toVector() / eps;
        v[k] = 0.;
      }

      M_placement_plus = M_placement;
    }

    BOOST_CHECK(kinematic_regressor_L.isApprox(kinematic_regressor_L_fd, sqrt(eps)));
    BOOST_CHECK(kinematic_regressor_LWA.isApprox(kinematic_regressor_LWA_fd, sqrt(eps)));
    BOOST_CHECK(kinematic_regressor_W.isApprox(kinematic_regressor_W_fd, sqrt(eps)));
  }
}

BOOST_AUTO_TEST_CASE(test_static_regressor)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  buildModels::humanoidRandom(model);

  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill(1.);

  VectorXd q = randomConfiguration(model);
  computeStaticRegressor(model, data, q);

  VectorXd phi(4 * (model.njoints - 1));
  for (int k = 1; k < model.njoints; ++k)
  {
    const Inertia & Y = model.inertias[(size_t)k];
    phi.segment<4>(4 * (k - 1)) << Y.mass(), Y.mass() * Y.lever();
  }

  Vector3d com = centerOfMass(model, data_ref, q);
  Vector3d static_com_ref;
  static_com_ref << com;

  Vector3d static_com = data.staticRegressor * phi;

  BOOST_CHECK(static_com.isApprox(static_com_ref));
}

BOOST_AUTO_TEST_CASE(test_body_regressor)
{
  using namespace Eigen;
  using namespace pinocchio;

  Inertia I(Inertia::Random());
  Motion v(Motion::Random());
  Motion a(Motion::Random());

  Force f = I * a + I.vxiv(v);

  Inertia::Vector6 f_regressor = bodyRegressor(v, a) * I.toDynamicParameters();

  BOOST_CHECK(f_regressor.isApprox(f.toVector()));
}

BOOST_AUTO_TEST_CASE(test_joint_body_regressor)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  buildModels::manipulator(model);
  pinocchio::Data data(model);

  JointIndex JOINT_ID = JointIndex(model.njoints) - 1;

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  VectorXd a = Eigen::VectorXd::Random(model.nv);

  rnea(model, data, q, v, a);

  Force f = data.f[JOINT_ID];

  Inertia::Vector6 f_regressor =
    jointBodyRegressor(model, data, JOINT_ID) * model.inertias[JOINT_ID].toDynamicParameters();

  BOOST_CHECK(f_regressor.isApprox(f.toVector()));
}

BOOST_AUTO_TEST_CASE(test_frame_body_regressor)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  buildModels::manipulator(model);

  JointIndex JOINT_ID = JointIndex(model.njoints) - 1;

  const SE3 & framePlacement = SE3::Random();
  FrameIndex FRAME_ID = model.addBodyFrame("test_body", JOINT_ID, framePlacement, -1);

  pinocchio::Data data(model);

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  VectorXd a = Eigen::VectorXd::Random(model.nv);

  rnea(model, data, q, v, a);

  Force f = framePlacement.actInv(data.f[JOINT_ID]);
  Inertia I = framePlacement.actInv(model.inertias[JOINT_ID]);

  Inertia::Vector6 f_regressor =
    frameBodyRegressor(model, data, FRAME_ID) * I.toDynamicParameters();

  BOOST_CHECK(f_regressor.isApprox(f.toVector()));
}

BOOST_AUTO_TEST_CASE(test_joint_torque_regressor)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  buildModels::humanoidRandom(model);

  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill(1.);

  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  VectorXd a = Eigen::VectorXd::Random(model.nv);

  rnea(model, data_ref, q, v, a);

  Eigen::VectorXd params(10 * (model.njoints - 1));
  for (JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
    params.segment<10>((int)((i - 1) * 10)) = model.inertias[i].toDynamicParameters();

  computeJointTorqueRegressor(model, data, q, v, a);

  Eigen::VectorXd tau_regressor = data.jointTorqueRegressor * params;

  BOOST_CHECK(tau_regressor.isApprox(data_ref.tau));
}

BOOST_AUTO_TEST_CASE(test_kinetic_energy_regressor)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  buildModels::humanoidRandom(model);

  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill(1.);

  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data_ref, q, v);
  auto target_energy = computeKineticEnergy(model, data_ref);

  const auto regressor = computeKineticEnergyRegressor(model, data, q, v);

  Eigen::VectorXd params(10 * (model.njoints - 1));
  for (JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
    params.segment<10>(Eigen::DenseIndex((i - 1) * 10)) = model.inertias[i].toDynamicParameters();

  const double kinetic_energy_regressor = data.kineticEnergyRegressor * params;

  BOOST_CHECK_CLOSE(kinetic_energy_regressor, target_energy, 1e-12);
}

BOOST_AUTO_TEST_CASE(test_potential_energy_regressor)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  buildModels::humanoidRandom(model);

  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill(1.);

  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data_ref, q, v);
  const double target_energy = computePotentialEnergy(model, data_ref);

  Eigen::VectorXd params(10 * (model.njoints - 1));
  for (JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
    params.segment<10>(Eigen::DenseIndex((i - 1) * 10)) = model.inertias[i].toDynamicParameters();

  computePotentialEnergyRegressor(model, data, q);
  const double potential_energy_regressor = data.potentialEnergyRegressor * params;

  BOOST_CHECK_CLOSE(potential_energy_regressor, target_energy, 1e-12);
}

BOOST_AUTO_TEST_CASE(test_coriolis_regressor)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  buildModels::manipulator(model);

  pinocchio::Data data(model);

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);

  Eigen::VectorXd params(10 * (model.njoints - 1));
  for (JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
    params.segment<10>(Eigen::DenseIndex((i - 1) * 10)) = model.inertias[i].toDynamicParameters();

  computeAllTerms(model, data, q, v);
  forwardKinematics(model, data, q, v);
  framesForwardKinematics(model, data, q);
  auto regressors = computeIndirectRegressors(model, data, q, v);
  auto Hregressor = regressors.first;
  auto CTregressor = regressors.second;
  auto CTv_regressor = CTregressor * params;

  auto CTv = computeCoriolisMatrix(model, data, q, v).transpose() * v;

  // they should match
  BOOST_CHECK(CTv_regressor.isApprox(CTv));

  // fill in the mass inertia matrix
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();

  const auto momentum = data.M * v;

  auto momentum_regressor = Hregressor * params;

  BOOST_CHECK(momentum_regressor.isApprox(momentum));
}

BOOST_AUTO_TEST_CASE(test_momentum_regressor)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  return;

  // buildModels::humanoid(model);
  // model.lowerPositionLimit.head<7>().fill(-1.);
  // model.upperPositionLimit.head<7>().fill(1.);

  buildModels::manipulator(model);

  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  VectorXd dv = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data_ref, q, v);
  forwardKinematics(model, data_ref, q, v);

  // fill in the mass inertia matrix
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  const VectorXd target_momentum = data_ref.M * v;

  computeMomentumRegressor(model, data, q, v);
  Eigen::VectorXd params(10 * (model.njoints - 1));
  for (JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
    params.segment<10>(Eigen::DenseIndex((i - 1) * 10)) = model.inertias[i].toDynamicParameters();

  const VectorXd momentum_regressor = data.momentumRegressor * params;
  BOOST_CHECK(momentum_regressor.isApprox(target_momentum));

  // perform simple simulation for one timestep to validate dpartial_lagrangian_q
  q = randomConfiguration(model);
  v = Eigen::VectorXd::Random(model.nv);
  dv = Eigen::VectorXd::Random(model.nv);
  VectorXd tau = rnea(model, data, q, v, dv);

  forwardKinematics(model, data_ref, q, v);
  computeMomentumRegressor(model, data, q, v);

  const Eigen::VectorXd momentum_test = data.momentumRegressor * params;
  const Eigen::MatrixXd dpartial_lagrangian_q_test = data.dpartial_lagrangian_q;

  const auto coriolis = computeCoriolisMatrix(model, data_ref, q, v);
  const auto gravity = computeGeneralizedGravity(model, data_ref, q);

  const auto p_dot = coriolis.transpose() * v - gravity + tau;

  std::cout << "p_dot: " << p_dot.transpose() << std::endl;

  const double alpha = 1e-8;
  VectorXd v_plus = v + alpha * dv;
  VectorXd q_plus = integrate(model, q, alpha * v);

  forwardKinematics(model, data_ref, q_plus, v_plus);
  computeMomentumRegressor(model, data, q_plus, v_plus);

  const Eigen::VectorXd momentum_plus = data.momentumRegressor * params;
  // const Eigen::VectorXd dpartial_lagrangian_q_plus = data.dpartial_lagrangian_q;

  // compare the time derivative of momentum
  const Eigen::VectorXd momentum_time_derivative_analytical =
    dpartial_lagrangian_q_test * params + tau;
  // const Eigen::VectorXd momentum_time_derivative_analytical2 =
  //   dpartial_lagrangian_q_plus * params + tau;
  const Eigen::VectorXd momentum_time_derivative_numerical =
    (momentum_plus - momentum_test) / alpha;

  // theoretically, the two should be approximately equal
  std::cout << "analytical: " << momentum_time_derivative_analytical.transpose() << std::endl;
  // std::cout << "analytical: " << momentum_time_derivative_analytical2.transpose() << std::endl;
  std::cout << "numerical: " << momentum_time_derivative_numerical.transpose() << std::endl;

  // // define a random trajectory for all joints
  // const Eigen::VectorXd amplitude = Eigen::VectorXd::Ones(model.nv);
  // auto positions = [](Eigen::VectorXd amplitude, double time) -> Eigen::VectorXd {
  //   return amplitude * std::cos(time);
  // };
  // auto velocities = [](Eigen::VectorXd amplitude, double time) -> Eigen::VectorXd {
  //   return -amplitude * std::sin(time);
  // };
  // auto accelerations = [](Eigen::VectorXd amplitude, double time) -> Eigen::VectorXd {
  //   return -amplitude * std::cos(time);
  // };

  // const double time_step = 1e-2;
  // double t = std::rand() / (double)RAND_MAX;
  // double t_plus = t + time_step;
  // double t_minus = t - time_step;

  // const Eigen::VectorXd q_test = positions(amplitude, t);
  // const Eigen::VectorXd v_test = velocities(amplitude, t);
  // const Eigen::VectorXd a_test = accelerations(amplitude, t);
  // const Eigen::VectorXd q_plus = positions(amplitude, t_plus);
  // const Eigen::VectorXd v_plus = velocities(amplitude, t_plus);
  // const Eigen::VectorXd q_minus = positions(amplitude, t_minus);
  // const Eigen::VectorXd v_minus = velocities(amplitude, t_minus);

  // // compute generalized torque according to the trajectories
  // rnea(model, data, q_test, v_test, a_test);
  // const Eigen::VectorXd tau = data.tau;
  // std::cout << "executed tau: " << tau.transpose() << std::endl;

  // // compute momemtum at each time instance
  // // computeAllTerms(model, data, q_test, v_test);
  // forwardKinematics(model, data_ref, q_test, v_test);

  // auto answer = computeMomentumRegressor(model, data, q_test, v_test);
  // const Eigen::VectorXd momentum_test = data.momentumRegressor * params;
  // const Eigen::MatrixXd dpartial_lagrangian_q_test = data.dpartial_lagrangian_q;

  // // std::cout << "dpartial langrangian q: " << std::endl;
  // // std::cout << dpartial_lagrangian_q_test << std::endl;

  // // computeAllTerms(model, data, q_plus, v_plus);
  // forwardKinematics(model, data_ref, q_plus, v_plus);
  // computeMomentumRegressor(model, data, q_plus, v_plus);
  // const Eigen::VectorXd momentum_plus = data.momentumRegressor * params;

  // // computeAllTerms(model, data, q_minus, v_minus);
  // forwardKinematics(model, data_ref, q_minus, v_minus);
  // computeMomentumRegressor(model, data, q_minus, v_minus);
  // const Eigen::VectorXd momentum_minus = data.momentumRegressor * params;

  // // compute the time derivative of momentum analytically and numerically
  // const Eigen::VectorXd momentum_time_derivative_analytical =
  //   dpartial_lagrangian_q_test * params + tau;
  // const Eigen::VectorXd momentum_time_derivative_numerical =
  //   (momentum_plus - momentum_minus) / (2 * time_step);

  // // theoretically, the two should be approximately equal
  // std::cout << "analytical: " << momentum_time_derivative_analytical.transpose() << std::endl;
  // std::cout << "numerical: " << momentum_time_derivative_numerical.transpose() << std::endl;
}

BOOST_AUTO_TEST_SUITE_END()
