//
// Copyright (c) 2024-2025 INRIA
//

#define BOOST_TEST_MODULE joint_limit_constraint

#include "pinocchio/constraints.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "utils/model-generator.hpp"

// Helpers
#include "constraints/jacobians-checker.hpp"

#include <iostream>

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

// TODO: refacto
using namespace pinocchio;
typedef JointLimitConstraintModel::EigenIndexVector EigenIndexVector;
typedef JointLimitConstraintModel::BooleanVector BooleanVector;

BOOST_AUTO_TEST_CASE(constraint_empty_constructor)
{
  Model model;
  buildModelWithAllJoints(model);

  const Data data(model);

  const Model::IndexVector empty_activable_joint_ids;

  JointLimitConstraintModel constraint(model, empty_activable_joint_ids);

  BOOST_CHECK(constraint.residualSize(MaximalSelection()) == 0);
  BOOST_CHECK(constraint.residualSize(CurrentSelection()) == 0);
}

BOOST_AUTO_TEST_CASE(constraint_model_constructor)
{
  Model model;
  buildModelWithAllJoints(model);

  const Data data(model);

  JointLimitConstraintModel constraint(model);
}

BOOST_AUTO_TEST_CASE(constraint_constructor_with_infinite_bounds)
{
  Model model;
  buildModelWithAllJoints(model);

  model.lowerPositionLimit.fill(-std::numeric_limits<double>::max());
  model.upperPositionLimit.fill(+std::numeric_limits<double>::max());

  const Data data(model);

  const JointIndex constraint_id = JointIndex(model.njoints) - 1;

  const Model::IndexVector & ee_support = model.supports[constraint_id];
  const Model::IndexVector activable_joint_ids(ee_support.begin() + 1, ee_support.end());
  JointLimitConstraintModel constraint(model, activable_joint_ids);

  BOOST_CHECK(constraint.residualSize(MaximalSelection()) == 0);
  BOOST_CHECK(constraint.residualSize(CurrentSelection()) == 0);
}

BOOST_AUTO_TEST_CASE(constraint_constructor)
{
  Model model;
  buildModelWithAllJoints(model);

  Data data(model);

  const JointIndex constraint_id = JointIndex(model.njoints) - 1;

  const Model::IndexVector & ee_support = model.supports[constraint_id];
  const Model::IndexVector activable_joint_ids(ee_support.begin() + 1, ee_support.end());

  for (const JointIndex joint_id : activable_joint_ids)
  {
    const auto & jmodel = model.joints[joint_id];
    std::cout << "joint type: " << jmodel.shortname() << std::endl;
  }
  JointLimitConstraintModel constraint(model, activable_joint_ids);

  // Check size
  {
    int total_size = 0;
    for (const JointIndex joint_id : activable_joint_ids)
    {
      const auto & jmodel = model.joints[joint_id];
      const int idx_q = jmodel.idx_q();
      const int nq = jmodel.nq();
      const auto has_configuration_limit = jmodel.hasConfigurationLimit();
      for (int k = 0; k < nq; ++k)
      {
        const int index_q = idx_q + k;
        if (has_configuration_limit[size_t(k)])
        {
          if (model.lowerPositionLimit[index_q] != -std::numeric_limits<double>::max())
            total_size++;
          if (model.upperPositionLimit[index_q] != +std::numeric_limits<double>::max())
            total_size++;
        }
      }
    }
    BOOST_CHECK(constraint.residualSize(MaximalSelection()) == total_size);
    BOOST_CHECK(constraint.residualSize(CurrentSelection()) == total_size);
    const Eigen::VectorXd q0 = randomConfiguration(model);
    constraint.makeSelectionFilteredByLimitProximity(q0);
    BOOST_CHECK(
      constraint.residualSize(CurrentSelection()) <= constraint.residualSize(MaximalSelection()));
  }

  // we set the margin to infinity so all limits are taken into account in what follows.
  {
    model.positionLimitMargin =
      Eigen::VectorXd::Constant(model.nq, std::numeric_limits<double>::max());
    constraint = JointLimitConstraintModel(model, activable_joint_ids);
    const Eigen::VectorXd q0 = randomConfiguration(model);
    constraint.makeSelectionFilteredByLimitProximity(q0);
    BOOST_CHECK(
      constraint.residualSize(CurrentSelection()) == constraint.residualSize(MaximalSelection()));
  }

  // Check projection on force sets
  {
    constraint = JointLimitConstraintModel(model, activable_joint_ids);

    auto constraint_data = constraint.createData();

    const Eigen::Index total_active_dofs = Eigen::Index(constraint.residualSize());

    const int num_projections = int(1e6);
    for (int k = 0; k < num_projections; ++k)
    {
      const Eigen::VectorXd f = Eigen::VectorXd::Random(total_active_dofs);
      const Eigen::VectorXd f_proj = constraint.set(constraint_data).project(f);

      BOOST_CHECK((f_proj.array() >= 0).all());
    }
  }
}

BOOST_AUTO_TEST_CASE(joint_limit_constraint_cast)
{
  Model model;
  buildModelWithAllJoints(model);

  const Data data(model);

  const JointIndex constraint_id = JointIndex(model.njoints) - 1;

  const Model::IndexVector & ee_support = model.supports[constraint_id];
  const Model::IndexVector active_joint_ids(ee_support.begin() + 1, ee_support.end());
  JointLimitConstraintModel cm(model, active_joint_ids);

  const auto cm_cast_double = cm.cast<double>();
  BOOST_CHECK(cm_cast_double == cm);

  const auto cm_cast_long_double = cm.cast<long double>();
  BOOST_CHECK(cm_cast_long_double.cast<double>() == cm);
}

BOOST_AUTO_TEST_CASE(constraint_jacobian)
{
  Model model;
  buildModelWithAllJoints(model);

  Data data(model);

  const JointIndex constraint_id = JointIndex(model.njoints) - 1;

  const Model::IndexVector & ee_support = model.supports[constraint_id];
  const Model::IndexVector activable_joint_ids(ee_support.begin() + 1, ee_support.end());

  // we set the margin to infinity so all limits are taken into account in what follows.
  model.positionLimitMargin =
    Eigen::VectorXd::Constant(model.nq, std::numeric_limits<double>::max());

  JointLimitConstraintModel constraint_model(model, activable_joint_ids);
  JointLimitConstraintData constraint_data(constraint_model);

  // Check against finite differences on the drift of the constraint
  const double eps_fd = 1e-8;
  for (int i = 0; i < 1e4; ++i)
  {
    const Eigen::VectorXd q0 = randomConfiguration(model);
    data.q_in = q0;
    constraint_model.calc(model, data, constraint_data);

    Eigen::MatrixXd jacobian_matrix(constraint_model.residualSize(), model.nv);
    constraint_model.jacobian(model, data, constraint_data, jacobian_matrix);
    Data data_fd(model);
    JointLimitConstraintData constraint_data_fd(constraint_model);
    Eigen::MatrixXd jacobian_matrix_fd(constraint_model.residualSize(), model.nv);

    for (Eigen::Index k = 0; k < model.nv; ++k)
    {
      Eigen::VectorXd v_eps = Eigen::VectorXd::Zero(model.nv);
      v_eps[k] = eps_fd;
      const Eigen::VectorXd q_plus = integrate(model, q0, v_eps);
      data_fd.q_in = q_plus;

      constraint_model.calc(model, data_fd, constraint_data_fd);

      jacobian_matrix_fd.col(k) =
        (constraint_data_fd.constraint_residual - constraint_data.constraint_residual) / eps_fd;
    }

    BOOST_CHECK(jacobian_matrix.isApprox(jacobian_matrix_fd, math::sqrt(eps_fd)));
  }

  check_jacobians_operations(model, data, constraint_model, constraint_data);
}

BOOST_AUTO_TEST_CASE(dynamic_constraint_residual)
{
  Model model;
  buildModelWithAllJoints(model);

  model.lowerPositionLimit.fill(0.);
  const Eigen::VectorXd qmin(Eigen::VectorXd::Zero(model.nq));
  const Eigen::VectorXd qmax(Eigen::VectorXd::Ones(model.nq));
  // We deactivate the upper limits
  model.upperPositionLimit.fill(std::numeric_limits<double>::max());
  model.positionLimitMargin = Eigen::VectorXd::Constant(model.nq, .5);

  Data data(model);

  int total_size = 0;
  Model::IndexVector activable_joint_ids;
  for (Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    activable_joint_ids.push_back(i);

    const auto & jmodel = model.joints[i];
    const int nq = jmodel.nq();
    const auto has_configuration_limit = jmodel.hasConfigurationLimit();
    for (int k = 0; k < nq; ++k)
    {
      if (has_configuration_limit[size_t(k)])
        total_size++;
    }
  }

  JointLimitConstraintModel constraint_model(model, activable_joint_ids);
  BOOST_CHECK(constraint_model.residualSize(MaximalSelection()) == total_size);

  JointLimitConstraintData constraint_data(constraint_model);

  for (int i = 0; i < 1e4; ++i)
  {
    const Eigen::VectorXd q0 = randomConfiguration(model, qmin, qmax);
    constraint_model.makeSelectionFilteredByLimitProximity(q0);
    data.q_in = q0;
    constraint_model.calc(model, data, constraint_data);

    int csize = 0;
    int set_idx = 0;

    std::vector<std::size_t> active_indexes;
    Eigen::VectorXd activable_residual(constraint_model.residualSize(MaximalSelection()));

    for (Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
    {
      const auto & jmodel = model.joints[i];
      const int idx_q = jmodel.idx_q();
      const int nq = jmodel.nq();

      const auto has_configuration_limit = jmodel.hasConfigurationLimit();
      for (int k = 0; k < nq; ++k)
      {
        if (!has_configuration_limit[size_t(k)])
          continue;
        const int q_index = idx_q + k;
        activable_residual(set_idx) = q0(q_index) - model.lowerPositionLimit[q_index];
        if (activable_residual(set_idx) < model.positionLimitMargin[q_index])
        {
          csize++;
          active_indexes.push_back((std::size_t)set_idx);
        }
        set_idx++;
      }
    }

    Eigen::VectorXd residual(csize);
    for (Eigen::Index j = 0; j < csize; j++)
    {
      residual(j) = activable_residual((Eigen::Index)active_indexes[static_cast<size_t>(j)]);
    }

    BOOST_CHECK(constraint_model.residualSize(MaximalSelection()) == set_idx);
    BOOST_CHECK(constraint_model.residualSize(CurrentSelection()) == csize);
    BOOST_CHECK(csize == constraint_data.constraint_residual.size());
    BOOST_CHECK(constraint_data.constraint_residual.isApprox(residual));
    BOOST_CHECK(active_indexes == constraint_model.active_idx_in_activable());
  }
}

BOOST_AUTO_TEST_CASE(dynamic_constraint_jacobian)
{
  Model model;
  buildModelWithAllJoints(model);

  model.lowerPositionLimit.fill(0.);
  const Eigen::VectorXd qmin(Eigen::VectorXd::Zero(model.nq));
  const Eigen::VectorXd qmax(Eigen::VectorXd::Ones(model.nq));
  // We deactivate the upper limits
  model.upperPositionLimit.fill(std::numeric_limits<double>::max());
  model.positionLimitMargin = Eigen::VectorXd::Constant(model.nq, .5);

  Data data(model);

  int total_size = 0;
  Model::IndexVector activable_joint_ids;
  for (Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    activable_joint_ids.push_back(i);

    const auto & jmodel = model.joints[i];
    const int nq = jmodel.nq();
    const auto has_configuration_limit = jmodel.hasConfigurationLimit();
    for (int k = 0; k < nq; ++k)
    {
      if (has_configuration_limit[size_t(k)])
        total_size++;
    }
  }

  JointLimitConstraintModel constraint_model(model, activable_joint_ids);
  JointLimitConstraintData constraint_data(constraint_model);

  const double eps_fd = 1e-8;
  const int num_tests = 1e4;
  for (int i = 0; i < num_tests; ++i)
  {
    const Eigen::VectorXd q0 = randomConfiguration(model, qmin, qmax);
    constraint_model.makeSelectionFilteredByLimitProximity(q0);

    BOOST_CHECK(constraint_model.residualSize(MaximalSelection()) == total_size);

    data.q_in = q0;
    constraint_model.calc(model, data, constraint_data);

    int csize = 0;
    int set_idx = 0;

    std::vector<std::size_t> active_indexes;
    Eigen::VectorXd activable_residual(constraint_model.residualSize(MaximalSelection()));

    for (Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
    {
      const auto & jmodel = model.joints[i];
      const int idx_q = jmodel.idx_q();
      const int nq = jmodel.nq();

      const auto has_configuration_limit = jmodel.hasConfigurationLimit();
      for (int k = 0; k < nq; ++k)
      {
        if (!has_configuration_limit[size_t(k)])
          continue;
        const int q_index = idx_q + k;
        activable_residual(set_idx) = -(q0(q_index) - model.lowerPositionLimit[q_index]);
        if (-activable_residual(set_idx) < model.positionLimitMargin[q_index])
        {
          csize++;
          active_indexes.push_back((std::size_t)set_idx);
        }
        set_idx++;
      }
    }

    BOOST_CHECK(constraint_model.residualSize(MaximalSelection()) == set_idx);
    BOOST_CHECK(constraint_model.residualSize(CurrentSelection()) == csize);
    BOOST_CHECK(csize == constraint_data.constraint_residual.size());
    BOOST_CHECK(active_indexes == constraint_model.active_idx_in_activable());

    Eigen::MatrixXd jacobian_matrix(constraint_model.residualSize(), model.nv);
    constraint_model.jacobian(model, data, constraint_data, jacobian_matrix);

    Data data_fd(model);
    JointLimitConstraintData constraint_data_fd(constraint_model);
    Eigen::MatrixXd jacobian_matrix_fd(constraint_model.residualSize(), model.nv);

    for (Eigen::Index k = 0; k < model.nv; ++k)
    {
      Eigen::VectorXd v_eps = Eigen::VectorXd::Zero(model.nv);
      v_eps[k] = eps_fd;
      const Eigen::VectorXd q_plus = integrate(model, q0, v_eps);
      data_fd.q_in = q_plus;
      constraint_model.calc(model, data_fd, constraint_data_fd);

      jacobian_matrix_fd.col(k) =
        (constraint_data_fd.constraint_residual - constraint_data.constraint_residual) / eps_fd;
      BOOST_CHECK((jacobian_matrix.col(k) - jacobian_matrix_fd.col(k)).isZero(math::sqrt(eps_fd)));
    }

    BOOST_CHECK(jacobian_matrix.isApprox(jacobian_matrix_fd, math::sqrt(eps_fd)));
  }
}

BOOST_AUTO_TEST_CASE(constraint_coupling_inertia)
{
  Model model;
  buildModelWithAllBoundedJoints(model);

  model.lowerPositionLimit.fill(-1.);
  model.upperPositionLimit.fill(+1.);

  const JointIndex last_joint_id = Model::JointIndex(model.njoints /*-1*/);

  Model::IndexVector activable_joint_ids;
  for (Model::JointIndex i = 1; i < last_joint_id; ++i)
  {
    activable_joint_ids.push_back(i);

    const auto & jmodel = model.joints[i];
    const int nq = jmodel.nq();
    const auto has_configuration_limit = jmodel.hasConfigurationLimit();
    for (size_t k = 0; k < size_t(nq); ++k)
    {
      BOOST_CHECK(has_configuration_limit[k] == true);
    }
  }

  JointLimitConstraintModel constraint_model(model, activable_joint_ids);
  const Eigen::VectorXd q = model.lowerPositionLimit;
  constraint_model.makeSelectionFilteredByLimitProximity(q);

  BOOST_CHECK(constraint_model.residualSize(MaximalSelection()) == 2 * model.nv);
  BOOST_CHECK(constraint_model.residualSize(CurrentSelection()) == model.nv);
  BOOST_CHECK(constraint_model.residualSize() == model.nv);

  Data data(model);
  JointLimitConstraintData constraint_data(constraint_model);
  computeJointJacobians(model, data, q);

  data.q_in = q;
  constraint_model.calc(model, data, constraint_data);

  const Eigen::VectorXd diagonal_inertia =
    Eigen::VectorXd::Random(constraint_model.residualSize()).array().square();
  constraint_model.appendCouplingConstraintInertias(
    model, data, constraint_data, diagonal_inertia, WorldFrameTag());

  Eigen::MatrixXd constraint_jacobian =
    Eigen::MatrixXd::Zero(constraint_model.residualSize(), model.nv);
  constraint_model.jacobian(model, data, constraint_data, constraint_jacobian);

  std::cout << "diagonal_inertia: " << diagonal_inertia.transpose() << std::endl;
  std::cout << "constraint_jacobian:\n" << constraint_jacobian << std::endl;

  const Eigen::MatrixXd joint_space_constraint_inertia =
    constraint_jacobian.transpose() * diagonal_inertia.asDiagonal() * constraint_jacobian;

  for (const auto joint_id : activable_joint_ids)
  {
    const auto & jmodel = model.joints[joint_id];
    const auto jmodel_nv = jmodel.nv();
    const auto jmodel_idx_v = jmodel.idx_v();

    BOOST_CHECK(
      joint_space_constraint_inertia.block(jmodel_idx_v, jmodel_idx_v, jmodel_nv, jmodel_nv)
        .isApprox(data.joint_apparent_inertia[joint_id]));
  }
}

BOOST_AUTO_TEST_CASE(check_maps)
{
  Model model;
  buildModelWithAllBoundedJoints(model);

  model.lowerPositionLimit.fill(-1.);
  model.upperPositionLimit.fill(+1.);

  const JointIndex last_joint_id = Model::JointIndex(model.njoints /*-1*/);

  Model::IndexVector activable_joint_ids;
  for (Model::JointIndex i = 1; i < last_joint_id; ++i)
  {
    activable_joint_ids.push_back(i);

    const auto & jmodel = model.joints[i];
    const int nq = jmodel.nq();
    const auto has_configuration_limit = jmodel.hasConfigurationLimit();
    for (size_t k = 0; k < size_t(nq); ++k)
    {
      BOOST_CHECK(has_configuration_limit[k] == true);
    }
  }

  JointLimitConstraintModel constraint_model(model, activable_joint_ids);
  const Eigen::VectorXd q = model.lowerPositionLimit;
  constraint_model.makeSelectionFilteredByLimitProximity(q);

  BOOST_CHECK(constraint_model.residualSize(MaximalSelection()) == 2 * model.nv);
  BOOST_CHECK(constraint_model.residualSize(CurrentSelection()) == model.nv);
  BOOST_CHECK(constraint_model.residualSize() == model.nv);

  Data data(model);
  JointLimitConstraintData constraint_data(constraint_model);

  data.q_in = q;
  computeJointJacobians(model, data, q);
  constraint_model.calc(model, data, constraint_data);

  // Use a second constraint model and associated data to compute the Jacobian
  JointLimitConstraintModel constraint_model_ref(model, activable_joint_ids);
  constraint_model_ref.makeSelectionFilteredByLimitProximity(q);

  BOOST_CHECK(constraint_model_ref.residualSize(MaximalSelection()) == 2 * model.nv);
  BOOST_CHECK(constraint_model_ref.residualSize(CurrentSelection()) == model.nv);
  BOOST_CHECK(constraint_model_ref.residualSize() == model.nv);

  Data data_ref(model);
  JointLimitConstraintData constraint_data_ref(constraint_model_ref);

  data_ref.q_in = q;
  computeJointJacobians(model, data_ref, q);
  constraint_model_ref.calc(model, data_ref, constraint_data_ref);

  const auto constraint_jacobian_ref =
    constraint_model.jacobian(model, data_ref, constraint_data_ref);

  // Test mapConstraintForcesToJointTorques
  {
    const Eigen::VectorXd constraint_forces =
      Eigen::VectorXd::Random(constraint_model.residualSize());

    Eigen::VectorXd joint_torques_ref = Eigen::VectorXd::Zero(model.nv);
    joint_torques_ref = constraint_jacobian_ref.transpose() * constraint_forces;

    Eigen::VectorXd joint_torques_ref2 = Eigen::VectorXd::Zero(model.nv);
    constraint_model.jacobianTransposeMatrixProduct(
      model, data_ref, constraint_data_ref, constraint_forces, joint_torques_ref2, SetTo());

    Eigen::VectorXd joint_torques = Eigen::VectorXd::Zero(model.nv);
    constraint_model.mapConstraintForceToJointTorques(
      model, data_ref, constraint_data, constraint_forces, joint_torques);

    BOOST_CHECK(joint_torques.isApprox(joint_torques_ref));
    BOOST_CHECK(joint_torques.isApprox(joint_torques_ref2));
    // std::cout << "joint_torques: " << joint_torques.transpose() << std::endl;
    // std::cout << "joint_torques_ref: " << joint_torques_ref.transpose() << std::endl;
    // std::cout << "joint_torques_ref2: " << joint_torques_ref2.transpose() << std::endl;
  }

  // Test mapJointMotionsToConstraintMotions
  {
    const Eigen::VectorXd joint_motions = Eigen::VectorXd::Random(model.nv);

    Eigen::VectorXd constraint_motions_ref = Eigen::VectorXd::Zero(constraint_model.residualSize());
    constraint_motions_ref = constraint_jacobian_ref * joint_motions;

    Eigen::VectorXd constraint_motions_ref2 =
      Eigen::VectorXd::Zero(constraint_model.residualSize());
    constraint_model.jacobianMatrixProduct(
      model, data_ref, constraint_data_ref, joint_motions, constraint_motions_ref2, SetTo());

    Eigen::VectorXd constraint_motions = -Eigen::VectorXd::Ones(constraint_model.residualSize());
    constraint_model.mapJointMotionsToConstraintMotion(
      model, data_ref, constraint_data, joint_motions, constraint_motions);

    BOOST_CHECK(constraint_motions.isApprox(constraint_motions_ref));
    BOOST_CHECK(constraint_motions.isApprox(constraint_motions_ref2));
    std::cout << "constraint_motions: " << constraint_motions.transpose() << std::endl;
    std::cout << "constraint_motions_ref: " << constraint_motions_ref.transpose() << std::endl;
    std::cout << "constraint_motions_ref2: " << constraint_motions_ref2.transpose() << std::endl;
  }
}

BOOST_AUTO_TEST_CASE(compliance)
{
  {
    JointLimitConstraintModel empty_jmodel;
    Eigen::VectorXd compliance;
    empty_jmodel.retrieveCompliance(compliance);
    BOOST_CHECK(compliance.size() == 0);
  }

  Model model;
  buildModelWithAllBoundedJoints(model);

  model.lowerPositionLimit.fill(-1.);
  model.upperPositionLimit.fill(+1.);

  const JointIndex last_joint_id = Model::JointIndex(model.njoints /*-1*/);

  Model::IndexVector activable_joint_ids;
  for (Model::JointIndex i = 1; i < last_joint_id; ++i)
  {
    activable_joint_ids.push_back(i);

    const auto & jmodel = model.joints[i];
    const int nq = jmodel.nq();
    const auto has_configuration_limit = jmodel.hasConfigurationLimit();
    for (size_t k = 0; k < size_t(nq); ++k)
    {
      BOOST_CHECK(has_configuration_limit[k] == true);
    }
  }
  JointLimitConstraintModel cmodel(model, activable_joint_ids);
  const Eigen::VectorXd q = model.lowerPositionLimit;

  {
    // check retrieve compliance
    Eigen::VectorXd compliance(cmodel.residualSize());
    cmodel.retrieveCompliance(compliance);
    BOOST_CHECK(compliance == Eigen::VectorXd::Zero(cmodel.residualSize()));
    //
    Eigen::VectorXd compliance_full(cmodel.residualSize(MaximalSelection()));
    cmodel.retrieveCompliance(compliance_full, MaximalSelection());
    BOOST_CHECK(compliance_full == Eigen::VectorXd::Zero(cmodel.residualSize(MaximalSelection())));
    BOOST_CHECK(compliance == compliance_full);

    cmodel.makeSelectionFilteredByLimitProximity(q);
    BOOST_CHECK(cmodel.residualSize() < cmodel.residualSize(MaximalSelection()));
    //
    compliance.resize(cmodel.residualSize());
    cmodel.retrieveCompliance(compliance);
    BOOST_CHECK(compliance == Eigen::VectorXd::Zero(cmodel.residualSize()));
    //
    cmodel.retrieveCompliance(compliance_full, MaximalSelection());
    BOOST_CHECK(compliance_full == Eigen::VectorXd::Zero(cmodel.residualSize(MaximalSelection())));
    BOOST_CHECK(compliance.size() < compliance_full.size());
  }

  {
    // check set compliance
    Eigen::VectorXd compliance_ref = Eigen::VectorXd::Random(cmodel.residualSize());
    compliance_ref = compliance_ref.cwiseAbs();
    Eigen::VectorXd compliance_full_ref =
      Eigen::VectorXd::Random(cmodel.residualSize(MaximalSelection()));
    compliance_full_ref = compliance_full_ref.cwiseAbs();

    cmodel.makeSelectionFilteredByLimitProximity(q);
    BOOST_CHECK(cmodel.residualSize() < cmodel.residualSize(MaximalSelection()));

    cmodel.setCompliance(compliance_ref);
    Eigen::VectorXd compliance(cmodel.residualSize());
    cmodel.retrieveCompliance(compliance);
    BOOST_CHECK(compliance == compliance_ref);

    cmodel.setCompliance(compliance_full_ref, MaximalSelection());
    Eigen::VectorXd compliance_full(cmodel.residualSize(MaximalSelection()));
    cmodel.retrieveCompliance(compliance_full, MaximalSelection());
    BOOST_CHECK(compliance_full == compliance_full_ref);
    BOOST_CHECK(compliance.size() < compliance_full_ref.size());
  }
}
