//
// Copyright (c) 2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_impulse_dynamics_hxx__
#define __pinocchio_algorithm_impulse_dynamics_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hxx"
#include <limits>

namespace pinocchio
{

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator>
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType & impulseDynamics(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v_before,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
      contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & contact_datas,
    const Scalar r_coeff,
    const ProximalSettingsTpl<Scalar> & settings)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      q.size() == model.nq, "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      v_before.size() == model.nv, "The joint velocity vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      check_expression_if_real<Scalar>(settings.mu >= Scalar(0)), "mu has to be positive");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      check_expression_if_real<Scalar>((r_coeff >= Scalar(0)) && (r_coeff <= Scalar(1))),
      "r_coeff has to be in [0,1]");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      contact_models.size() == contact_datas.size(),
      "The contact models and data do not have the same vector size.");

    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef RigidConstraintModelTpl<Scalar, Options> RigidConstraintModel;
    typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;

    typename Data::TangentVectorType & dq_after = data.dq_after;
    typename Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
    typename Data::VectorXs & primal_dual_contact_solution = data.primal_dual_contact_solution;

    data.oYcrb[0].setZero();
    typedef ContactAndImpulseDynamicsForwardStep<
      Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1, false>
      Pass1;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Pass1::run(
        model.joints[i], data.joints[i],
        typename Pass1::ArgsType(model, data, q.derived(), v_before.derived()));
    }

    typedef ContactAndImpulseDynamicsBackwardStep<Scalar, Options, JointCollectionTpl, false> Pass2;
    for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
    {
      Pass2::run(model.joints[i], typename Pass2::ArgsType(model, data));
    }

    data.M.diagonal() += model.armature;
    contact_chol.compute(model, data, contact_models, contact_datas, settings.mu);

    // Centroidal computations
    typedef typename Data::Force Force;
    typedef Eigen::Block<typename Data::Matrix6x, 3, -1> Block3x;
    data.com[0] = data.oYcrb[0].lever();

    data.Ag = data.dFda;
    const Block3x Ag_lin = data.Ag.template middleRows<3>(Force::LINEAR);
    Block3x Ag_ang = data.Ag.template middleRows<3>(Force::ANGULAR);
    for (long i = 0; i < model.nv; ++i)
      Ag_ang.col(i) += Ag_lin.col(i).cross(data.com[0]);

    primal_dual_contact_solution.tail(model.nv).setZero();
    Eigen::DenseIndex current_row_id = 0;
    for (size_t contact_id = 0; contact_id < contact_models.size(); ++contact_id)
    {
      const RigidConstraintModel & contact_model = contact_models[contact_id];
      RigidConstraintData & contact_data = contact_datas[contact_id];
      const int contact_dim = contact_model.size();

      const JointIndex joint1_id = contact_model.joint1_id;
      const typename Data::SE3 & oMi = data.oMi[joint1_id];
      typename Data::SE3 & oMc = contact_data.oMc1;

      // Update contact placement
      oMc = oMi * contact_model.joint1_placement;

      typename Data::Motion pre_impact_velocity;
      switch (contact_model.reference_frame)
      {
      case WORLD: {
        // Temporary assignment
        pre_impact_velocity = data.ov[joint1_id];
        break;
      }
      case LOCAL_WORLD_ALIGNED: {
        // Temporary assignment
        pre_impact_velocity = data.ov[joint1_id];
        pre_impact_velocity.linear() -= oMc.translation().cross(data.ov[joint1_id].angular());
        break;
      }
      case LOCAL: {
        // Temporary assignment
        pre_impact_velocity = oMc.actInv(data.ov[joint1_id]);
        break;
      }
      default:
        assert(false && "must never happened");
        break;
      }

      switch (contact_model.type)
      {
      case CONTACT_3D:
        primal_dual_contact_solution.segment(current_row_id, contact_dim) =
          -(1 + r_coeff) * pre_impact_velocity.linear();
        break;
      case CONTACT_6D:
        primal_dual_contact_solution.segment(current_row_id, contact_dim) =
          -(1 + r_coeff) * pre_impact_velocity.toVector();
        break;
      default:
        assert(false && "must never happened");
        break;
      }

      current_row_id += contact_dim;
    }

    // Solve the system
    contact_chol.solveInPlace(primal_dual_contact_solution);

    // Retrieve the joint space delta v
    data.ddq = primal_dual_contact_solution.tail(model.nv);
    dq_after = data.ddq + v_before;
    data.impulse_c = -primal_dual_contact_solution.head(contact_chol.constraintDim());

    // Retrieve the impulses
    Eigen::DenseIndex current_row_sol_id = 0;
    for (size_t contact_id = 0; contact_id < contact_models.size(); ++contact_id)
    {
      const RigidConstraintModel & contact_model = contact_models[contact_id];
      RigidConstraintData & contact_data = contact_datas[contact_id];
      typename RigidConstraintData::Force & impulse = contact_data.contact_force;
      const int contact_dim = contact_model.size();

      switch (contact_model.type)
      {
      case CONTACT_3D: {
        impulse.linear() = -primal_dual_contact_solution.template segment<3>(current_row_sol_id);
        impulse.angular().setZero();
        break;
      }
      case CONTACT_6D: {
        typedef typename Data::VectorXs::template FixedSegmentReturnType<6>::Type Segment6d;
        const ForceRef<Segment6d> i_sol(
          primal_dual_contact_solution.template segment<6>(current_row_sol_id));
        impulse = -i_sol;
        break;
      }
      default:
        assert(false && "must never happened");
        break;
      }

      current_row_sol_id += contact_dim;
    }

    return dq_after;
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_impulse_dynamics_hxx__
