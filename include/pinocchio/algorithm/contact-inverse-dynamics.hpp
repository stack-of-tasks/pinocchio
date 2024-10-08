//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_algorithm_contact_inverse_dynamics__hpp__
#define __pinocchio_algorithm_contact_inverse_dynamics__hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/constraints/coulomb-friction-cone.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <boost/optional/optional.hpp>
#include <pinocchio/algorithm/contact-cholesky.hpp>
#include <pinocchio/algorithm/contact-jacobian.hpp>
#include "pinocchio/algorithm/proximal.hpp"

#include <boost/optional.hpp>

namespace pinocchio
{

  ///
  /// \brief Compute the contact impulses given a target velocity of contact points.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint acceleration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] c_ref The contact point velocity
  /// \param[in] contact_models The list of contact models.
  /// \param[in] contact_datas The list of contact_datas.
  /// \param[in] cones list of friction cones.
  /// \param[in] R vector representing the diagonal of the compliance matrix.
  /// \param[in] constraint_correction vector representing the constraint correction.
  /// \param[in] settings The settings for the proximal algorithm.
  /// \param[in] impulse_guess initial guess for the contact impulses.
  ///
  /// \return The desired joint torques stored in data.tau.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename VectorLikeC,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator,
    class CoulombFrictionConelAllocator,
    typename VectorLikeR,
    typename VectorLikeGamma,
    typename VectorLikeImp>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::
    TangentVectorType & computeContactImpulses(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<VectorLikeC> & c_ref,
      const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
        contact_models,
      std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & contact_datas,
      const std::vector<CoulombFrictionConeTpl<Scalar>, CoulombFrictionConelAllocator> & cones,
      const Eigen::MatrixBase<VectorLikeR> & R,
      const Eigen::MatrixBase<VectorLikeGamma> & constraint_correction,
      ProximalSettingsTpl<Scalar> & settings,
      const boost::optional<VectorLikeImp> & impulse_guess = boost::none)
  {

    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> MatrixXs;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> VectorXs;
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;

    int problem_size = R.size();
    int n_contacts = (int)problem_size / 3;
    PINOCCHIO_CHECK_ARGUMENT_SIZE(constraint_correction.size(), problem_size);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(contact_models.size(), n_contacts);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(contact_datas.size(), n_contacts);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      check_expression_if_real<Scalar>(settings.mu > Scalar(0)), "mu has to be strictly positive");
    MatrixXs J = MatrixXs::Zero(problem_size, model.nv); // TODO: malloc
    getConstraintsJacobian(model, data, contact_models, contact_datas, J);
    VectorXs c_ref_cor, desaxce_correction, R_prox, impulse_c_prev, dimpulse_c; // TODO: malloc
    R_prox = R + VectorXs::Constant(problem_size, settings.mu);
    c_ref_cor = c_ref + constraint_correction;
    if (impulse_guess)
    {
      data.impulse_c = impulse_guess.get();
      PINOCCHIO_CHECK_ARGUMENT_SIZE(data.impulse_c.size(), problem_size);
    }
    else
    {
      data.impulse_c.setZero();
    }
    Scalar impulse_c_prev_norm_inf = data.impulse_c.template lpNorm<Eigen::Infinity>();
    Scalar complementarity, dual_feasibility;
    bool abs_prec_reached = false, rel_prec_reached = false;
    const size_t nc = cones.size(); // num constraints
    settings.iter = 1;
    for (; settings.iter <= settings.max_iter; ++settings.iter)
    {
      impulse_c_prev = data.impulse_c;
      for (size_t cone_id = 0; cone_id < nc; ++cone_id)
      {
        const Eigen::DenseIndex row_id = 3 * cone_id;
        const auto & cone = cones[cone_id];
        auto impulse_segment = data.impulse_c.template segment<3>(row_id);
        auto impulse_prev_segment = impulse_c_prev.template segment<3>(row_id);
        auto R_prox_segment = R_prox.template segment<3>(row_id);
        // Vector3 desaxce_segment;
        // auto desaxce_segment = desaxce_correction.template segment<3>(row_id);
        auto c_ref_segment = c_ref.template segment<3>(row_id);
        Vector3 desaxce_segment = cone.computeNormalCorrection(
          c_ref_segment
          + (R.template segment<3>(row_id).array() * impulse_segment.array()).matrix());
        impulse_segment =
          -((c_ref_segment + desaxce_segment - settings.mu * impulse_prev_segment).array()
            / R_prox_segment.array())
             .matrix();
        impulse_segment = cone.weightedProject(impulse_segment, R_prox_segment);
      }
      dimpulse_c = data.impulse_c - impulse_c_prev;
      settings.relative_residual = dimpulse_c.template lpNorm<Eigen::Infinity>();

      // if(   check_expression_if_real<Scalar,false>(complementarity <= this->absolute_precision)
      //    && check_expression_if_real<Scalar,false>(dual_feasibility <= this->absolute_precision)
      //    && check_expression_if_real<Scalar,false>(primal_feasibility <=
      //    this->absolute_precision))
      //   abs_prec_reached = true;
      // else
      //   abs_prec_reached = false;

      const Scalar impulse_c_norm_inf = data.impulse_c.template lpNorm<Eigen::Infinity>();
      if (check_expression_if_real<Scalar, false>(
            settings.relative_residual
            <= settings.relative_accuracy * math::max(impulse_c_norm_inf, impulse_c_prev_norm_inf)))
        rel_prec_reached = true;
      else
        rel_prec_reached = false;

      if (abs_prec_reached || rel_prec_reached)
        break;

      impulse_c_prev_norm_inf = impulse_c_norm_inf;
    }
    return data.impulse_c;
  }

  ///
  /// \brief The Contact Inverse Dynamics algorithm. It computes the inverse dynamics in the
  /// presence of contacts, aka the joint torques according to the current state of the system and
  /// the desired joint accelerations.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint acceleration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] a The joint acceleration vector (dim model.nv).
  /// \param[in] dt The time step.
  /// \param[in] contact_models The list of contact models.
  /// \param[in] contact_datas The list of contact_datas.
  /// \param[in] cones list of friction cones.
  /// \param[in] R vector representing the diagonal of the compliance matrix.
  /// \param[in] constraint_correction vector representing the constraint correction.
  /// \param[in] settings The settings for the proximal algorithm.
  /// \param[in] lambda_guess initial guess for the contact forces.
  ///
  /// \return The desired joint torques stored in data.tau.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator,
    class CoulombFrictionConelAllocator,
    typename VectorLikeR,
    typename VectorLikeGamma,
    typename VectorLikeLam>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::
    TangentVectorType & contactInverseDynamics(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & a,
      Scalar dt,
      const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
        contact_models,
      std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & contact_datas,
      const std::vector<CoulombFrictionConeTpl<Scalar>, CoulombFrictionConelAllocator> & cones,
      const Eigen::MatrixBase<VectorLikeR> & R,
      const Eigen::MatrixBase<VectorLikeGamma> & constraint_correction,
      ProximalSettingsTpl<Scalar> & settings,
      const boost::optional<VectorLikeLam> & lambda_guess = boost::none)
  {

    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> MatrixXs;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> VectorXs;
    typedef ForceTpl<Scalar, Options> Force;
    typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;

    int problem_size = R.size();
    int n_contacts = (int)problem_size / 3;
    MatrixXs J = MatrixXs::Zero(problem_size, model.nv); // TODO: malloc
    getConstraintsJacobian(model, data, contact_models, contact_datas, J);
    VectorXs v_ref, c_ref, tau_c;
    v_ref = v + dt * a;
    c_ref.noalias() = J * v_ref; // TODO should rather use the displacement
    boost::optional<VectorXs> impulse_guess = boost::none;
    if (lambda_guess)
    {
      data.impulse_c = lambda_guess.get();
      data.impulse_c *= dt;
      impulse_guess = boost::make_optional(data.impulse_c);
    }
    computeContactImpulses(
      model, data, c_ref, contact_models, contact_datas, cones, R, constraint_correction, settings,
      impulse_guess);
    data.lambda_c = data.impulse_c / dt;
    container::aligned_vector<Force> fext(model.njoints);
    for (int i = 0; i < model.njoints; i++)
    {
      fext[i] = Force::Zero();
    }
    for (int i = 0; i < n_contacts; i++)
    {
      const auto & cmodel = contact_models[i];
      const Eigen::DenseIndex row_id = 3 * i;
      auto lambda_segment = data.lambda_c.template segment<3>(row_id);
      typename RigidConstraintData::Matrix6 actInv_transpose1 =
        cmodel.joint1_placement.toActionMatrixInverse();
      actInv_transpose1.transposeInPlace();
      fext[cmodel.joint1_id] += Force(actInv_transpose1.template leftCols<3>() * lambda_segment);
      typename RigidConstraintData::Matrix6 actInv_transpose2 =
        cmodel.joint2_placement.toActionMatrixInverse();
      actInv_transpose2.transposeInPlace();
      fext[cmodel.joint2_id] += Force(actInv_transpose2.template leftCols<3>() * lambda_segment);
    }
    rnea(model, data, q, v, a, fext);
    return data.tau;
  }

} // namespace pinocchio

// #if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
// #include "pinocchio/algorithm/contact-inverse-dynamics.txx"
// #endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

#endif // ifndef __pinocchio_algorithm_contact_inverse_dynamics_hpp__
