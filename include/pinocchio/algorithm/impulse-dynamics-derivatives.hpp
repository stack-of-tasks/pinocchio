//
// Copyright (c) 2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_impulse_dynamics_derivatives_hpp__
#define __pinocchio_algorithm_impulse_dynamics_derivatives_hpp__

#include "pinocchio/algorithm/contact-info.hpp"

namespace pinocchio
{

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator,
    typename MatrixType1,
    typename MatrixType2,
    typename MatrixType3,
    typename MatrixType4>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  inline void computeImpulseDynamicsDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
      contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & contact_data,
    const Scalar r_coeff,
    const ProximalSettingsTpl<Scalar> & settings,
    const Eigen::MatrixBase<MatrixType1> & dvimpulse_partial_dq,
    const Eigen::MatrixBase<MatrixType2> & dvimpulse_partial_dv,
    const Eigen::MatrixBase<MatrixType3> & impulse_partial_dq,
    const Eigen::MatrixBase<MatrixType4> & impulse_partial_dv);

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  inline void computeImpulseDynamicsDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
      contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & contact_data,
    const Scalar r_coeff,
    const ProximalSettingsTpl<Scalar> & settings)
  {
    computeImpulseDynamicsDerivatives(
      model, data, contact_models, contact_data, r_coeff, settings, data.ddq_dq, data.ddq_dv,
      data.dlambda_dq, data.dlambda_dv);
  };

} // namespace pinocchio

#include "pinocchio/algorithm/impulse-dynamics-derivatives.hxx"

#endif // ifndef __pinocchio_algorithm_impulse_dynamics_derivatives_hpp__
