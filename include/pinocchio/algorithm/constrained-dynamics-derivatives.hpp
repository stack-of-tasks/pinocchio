//
// Copyright (c) 2020-2022 CNRS INRIA
//

#ifndef __pinocchio_algorithm_constrained_dynamics_derivatives_hpp__
#define __pinocchio_algorithm_constrained_dynamics_derivatives_hpp__

#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/proximal.hpp"

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
    typename MatrixType4,
    typename MatrixType5,
    typename MatrixType6>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  inline void computeConstraintDynamicsDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
      contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & contact_data,
    const ProximalSettingsTpl<Scalar> & settings,
    const Eigen::MatrixBase<MatrixType1> & ddq_partial_dq,
    const Eigen::MatrixBase<MatrixType2> & ddq_partial_dv,
    const Eigen::MatrixBase<MatrixType3> & ddq_partial_dtau,
    const Eigen::MatrixBase<MatrixType4> & lambda_partial_dq,
    const Eigen::MatrixBase<MatrixType5> & lambda_partial_dv,
    const Eigen::MatrixBase<MatrixType6> & lambda_partial_dtau);

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator,
    typename MatrixType1,
    typename MatrixType2,
    typename MatrixType3,
    typename MatrixType4,
    typename MatrixType5,
    typename MatrixType6>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  inline void computeConstraintDynamicsDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
      contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & contact_data,
    const Eigen::MatrixBase<MatrixType1> & ddq_partial_dq,
    const Eigen::MatrixBase<MatrixType2> & ddq_partial_dv,
    const Eigen::MatrixBase<MatrixType3> & ddq_partial_dtau,
    const Eigen::MatrixBase<MatrixType4> & lambda_partial_dq,
    const Eigen::MatrixBase<MatrixType5> & lambda_partial_dv,
    const Eigen::MatrixBase<MatrixType6> & lambda_partial_dtau)
  {
    ProximalSettingsTpl<Scalar> settings;
    computeConstraintDynamicsDerivatives(
      model, data, contact_models, contact_data, settings, ddq_partial_dq.const_cast_derived(),
      ddq_partial_dv.const_cast_derived(), ddq_partial_dtau.const_cast_derived(),
      lambda_partial_dq.const_cast_derived(), lambda_partial_dv.const_cast_derived(),
      lambda_partial_dtau.const_cast_derived());
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  inline void computeConstraintDynamicsDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
      contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & contact_data,
    const ProximalSettingsTpl<Scalar> & settings)
  {
    computeConstraintDynamicsDerivatives(
      model, data, contact_models, contact_data, settings, data.ddq_dq, data.ddq_dv, data.ddq_dtau,
      data.dlambda_dq, data.dlambda_dv, data.dlambda_dtau);
  };

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  inline void computeConstraintDynamicsDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
      contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & contact_data)
  {
    ProximalSettingsTpl<Scalar> settings;
    computeConstraintDynamicsDerivatives(model, data, contact_models, contact_data, settings);
  };

} // namespace pinocchio

#include "pinocchio/algorithm/constrained-dynamics-derivatives.hxx"

#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  #include "pinocchio/algorithm/constrained-dynamics-derivatives.txx"
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

#endif // ifndef __pinocchio_algorithm_constrained_dynamics_derivatives_hpp__
