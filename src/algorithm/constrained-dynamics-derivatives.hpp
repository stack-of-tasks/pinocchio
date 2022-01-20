//
// Copyright (c) 2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_constrained_dynamics_derivatives_hpp__
#define __pinocchio_algorithm_constrained_dynamics_derivatives_hpp__

#include "pinocchio/algorithm/contact-info.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, class ContactModelAllocator, class ContactDataAllocator, typename MatrixType1, typename MatrixType2, typename MatrixType3, typename MatrixType4, typename MatrixType5, typename MatrixType6>
  inline void computeConstraintDynamicsDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                const std::vector<RigidConstraintModelTpl<Scalar,Options>,ContactModelAllocator> & contact_models,
                                                std::vector<RigidConstraintDataTpl<Scalar,Options>,ContactDataAllocator> & contact_data,
                                                const Eigen::MatrixBase<MatrixType1> & ddq_partial_dq,
                                                const Eigen::MatrixBase<MatrixType2> & ddq_partial_dv,
                                                const Eigen::MatrixBase<MatrixType3> & ddq_partial_dtau,
                                                const Eigen::MatrixBase<MatrixType4> & lambda_partial_dq,
                                                const Eigen::MatrixBase<MatrixType5> & lambda_partial_dv,
                                                const Eigen::MatrixBase<MatrixType6> & lambda_partial_dtau);
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, class ContactModelAllocator, class ContactDataAllocator>
  inline void computeConstraintDynamicsDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                const std::vector<RigidConstraintModelTpl<Scalar,Options>,ContactModelAllocator> & contact_models,
                                                std::vector<RigidConstraintDataTpl<Scalar,Options>,ContactDataAllocator> & contact_data)
  {
    computeConstraintDynamicsDerivatives(model, data, contact_models, contact_data,
                                      data.ddq_dq, data.ddq_dv, data.ddq_dtau,
                                      data.dlambda_dq, data.dlambda_dv, data.dlambda_dtau);
  };
  
} // namespace pinocchio

#include "pinocchio/algorithm/constrained-dynamics-derivatives.hxx"

#endif // ifndef __pinocchio_algorithm_constrained_dynamics_derivatives_hpp__
