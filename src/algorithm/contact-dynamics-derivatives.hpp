//
// Copyright (c) 2020 LAAS, INRIA
//

#ifndef __pinocchio_algorithm_contact_dynamics_derivatives_hpp__
#define __pinocchio_algorithm_contact_dynamics_derivatives_hpp__

#include "pinocchio/algorithm/contact-info.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2, class Allocator, class AllocatorData>//, typename MatrixType1, typename MatrixType2, typename MatrixType3, typename MatrixType4, typename MatrixType5, typename MatrixType6>  
  inline void computeContactDynamicsDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                const Eigen::MatrixBase<ConfigVectorType> & q,
                                                const Eigen::MatrixBase<TangentVectorType1> & v,
                                                const Eigen::MatrixBase<TangentVectorType2> & tau,
                                                const std::vector<RigidContactModelTpl<Scalar,Options>,Allocator> & contact_models,
                                                const std::vector<RigidContactDataTpl<Scalar,Options>,AllocatorData> & contact_datas,
                                                const Scalar mu = Scalar(0.)/*,
                                                const Eigen::MatrixBase<MatrixType1> & ddq_partial_dq,
                                                const Eigen::MatrixBase<MatrixType2> & ddq_partial_dv,
                                                const Eigen::MatrixBase<MatrixType3> & ddq_partial_dtau,
                                                const Eigen::MatrixBase<MatrixType4> & lambda_partial_dq,
                                                const Eigen::MatrixBase<MatrixType5> & lambda_partial_dv,
                                                const Eigen::MatrixBase<MatrixType6> & lambda_partial_dtau*/);
  
} // namespace pinocchio

#include "pinocchio/algorithm/contact-dynamics-derivatives.hxx"

#endif // ifndef __pinocchio_algorithm_contact_dynamics_hpp__
