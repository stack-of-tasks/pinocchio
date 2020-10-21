//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_contact_dynamics_hxx__
#define __pinocchio_contact_dynamics_hxx__

#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/check.hpp"

#include <Eigen/Cholesky>

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename TangentVectorType,
  typename ConstraintMatrixType, typename DriftVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  forwardDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<TangentVectorType> & tau,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Eigen::MatrixBase<DriftVectorType> & gamma,
                  const Scalar inv_damping)
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(tau.size(), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(J.cols(), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(J.rows(), gamma.size());
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typename Data::TangentVectorType & a = data.ddq;
    typename Data::VectorXs & lambda_c = data.lambda_c;
    
    // Compute the UDUt decomposition of data.M
    cholesky::decompose(model, data);
    
    // Compute the dynamic drift (control - nle)
    data.torque_residual = tau - data.nle;
    cholesky::solve(model, data, data.torque_residual);
    
    data.sDUiJt = J.transpose();
    // Compute U^-1 * J.T
    cholesky::Uiv(model, data, data.sDUiJt);
    for(Eigen::DenseIndex k=0;k<model.nv;++k)
      data.sDUiJt.row(k) /= sqrt(data.D[k]);
    
    data.JMinvJt.noalias() = data.sDUiJt.transpose() * data.sDUiJt;
    
    data.JMinvJt.diagonal().array() += inv_damping;
    data.llt_JMinvJt.compute(data.JMinvJt);
    
    // Compute the Lagrange Multipliers
    lambda_c.noalias() = -J*data.torque_residual;
    lambda_c -= gamma;
    data.llt_JMinvJt.solveInPlace(lambda_c);
    
    // Compute the joint acceleration
    a.noalias() = J.transpose() * lambda_c;
    cholesky::solve(model, data, a);
    a += data.torque_residual;
    
    return a;
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2,
  typename ConstraintMatrixType, typename DriftVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  forwardDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType1> & v,
                  const Eigen::MatrixBase<TangentVectorType2> & tau,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Eigen::MatrixBase<DriftVectorType> & gamma,
                  const Scalar inv_damping)
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.size(), model.nq);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv);

    computeAllTerms(model, data, q, v);

    return forwardDynamics(model,data,tau,J,gamma,inv_damping);
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
           typename ConfigVectorType, typename ConstraintMatrixType, typename KKTMatrixType>
  void computeKKTContactDynamicMatrixInverse(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                             DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                             const Eigen::MatrixBase<ConfigVectorType> & q,
                                             const Eigen::MatrixBase<ConstraintMatrixType> & J,
                                             const Eigen::MatrixBase<KKTMatrixType> & KKTMatrix_inv,
                                             const Scalar & inv_damping)
  {
    assert(model.check(data));
    PINOCCHIO_CHECK_INPUT_ARGUMENT(inv_damping >= 0., "mu must be positive.");
    
    // Compute the mass matrix.
    crbaMinimal(model,data,q);
    
    // Compute the UDUt decomposition of data.M.
    cholesky::decompose(model, data);
    
    using std::sqrt;
    data.sDUiJt = J.transpose();
    // Compute U^-1 * J.T
    cholesky::Uiv(model, data, data.sDUiJt);
    for(Eigen::DenseIndex k=0;k<model.nv;++k)
      data.sDUiJt.row(k) /= sqrt(data.D[k]);
    
    data.JMinvJt.noalias() = data.sDUiJt.transpose() * data.sDUiJt;
    
    data.JMinvJt.diagonal().array() += inv_damping;
    data.llt_JMinvJt.compute(data.JMinvJt);
    
    getKKTContactDynamicMatrixInverse(model,data,J,KKTMatrix_inv.const_cast_derived());
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
  typename ConstraintMatrixType, typename KKTMatrixType>
  inline void getKKTContactDynamicMatrixInverse(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                const Eigen::MatrixBase<ConstraintMatrixType> & J,
                                                const Eigen::MatrixBase<KKTMatrixType> & KKTMatrix_inv)
  {
    assert(model.check(data));
    PINOCCHIO_CHECK_ARGUMENT_SIZE(KKTMatrix_inv.cols(), data.JMinvJt.cols() + model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(KKTMatrix_inv.rows(), data.JMinvJt.rows() + model.nv);
    
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    const Eigen::DenseIndex nc = data.JMinvJt.cols();
    
    KKTMatrixType & KKTMatrix_inv_ = PINOCCHIO_EIGEN_CONST_CAST(KKTMatrixType,KKTMatrix_inv);
    
    typedef Eigen::Block<KKTMatrixType> BlockType;
    BlockType topLeft = KKTMatrix_inv_.topLeftCorner(model.nv, model.nv);
    BlockType topRight = KKTMatrix_inv_.topRightCorner(model.nv, nc);
    BlockType bottomLeft = KKTMatrix_inv_.bottomLeftCorner(nc, model.nv);
    BlockType bottomRight = KKTMatrix_inv_.bottomRightCorner(nc, nc);
    
    bottomRight = -Data::MatrixXs::Identity(nc,nc); data.llt_JMinvJt.solveInPlace(bottomRight);
    topLeft.setIdentity(); cholesky::solve(model, data, topLeft);
    
    bottomLeft.noalias() = J*topLeft;
    topRight.noalias() = bottomLeft.transpose() * (-bottomRight);
    topLeft.noalias() -= topRight*bottomLeft;
    bottomLeft = topRight.transpose();
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename ConstraintMatrixType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  impulseDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType> & v_before,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Scalar r_coeff,
                  const Scalar inv_damping)
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.size(), model.nq);
    
    // Compute the mass matrix
    crba(model, data, q);
    
    return impulseDynamics(model,data,v_before,J,r_coeff,inv_damping);
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename TangentVectorType, typename ConstraintMatrixType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  impulseDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<TangentVectorType> & v_before,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Scalar r_coeff,
                  const Scalar inv_damping)
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(v_before.size(), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(J.cols(), model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typename Data::VectorXs & impulse_c = data.impulse_c;
    typename Data::TangentVectorType & dq_after = data.dq_after;
    
    // Compute the UDUt decomposition of data.M
    cholesky::decompose(model, data);
    
    data.sDUiJt = J.transpose();
    // Compute U^-1 * J.T
    cholesky::Uiv(model, data, data.sDUiJt);
    for(int k=0;k<model.nv;++k) data.sDUiJt.row(k) /= sqrt(data.D[k]);
    
    data.JMinvJt.noalias() = data.sDUiJt.transpose() * data.sDUiJt;
    
    data.JMinvJt.diagonal().array() += inv_damping;
    data.llt_JMinvJt.compute(data.JMinvJt);
    
    // Compute the Lagrange Multipliers related to the contact impulses
    impulse_c.noalias() = (-r_coeff - 1.) * (J * v_before);
    data.llt_JMinvJt.solveInPlace(impulse_c);
    
    // Compute the joint velocity after impacts
    dq_after.noalias() = J.transpose() * impulse_c;
    cholesky::solve(model, data, dq_after);
    dq_after += v_before;
    
    return dq_after;
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_contact_dynamics_hxx__
