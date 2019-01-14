//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_contact_dynamics_hpp__
#define __pinocchio_contact_dynamics_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/check.hpp"

#include <Eigen/Cholesky>

namespace pinocchio
{
  
  ///
  /// \brief Compute the forward dynamics with contact constraints.
  /// \note It computes the following problem: <BR>
  ///       <CENTER> \f$ \begin{eqnarray} \underset{\ddot{q}}{\min} & & \| \ddot{q} - \ddot{q}_{\text{free}} \|_{M(q)} \\
  ///           \text{s.t.} & & J (q) \ddot{q} + \gamma (q, \dot{q}) = 0 \end{eqnarray} \f$ </CENTER> <BR>
  ///       where \f$ \ddot{q}_{\text{free}} \f$ is the free acceleration (i.e. without constraints),
  ///       \f$ M \f$ is the mass matrix, \f$ J \f$ the constraint Jacobian and \f$ \gamma \f$ is the constraint drift.
  ///  By default, the constraint Jacobian is assumed to be full rank, and undamped Cholesky inverse is performed.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint torque vector.
  /// \tparam ConstraintMatrixType Type of the constraint matrix.
  /// \tparam DriftVectorType Type of the drift vector.

  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  /// \param[in] v The joint velocity (vector dim model.nv).
  /// \param[in] tau The joint torque vector (dim model.nv).
  /// \param[in] J The Jacobian of the constraints (dim nb_constraints*model.nv).
  /// \param[in] gamma The drift of the constraints (dim nb_constraints).
  /// \param[in] inv_damping Damping factor for cholesky decomposition of JMinvJt. Set to zero if constraints are full rank.    
  /// \param[in] updateKinematics If true, the algorithm calls first pinocchio::computeAllTerms. Otherwise, it uses the current dynamic values stored in data. \\
  ///            \note A hint: 1e-12 as the damping factor gave good result in the particular case of redundancy in contact constraints on the two feet.
  ///
  /// \return A reference to the joint acceleration stored in data.ddq. The Lagrange Multipliers linked to the contact forces are available throw data.lambda_c vector.
  ///
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
                  const Scalar inv_damping = 0.,
                  const bool updateKinematics = true
                  )
  {
    assert(q.size() == model.nq);
    assert(v.size() == model.nv);
    assert(tau.size() == model.nv);
    assert(J.cols() == model.nv);
    assert(J.rows() == gamma.size());
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typename Data::TangentVectorType & a = data.ddq;
    typename Data::VectorXs & lambda_c = data.lambda_c;
    
    if (updateKinematics)
      computeAllTerms(model, data, q, v);
    
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

  
  ///
  /// \brief Computes the inverse of the KKT matrix for dynamics with contact constraints, [[M JT], [J 0]].
  /// The matrix is defined when we call forwardDynamics/impulsedynamics. This method makes use of
  /// the matrix decompositions performed during the forwardDynamics/impulasedynamics and returns the inverse.
  /// The jacobian should be the same that was provided to forwardDynamics/impulasedynamics.
  /// Thus you should call forward Dynamics/impulsedynamics first.
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[out] MJtJ_inv inverse of the MJtJ matrix.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
           typename ConstraintMatrixType, typename KKTMatrixType>
  inline void getKKTContactDynamicMatrixInverse(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                const Eigen::MatrixBase<ConstraintMatrixType> & J,
                                                const Eigen::MatrixBase<KKTMatrixType> & MJtJ_inv)
  {
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    assert(MJtJ_inv.cols() == data.JMinvJt.cols() + model.nv);
    assert(MJtJ_inv.rows() == data.JMinvJt.rows() + model.nv);
    const typename Data::MatrixXs::Index& nc = data.JMinvJt.cols();

    KKTMatrixType& MJtJ_inv_ = PINOCCHIO_EIGEN_CONST_CAST(KKTMatrixType,MJtJ_inv);
    
    Eigen::Block<typename Data::MatrixXs> topLeft = MJtJ_inv_.topLeftCorner(model.nv, model.nv);
    Eigen::Block<typename Data::MatrixXs> topRight = MJtJ_inv_.topRightCorner(model.nv, nc);
    Eigen::Block<typename Data::MatrixXs> bottomLeft = MJtJ_inv_.bottomLeftCorner(nc, model.nv);
    Eigen::Block<typename Data::MatrixXs> bottomRight = MJtJ_inv_.bottomRightCorner(nc, nc); 

    bottomRight = -Data::MatrixXs::Identity(nc,nc);    topLeft.setIdentity();
    data.llt_JMinvJt.solveInPlace(bottomRight);    cholesky::solve(model, data, topLeft);

    bottomLeft.noalias() = J*topLeft;
    topRight.noalias() = bottomLeft.transpose() * (-bottomRight);
    topLeft.noalias() -= topRight*bottomLeft;
    bottomLeft = topRight.transpose();
  }
  
  ///
  /// \brief Compute the impulse dynamics with contact constraints.
  /// \note It computes the following problem: <BR>
  ///       <CENTER> \f$ \begin{eqnarray} \underset{\dot{q}^{+}}{\min} & & \| \dot{q}^{+} - \dot{q}^{-} \|_{M(q)} \\
  ///           \text{s.t.} & & J (q) \dot{q}^{+} = - \epsilon J (q) \dot{q}^{-}  \end{eqnarray} \f$ </CENTER> <BR>
  ///       where \f$ \dot{q}^{-} \f$ is the generalized velocity before impact,
  ///       \f$ M \f$ is the joint space mass matrix, \f$ J \f$ the constraint Jacobian and \f$ \epsilon \f$ is the coefficient of restitution (1 for a fully elastic impact or 0 for a rigid impact).
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType Type of the joint velocity vector.
  /// \tparam ConstraintMatrixType Type of the constraint matrix.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  /// \param[in] v_before The joint velocity before impact (vector dim model.nv).
  /// \param[in] J The Jacobian of the constraints (dim nb_constraints*model.nv).
  /// \param[in] r_coeff The coefficient of restitution. Must be in [0;1].
  /// \param[in] updateKinematics If true, the algorithm calls first pinocchio::crba. Otherwise, it uses the current mass matrix value stored in data.
  ///
  /// \return A reference to the generalized velocity after impact stored in data.dq_after. The Lagrange Multipliers linked to the contact impulsed are available throw data.impulse_c vector.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename ConstraintMatrixType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  impulseDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType> & v_before,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Scalar r_coeff = 0,
                  const bool updateKinematics = true
                  )
  {
    assert(q.size() == model.nq);
    assert(v_before.size() == model.nv);
    assert(J.cols() == model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typename Data::VectorXs & impulse_c = data.impulse_c;
    typename Data::TangentVectorType & dq_after = data.dq_after;
    
    // Compute the mass matrix
    if (updateKinematics)
      crba(model, data, q);
    
    // Compute the UDUt decomposition of data.M
    cholesky::decompose(model, data);
    
    data.sDUiJt = J.transpose();
    // Compute U^-1 * J.T
    cholesky::Uiv(model, data, data.sDUiJt);
    for(int k=0;k<model.nv;++k) data.sDUiJt.row(k) /= sqrt(data.D[k]);
    
    data.JMinvJt.noalias() = data.sDUiJt.transpose() * data.sDUiJt;
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


#endif // ifndef __pinocchio_contact_dynamics_hpp__
