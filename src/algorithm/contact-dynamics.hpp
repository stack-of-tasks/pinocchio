//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_contact_dynamics_hpp__
#define __pinocchio_contact_dynamics_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{

  ///
  /// \brief Compute the forward dynamics with contact constraints. Internally, pinocchio::computeAllTerms is called.
  /// \note It computes the following problem: <BR>
  ///       <CENTER> \f$ \begin{eqnarray} \underset{\ddot{q}}{\min} & & \| \ddot{q} - \ddot{q}_{\text{free}} \|_{M(q)} \\ %
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
  /// \param[in] inv_damping Damping factor for Cholesky decomposition of JMinvJt. Set to zero if constraints are full rank.
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
                  const Scalar inv_damping = 0.);

  ///
  /// \brief Compute the forward dynamics with contact constraints, assuming pinocchio::computeAllTerms has been called.
  /// \note It computes the following problem: <BR>
  ///       <CENTER> \f$ \begin{eqnarray} \underset{\ddot{q}}{\min} & & \| \ddot{q} - \ddot{q}_{\text{free}} \|_{M(q)} \\ %
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
  /// \param[in] v The joint velocity (vector dim model.nv).
  /// \param[in] tau The joint torque vector (dim model.nv).
  /// \param[in] J The Jacobian of the constraints (dim nb_constraints*model.nv).
  /// \param[in] gamma The drift of the constraints (dim nb_constraints).
  /// \param[in] inv_damping Damping factor for Cholesky decomposition of JMinvJt. Set to zero if constraints are full rank.
  ///            \note A hint: 1e-12 as the damping factor gave good result in the particular case of redundancy in contact constraints on the two feet.
  ///
  /// \return A reference to the joint acceleration stored in data.ddq. The Lagrange Multipliers linked to the contact forces are available throw data.lambda_c vector.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename TangentVectorType,
  typename ConstraintMatrixType, typename DriftVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  forwardDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<TangentVectorType> & tau,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Eigen::MatrixBase<DriftVectorType> & gamma,
                  const Scalar inv_damping = 0.);

  ///
  /// \brief Compute the forward dynamics with contact constraints.
  ///
  /// \deprecated This function signature has been deprecated and will be removed in future releases of Pinocchio.
  ///             Please change for the new signature of forwardDynamics(model,data[,q],v,tau,J,gamma[,inv_damping]).
  ///
  /// \note It computes the following problem: <BR>
  ///       <CENTER> \f$ \begin{eqnarray} \underset{\ddot{q}}{\min} & & \| \ddot{q} - \ddot{q}_{\text{free}} \|_{M(q)} \\ %
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
  /// \param[in] inv_damping Damping factor for Cholesky decomposition of JMinvJt. Set to zero if constraints are full rank.
  /// \param[in] updateKinematics If true, the algorithm calls first pinocchio::computeAllTerms. Otherwise, it uses the current dynamic values stored in data. \\ %
  ///            \note A hint: 1e-12 as the damping factor gave good result in the particular case of redundancy in contact constraints on the two feet.
  ///
  /// \return A reference to the joint acceleration stored in data.ddq. The Lagrange Multipliers linked to the contact forces are available throw data.lambda_c vector.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2,
  typename ConstraintMatrixType, typename DriftVectorType>
  PINOCCHIO_DEPRECATED
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  forwardDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType1> & v,
                  const Eigen::MatrixBase<TangentVectorType2> & tau,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Eigen::MatrixBase<DriftVectorType> & gamma,
                  const Scalar inv_damping,
                  const bool updateKinematics)
  {
    if(updateKinematics)
      return forwardDynamics(model,data,q,v,tau,J,gamma,inv_damping);
    else
      return forwardDynamics(model,data,tau,J,gamma,inv_damping);
  }

  
  ///
  /// \brief Computes the inverse of the KKT matrix for dynamics with contact constraints, [[M JT], [J 0]].
  /// The matrix is defined when we call forwardDynamics/impulseDynamics. This method makes use of
  /// the matrix decompositions performed during the forwardDynamics/impulseDynamics and returns the inverse.
  /// The jacobian should be the same that was provided to forwardDynamics/impulseDynamics.
  /// Thus you should call forward Dynamics/impulseDynamics first.
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[out] MJtJ_inv inverse of the MJtJ matrix.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
           typename ConstraintMatrixType, typename KKTMatrixType>
  inline void getKKTContactDynamicMatrixInverse(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                const Eigen::MatrixBase<ConstraintMatrixType> & J,
                                                const Eigen::MatrixBase<KKTMatrixType> & MJtJ_inv);
  
  ///
  /// \brief Compute the impulse dynamics with contact constraints. Internally, pinocchio::crba is called.
  /// \note It computes the following problem: <BR>
  ///       <CENTER> \f$ \begin{eqnarray} \underset{\dot{q}^{+}}{\min} & & \| \dot{q}^{+} - \dot{q}^{-} \|_{M(q)} \\ %
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
  /// \param[in] inv_damping Damping factor for Cholesky decomposition of JMinvJt. Set to zero if constraints are full rank.
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
                  const Scalar r_coeff = 0.,
                  const Scalar inv_damping = 0.);

  ///
  /// \brief Compute the impulse dynamics with contact constraints, assuming pinocchio::crba has been called.
  /// \note It computes the following problem: <BR>
  ///       <CENTER> \f$ \begin{eqnarray} \underset{\dot{q}^{+}}{\min} & & \| \dot{q}^{+} - \dot{q}^{-} \|_{M(q)} \\ %
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
  /// \param[in] v_before The joint velocity before impact (vector dim model.nv).
  /// \param[in] J The Jacobian of the constraints (dim nb_constraints*model.nv).
  /// \param[in] r_coeff The coefficient of restitution. Must be in [0;1].
  /// \param[in] inv_damping Damping factor for Cholesky decomposition of JMinvJt. Set to zero if constraints are full rank.
  ///
  /// \return A reference to the generalized velocity after impact stored in data.dq_after. The Lagrange Multipliers linked to the contact impulsed are available throw data.impulse_c vector.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename ConstraintMatrixType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  impulseDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<TangentVectorType> & v_before,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Scalar r_coeff = 0.,
                  const Scalar inv_damping = 0.);
  
  ///
  /// \brief Compute the impulse dynamics with contact constraints.
  ///
  /// \deprecated This function signature has been deprecated and will be removed in future releases of Pinocchio.
  ///             Please change for the new signature of impulseDynamics(model,data[,q],v_before,J[,r_coeff[,inv_damping]]).
  ///
  /// \note It computes the following problem: <BR>
  ///       <CENTER> \f$ \begin{eqnarray} \underset{\dot{q}^{+}}{\min} & & \| \dot{q}^{+} - \dot{q}^{-} \|_{M(q)} \\ %
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
  PINOCCHIO_DEPRECATED
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  impulseDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType> & v_before,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Scalar r_coeff,
                  const bool updateKinematics)
  {
    if(updateKinematics)
      return impulseDynamics(model,data,q,v_before,J,r_coeff,Scalar(0));
    else
      return impulseDynamics(model,data,v_before,J,r_coeff,Scalar(0));
  }

} // namespace pinocchio

#include "pinocchio/algorithm/contact-dynamics.hxx"

#endif // ifndef __pinocchio_contact_dynamics_hpp__
