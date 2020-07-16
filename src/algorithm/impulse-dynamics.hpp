//
// Copyright (c) 2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_impulse_dynamics_hpp__
#define __pinocchio_algorithm_impulse_dynamics_hpp__

#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/contact-info.hpp"

#include "pinocchio/algorithm/proximal.hpp"

namespace pinocchio
{
  ///
  /// \brief Init the impulse dynamics data according to the information contained in contact_models.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint torque vector.
  /// \tparam Allocator Allocator class for the std::vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] contact_models Vector of impulse information related to the problem.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, class Allocator>
  inline void
  initImpulseDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      DataTpl<Scalar,Options,JointCollectionTpl> & data,
                      const std::vector<RigidContactModelTpl<Scalar,Options>,Allocator> & contact_models);
  
  ///
  /// \brief Compute the impulse dynamics with contact constraints. Internally, pinocchio::crba is called.
  /// \note It computes the following problem: <BR>
  ///       <CENTER> \f$ \begin{eqnarray} \underset{\dot{q}^{+}}{\min} & & \| \dot{q}^{+} - \dot{q}^{-} \|_{M(q)} \\
  ///           \text{s.t.} & & J (q) \dot{q}^{+} = - \epsilon J (q) \dot{q}^{-}  \end{eqnarray} \f$ </CENTER> <BR>
  ///       where \f$ \dot{q}^{-} \f$ is the generalized velocity before impact,
  ///       \f$ M \f$ is the joint space mass matrix, \f$ J \f$ the constraint Jacobian and \f$ \epsilon \f$ is the coefficient of restitution (1 for a fully elastic impact or 0 for a rigid impact).
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint torque vector.
  /// \tparam Allocator Allocator class for the std::vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (size model.nq).
  /// \param[in] v_before The joint velocity (size model.nv).
  /// \param[in] contact_models Vector of contact information related to the problem.
  /// \param[in] contact_datas Vector of contact datas related to the contact models.
  /// \param[in] mu Damping factor for cholesky decomposition. Set to zero if constraints are full rank.
  /// \param[in] r_coeff coefficient of restitution: must be in [0.,1.]
  ///
  /// \note A hint: a typical value for mu is 1e-12 when two contact constraints are redundant.
  ///
  /// \return A reference to the joint velocities stored in data.dq_after. The Lagrange Multipliers linked to the contact forces are available throw data.impulse_c vector.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2, class ContactModelAllocator, class ContactDataAllocator>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  impulseDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType1> & v_before,
                  const std::vector<RigidContactModelTpl<Scalar,Options>,ContactModelAllocator> & contact_models,
                  std::vector<RigidContactDataTpl<Scalar,Options>,ContactDataAllocator> & contact_datas,
                  const Scalar mu = Scalar(0.),
                  const Scalar r_coeff = Scalar(0.));

} // namespace pinocchio

#include "pinocchio/algorithm/impulse-dynamics.hxx"

#endif // ifndef __pinocchio_algorithm_impulse_dynamics_hpp__
