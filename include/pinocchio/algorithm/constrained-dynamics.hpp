//
// Copyright (c) 2019-2021 INRIA
//

#ifndef __pinocchio_algorithm_constrained_dynamics_hpp__
#define __pinocchio_algorithm_constrained_dynamics_hpp__

#include "pinocchio/algorithm/contact-info.hpp"

#include "pinocchio/algorithm/proximal.hpp"

namespace pinocchio
{

  ///
  /// \brief Init the forward dynamics data according to the contact information contained in
  /// contact_models.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint torque vector.
  /// \tparam Allocator Allocator class for the std::vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] contact_models Vector of contact information related to the problem.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class Allocator>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  inline void initConstraintDynamics(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, Allocator> & contact_models);

  ///
  /// \brief Computes the forward dynamics with contact constraints according to a given list of
  /// contact information.
  ///
  /// \note  When using forwardDynamics for the first time, you should call first
  ///        initConstraintDynamics to initialize the internal memory used in the algorithm.
  ///
  /// It computes the following problem: \[
  ///       \f[
  ///       \begin{eqnarray}
  ///       \underset{\ddot{q}}{\min} & & \| \ddot{q} - \ddot{q}_{\text{free}} \|_{M(q)} \\
  ///           \text{s.t.} & & J (q) \ddot{q} + \gamma (q, \dot{q}) = 0
  ///       \end{eqnarray}
  ///       \f] where \f$ \ddot{q}_{\text{free}} \f$ is the free acceleration (i.e. without
  ///       constraints), \f$ M \f$ is the mass matrix, \f$ J \f$ the constraint Jacobian and \f$
  ///       \gamma \f$ is the constraint drift.
  ///  By default, the constraint Jacobian is assumed to be full rank, and undamped Cholesky inverse
  ///  is performed.
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
  /// \param[in] v The joint velocity (size model.nv).
  /// \param[in] tau The joint torque vector (size model.nv).
  /// \param[in] contact_models Vector of contact models.
  /// \param[in] contact_datas Vector of contact data.
  /// \param[in] settings Proximal settings (mu, accuracy and maximal number of iterations).
  ///
  /// \note A hint: a typical value for mu is 1e-12 when two contact constraints are redundant.
  ///
  /// \return A reference to the joint acceleration stored in data.ddq. The Lagrange Multipliers
  /// linked to the contact forces are available throw data.lambda_c vector.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  inline const
    typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType & constraintDynamics(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & tau,
      const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
        contact_models,
      std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & contact_datas,
      ProximalSettingsTpl<Scalar> & settings);

  ///
  /// \brief Computes the forward dynamics with contact constraints according to a given list of
  /// Contact information.
  ///
  /// \note  When using forwardDynamics for the first time, you should call first
  ///        initConstraintDynamics to initialize the internal memory used in the algorithm.
  ///
  /// It computes the following problem: \f[
  ///       \begin{eqnarray}
  ///           \underset{\ddot{q}}{\min} & & \| \ddot{q} - \ddot{q}_{\text{free}} \|_{M(q)} \\
  ///           \text{s.t.} & & J (q) \ddot{q} + \gamma (q, \dot{q}) = 0
  ///       \end{eqnarray}
  ///       \f] where \f$ \ddot{q}_{\text{free}} \f$ is the free acceleration (i.e. without
  ///       constraints), \f$ M \f$ is the mass matrix, \f$ J \f$ the constraint Jacobian and \f$
  ///       \gamma \f$ is the constraint drift.
  ///  By default, the constraint Jacobian is assumed to be full rank, and undamped Cholesky inverse
  ///  is performed.
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
  /// \param[in] v The joint velocity (size model.nv).
  /// \param[in] tau The joint torque vector (size model.nv).
  /// \param[in] contact_models Vector of contact models.
  /// \param[in] contact_datas Vector of contact data.
  ///
  /// \return A reference to the joint acceleration stored in data.ddq. The Lagrange Multipliers
  /// linked to the contact forces are available throw data.lambda_c vector.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  inline const
    typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType & constraintDynamics(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & tau,
      const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
        contact_models,
      std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & contact_datas)
  {
    ProximalSettingsTpl<Scalar> settings;
    return constraintDynamics(model, data, q, v, tau, contact_models, contact_datas, settings);
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    class ModelAllocator,
    class DataAllocator>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  inline const
    typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType & contactABA(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & tau,
      const std::vector<RigidConstraintModelTpl<Scalar, Options>, ModelAllocator> & contact_models,
      std::vector<RigidConstraintDataTpl<Scalar, Options>, DataAllocator> & contact_data)
  {
    ProximalSettingsTpl<Scalar> settings = ProximalSettingsTpl<Scalar>();
    return contactABA(
      model, data, q.derived(), v.derived(), tau.derived(), contact_models, contact_data, settings);
  }

} // namespace pinocchio

#include "pinocchio/algorithm/constrained-dynamics.hxx"

#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  #include "pinocchio/algorithm/constrained-dynamics.txx"
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

#endif // ifndef __pinocchio_algorithm_constrained_dynamics_hpp__
