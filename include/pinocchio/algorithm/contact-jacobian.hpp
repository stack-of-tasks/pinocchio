//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_algorithm_contact_jacobian_hpp__
#define __pinocchio_algorithm_contact_jacobian_hpp__

#include "pinocchio/algorithm/contact-info.hpp"

namespace pinocchio
{

  ///
  /// \brief Computes the kinematic Jacobian associatied to a given constraint model.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] constraint_model Constraint model.
  /// \param[in] constraint_data Constraint data.
  /// \param[out] J A reference on the Jacobian matrix where the results will be stored in (dim 6 x
  /// model.nv). You must fill J with zero elements, e.g. J.fill(0.).
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6Like>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  void getConstraintJacobian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const RigidConstraintModelTpl<Scalar, Options> & constraint_model,
    RigidConstraintDataTpl<Scalar, Options> & constraint_data,
    const Eigen::MatrixBase<Matrix6Like> & J);

  ///
  /// \brief Computes the kinematic Jacobian associatied to a given set of constraint models.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] constraint_models Vector of constraint models.
  /// \param[in] constraint_datas Vector of constraint data.
  /// \param[out] J A reference on the Jacobian matrix where the results will be stored in (dim nc x
  /// model.nv). You must fill J with zero elements, e.g. J.fill(0.).
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename DynamicMatrixLike,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  void getConstraintsJacobian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintDataAllocator> &
      constraint_model,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & constraint_data,
    const Eigen::MatrixBase<DynamicMatrixLike> & J);

} // namespace pinocchio

#include "pinocchio/algorithm/contact-jacobian.hxx"

#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  #include "pinocchio/algorithm/contact-jacobian.txx"
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

#endif // ifndef __pinocchio_algorithm_contact_jacobian_hpp__
