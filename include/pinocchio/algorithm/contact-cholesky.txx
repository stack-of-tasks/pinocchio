//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_contact_cholesky_txx__
#define __pinocchio_algorithm_contact_cholesky_txx__

#ifndef PINOCCHIO_SKIP_ALGORITHM_CONTACT_CHOLESKY

namespace pinocchio
{
  // TODO Remove when API is stabilized
  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  namespace details
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::VectorXs &
    inverseAlgo<context::Scalar, context::Options, context::VectorXs>(
      const ContactCholeskyDecompositionTpl<context::Scalar, context::Options> &,
      const Eigen::DenseIndex,
      const Eigen::MatrixBase<context::VectorXs> &);
  }

  extern template struct PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    ContactCholeskyDecompositionTpl<context::Scalar, context::Options>;

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::allocate<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    typename context::RigidConstraintModelVector::allocator_type>(
    const context::Model &, const context::RigidConstraintModelVector &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::
    getInverseOperationalSpaceInertiaMatrix<context::MatrixXs>(
      const Eigen::MatrixBase<context::MatrixXs> &) const;

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::
    getOperationalSpaceInertiaMatrix<context::MatrixXs>(
      const Eigen::MatrixBase<context::MatrixXs> &) const;

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::getInverseMassMatrix<
    context::MatrixXs>(const Eigen::MatrixBase<context::MatrixXs> &) const;

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::compute<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    typename context::RigidConstraintModelVector::allocator_type,
    typename context::RigidConstraintDataVector::allocator_type>(
    const context::Model &,
    context::Data &,
    const context::RigidConstraintModelVector &,
    context::RigidConstraintDataVector &,
    const context::Scalar);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::solveInPlace<
    context::MatrixXs>(const Eigen::MatrixBase<context::MatrixXs> &) const;

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::Matrix
    ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::solve<context::MatrixXs>(
      const Eigen::MatrixBase<
        ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::Matrix> &) const;

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    ContactCholeskyDecompositionTpl<context::Scalar, context::Options>
    ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::
      getMassMatrixChoeslkyDecomposition<
        context::Scalar,
        context::Options,
        JointCollectionDefaultTpl>(const context::Model &) const;

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::Uv<context::MatrixXs>(
    const Eigen::MatrixBase<context::MatrixXs> &) const;

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::Utv<context::MatrixXs>(
    const Eigen::MatrixBase<context::MatrixXs> &) const;

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::Uiv<context::MatrixXs>(
    const Eigen::MatrixBase<context::MatrixXs> &) const;

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::Utiv<context::MatrixXs>(
    const Eigen::MatrixBase<context::MatrixXs> &) const;

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::matrix<context::MatrixXs>(
    const Eigen::MatrixBase<context::MatrixXs> &) const;

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::inverse<context::MatrixXs>(
    const Eigen::MatrixBase<context::MatrixXs> &) const;

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI bool
    ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::
    operator== <context::Scalar, context::Options>(
      const ContactCholeskyDecompositionTpl<context::Scalar, context::Options> &) const;

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI bool
    ContactCholeskyDecompositionTpl<context::Scalar, context::Options>::
    operator!= <context::Scalar, context::Options>(
      const ContactCholeskyDecompositionTpl<context::Scalar, context::Options> &) const;

  PINOCCHIO_COMPILER_DIAGNOSTIC_POP

} // namespace pinocchio

#endif // PINOCCHIO_SKIP_ALGORITHM_CONTACT_CHOLESKY

#endif // ifndef __pinocchio_algorithm_contact_cholesky_txx__
