//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_cholesky_txx__
#define __pinocchio_algorithm_cholesky_txx__

#ifndef PINOCCHIO_SKIP_ALGORITHM_CHOLESKY

namespace pinocchio
{
  namespace cholesky
  {

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::MatrixXs &
    decompose<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, context::Data &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::MatrixXs &
    solve<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::MatrixXs
    Mv<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::MatrixXs & Mv<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      context::MatrixXs,
      context::MatrixXs>(
      const context::Model &,
      const context::Data &,
      const Eigen::MatrixBase<context::MatrixXs> &,
      const Eigen::MatrixBase<context::MatrixXs> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::MatrixXs &
    UDUtv<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::MatrixXs &
    Uv<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::MatrixXs &
    Utv<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::MatrixXs &
    Uiv<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::MatrixXs &
    Utiv<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::MatrixXs &
    computeMinv<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

  } // namespace cholesky
} // namespace pinocchio

#endif // PINOCCHIO_SKIP_ALGORITHM_CHOLESKY

#endif // ifndef __pinocchio_algorithm_cholesky_txx__
