//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/spatial/fwd.hpp"

#ifndef PINOCCHIO_SKIP_ALGORITHM_CHOLESKY

  #include "pinocchio/algorithm/cholesky.hpp"

namespace pinocchio
{
  namespace cholesky
  {

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::MatrixXs &
    decompose<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, context::Data &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::MatrixXs &
    solve<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::MatrixXs
    Mv<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::MatrixXs & Mv<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      context::MatrixXs,
      context::MatrixXs>(
      const context::Model &,
      const context::Data &,
      const Eigen::MatrixBase<context::MatrixXs> &,
      const Eigen::MatrixBase<context::MatrixXs> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::MatrixXs &
    UDUtv<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::MatrixXs &
    Uv<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::MatrixXs &
    Utv<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::MatrixXs &
    Uiv<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::MatrixXs &
    Utiv<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::MatrixXs &
    computeMinv<context::Scalar, context::Options, JointCollectionDefaultTpl, context::MatrixXs>(
      const context::Model &, const context::Data &, const Eigen::MatrixBase<context::MatrixXs> &);
  } // namespace cholesky
} // namespace pinocchio

#endif // PINOCCHIO_SKIP_ALGORITHM_CHOLESKY
