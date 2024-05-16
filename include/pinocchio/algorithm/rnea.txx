//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_rnea_txx__
#define __pinocchio_algorithm_rnea_txx__

namespace pinocchio
{
  namespace impl
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::VectorXs &
    rnea<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::VectorXs &
    rnea<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      context::Force>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const container::aligned_vector<context::Force> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::VectorXs &
    nonLinearEffects<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::VectorXs &
    computeGeneralizedGravity<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::VectorXs &
    computeStaticTorque<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const container::aligned_vector<context::Force> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::MatrixXs &
    computeCoriolisMatrix<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);
  } // namespace impl
  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::MatrixXs &
  getCoriolisMatrix<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_rnea_txx__
