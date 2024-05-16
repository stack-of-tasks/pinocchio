//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_aba_derivatives_txx__
#define __pinocchio_algorithm_aba_derivatives_txx__

namespace pinocchio
{
  namespace impl
  {

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void computeABADerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<context::MatrixXs>,
      Eigen::Ref<context::MatrixXs>,
      Eigen::Ref<context::MatrixXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void computeABADerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<context::MatrixXs>,
      Eigen::Ref<context::MatrixXs>,
      Eigen::Ref<context::RowMatrixXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::RowMatrixXs>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void computeABADerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<context::MatrixXs>,
      Eigen::Ref<context::MatrixXs>,
      Eigen::Ref<context::MatrixXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const container::aligned_vector<ForceTpl<context::Scalar, context::Options>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void computeABADerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<context::MatrixXs>,
      Eigen::Ref<context::MatrixXs>,
      Eigen::Ref<context::RowMatrixXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const container::aligned_vector<ForceTpl<context::Scalar, context::Options>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::RowMatrixXs>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void computeABADerivatives<
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

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void computeABADerivatives<
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
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const container::aligned_vector<ForceTpl<context::Scalar, context::Options>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void computeABADerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::MatrixXs>,
      Eigen::Ref<context::MatrixXs>,
      Eigen::Ref<context::MatrixXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &);
  } // namespace impl

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  computeABADerivatives<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);

  namespace impl
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void computeABADerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::MatrixXs>,
      Eigen::Ref<context::MatrixXs>,
      Eigen::Ref<context::MatrixXs>>(
      const context::Model &,
      context::Data &,
      const container::aligned_vector<ForceTpl<context::Scalar, context::Options>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &);
  } // namespace impl

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  computeABADerivatives<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &,
    context::Data &,
    const container::aligned_vector<ForceTpl<context::Scalar, context::Options>> &);
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_aba_derivatives_txx__
