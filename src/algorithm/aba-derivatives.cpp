//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/aba-derivatives.hpp"

namespace pinocchio
{
  namespace impl
  {

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeABADerivatives<
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

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeABADerivatives<
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

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeABADerivatives<
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

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeABADerivatives<
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

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeABADerivatives<
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

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeABADerivatives<
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

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeABADerivatives<
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

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  computeABADerivatives<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);

  namespace impl
  {
    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeABADerivatives<
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

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  computeABADerivatives<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &,
    context::Data &,
    const container::aligned_vector<ForceTpl<context::Scalar, context::Options>> &);
} // namespace pinocchio
