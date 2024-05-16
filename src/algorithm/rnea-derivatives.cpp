//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/rnea-derivatives.hpp"

namespace pinocchio
{
  namespace impl
  {
    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
    computeGeneralizedGravityDerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<context::MatrixXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeStaticTorqueDerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<context::MatrixXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const container::aligned_vector<context::Force> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeRNEADerivatives<
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

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeRNEADerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<context::RowMatrixXs>,
      Eigen::Ref<context::RowMatrixXs>,
      Eigen::Ref<context::MatrixXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::RowMatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::RowMatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeRNEADerivatives<
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
      const container::aligned_vector<context::Force> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeRNEADerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<context::RowMatrixXs>,
      Eigen::Ref<context::RowMatrixXs>,
      Eigen::Ref<context::MatrixXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const container::aligned_vector<context::Force> &,
      const Eigen::MatrixBase<Eigen::Ref<context::RowMatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::RowMatrixXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::MatrixXs>> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeRNEADerivatives<
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

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeRNEADerivatives<
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
      const container::aligned_vector<context::Force> &);
  } // namespace impl
} // namespace pinocchio
