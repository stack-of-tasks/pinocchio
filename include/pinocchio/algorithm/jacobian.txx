//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_jacobian_txx__
#define __pinocchio_algorithm_jacobian_txx__

namespace pinocchio
{
  namespace impl
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::Matrix6xs &
    computeJointJacobians<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);
  } // namespace impl
  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::Matrix6xs &
  computeJointJacobians<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);
  namespace impl
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void getJointJacobian<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::Matrix6xs>>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const ReferenceFrame,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &);
  } // namespace impl
  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::Matrix6xs
  getJointJacobian<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame);
  namespace impl
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void computeJointJacobian<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<context::Matrix6xs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const JointIndex,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::Matrix6xs &
    computeJointJacobiansTimeVariation<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
    getJointJacobianTimeVariation<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::Matrix6xs>>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const ReferenceFrame,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &);
  } // namespace impl
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_jacobian_txx__
