//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_energy_txx__
#define __pinocchio_algorithm_energy_txx__

namespace pinocchio
{

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::Scalar
  computeKineticEnergy<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);
  namespace impl
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::Scalar
    computeKineticEnergy<
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
  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::Scalar
  computePotentialEnergy<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);
  namespace impl
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::Scalar
    computePotentialEnergy<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);
  } // namespace impl
  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::Scalar
  computeMechanicalEnergy<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);
  namespace impl
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::Scalar
    computeMechanicalEnergy<
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
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_energy_txx__
