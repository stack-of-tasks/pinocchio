//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_center_of_mass_txx__
#define __pinocchio_algorithm_center_of_mass_txx__

namespace pinocchio
{

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::Scalar
  computeTotalMass<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::Scalar
  computeTotalMass<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  computeSubtreeMasses<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);
  namespace impl
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::Vector3 &
    centerOfMass<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const bool computeSubtreeComs);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::Vector3 &
    centerOfMass<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const bool computeSubtreeComs);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::Vector3 &
    centerOfMass<
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
      const bool computeSubtreeComs);
  } // namespace impl
  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::Vector3 &
  centerOfMass<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &, KinematicLevel, const bool computeSubtreeComs);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::Vector3 &
  centerOfMass<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &, const bool computeSubtreeComs);
  namespace impl
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::Matrix3x &
    jacobianCenterOfMass<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const bool computeSubtreeComs);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
    jacobianSubtreeCenterOfMass<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<context::Matrix3x>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const JointIndex &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
    jacobianSubtreeCenterOfMass<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::Matrix3x>>(
      const context::Model &,
      context::Data &,
      const JointIndex &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
    getJacobianSubtreeCenterOfMass<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::Matrix3x>>(
      const context::Model &,
      const context::Data &,
      const JointIndex &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &);
  } // namespace impl
  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::Vector3 &
  getComFromCrba<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::Matrix3x &
  getJacobianComFromCrba<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_center_of_mass_txx__
