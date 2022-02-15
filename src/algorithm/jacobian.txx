//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_jacobian_txx__
#define __pinocchio_algorithm_jacobian_txx__

namespace pinocchio {
  extern template PINOCCHIO_DLLAPI DataTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::Matrix6x & computeJointJacobians
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const Model &, const Data &, const context::VectorXs);

  extern template PINOCCHIO_DLLAPI DataTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::Matrix6x & computeJointJacobians
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const Model &, const Data &);

  extern template PINOCCHIO_DLLAPI void getJointJacobian
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Matrix6Like>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame, const Eigen::MatrixBase<Matrix6Like> &);

  extern template PINOCCHIO_DLLAPI Eigen::Matrix<context::Scalar, 6, Eigen::Dynamic, context::Options> getJointJacobian
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame);

  extern template PINOCCHIO_DLLAPI void computeJointJacobian
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, Matrix6Like>
    (const Model &, const Data &, const context::VectorXs &, const JointIndex, const Eigen::MatrixBase<Matrix6Like> &);

  extern template PINOCCHIO_DLLAPI DataTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::Matrix6x & computeJointJacobiansTimeVariation
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
    (const Model &, const Data &, const context::VectorXs &, const context::VectorXs &);

  extern template PINOCCHIO_DLLAPI void computeJointJacobiansTimeVariation
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Matrix6Like>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame, const Eigen::MatrixBase<Matrix6Like> &);

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_jacobian_txx__
