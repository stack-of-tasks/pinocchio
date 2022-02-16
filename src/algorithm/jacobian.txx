//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_jacobian_txx__
#define __pinocchio_algorithm_jacobian_txx__

namespace pinocchio {
  extern template PINOCCHIO_DLLAPI const context::Matrix6xs & computeJointJacobians
  <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const Model &, Data &, const Eigen::MatrixBase<context::VectorXs> &);
  
  extern template PINOCCHIO_DLLAPI const context::Matrix6xs & computeJointJacobians
  <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const Model &, Data &);
  
  extern template PINOCCHIO_DLLAPI void getJointJacobian
  <context::Scalar, context::Options, JointCollectionDefaultTpl, context::Matrix6xs>
  (const Model &, const Data &, const JointIndex, const ReferenceFrame, const Eigen::MatrixBase<context::Matrix6xs> &);
  
  extern template PINOCCHIO_DLLAPI context::Matrix6xs getJointJacobian
  <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const Model &, const Data &, const JointIndex, const ReferenceFrame);
  
  extern template PINOCCHIO_DLLAPI void computeJointJacobian
  <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::Matrix6xs>
  (const Model &, Data &, const Eigen::MatrixBase<context::VectorXs> &, const JointIndex, const Eigen::MatrixBase<context::Matrix6xs> &);
  
  extern template PINOCCHIO_DLLAPI const context::Matrix6xs & computeJointJacobiansTimeVariation
  <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const Model &, Data &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);
  
  extern template PINOCCHIO_DLLAPI void getJointJacobianTimeVariation
  <context::Scalar, context::Options, JointCollectionDefaultTpl, context::Matrix6xs>
  (const Model &, const Data &, const JointIndex, const ReferenceFrame, const Eigen::MatrixBase<context::Matrix6xs> &);
  
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_jacobian_txx__
