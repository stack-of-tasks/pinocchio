#include "pinocchio/algorithm/jacobian.hpp"

#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
namespace pinocchio {
  
  template const context::Matrix6xs &
  computeJointJacobians <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const Model &, Data &, const Eigen::MatrixBase<context::VectorXs> &);
  
  template const context::Matrix6xs &
  computeJointJacobians <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const Model &, Data &);
  
  template void getJointJacobian
  <context::Scalar, context::Options, JointCollectionDefaultTpl, context::Matrix6xs>
  (const Model &, const Data &, const JointIndex, const ReferenceFrame, const Eigen::MatrixBase<context::Matrix6xs> &);
  
  template context::Matrix6xs
  getJointJacobian <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const Model &, const Data &, const JointIndex, const ReferenceFrame);
  
  template void computeJointJacobian
  <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::Matrix6xs>
  (const Model &, Data &, const Eigen::MatrixBase<context::VectorXs> &, const JointIndex, const Eigen::MatrixBase<context::Matrix6xs> &);
  
  template const context::Matrix6xs &
  computeJointJacobiansTimeVariation <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const Model &, Data &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);
  
  template void getJointJacobianTimeVariation <context::Scalar, context::Options, JointCollectionDefaultTpl, context::Matrix6xs>
  (const Model &, const Data &, const JointIndex, const ReferenceFrame, const Eigen::MatrixBase<context::Matrix6xs> &);
  
} // namespace pinocchio 
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
