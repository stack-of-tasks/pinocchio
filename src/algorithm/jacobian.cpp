#include "pinocchio/algorithm/jacobian.hpp"

#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
namespace pinocchio {
  
  template context::Matrix6xs &
    computeJointJacobians <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const Model &, const Data &, const context::VectorXs);

  template context::Matrix6xs &
    computeJointJacobians <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const Model &, const Data &);

  template void getJointJacobian
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Matrix6Like>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame, const Eigen::MatrixBase<Matrix6Like> &);

  template context::Matrix6xs
  getJointJacobian <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame);

  template void computeJointJacobian
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, Matrix6Like>
    (const Model &, const Data &, const context::VectorXs &, const JointIndex, const Eigen::MatrixBase<Matrix6Like> &);

  template context::Matrix6xs &
  computeJointJacobiansTimeVariation <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
    (const Model &, const Data &, const context::VectorXs &, const context::VectorXs &);

  template void computeJointJacobiansTimeVariation <context::Scalar, context::Options, JointCollectionDefaultTpl, Matrix6Like>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame, const Eigen::MatrixBase<Matrix6Like> &);

} // namespace pinocchio 
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
