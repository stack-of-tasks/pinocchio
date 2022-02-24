#include "pinocchio/algorithm/kinematics.hpp"

#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
namespace pinocchio {
  
  template void updateGlobalPlacements
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const Model &, Data & data);

  template void forwardKinematics
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
  (const Model &, Data &, const Eigen::MatrixBase<context::VectorXs> &);

  template void forwardKinematics
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs>
  (const Model &, Data &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template void forwardKinematics
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs>
    (const Model &, Data &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);

  template MotionTpl<context::Scalar, context::Options> getVelocity
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame);

  template MotionTpl<context::Scalar, context::Options> getAcceleration
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame);

  template MotionTpl<context::Scalar, context::Options> getClassicalAcceleration
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame);
} // namespace pinocchio 
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
