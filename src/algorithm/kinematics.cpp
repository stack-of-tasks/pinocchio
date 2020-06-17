#include "pinocchio/algorithm/kinematics.hpp"

#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
namespace pinocchio {
  template void updateGlobalPlacements
    <double, 0, JointCollectionDefaultTpl>
    (const Model &, Data & data);

  template void forwardKinematics
    <double, 0, JointCollectionDefaultTpl, Model::ConfigVectorType>
    (const Model &, Data &, const Eigen::MatrixBase<Model::ConfigVectorType> &);

  template void forwardKinematics
    <double, 0, JointCollectionDefaultTpl, Model::ConfigVectorType, Model::TangentVectorType>
    (const Model &, Data &, const Eigen::MatrixBase<Model::ConfigVectorType> &, const Eigen::MatrixBase<Model::TangentVectorType> &);

  template void forwardKinematics
    <double, 0, JointCollectionDefaultTpl, Model::ConfigVectorType, Model::TangentVectorType, Model::TangentVectorType>
    (const Model &, Data &, const Eigen::MatrixBase<Model::ConfigVectorType> &, const Eigen::MatrixBase<Model::TangentVectorType> &, const Eigen::MatrixBase<Model::TangentVectorType> &);

  template MotionTpl<double, 0> getVelocity
    <double, 0, JointCollectionDefaultTpl>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame);

  template MotionTpl<double, 0> getAcceleration
    <double, 0, JointCollectionDefaultTpl>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame);

  template MotionTpl<double, 0> getClassicalAcceleration
    <double, 0, JointCollectionDefaultTpl>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame);
} // namespace pinocchio 
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
