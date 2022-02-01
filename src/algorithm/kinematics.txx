//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_algorithm_kinematics_txx__
#define __pinocchio_algorithm_kinematics_txx__

namespace pinocchio {
  extern template PINOCCHIO_DLLAPI void updateGlobalPlacements
    <double, 0, JointCollectionDefaultTpl>
    (const Model &, Data & data);

  extern template PINOCCHIO_DLLAPI void forwardKinematics
    <double, 0, JointCollectionDefaultTpl, Model::ConfigVectorType>
    (const Model &, Data &, const Eigen::MatrixBase<Model::ConfigVectorType> &);

  extern template PINOCCHIO_DLLAPI void forwardKinematics
    <double, 0, JointCollectionDefaultTpl, Model::ConfigVectorType, Model::TangentVectorType>
    (const Model &, Data &, const Eigen::MatrixBase<Model::ConfigVectorType> &, const Eigen::MatrixBase<Model::TangentVectorType> &);

  extern template PINOCCHIO_DLLAPI void forwardKinematics
    <double, 0, JointCollectionDefaultTpl, Model::ConfigVectorType, Model::TangentVectorType, Model::TangentVectorType>
    (const Model &, Data &, const Eigen::MatrixBase<Model::ConfigVectorType> &, const Eigen::MatrixBase<Model::TangentVectorType> &, const Eigen::MatrixBase<Model::TangentVectorType> &);

  extern template PINOCCHIO_DLLAPI MotionTpl<double, 0> getVelocity
    <double, 0, JointCollectionDefaultTpl>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame);

  extern template PINOCCHIO_DLLAPI MotionTpl<double, 0> getAcceleration
    <double, 0, JointCollectionDefaultTpl>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame);

  extern template PINOCCHIO_DLLAPI MotionTpl<double, 0> getClassicalAcceleration
    <double, 0, JointCollectionDefaultTpl>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame);
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_kinematics_txx__
