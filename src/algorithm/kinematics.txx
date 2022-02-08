//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_algorithm_kinematics_txx__
#define __pinocchio_algorithm_kinematics_txx__

namespace pinocchio {
  extern template PINOCCHIO_DLLAPI void updateGlobalPlacements
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const Model &, Data & data);

  extern template PINOCCHIO_DLLAPI void forwardKinematics
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Model::ConfigVectorType>
    (const Model &, Data &, const Eigen::MatrixBase<Model::ConfigVectorType> &);

  extern template PINOCCHIO_DLLAPI void forwardKinematics
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Model::ConfigVectorType, Model::TangentVectorType>
    (const Model &, Data &, const Eigen::MatrixBase<Model::ConfigVectorType> &, const Eigen::MatrixBase<Model::TangentVectorType> &);

  extern template PINOCCHIO_DLLAPI void forwardKinematics
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Model::ConfigVectorType, Model::TangentVectorType, Model::TangentVectorType>
    (const Model &, Data &, const Eigen::MatrixBase<Model::ConfigVectorType> &, const Eigen::MatrixBase<Model::TangentVectorType> &, const Eigen::MatrixBase<Model::TangentVectorType> &);

  extern template PINOCCHIO_DLLAPI MotionTpl<context::Scalar, context::Options> getVelocity
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame);

  extern template PINOCCHIO_DLLAPI MotionTpl<context::Scalar, context::Options> getAcceleration
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame);

  extern template PINOCCHIO_DLLAPI MotionTpl<context::Scalar, context::Options> getClassicalAcceleration
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const Model &, const Data &, const JointIndex, const ReferenceFrame);
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_kinematics_txx__
