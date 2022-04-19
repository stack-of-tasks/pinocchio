//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/multibody/model.hpp"

namespace pinocchio {
  
  template ModelTpl<context::Scalar,context::Options,JointCollectionDefaultTpl>::ModelTpl();


  template JointIndex ModelTpl<context::Scalar,context::Options,JointCollectionDefaultTpl>::addJoint
  (const JointIndex, const JointModel &, const SE3 &, const std::string &);

  template JointIndex ModelTpl<context::Scalar,context::Options,JointCollectionDefaultTpl>::addJoint
  (const JointIndex, const JointModel &, const SE3 &, const std::string &, const context::VectorXs &,
   const context::VectorXs &, const context::VectorXs &, const context::VectorXs &);   

  template JointIndex ModelTpl<context::Scalar,context::Options,JointCollectionDefaultTpl>::addJoint
  (const JointIndex, const JointModel &, const SE3 &, const std::string &, const context::VectorXs &,
   const context::VectorXs &, const context::VectorXs &, const context::VectorXs &, const context::VectorXs &, const context::VectorXs &);   

  template FrameIndex ModelTpl<context::Scalar,context::Options,JointCollectionDefaultTpl>::addJointFrame
  (const JointIndex &, int);   
  
  template void ModelTpl<context::Scalar,context::Options,JointCollectionDefaultTpl>::appendBodyToJoint
  (const JointIndex, const Inertia &, const SE3 &);   

  template FrameIndex ModelTpl<context::Scalar,context::Options,JointCollectionDefaultTpl>::addBodyFrame
  (const std::string &, const JointIndex &, const SE3 &, int);   
    
  template FrameIndex ModelTpl<context::Scalar,context::Options,JointCollectionDefaultTpl>::addFrame
  (const Frame &, const bool);   

} // namespace pinocchio 
