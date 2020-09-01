//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_multibody_joint_motion_subspace_hpp__
#define __pinocchio_multibody_joint_motion_subspace_hpp__

#include "pinocchio/macros.hpp"

namespace pinocchio
{
  
  template<int _Dim, typename _Scalar, int _Options=0> struct JointMotionSubspaceTpl;
  
  typedef JointMotionSubspaceTpl<1,double,0> JointMotionSubspace1d;
  typedef JointMotionSubspaceTpl<3,double,0> JointMotionSubspace3d;
  typedef JointMotionSubspaceTpl<6,double,0> JointMotionSubspace6d;
  typedef JointMotionSubspaceTpl<Eigen::Dynamic,double,0> JointMotionSubspaceXd;
  
} // namespace pinocchio

#include "pinocchio/multibody/joint-motion-subspace-base.hpp"
#include "pinocchio/multibody/joint-motion-subspace-generic.hpp"

#endif // ifndef __pinocchio_multibody_joint_motion_subspace_hpp__
