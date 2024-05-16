//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_multibody_joint_motion_subspace_hpp__
#define __pinocchio_multibody_joint_motion_subspace_hpp__

#include "pinocchio/macros.hpp"

namespace pinocchio
{

  template<int _Dim, typename _Scalar, int _Options = context::Options>
  struct JointMotionSubspaceTpl;

  typedef JointMotionSubspaceTpl<1, context::Scalar, context::Options> JointMotionSubspace1d;
  typedef JointMotionSubspaceTpl<3, context::Scalar, context::Options> JointMotionSubspace3d;
  typedef JointMotionSubspaceTpl<6, context::Scalar, context::Options> JointMotionSubspace6d;
  typedef JointMotionSubspaceTpl<Eigen::Dynamic, context::Scalar, context::Options>
    JointMotionSubspaceXd;

} // namespace pinocchio

#include "pinocchio/multibody/joint-motion-subspace-base.hpp"
#include "pinocchio/multibody/joint-motion-subspace-generic.hpp"

#endif // ifndef __pinocchio_multibody_joint_motion_subspace_hpp__
