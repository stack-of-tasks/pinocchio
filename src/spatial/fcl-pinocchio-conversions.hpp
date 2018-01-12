//
// Copyright (c) 2015-2016 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_fcl_convertion_hpp__
#define __se3_fcl_convertion_hpp__

#include <hpp/fcl/math/transform.h>
#include "pinocchio/spatial/se3.hpp"

namespace se3
{

  // TODO When not supporting the version of hpp-fcl (v0.5) not using plain eigen types
  // the following functions can be removed because
  //  - fcl::Matrix3f = Eigen::Matrix3d
  //  - fcl::Vec3f    = Eigen::Vector3d
  // Currently, they are mandatory to support both v0.5 with FCL_HAVE_EIGEN and
  // future version
  inline fcl::Matrix3f   & toFclMatrix3f (Eigen::Matrix3d & mat) { return mat; }
  inline Eigen::Matrix3d & toMatrix3d    (fcl::Matrix3f   & mat) { return mat; }
  inline fcl::Vec3f      & toFclVec3f    (Eigen::Vector3d & vec) { return vec; }
  inline Eigen::Vector3d & toVector3d    (fcl::Vec3f      & vec) { return vec; }
  inline fcl::Matrix3f   const & toFclMatrix3f (Eigen::Matrix3d const & mat) { return mat; }
  inline Eigen::Matrix3d const & toMatrix3d    (fcl::Matrix3f   const & mat) { return mat; }
  inline fcl::Vec3f      const & toFclVec3f    (Eigen::Vector3d const & vec) { return vec; }
  inline Eigen::Vector3d const & toVector3d    (fcl::Vec3f      const & vec) { return vec; }

  inline fcl::Transform3f toFclTransform3f(const SE3 & m)
  {
    return fcl::Transform3f(toFclMatrix3f(m.rotation()), toFclVec3f(m.translation()));
  }

  inline SE3 toPinocchioSE3(const fcl::Transform3f & tf)
  {
    return SE3(toMatrix3d(tf.getRotation()), toVector3d(tf.getTranslation()));
  }

} // namespace se3

#endif // ifndef __se3_fcl_convertion_hpp__
