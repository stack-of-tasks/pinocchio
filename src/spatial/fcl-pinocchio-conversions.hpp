//
// Copyright (c) 2015 CNRS
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

#include <Eigen/Geometry>

#include "pinocchio/spatial/se3.hpp"
# include <hpp/fcl/math/transform.h>

namespace se3
{

  inline fcl::Matrix3f toFclMatrix3f(const Eigen::Matrix3d & mat)
  {
    return fcl::Matrix3f( mat(0,0),mat(0,1), mat(0,2),
                          mat(1,0),mat(1,1), mat(1,2),
                          mat(2,0),mat(2,1), mat(2,2));
  }

  inline Eigen::Matrix3d toMatrix3d(const fcl::Matrix3f & mat)
  {
    Eigen::Matrix3d res;

    res <<  mat(0,0),mat(0,1), mat(0,2),
            mat(1,0),mat(1,1), mat(1,2),
            mat(2,0),mat(2,1), mat(2,2);
    return res;
  }

  inline fcl::Vec3f toFclVec3f(const Eigen::Vector3d & vec)
  {
    return fcl::Vec3f( vec(0),vec(1), vec(2));
  }

  inline Eigen::Vector3d toVector3d(const fcl::Vec3f & vec)
  {
    Eigen::Vector3d res;
    res << vec[0],vec[1], vec[2];
    return res;
  }

  inline fcl::Transform3f toFclTransform3f(const se3::SE3 & m)
  {
    return fcl::Transform3f(toFclMatrix3f(m.rotation()), toFclVec3f(m.translation()));
  }

  inline se3::SE3 toPinocchioSE3(const fcl::Transform3f & tf)
  {
    return se3::SE3(toMatrix3d(tf.getRotation()), toVector3d(tf.getTranslation()));
  }

} // namespace se3

#endif // ifndef __se3_fcl_convertion_hpp__

