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

#ifndef __math_quaternion_hpp__
#define __math_quaternion_hpp__

#include <cmath>

# include <Eigen/Geometry>


/// Compute angle between two quaternions
///
/// \param q1, q2, unit quaternions
/// \return angle between both quaternions
template <typename D>
typename D::Scalar angleBetweenQuaternions(const Eigen::MatrixBase<D> & q1,
                                            const Eigen::MatrixBase<D> & q2)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,4);

  Eigen::Quaternion<typename D::Scalar> p1 (q1);
  Eigen::Quaternion<typename D::Scalar> p2 (q2);

  Eigen::Quaternion<typename D::Scalar> p (p1*p2.conjugate());
  Eigen::AngleAxis<typename D::Scalar> angle_axis(p);

  return (angle_axis.angle() * angle_axis.axis()).norm();
}

#endif //#ifndef __math_quaternion_hpp__
