//
// Copyright (c) 2016 CNRS
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

#include "pinocchio/math/fwd.hpp"

/// Compute minimal angle between q1 and q2 or between q1 and -q2
///
/// \param q1, q2, unit quaternions
/// \return angle between both quaternions
template <typename D>
D angleBetweenQuaternions(const Eigen::Quaternion< D> & q1,
                          const Eigen::Quaternion< D> & q2)
{
  double innerprod = q1.dot(q2);
  double theta = acos(innerprod);
  if (innerprod < 0)
    theta = PI - theta;
  return theta;
}

template <typename D>
bool quaternion_are_same_rotation(const Eigen::Quaternion< D> & q1,
                                  const Eigen::Quaternion< D> & q2)
{
  return (q1.coeffs() == q2.coeffs() || q1.coeffs() == -q2.coeffs() );
}

#endif //#ifndef __math_quaternion_hpp__
