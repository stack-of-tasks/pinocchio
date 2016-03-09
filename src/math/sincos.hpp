//
// Copyright (c) 2015-2016 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __math_sincos_hpp__
#define __math_sincos_hpp__

#include <cmath>

# include <Eigen/Geometry>

#ifdef __linux__
  #define SINCOS sincos
#elif __APPLE__
  #define SINCOS __sincos
#else // if sincos specialization does not exist
  #define SINCOS(a,sa,ca) (*sa) = std::sin(a); (*ca) = std::cos(a)
#endif

/// Compute quaternion and angle from a SO(3) joint configuration
///
/// \param q1, q2, robot configurations
/// \param index index of joint configuration in robot configuration vector
/// \param unit quaternion corresponding to both joint configuration
/// \return angle between both joint configuration
template <typename D>
static double angleBetweenQuaternions(const Eigen::MatrixBase<D> & q1,
                                      const Eigen::MatrixBase<D> & q2)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,4);

  double innerprod = q1.dot(q2);
  assert (fabs (innerprod) < 1.0001);
  if (innerprod < -1) innerprod = -1;
  if (innerprod >  1) innerprod =  1;
  double theta = acos (innerprod);
  return theta;
}

#endif //#ifndef __math_sincos_hpp__
