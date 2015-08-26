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

#ifndef __se3_fwd_hpp__
#define __se3_fwd_hpp__

#include <Eigen/Core>

namespace se3
{
  template<typename _Scalar, int _Options=0> class SE3Tpl;
  template<typename _Scalar, int _Options=0> class MotionTpl;
  template<typename _Scalar, int _Options=0> class ForceTpl;
  template<typename _Scalar, int _Options=0> class InertiaTpl;
  template<typename _Scalar, int _Options=0> class Symmetric3Tpl;

  template<class C> struct traits {};

  #define SPATIAL_TYPEDEF_TEMPLATE(derived)              \
    typedef typename traits<derived>::Scalar_t Scalar_t; \
    typedef typename traits<derived>::Vector3 Vector3; \
    typedef typename traits<derived>::Vector4 Vector4; \
    typedef typename traits<derived>::Vector6 Vector6; \
    typedef typename traits<derived>::Matrix3 Matrix3; \
    typedef typename traits<derived>::Matrix4 Matrix4; \
    typedef typename traits<derived>::Matrix6 Matrix6; \
    typedef typename traits<derived>::Angular_t Angular_t; \
    typedef typename traits<derived>::Linear_t Linear_t; \
    typedef typename traits<derived>::ActionMatrix_t ActionMatrix_t; \
    typedef typename traits<derived>::Quaternion_t Quaternion_t; \
    typedef typename traits<derived>::SE3 SE3; \
    typedef typename traits<derived>::Force Force; \
    typedef typename traits<derived>::Motion Motion; \
    typedef typename traits<derived>::Symmetric3 Symmetric3; \
    enum {  \
      LINEAR = traits<derived>::LINEAR,  \
      ANGULAR = traits<derived>::ANGULAR   \
    }

  #define SPATIAL_TYPEDEF_NO_TEMPLATE(derived)              \
    typedef traits<derived>::Scalar_t Scalar_t; \
    typedef traits<derived>::Vector3 Vector3; \
    typedef traits<derived>::Vector4 Vector4; \
    typedef traits<derived>::Vector6 Vector6; \
    typedef traits<derived>::Matrix3 Matrix3; \
    typedef traits<derived>::Matrix4 Matrix4; \
    typedef traits<derived>::Matrix6 Matrix6; \
    typedef traits<derived>::Angular_t Angular_t; \
    typedef traits<derived>::Linear_t Linear_t; \
    typedef traits<derived>::ActionMatrix_t ActionMatrix_t; \
    typedef traits<derived>::Quaternion_t Quaternion_t; \
    typedef traits<derived>::SE3 SE3; \
    typedef traits<derived>::Force Force; \
    typedef traits<derived>::Motion Motion; \
    typedef traits<derived>::Symmetric3 Symmetric3; \
    enum {  \
      LINEAR = traits<derived>::LINEAR,  \
      ANGULAR = traits<derived>::ANGULAR   \
    }


} // namespace se3

#endif // ifndef __se3_fwd_hpp__
