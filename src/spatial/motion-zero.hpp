//
// Copyright (c) 2015-2017 CNRS
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __se3_motion_zero_hpp__
#define __se3_motion_zero_hpp__

#include <Eigen/Core>

namespace se3
{
  struct BiasZero;
  
  template<>
  struct traits< BiasZero >
  {
    typedef double Scalar;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,4,1,0> Vector4;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,3,3,0> Matrix3;
    typedef Eigen::Matrix<double,4,4,0> Matrix4;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef Matrix6 ActionMatrix_t;
    typedef Vector3 Angular_t;
    typedef const Vector3 ConstAngular_t;
    typedef Vector3 Linear_t;
    typedef const Vector3 ConstLinear_t;
    typedef Eigen::Quaternion<double,0> Quaternion_t;
    typedef SE3Tpl<double,0> SE3;
    typedef ForceTpl<double,0> Force;
    typedef MotionTpl<double,0> Motion;
    typedef Symmetric3Tpl<double,0> Symmetric3;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits BiasZero
  
  struct BiasZero : public MotionBase<BiasZero>
  {
    SPATIAL_TYPEDEF_NO_TEMPLATE(BiasZero);
    operator Motion () const { return Motion::Zero(); }
  }; // struct BiasZero
  
  template<typename M1>
  inline const M1 & operator+(const MotionBase<M1> & v,
                              const BiasZero&)
  { return v.derived(); }
  
  template<typename M1>
  inline const M1 & operator+(const BiasZero&,
                              const MotionBase<M1> & v)
  { return v.derived(); }
  
} // namespace se3

#endif // ifndef __se3_motion_zero_hpp__
