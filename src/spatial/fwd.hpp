//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#include "pinocchio/macros.hpp"

namespace se3
{
  
  template<typename _Scalar, int _Options=0> class SE3Tpl;

  template<typename Derived> class MotionBase;
  template<typename Derived> class MotionDense;
  template<typename Vector6ArgType> class MotionRef;
  template<typename _Scalar, int _Options=0> class MotionTpl;
  struct BiasZero;
  
  template<typename Derived> class ForceBase;
  template<typename Derived> class ForceDense;
  template<typename Vector6ArgType> class ForceRef;
  template<typename _Scalar, int _Options=0> class ForceTpl;
  
  template<typename _Scalar, int _Options=0> class InertiaTpl;
  template<typename _Scalar, int _Options=0> class Symmetric3Tpl;

  typedef SE3Tpl        <double,0> SE3;
  typedef MotionTpl     <double,0> Motion;
  typedef ForceTpl      <double,0> Force;
  typedef InertiaTpl    <double,0> Inertia;
  typedef Symmetric3Tpl <double,0> Symmetric3;

  template<class C> struct traits {};

  #define SPATIAL_TYPEDEF_TEMPLATE_GENERIC(derived,TYPENAME)              \
    typedef TYPENAME traits<derived>::Scalar Scalar; \
    typedef TYPENAME traits<derived>::Vector3 Vector3; \
    typedef TYPENAME traits<derived>::Vector4 Vector4; \
    typedef TYPENAME traits<derived>::Vector6 Vector6; \
    typedef TYPENAME traits<derived>::Matrix3 Matrix3; \
    typedef TYPENAME traits<derived>::Matrix4 Matrix4; \
    typedef TYPENAME traits<derived>::Matrix6 Matrix6; \
    typedef TYPENAME traits<derived>::Angular_t Angular_t; \
    typedef TYPENAME traits<derived>::Linear_t Linear_t; \
    typedef TYPENAME traits<derived>::ConstAngular_t ConstAngular_t; \
    typedef TYPENAME traits<derived>::ConstLinear_t ConstLinear_t; \
    typedef TYPENAME traits<derived>::ActionMatrix_t ActionMatrix_t; \
    typedef TYPENAME traits<derived>::Quaternion_t Quaternion_t; \
    typedef TYPENAME traits<derived>::SE3 SE3; \
    typedef TYPENAME traits<derived>::Force Force; \
    typedef TYPENAME traits<derived>::Motion Motion; \
    typedef TYPENAME traits<derived>::Symmetric3 Symmetric3; \
    enum {  \
      LINEAR = traits<derived>::LINEAR,  \
      ANGULAR = traits<derived>::ANGULAR   \
    }

  #define SPATIAL_TYPEDEF_TEMPLATE(derived)                 \
    SPATIAL_TYPEDEF_TEMPLATE_GENERIC(derived,typename)
  
  #define SPATIAL_TYPEDEF_NO_TEMPLATE(derived)              \
    SPATIAL_TYPEDEF_TEMPLATE_GENERIC(derived,PINOCCHIO_MACRO_EMPTY_ARG)


} // namespace se3

#endif // ifndef __se3_fwd_hpp__
