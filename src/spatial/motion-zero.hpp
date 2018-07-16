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

namespace se3
{
  
  namespace internal
  {
    
    template<>
    struct SE3GroupAction<BiasZero>
    {
      typedef BiasZero ReturnType;
    };
    
    template<typename MotionDerived>
    struct MotionAlgebraAction<BiasZero, MotionDerived>
    {
      typedef BiasZero ReturnType;
    };
  }

  template<>
  struct traits<BiasZero>
  {
    typedef double Scalar;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,3,3,0> Matrix3;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef EIGEN_REF_CONSTTYPE(Vector6) ToVectorConstReturnType;
    typedef EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef Matrix6 ActionMatrixType;
    typedef Vector3 AngularType;
    typedef const Vector3 ConstAngularType;
    typedef Vector3 LinearType;
    typedef const Vector3 ConstLinearType;
    typedef Motion MotionPlain;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits BiasZero
  
  struct BiasZero : public MotionBase<BiasZero>
  {
    typedef traits<BiasZero>::MotionPlain MotionPlain;
    operator MotionPlain () const { return MotionPlain::Zero(); }
    
    template<typename D2>
    static bool isEqual_impl(const MotionDense<D2> & other)
    { return other.linear().isZero() && other.angular().isZero(); }
    
    template<typename D2>
    static void addTo(const MotionDense<D2> &) {}
    
    template<typename M1>
    BiasZero motionAction(const MotionBase<M1> &) const
    {
      return BiasZero();
    }
    
    template<typename S2, int O2, typename D2>
    void se3Action_impl(const SE3Tpl<S2,O2> &, MotionDense<D2> & v) const
    {
      v.setZero();
    }
    
    template<typename S2, int O2>
    BiasZero se3Action_impl(const SE3Tpl<S2,O2> &) const
    {
      return BiasZero();
    }
    
    template<typename S2, int O2, typename D2>
    void se3ActionInverse_impl(const SE3Tpl<S2,O2> &, MotionDense<D2> & v) const
    {
      v.setZero();
    }
    
    template<typename S2, int O2>
    BiasZero se3ActionInverse_impl(const SE3Tpl<S2,O2> &) const
    {
      return BiasZero();
    }
    
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
