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
    
    template<typename Scalar, int Options>
    struct SE3GroupAction< BiasZeroTpl<Scalar,Options> >
    {
      typedef BiasZeroTpl<Scalar,Options> ReturnType;
    };
    
    template<typename Scalar, int Options, typename MotionDerived>
    struct MotionAlgebraAction< BiasZeroTpl<Scalar,Options>, MotionDerived>
    {
      typedef BiasZeroTpl<Scalar,Options> ReturnType;
    };
  }

  template<typename _Scalar, int _Options>
  struct traits< BiasZeroTpl<_Scalar,_Options> >
  {
    enum {
      Options = _Options,
      LINEAR = 0,
      ANGULAR = 3
    };
    typedef _Scalar Scalar;
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef typename EIGEN_REF_CONSTTYPE(Vector6) ToVectorConstReturnType;
    typedef typename EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef Matrix6 ActionMatrixType;
    typedef Vector3 AngularType;
    typedef const Vector3 ConstAngularType;
    typedef Vector3 LinearType;
    typedef const Vector3 ConstLinearType;
    typedef Motion MotionPlain;
    
  }; // traits BiasZeroTpl
  
  template<typename Scalar, int Options>
  struct BiasZeroTpl : public MotionBase< BiasZeroTpl<Scalar,Options> >
  {
    typedef typename traits<BiasZeroTpl>::MotionPlain MotionPlain;
    
    operator MotionPlain () const { return MotionPlain::Zero(); }
    
    template<typename D2>
    static bool isEqual_impl(const MotionDense<D2> & other)
    { return other.linear().isZero() && other.angular().isZero(); }
    
    template<typename D2>
    static void addTo(const MotionDense<D2> &) {}
    
    template<typename M1>
    BiasZeroTpl motionAction(const MotionBase<M1> &) const
    {
      return BiasZeroTpl();
    }
    
    template<typename S2, int O2, typename D2>
    void se3Action_impl(const SE3Tpl<S2,O2> &, MotionDense<D2> & v) const
    {
      v.setZero();
    }
    
    template<typename S2, int O2>
    BiasZeroTpl se3Action_impl(const SE3Tpl<S2,O2> &) const
    {
      return BiasZeroTpl();
    }
    
    template<typename S2, int O2, typename D2>
    void se3ActionInverse_impl(const SE3Tpl<S2,O2> &, MotionDense<D2> & v) const
    {
      v.setZero();
    }
    
    template<typename S2, int O2>
    BiasZeroTpl se3ActionInverse_impl(const SE3Tpl<S2,O2> &) const
    {
      return BiasZeroTpl();
    }
    
  }; // struct BiasZeroTpl
  
  template<typename M1, typename Scalar, int Options>
  inline const M1 & operator+(const MotionBase<M1> & v,
                              const BiasZeroTpl<Scalar,Options> &)
  { return v.derived(); }
  
  template<typename Scalar, int Options, typename M1>
  inline const M1 & operator+(const BiasZeroTpl<Scalar,Options> &,
                              const MotionBase<M1> & v)
  { return v.derived(); }
  
} // namespace se3

#endif // ifndef __se3_motion_zero_hpp__
