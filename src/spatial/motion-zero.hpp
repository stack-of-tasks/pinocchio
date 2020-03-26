//
// Copyright (c) 2015-2019 CNRS, INRIA
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_motion_zero_hpp__
#define __pinocchio_motion_zero_hpp__

namespace pinocchio
{
  
  template<typename Scalar, int Options>
  struct SE3GroupAction< MotionZeroTpl<Scalar,Options> >
  {
    typedef MotionZeroTpl<Scalar,Options> ReturnType;
  };
  
  template<typename Scalar, int Options, typename MotionDerived>
  struct MotionAlgebraAction< MotionZeroTpl<Scalar,Options>, MotionDerived>
  {
    typedef MotionZeroTpl<Scalar,Options> ReturnType;
  };

  template<typename _Scalar, int _Options>
  struct traits< MotionZeroTpl<_Scalar,_Options> >
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
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector6) ToVectorConstReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef Matrix6 ActionMatrixType;
    typedef Vector3 AngularType;
    typedef const Vector3 ConstAngularType;
    typedef Vector3 LinearType;
    typedef const Vector3 ConstLinearType;
    typedef Motion MotionPlain;
    typedef MotionPlain PlainReturnType;
    
  }; // traits MotionZeroTpl
  
  template<typename Scalar, int Options>
  struct MotionZeroTpl
  : public MotionBase< MotionZeroTpl<Scalar,Options> >
  {
    typedef typename traits<MotionZeroTpl>::MotionPlain MotionPlain;
    typedef typename traits<MotionZeroTpl>::PlainReturnType PlainReturnType;
    
    static PlainReturnType plain() { return MotionPlain::Zero(); }
    
    template<typename D2>
    static bool isEqual_impl(const MotionDense<D2> & other)
    {
      return other.linear().isZero(0) && other.angular().isZero(0);
    }
    
    static bool isEqual_impl(const MotionZeroTpl &)
    {
      return true;
    }
    
    template<typename D2>
    static void addTo(const MotionBase<D2> &) {}
    
    template<typename D2>
    static void setTo(MotionBase<D2> & other)
    {
      other.setZero();
    }
    
    template<typename M1>
    MotionZeroTpl motionAction(const MotionBase<M1> &) const
    {
      return MotionZeroTpl();
    }
    
    template<typename S2, int O2, typename D2>
    void se3Action_impl(const SE3Tpl<S2,O2> &, MotionDense<D2> & v) const
    {
      v.setZero();
    }
    
    template<typename S2, int O2>
    MotionZeroTpl se3Action_impl(const SE3Tpl<S2,O2> &) const
    {
      return MotionZeroTpl();
    }
    
    template<typename S2, int O2, typename D2>
    void se3ActionInverse_impl(const SE3Tpl<S2,O2> &, MotionDense<D2> & v) const
    {
      v.setZero();
    }
    
    template<typename S2, int O2>
    MotionZeroTpl se3ActionInverse_impl(const SE3Tpl<S2,O2> &) const
    {
      return MotionZeroTpl();
    }
    
  }; // struct MotionZeroTpl
  
  template<typename M1, typename Scalar, int Options>
  inline const M1 & operator+(const MotionBase<M1> & v,
                              const MotionZeroTpl<Scalar,Options> &)
  { return v.derived(); }
  
  template<typename Scalar, int Options, typename M1>
  inline const M1 & operator+(const MotionZeroTpl<Scalar,Options> &,
                              const MotionBase<M1> & v)
  { return v.derived(); }

  /// \brief BiasZeroTpl has been replaced by MotionZeroTpl. Please use this naming instead.
  template<typename Scalar, int Options>
  struct PINOCCHIO_DEPRECATED BiasZeroTpl : MotionZeroTpl<Scalar,Options>
  {
    typedef MotionZeroTpl<Scalar,Options> Base;
    BiasZeroTpl(const Base &) {}
  };

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
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
#pragma GCC diagnostic pop
  
} // namespace pinocchio

#endif // ifndef __pinocchio_motion_zero_hpp__
