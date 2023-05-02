//
// Copyright (c) 2017-2019 CNRS INRIA
//

#ifndef __pinocchio_motion_dense_hpp__
#define __pinocchio_motion_dense_hpp__

#include "pinocchio/spatial/skew.hpp"

namespace pinocchio
{
  
  template<typename Derived>
  struct SE3GroupAction< MotionDense<Derived> >
  {
    typedef typename SE3GroupAction< Derived >::ReturnType ReturnType;
  };
  
  template<typename Derived, typename MotionDerived>
  struct MotionAlgebraAction< MotionDense<Derived>, MotionDerived >
  {
    typedef typename MotionAlgebraAction< Derived, MotionDerived >::ReturnType ReturnType;
  };

  template<typename Derived>
  class MotionDense : public MotionBase<Derived>
  {
  public:
    typedef MotionBase<Derived> Base;
    MOTION_TYPEDEF_TPL(Derived);
    typedef typename traits<Derived>::MotionRefType MotionRefType;

    using Base::linear;
    using Base::angular;
    using Base::derived;
    using Base::isApprox;
    using Base::isZero;

    Derived & setZero() { linear().setZero(); angular().setZero(); return derived(); }
    Derived & setRandom() { linear().setRandom(); angular().setRandom(); return derived(); }
    
    ActionMatrixType toActionMatrix_impl() const
    {
      ActionMatrixType X;
      X.template block <3,3> (ANGULAR, ANGULAR) = X.template block <3,3> (LINEAR, LINEAR) = skew(angular());
      X.template block <3,3> (LINEAR, ANGULAR) = skew(linear());
      X.template block <3,3> (ANGULAR, LINEAR).setZero();
      
      return X;
    }
    
    ActionMatrixType toDualActionMatrix_impl() const
    {
      ActionMatrixType X;
      X.template block <3,3> (ANGULAR, ANGULAR) = X.template block <3,3> (LINEAR, LINEAR) = skew(angular());
      X.template block <3,3> (ANGULAR, LINEAR) = skew(linear());
      X.template block <3,3> (LINEAR, ANGULAR).setZero();
      
      return X;
    }

    HomogeneousMatrixType toHomogeneousMatrix_impl() const
    {
      HomogeneousMatrixType M;
      M.template block<3,3>(0, 0) = skew(angular());
      M.template block<3,1>(0, 3) = linear();
      M.template block<1,4>(3, 0).setZero();
      return M;
    }
    
    template<typename D2>
    bool isEqual_impl(const MotionDense<D2> & other) const
    { return linear() == other.linear() && angular() == other.angular(); }
    
    template<typename D2>
    bool isEqual_impl(const MotionBase<D2> & other) const
    { return other.derived() == derived(); }
    
    // Arithmetic operators
    template<typename D2>
    Derived & operator=(const MotionDense<D2> & other)
    {
      linear() = other.linear();
      angular() = other.angular();
      return derived();
    }
    
    template<typename D2>
    Derived & operator=(const MotionBase<D2> & other)
    {
      other.derived().setTo(derived());
      return derived();
    }
    
    template<typename V6>
    Derived & operator=(const Eigen::MatrixBase<V6> & v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(V6); assert(v.size() == 6);
      linear() = v.template segment<3>(LINEAR);
      angular() = v.template segment<3>(ANGULAR);
      return derived();
    }
    
    MotionPlain operator-() const { return derived().__opposite__(); }
    template<typename M1>
    MotionPlain operator+(const MotionDense<M1> & v) const { return derived().__plus__(v.derived()); }
    template<typename M1>
    MotionPlain operator-(const MotionDense<M1> & v) const { return derived().__minus__(v.derived()); }
    
    template<typename M1>
    Derived & operator+=(const MotionDense<M1> & v) { return derived().__pequ__(v.derived()); }
    template<typename M1>
    Derived & operator+=(const MotionBase<M1> & v)
    { v.derived().addTo(derived()); return derived(); }
    
    template<typename M1>
    Derived & operator-=(const MotionDense<M1> & v) { return derived().__mequ__(v.derived()); }

    MotionPlain __opposite__() const { return MotionPlain(-linear(),-angular()); }
    
    template<typename M1>
    MotionPlain __plus__(const MotionDense<M1> & v) const
    { return MotionPlain(linear()+v.linear(), angular()+v.angular()); }
    
    template<typename M1>
    MotionPlain __minus__(const MotionDense<M1> & v) const
    { return MotionPlain(linear()-v.linear(), angular()-v.angular()); }
    
    template<typename M1>
    Derived & __pequ__(const MotionDense<M1> & v)
    { linear() += v.linear(); angular() += v.angular(); return derived(); }
    
    template<typename M1>
    Derived & __mequ__(const MotionDense<M1> & v)
    { linear() -= v.linear(); angular() -= v.angular(); return derived(); }
    
    template<typename OtherScalar>
    MotionPlain __mult__(const OtherScalar & alpha) const
    { return MotionPlain(alpha*linear(),alpha*angular()); }
    
    template<typename OtherScalar>
    MotionPlain __div__(const OtherScalar & alpha) const
    { return derived().__mult__((OtherScalar)(1)/alpha); }
    
    template<typename F1>
    Scalar dot(const ForceBase<F1> & phi) const
    { return phi.linear().dot(linear()) + phi.angular().dot(angular()); }
    
    template<typename D>
    typename MotionAlgebraAction<D,Derived>::ReturnType cross_impl(const D & d) const
    {
      return d.motionAction(derived());
    }
    
    template<typename M1, typename M2>
    void motionAction(const MotionDense<M1> & v, MotionDense<M2> & mout) const
    {
      mout.linear() = v.linear().cross(angular())+v.angular().cross(linear());
      mout.angular() = v.angular().cross(angular());
    }
    
    template<typename M1>
    MotionPlain motionAction(const MotionDense<M1> & v) const
    {
      MotionPlain res;
      motionAction(v,res);
      return res;
    }
    
    template<typename M2>
    bool isApprox(const MotionDense<M2> & m2, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return derived().isApprox_impl(m2, prec);
    }
    
    template<typename D2>
    bool isApprox_impl(const MotionDense<D2> & m2, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return linear().isApprox(m2.linear(), prec) && angular().isApprox(m2.angular(), prec);
    }
    
    bool isZero_impl(const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return linear().isZero(prec) && angular().isZero(prec);
    }
    
    template<typename S2, int O2, typename D2>
    void se3Action_impl(const SE3Tpl<S2,O2> & m, MotionDense<D2> & v) const
    {
      v.angular().noalias() = m.rotation()*angular();
      v.linear().noalias() = m.rotation()*linear() + m.translation().cross(v.angular());
    }
    
    template<typename S2, int O2>
    typename SE3GroupAction<Derived>::ReturnType
    se3Action_impl(const SE3Tpl<S2,O2> & m) const
    {
      typename SE3GroupAction<Derived>::ReturnType res;
      se3Action_impl(m,res);
      return res;
    }
    
    template<typename S2, int O2, typename D2>
    void se3ActionInverse_impl(const SE3Tpl<S2,O2> & m, MotionDense<D2> & v) const
    {
      v.linear().noalias() = m.rotation().transpose()*(linear()-m.translation().cross(angular()));
      v.angular().noalias() = m.rotation().transpose()*angular();
    }
    
    template<typename S2, int O2>
    typename SE3GroupAction<Derived>::ReturnType
    se3ActionInverse_impl(const SE3Tpl<S2,O2> & m) const
    {
      typename SE3GroupAction<Derived>::ReturnType res;
      se3ActionInverse_impl(m,res);
      return res;
    }
    
    void disp_impl(std::ostream & os) const
    {
      os
      << "  v = " << linear().transpose () << std::endl
      << "  w = " << angular().transpose () << std::endl;
    }
    
    /// \returns a MotionRef on this.
    MotionRefType ref() { return derived().ref(); }
    
  }; // class MotionDense
  
  /// Basic operations specialization
  template<typename M1, typename M2>
  typename traits<M1>::MotionPlain operator^(const MotionDense<M1> & v1,
                                             const MotionDense<M2> & v2)
  { return v1.derived().cross(v2.derived()); }
  
  template<typename M1, typename F1>
  typename traits<F1>::ForcePlain operator^(const MotionDense<M1> & v,
                                            const ForceBase<F1> & f)
  { return v.derived().cross(f.derived()); }
  
  template<typename M1>
  typename traits<M1>::MotionPlain operator*(const typename traits<M1>::Scalar alpha,
                                             const MotionDense<M1> & v)
  { return v*alpha; }
  
} // namespace pinocchio

#endif // ifndef __pinocchio_motion_dense_hpp__
