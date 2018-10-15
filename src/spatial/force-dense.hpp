//
// Copyright (c) 2017-2018 CNRS
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

#ifndef __se3_force_dense_hpp__
#define __se3_force_dense_hpp__

namespace se3
{
  
  namespace internal
  {
    template<typename Derived>
    struct SE3GroupAction< ForceDense<Derived> >
    {
      typedef typename SE3GroupAction< Derived >::ReturnType ReturnType;
    };
    
    template<typename Derived, typename MotionDerived>
    struct MotionAlgebraAction< ForceDense<Derived>, MotionDerived >
    {
      typedef typename MotionAlgebraAction< Derived, MotionDerived >::ReturnType ReturnType;
    };
  }
  
  template<typename Derived>
  class ForceDense : public ForceBase<Derived>
  {
  public:
    typedef ForceBase<Derived> Base;
    FORCE_TYPEDEF_TPL(Derived);
    typedef typename traits<Derived>::ForceRefType ForceRefType;
    
    using Base::linear;
    using Base::angular;
    using Base::derived;
    using Base::isApprox;
    
    Derived & setZero() { linear().setZero(); angular().setZero(); return derived(); }
    Derived & setRandom() { linear().setRandom(); angular().setRandom(); return derived(); }
    
    template<typename D2>
    bool isEqual_impl(const ForceDense<D2> & other) const
    { return linear() == other.linear() && angular() == other.angular(); }
    
    template<typename D2>
    bool isEqual_impl(const ForceBase<D2> & other) const
    { return other.derived() == derived(); }
    
    // Arithmetic operators
    template<typename D2>
    Derived & operator=(const ForceDense<D2> & other)
    {
      linear() = other.linear();
      angular() = other.angular();
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
    
    ForcePlain operator-() const { return derived().__opposite__(); }
    template<typename M1>
    ForcePlain operator+(const ForceDense<M1> & v) const { return derived().__plus__(v.derived()); }
    template<typename M1>
    ForcePlain operator-(const ForceDense<M1> & v) const { return derived().__minus__(v.derived()); }
    
    template<typename M1>
    Derived & operator+=(const ForceDense<M1> & v) { return derived().__pequ__(v.derived()); }
    template<typename M1>
    Derived & operator+=(const ForceBase<M1> & v)
    { v.derived().addTo(derived()); return derived(); }
    
    template<typename M1>
    Derived & operator-=(const ForceDense<M1> & v) { return derived().__mequ__(v.derived()); }

    ForcePlain __opposite__() const { return ForcePlain(-linear(),-angular()); }
    
    template<typename M1>
    ForcePlain __plus__(const ForceDense<M1> & v) const
    { return ForcePlain(linear()+v.linear(), angular()+v.angular()); }
    
    template<typename M1>
    ForcePlain __minus__(const ForceDense<M1> & v) const
    { return ForcePlain(linear()-v.linear(), angular()-v.angular()); }
    
    template<typename M1>
    Derived & __pequ__(const ForceDense<M1> & v)
    { linear() += v.linear(); angular() += v.angular(); return derived(); }
    
    template<typename M1>
    Derived & __mequ__(const ForceDense<M1> & v)
    { linear() -= v.linear(); angular() -= v.angular(); return derived(); }
    
    template<typename OtherScalar>
    ForcePlain __mult__(const OtherScalar & alpha) const
    { return ForcePlain(alpha*linear(),alpha*angular()); }
    
    template<typename OtherScalar>
    ForcePlain __div__(const OtherScalar & alpha) const
    { return derived().__mult__((OtherScalar)(1)/alpha); }
    
    template<typename F1>
    Scalar dot(const MotionDense<F1> & phi) const
    { return phi.linear().dot(linear()) + phi.angular().dot(angular()); }
    
    template<typename M1, typename M2>
    void motionAction(const MotionDense<M1> & v, ForceDense<M2> & fout) const
    {
      fout.linear() = v.angular().cross(linear());
      fout.angular() = v.angular().cross(angular())+v.linear().cross(linear());
    }
    
    template<typename M1>
    ForcePlain motionAction(const MotionDense<M1> & v) const
    {
      ForcePlain res;
      motionAction(v,res);
      return res;
    }
    
    template<typename M2>
    bool isApprox(const ForceDense<M2> & f, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return derived().isApprox_impl(f, prec);}
    
    template<typename D2>
    bool isApprox_impl(const ForceDense<D2> & f, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return linear().isApprox(f.linear(), prec) && angular().isApprox(f.angular(), prec);
    }
    
    template<typename S2, int O2, typename D2>
    void se3Action_impl(const SE3Tpl<S2,O2> & m, ForceDense<D2> & f) const
    {
      f.linear() = m.rotation()*linear();
      f.angular() = m.rotation()*angular() + m.translation().cross(f.linear());
    }
    
    template<typename S2, int O2>
    ForcePlain se3Action_impl(const SE3Tpl<S2,O2> & m) const
    {
      ForcePlain res;
      se3Action_impl(m,res);
      return res;
    }
    
    template<typename S2, int O2, typename D2>
    void se3ActionInverse_impl(const SE3Tpl<S2,O2> & m, ForceDense<D2> & f) const
    {
      f.linear() = angular()-m.translation().cross(linear());
      f.angular() = m.rotation().transpose()*(f.linear());
      f.linear() = m.rotation().transpose()*linear();
    }
    
    template<typename S2, int O2>
    ForcePlain se3ActionInverse_impl(const SE3Tpl<S2,O2> & m) const
    {
      ForcePlain res;
      se3ActionInverse_impl(m,res);
      return res;
    }
    
    void disp_impl(std::ostream & os) const
    {
      os
      << "  f = " << linear().transpose () << std::endl
      << "tau = " << angular().transpose () << std::endl;
    }
    
    /// \returns a MotionRef on this.
    ForceRefType ref() { return derived().ref(); }
    
  }; // class ForceDense
  
  /// Basic operations specialization
  template<typename F1>
  typename traits<F1>::ForcePlain operator*(const typename traits<F1>::Scalar alpha,
                                             const ForceDense<F1> & f)
  { return f.derived()*alpha; }
  
} // namespace se3

#endif // ifndef __se3_force_dense_hpp__
