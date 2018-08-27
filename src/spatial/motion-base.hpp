//
// Copyright (c) 2015-2018 CNRS
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

#ifndef __se3_motion_base_hpp__
#define __se3_motion_base_hpp__

namespace se3
{
  
  template<class Derived>
  class MotionBase
  {
  public:
    MOTION_TYPEDEF_TPL(Derived);
    
    Derived & derived() { return *static_cast<Derived*>(this); }
    const Derived& derived() const { return *static_cast<const Derived*>(this); }
    
    ConstAngularType angular() const  { return derived().angular_impl(); }
    ConstLinearType linear() const  { return derived().linear_impl(); }
    AngularType angular() { return derived().angular_impl(); }
    LinearType linear() { return derived().linear_impl(); }
    
    template<typename V3Like>
    void angular(const Eigen::MatrixBase<V3Like> & w)
    { derived().angular_impl(w.derived()); }
    
    template<typename V3Like>
    void linear(const Eigen::MatrixBase<V3Like> & v)
    { derived().linear_impl(v.derived()); }
    
    ToVectorConstReturnType toVector() const { return derived().toVector_impl(); }
    ToVectorReturnType toVector() { return derived().toVector_impl(); }
    operator Vector6() const { return toVector(); }
    
    ActionMatrixType toActionMatrix() const { return derived().toActionMatrix_impl(); }
    ActionMatrixType toDualActionMatrix() const { return derived().toDualActionMatrix_impl(); }
    operator Matrix6() const { return toActionMatrix(); }
    
    template<typename M2>
    bool operator==(const MotionBase<M2> & other) const
    { return derived().isEqual_impl(other.derived()); }
    
    template<typename M2>
    bool operator!=(const MotionBase<M2> & other) const
    { return !(derived() == other.derived()); }
    
    MotionPlain operator-() const { return derived().__opposite__(); }
    MotionPlain operator+(const Derived & v) const { return derived().__plus__(v); }
    MotionPlain operator-(const Derived & v) const { return derived().__minus__(v); }
    Derived & operator+=(const Derived & v) { return derived().__pequ__(v); }
    Derived & operator-=(const Derived & v) { return derived().__mequ__(v); }
    
    template<typename OtherScalar>
    MotionPlain operator*(const OtherScalar & alpha) const
    { return derived().__mult__(alpha); }
    template<typename OtherScalar>
    MotionPlain operator/(const OtherScalar & alpha) const
    { return derived().__div__(alpha); }
    
    template<typename OtherSpatialType>
    typename internal::MotionAlgebraAction<OtherSpatialType,Derived>::ReturnType
    cross(const OtherSpatialType & d) const
    {
      return derived().cross_impl(d);
    }
    
    bool isApprox(const Derived & other, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return derived().isApprox_impl(other, prec);}
    
    template<typename S2, int O2>
    MotionPlain se3Action(const SE3Tpl<S2,O2> & m) const
    { return derived().se3Action_impl(m); }
    
    template<typename S2, int O2>
    MotionPlain se3ActionInverse(const SE3Tpl<S2,O2> & m) const
    { return derived().se3ActionInverse_impl(m); }
    
    Scalar dot(const Force & f) const { return derived().dot(f); }
    
    void disp(std::ostream & os) const { derived().disp_impl(os); }
    friend std::ostream & operator << (std::ostream & os, const MotionBase<Derived> & v)
    {
      v.disp(os);
      return os;
    }
    
  }; // class MotionBase
  
} // namespace se3

#endif // ifndef __se3_motion_base_hpp__
