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

#ifndef __se3_motion_base_hpp__
#define __se3_motion_base_hpp__

#include <Eigen/Core>
#include "pinocchio/macros.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
//#include "pinocchio/spatial/force.hpp"

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
    
    template<typename V3>
    void angular(const Eigen::MatrixBase<V3> & w) { derived().angular_impl(w); }
    template<typename V3>
    void linear(const Eigen::MatrixBase<V3> & v) { derived().linear_impl(v); }
    
    const Vector6 & toVector() const { return derived().toVector_impl(); }
    Vector6 & toVector() { return derived().toVector_impl(); }
    operator Vector6 () const { return toVector(); }
    
    ActionMatrixType toActionMatrix() const { return derived().toActionMatrix_impl(); }
    ActionMatrixType toDualActionMatrix() const { return derived().toDualActionMatrix_impl(); }
    operator Matrix6 () const { return toActionMatrix(); }
    
    bool operator==(const Derived & other) const { return derived().isEqual_impl(other);}
    bool operator!=(const Derived & other) const { return !(*this == other); }
    MotionPlain operator-() const { return derived().__minus__(); }
    MotionPlain operator+(const Derived & v) const { return derived().__plus__(v); }
    MotionPlain operator-(const Derived & v) const { return derived().__minus__(v); }
    Derived & operator+=(const Derived & v) { return derived().__pequ__(v); }
    Derived & operator-=(const Derived & v) { return derived().__mequ__(v); }
    MotionPlain operator*(const Scalar alpha) const { return derived().__mult__(alpha); }
    
    template<typename D>
    typename internal::MotionAlgebraAction<D>::ReturnType cross(const D & d) const
    {
      return derived().cross_impl(d);
    }
    
    bool isApprox(const Derived & other, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return derived().isApprox_impl(other, prec);}
    
    MotionPlain se3Action(const SE3 & m) const { return derived().se3Action_impl(m); }
    MotionPlain se3ActionInverse(const SE3 & m) const { return derived().se3ActionInverse_impl(m); }
    
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
