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
    SPATIAL_TYPEDEF_TEMPLATE(Derived);
    
    Derived & derived() { return *static_cast<Derived*>(this); }
    const Derived& derived() const { return *static_cast<const Derived*>(this); }
    
    ConstAngular_t angular() const  { return derived().angular_impl(); }
    ConstLinear_t linear() const  { return derived().linear_impl(); }
    Angular_t angular() { return derived().angular_impl(); }
    Linear_t linear() { return derived().linear_impl(); }
    
    template<typename V3>
    void angular(const Eigen::MatrixBase<V3> & w) { derived().angular_impl(w); }
    template<typename V3>
    void linear(const Eigen::MatrixBase<V3> & v) { derived().linear_impl(v); }
    
    const Vector6 & toVector() const { return derived().toVector_impl(); }
    Vector6 & toVector() { return derived().toVector_impl(); }
    operator Vector6 () const { return toVector(); }
    
    ActionMatrix_t toActionMatrix() const { return derived().toActionMatrix_impl(); }
    ActionMatrix_t toDualActionMatrix() const { return derived().toDualActionMatrix_impl(); }
    operator Matrix6 () const { return toActionMatrix(); }
    
    bool operator==(const Derived & other) const { return derived().isEqual_impl(other);}
    bool operator!=(const Derived & other) const { return !(*this == other); }
    Derived operator-() const { return derived().__minus__(); }
    Derived operator+(const Derived & v) const { return derived().__plus__(v); }
    Derived operator-(const Derived & v) const { return derived().__minus__(v); }
    Derived & operator+=(const Derived & v) { return derived().__pequ__(v); }
    Derived & operator-=(const Derived & v) { return derived().__mequ__(v); }
    Derived operator*(const Scalar alpha) const { return derived().__mult__(alpha); }
    
    template<typename D>
    typename internal::MotionAlgebraAction<D>::ReturnType cross(const D & d) const
    {
      return derived().cross_impl(d);
    }
    
    bool isApprox(const Derived & other, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return derived().isApprox_impl(other, prec);}
    
    Derived se3Action(const SE3 & m) const { return derived().se3Action_impl(m); }
    Derived se3ActionInverse(const SE3 & m) const { return derived().se3ActionInverse_impl(m); }
    
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
