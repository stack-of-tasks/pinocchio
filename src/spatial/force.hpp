//
// Copyright (c) 2015 CNRS
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

#ifndef __se3_force_hpp__
#define __se3_force_hpp__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "pinocchio/spatial/fwd.hpp"


namespace se3
{

  template< class Derived>
  class ForceBase
  {
  protected:

    typedef Derived  Derived_t;
    SPATIAL_TYPEDEF_TEMPLATE(Derived_t);
    
//    typedef typename Vector6::template ConstFixedSegmentReturnType<3>::Type ConstLinear_t;
//    typedef typename Vector6::template ConstFixedSegmentReturnType<3>::Type ConstAngular_t;
    typedef typename Eigen::VectorBlock<const Vector6,3> ConstLinear_t;
    typedef ConstLinear_t ConstAngular_t;

  public:
    Derived_t & derived() { return *static_cast<Derived_t*>(this); }
    const Derived_t& derived() const { return *static_cast<const Derived_t*>(this); }

    ConstAngular_t angular() const  { return derived().angular_impl(); }
    ConstLinear_t linear() const  { return derived().linear_impl(); }
    Angular_t angular()  { return derived().angular_impl(); }
    Linear_t linear()   { return derived().linear_impl(); }
    void angular(const Vector3 & n) { derived().angular_impl(n); }
    void linear(const Vector3 & f) { derived().linear_impl(f); }

    const Vector6 & toVector() const { return derived().toVector_impl(); }
    Vector6 & toVector() { return derived().toVector_impl(); }
    operator Vector6 () const { return toVector(); }
    
//    void disp(std::ostream & os) const
//    {
//      static_cast<const Derived_t*>(this)->disp_impl(os);
//    }

    bool operator== (const Derived_t & other) const {return derived().isEqual(other);}
    Derived_t & operator= (const Derived_t & other) { return derived().__equl__(other); }
    Derived_t& operator+= (const Derived_t & phi) { return derived().__pequ__(phi); }
    Derived_t operator+(const Derived_t & phi) const { return derived().__plus__(phi); }
    Derived_t operator*(double a) const    { return derived().__mult__(a); }
    Derived_t operator-() const { return derived().__minus__(); }
    Derived_t operator-(const Derived_t & phi) const { return derived().__minus__(phi); }
    
    Scalar_t dot(const Motion & m) const { return static_cast<Derived_t*>(this)->dot(m); }

    Derived_t se3Action(const SE3 & m) const { return derived().se3Action_impl(m); }
    Derived_t se3ActionInverse(const SE3 & m) const { return derived().se3ActionInverse_impl(m); }

    void disp(std::ostream & os) const { derived().disp_impl(os); }
    friend std::ostream & operator << (std::ostream & os, const ForceBase<Derived_t> & X)
    { 
      X.disp(os);
      return os;
    }

  }; // class ForceBase


  template<typename T, int U>
  struct traits< ForceTpl<T, U> >
  {
    typedef T Scalar_t;
    typedef Eigen::Matrix<T,3,1,U> Vector3;
    typedef Eigen::Matrix<T,4,1,U> Vector4;
    typedef Eigen::Matrix<T,6,1,U> Vector6;
    typedef Eigen::Matrix<T,3,3,U> Matrix3;
    typedef Eigen::Matrix<T,4,4,U> Matrix4;
    typedef Eigen::Matrix<T,6,6,U> Matrix6;
    typedef typename Eigen::VectorBlock<Vector6,3> Linear_t;
    //    typedef typename Vector6::template FixedSeegmentReturnType<3>::Type Linear_t;
    typedef Linear_t Angular_t;
//    typedef Vector3 Angular_t;
//    typedef Vector3 Linear_t;
    typedef Matrix6 ActionMatrix_t;
    typedef Eigen::Quaternion<T,U> Quaternion_t;
    typedef SE3Tpl<T,U> SE3;
    typedef ForceTpl<T,U> Force;
    typedef MotionTpl<T,U> Motion;
    typedef Symmetric3Tpl<T,U> Symmetric3;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits ForceTpl

  template<typename _Scalar, int _Options>
  class ForceTpl : public ForceBase< ForceTpl< _Scalar, _Options > >
  {

  public:
    friend class ForceBase< ForceTpl< _Scalar, _Options > >;
    SPATIAL_TYPEDEF_TEMPLATE(ForceTpl);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
//    typedef typename Vector6::template ConstFixedSegmentReturnType<3>::Type ConstLinear_t;
//    typedef typename Vector6::template ConstFixedSegmentReturnType<3>::Type ConstAngular_t;
    typedef typename Eigen::VectorBlock<const Vector6,3> ConstLinear_t;
    typedef ConstLinear_t ConstAngular_t;

    ForceTpl() : data() {}


    template<typename f3_t,typename n3_t>
    ForceTpl(const Eigen::MatrixBase<f3_t> & f,const Eigen::MatrixBase<n3_t> & n)
    {
      data << f, n;
    }

    template<typename f6>
    explicit ForceTpl(const Eigen::MatrixBase<f6> & f)
    : data(f)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(f6);
      assert( f.size() == 6 );
    }

    template<typename S2,int O2>
    explicit ForceTpl(const ForceTpl<S2,O2> & clone)
    : data(clone.toVector())
    {}

    static ForceTpl Zero() { return ForceTpl(Linear_t::Zero(), Angular_t::Zero()); }
    static ForceTpl Random() { return ForceTpl(Linear_t::Random(), Angular_t::Random()); }

    ForceTpl & setZero () { data.setZero (); return *this; }
    ForceTpl & setRandom () { data.setRandom (); return *this; }

    const Vector6 & toVector_impl() const { return data; }
    Vector6 & toVector_impl() { return data; }

    void disp_impl(std::ostream & os) const
    {
      os << "  f = " << linear_impl().transpose() << std::endl
      << "  tau = " << angular_impl().transpose() << std::endl;
    }

    /// af = aXb.act(bf)
    ForceTpl se3Action_impl(const SE3 & m) const
    {
      Vector3 Rf (static_cast<Vector3>( (m.rotation()) * linear_impl() ) );
      return ForceTpl(Rf,m.translation().cross(Rf)+m.rotation()*angular_impl());
    }


    ForceTpl se3ActionInverse_impl(const SE3 & m) const
    {
      return ForceTpl(m.rotation().transpose()*linear_impl(),
        m.rotation().transpose()*(angular_impl() - m.translation().cross(linear_impl())) );
    }
    
    bool isEqual (const ForceTpl & other) const { return data == other.data; }

    // Arithmetic operators
    template<typename S2, int O2>
    ForceTpl & operator= (const ForceTpl<S2,O2> & other)
    {
      data = other.toVector();
      return *this;
    }

    template<typename F6>
    ForceTpl & operator= (const Eigen::MatrixBase<F6> & phi)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(F6); assert(phi.size() == 6);
      data = phi;
      return *this;
    }

    ForceTpl & __equl__(const ForceTpl & other) { data = other.data; return *this; }
    ForceTpl & __pequ__ (const ForceTpl & phi) { data += phi.data; return *this; }
    ForceTpl __plus__(const ForceTpl & phi) const { return ForceTpl(data + phi.data); }
    ForceTpl __mult__(const double a) const { return ForceTpl(a*data); }
    ForceTpl __minus__() const { return ForceTpl(-data); }
    ForceTpl __minus__(const ForceTpl & phi) const { return ForceTpl(data - phi.data); }


    ConstAngular_t angular_impl() const { return data.template segment<3> (ANGULAR); }
    Angular_t angular_impl() { return data.template segment<3> (ANGULAR); }
    void angular_impl(const Vector3 & n) { data.template segment<3> (ANGULAR) = n; }
    ConstLinear_t linear_impl() const { return data.template segment<3> (LINEAR);}
    Linear_t linear_impl() { return data.template segment<3> (LINEAR);}
    void linear_impl(const Vector3 & f) { data.template segment<3> (LINEAR) = f; }
    
    Scalar_t dot(const Motion & m) const { return data.dot(m.toVector()); }

  protected:
    Vector6 data;

  }; // class ForceTpl

  typedef ForceTpl<double,0> Force;

} // namespace se3

#endif // ifndef __se3_force_hpp__

