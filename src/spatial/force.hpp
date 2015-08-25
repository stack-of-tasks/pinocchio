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
    SPATIAL_TYPEDEF_ARG(Derived_t);

  public:
    Derived_t & derived() { return *static_cast<Derived_t*>(this); }
    const Derived_t& derived() const { return *static_cast<const Derived_t*>(this); }

    const Angular_t & angular() const  { return derived().angular_impl(); }
    const Linear_t & linear() const  { return derived().linear_impl(); }
    Angular_t & angular()  { return derived().angular_impl(); }
    Linear_t & linear()   { return derived().linear_impl(); }
    void angular(const Angular_t & R) { derived().angular_impl(R); }
    void linear(const Linear_t & R) { derived().linear_impl(R); }


    Vector6 toVector() const
    {
      return derived().toVector_impl();
    }
    operator Vector6 () const { return toVector(); }


    void disp(std::ostream & os) const
    {
      os << "base disp" << std::endl;
      static_cast<const Derived_t*>(this)->disp_impl(os);
    }

    Derived_t & operator= (const Derived_t & other) { return derived().__equl__(other); }
    Derived_t& operator+= (const Derived_t & phi) { return derived().__pequ__(phi); }
    Derived_t operator+(const Derived_t & phi) const { return derived().__plus__(phi); }
    Derived_t operator*(double a) const    { return derived().__mult__(a); }
    Derived_t operator-() const { return derived().__minus__(); }
    Derived_t operator-(const Derived_t & phi) const { return derived().__minus__(phi); }


    /// af = aXb.act(bf)
    Derived_t se3Action(const SE3 & m) const
    {
      return derived().se3Action_impl(m);
    }

    Derived_t se3ActionInverse(const SE3 & m) const
    {
      return derived().se3ActionInverse_impl(m);
    }

    friend std::ostream & operator << (std::ostream & os,const ForceBase<Derived> & X)
    { 
      os << "base <<" << std::endl;
      X.disp(os);
      return os;
    }

  };


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
    typedef Vector3 Angular_t;
    typedef Vector3 Linear_t;
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
  };

  template<typename _Scalar, int _Options>
  class ForceTpl : public ForceBase< ForceTpl< _Scalar, _Options > >
  {

  public:
    friend class ForceBase< ForceTpl< _Scalar, _Options > >;
    SPATIAL_TYPEDEF_ARG(ForceTpl);


  public:
    ForceTpl() : m_n(), m_f() {}


    template<typename f3_t,typename n3_t>
    ForceTpl(const Eigen::MatrixBase<f3_t> & f,const Eigen::MatrixBase<n3_t> & n)
    : m_n(n),
    m_f(f)
    {

    }

    template<typename f6>
    explicit ForceTpl(const Eigen::MatrixBase<f6> & f)
    : m_n(f.template segment<3>(ANGULAR)),
    m_f(f.template segment<3>(LINEAR)) 
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(f6);
      assert( f.size() == 6 );
    }


    template<typename S2,int O2>
    explicit ForceTpl(const ForceTpl<S2,O2> & clone)
    : m_n(clone.angular()),
    m_f(clone.linear())
    {

    }


    static ForceTpl Zero() { return ForceTpl(Linear_t::Zero(), Angular_t::Zero()); }
    static ForceTpl Random() { return ForceTpl(Linear_t::Random(), Angular_t::Random()); }


    ForceTpl & setZero () { m_n.setZero (); m_f.setZero (); return *this; } 
    ForceTpl & setRandom () { m_n.setRandom (); m_f.setRandom (); return *this; }


  public:
    Vector6 toVector_impl() const
    {
      Vector6 f;
      f.template segment<3>(ANGULAR) = m_n;
      f.template segment<3>(LINEAR)  = m_f;
      return f;
    }



    void disp_impl(std::ostream & os) const
    {
      os
      << "f =\n" << m_f << std::endl
      << "tau =\n" << m_n << std::endl;
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

    // Arithmetic operators
    template<typename S2, int O2>
    ForceTpl & operator= (const ForceTpl<S2,O2> & other)
    {
      m_n = other.angular ();
      m_f = other.linear ();
      return *this;
    }

    template<typename F6>
    ForceTpl & operator=(const Eigen::MatrixBase<F6> & phi)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(F6); assert(phi.size() == 6);
      m_n = phi.template segment<3>(ANGULAR);
      m_f = phi.template segment<3>(LINEAR);
      return *this;
    }

    // template <typename D>
    // ForceTpl operator + (ForceBase<D> a){
    //   return ForceTpl(m_f+a.linear(), m_n + a.angular());
    // }

     // friend ForceTpl operator*(Scalar a, const ForceTpl & phi)
    // {
    //   return ForceTpl(phi.n()*a, phi.f()*a);
    // }

    ForceTpl & __equl__(const ForceTpl & other)
    {
      m_n = other.angular();
      m_f = other.linear();
      return *this;
    }


    ForceTpl& __pequ__ (const ForceTpl & phi)
    {
      m_f += phi.m_f;
      m_n += phi.m_n;
      return *this;
    }

    ForceTpl __plus__(const ForceTpl & phi) const
    {
      return ForceTpl(m_f+phi.m_f,m_n+phi.m_n);
    }

    ForceTpl __mult__(double a) const
    {
      return ForceTpl(m_f*a, m_n*a);
    }

    ForceTpl __minus__() const
    {
      return ForceTpl(-m_f, -m_n);
    }

    ForceTpl __minus__(const ForceTpl & phi) const
    {
      return ForceTpl(m_f-phi.m_f,m_n-phi.m_n);
    }


  public:
    const Angular_t & angular_impl() const { return m_n; }
    Angular_t & angular_impl() { return m_n; }
    void angular_impl(const Angular_t & R) { m_n = R; }
    const Linear_t & linear_impl() const { return m_f;}
    Linear_t & linear_impl() { return m_f;}
    void linear_impl(const Linear_t & p) { m_f=p; }

  protected:
    Angular_t m_n;
    Linear_t m_f;
  };


  typedef ForceTpl<double,0> Force;


} // namespace se3

#endif // ifndef __se3_force_hpp__

