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

#ifndef __se3_motion_hpp__
#define __se3_motion_hpp__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/force.hpp"

namespace se3
{


  template< class Derived>
  class MotionBase
  {
  protected:


    typedef Derived  Derived_t;
    SPATIAL_TYPEDEF_TEMPLATE(Derived_t);

  public:
    Derived_t & derived() { return *static_cast<Derived_t*>(this); }
    const Derived_t& derived() const { return *static_cast<const Derived_t*>(this); }

    const Angular_t & angular() const  { return static_cast<const Derived_t*>(this)->angular_impl(); }
    const Linear_t & linear() const  { return static_cast<const Derived_t*>(this)->linear_impl(); }
    Angular_t & angular()  { return static_cast<Derived_t*>(this)->angular_impl(); }
    Linear_t & linear()   { return static_cast<Derived_t*>(this)->linear_impl(); }
    void angular(const Angular_t & R) { static_cast< Derived_t*>(this)->angular_impl(R); }
    void linear(const Linear_t & R) { static_cast< Derived_t*>(this)->linear_impl(R); }

    Vector6 toVector() const
    {
      return derived().toVector_impl();
    }

    operator Vector6 () const { return toVector(); }

    ActionMatrix_t toActionMatrix() const
    {
      return derived().toActionMatrix_impl();
    }

    operator Matrix6 () const { return toActionMatrix(); }


    Derived_t operator-() const
    {
      return derived().__minus__();
    }

    Derived_t operator+(const Derived_t & v2) const
    {
      return derived().__plus__(v2);
    }
    Derived_t operator-(const Derived_t & v2) const
    {
      return derived().__minus__(v2);
    }
    Derived_t& operator+=(const Derived_t & v2)
    {
      return derived().__pequ__(v2);
    }

    Derived_t se3Action(const SE3 & m) const
    {
      return derived().se3Action_impl(m);
    }
    /// bv = aXb.actInv(av)
    Derived_t se3ActionInverse(const SE3 & m) const
    {
      return derived().se3ActionInverse_impl(m);
    }

    void disp(std::ostream & os) const
    {
      os << "base disp" << std::endl;
      derived().disp_impl(os);
    }

    friend std::ostream & operator << (std::ostream & os, const MotionBase<Derived_t> & mv)
    {
      os << "base <<" << std::endl;
      mv.disp(os);
      return os;
    }

  };


  template<typename T, int U>
  struct traits< MotionTpl<T, U> >
  {
    typedef T Scalar_t;
    typedef Eigen::Matrix<T,3,1,U> Vector3;
    typedef Eigen::Matrix<T,4,1,U> Vector4;
    typedef Eigen::Matrix<T,6,1,U> Vector6;
    typedef Eigen::Matrix<T,3,3,U> Matrix3;
    typedef Eigen::Matrix<T,4,4,U> Matrix4;
    typedef Eigen::Matrix<T,6,6,U> Matrix6;
    typedef Matrix6 ActionMatrix_t;
    typedef Vector3 Angular_t;
    typedef Vector3 Linear_t;
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
  class MotionTpl : public MotionBase< MotionTpl< _Scalar, _Options > >
  {
  public:
    SPATIAL_TYPEDEF_TEMPLATE(MotionTpl);


  public:
    // Constructors
    MotionTpl() : m_w(), m_v() {}

    template<typename v1,typename v2>
    MotionTpl(const Eigen::MatrixBase<v1> & v, const Eigen::MatrixBase<v2> & w)
    : m_w(w), m_v(v) {}

    template<typename v6>
    explicit MotionTpl(const Eigen::MatrixBase<v6> & v)
    : m_w(v.template segment<3>(ANGULAR))
    , m_v(v.template segment<3>(LINEAR)) 
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(v6);
      assert( v.size() == 6 );
    }


    template<typename S2,int O2>
    explicit MotionTpl(const MotionTpl<S2,O2> & clone)
    : m_w(clone.angular()),m_v(clone.linear()) {}

    // initializers
    static MotionTpl Zero()   { return MotionTpl(Vector3::Zero(),  Vector3::Zero());   }
    static MotionTpl Random() { return MotionTpl(Vector3::Random(),Vector3::Random()); }

    MotionTpl & setZero () { m_v.setZero (); m_w.setZero (); return *this; }
    MotionTpl & setRandom () { m_v.setRandom (); m_w.setRandom (); return *this; }


    Vector6 toVector_impl() const
    {
      Vector6 v;
      v.template segment<3>(ANGULAR) = m_w;
      v.template segment<3>(LINEAR)  = m_v;
      return v;
    }
    

    ActionMatrix_t toActionMatrix_impl () const
    {
      ActionMatrix_t X;
      X.block <3,3> (ANGULAR, ANGULAR) = X.block <3,3> (LINEAR, LINEAR) = skew (m_w);
      X.block <3,3> (LINEAR, ANGULAR) = skew (m_v);
      X.block <3,3> (ANGULAR, LINEAR).setZero ();

      return X;
    }

    // Getters
    const Vector3 & angular_impl() const { return m_w; }
    const Vector3 & linear_impl()  const { return m_v; }
    Vector3 & angular_impl() { return m_w; }
    Vector3 & linear_impl()  { return m_v; }
    void angular_impl(const Vector3 & w) { m_w=w; }
    void linear_impl(const Vector3 & v) { m_v=v; }

    // Arithmetic operators
    template<typename S2, int O2>
    MotionTpl & operator= (const MotionTpl<S2,O2> & other)
    {
      m_w = other.angular ();
      m_v = other.linear ();
      return *this;
    }
    
    template<typename V6>
    MotionTpl & operator=(const Eigen::MatrixBase<V6> & v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(V6); assert(v.size() == 6);
      m_w = v.template segment<3>(ANGULAR);
      m_v = v.template segment<3>(LINEAR);
      return *this;
    }

    MotionTpl __minus__() const
    {
      return MotionTpl(-m_v, -m_w);
    }

    MotionTpl __plus__(const MotionTpl & v2) const
    {
      return MotionTpl(m_v+v2.m_v,m_w+v2.m_w);
    }
    MotionTpl __minus__(const MotionTpl & v2) const
    {
      return MotionTpl(m_v-v2.m_v,m_w-v2.m_w);
    }
    MotionTpl& __pequ__(const MotionTpl & v2)
    {
      m_v+=v2.m_v;
      m_w+=v2.m_w;
      return *this;
    }


    // MotionTpl operator*(Scalar a) const
    // {
    //   return MotionTpl(m_w*a, m_v*a);
    // }

    // friend MotionTpl operator*(Scalar a, const MotionTpl & mv)
    // {
    //   return MotionTpl(mv.w()*a, mv.v()*a);
    // }

    MotionTpl cross(const MotionTpl& v2) const
    {
      return MotionTpl( m_v.cross(v2.m_w)+m_w.cross(v2.m_v),
                        m_w.cross(v2.m_w) );
    }

    Force cross(const Force& phi) const
    {
      return Force( m_w.cross(phi.linear()),
                    m_w.cross(phi.angular())+m_v.cross(phi.linear()) );
    }

    MotionTpl se3Action_impl(const SE3 & m) const
    {
      Vector3 Rw (static_cast<Vector3>(m.rotation() * angular_impl()));
      return MotionTpl( m.rotation()*linear_impl() + m.translation().cross(Rw),
                        Rw);
    }
    /// bv = aXb.actInv(av)
    MotionTpl se3ActionInverse_impl(const SE3 & m) const
    {
      return MotionTpl( m.rotation().transpose()*(linear_impl()-m.translation().cross(angular_impl())),
                        m.rotation().transpose()*angular_impl());
    }

    void disp_impl(std::ostream & os) const
    {
      os << "  v = " << linear_impl().transpose () << std::endl
      << "  w = " << angular_impl().transpose () << std::endl;
    }

    /** \brief Compute the classical acceleration of point according to the spatial velocity and spatial acceleration of the frame centered on this point
     */
    static inline Vector3 computeLinearClassicalAcceleration (const MotionTpl & spatial_velocity, const MotionTpl & spatial_acceleration)
    {
      return spatial_acceleration.linear () + spatial_velocity.angular ().cross (spatial_velocity.linear ());
    }

    /**
      \brief Compute the spatial motion quantity of the parallel frame translated by translation_vector
     */
    MotionTpl translate (const Vector3 & translation_vector) const
    {
      return MotionTpl (m_v + m_w.cross (translation_vector), m_w);
    }

  public:
  private:
    Vector3 m_w;
    Vector3 m_v;
  }; // class MotionTpl

  template<typename S,int O>
  MotionTpl<S,O> operator^( const MotionTpl<S,O> &m1, const MotionTpl<S,O> &m2 ) { return m1.cross(m2); }
  template<typename S,int O>
  ForceTpl<S,O> operator^( const MotionTpl<S,O> &m, const ForceTpl<S,O> &f ) { return m.cross(f); }

  typedef MotionTpl<double> Motion;


  struct BiasZero;

  template<>
  struct traits< BiasZero >
  {
    typedef double Scalar_t;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,4,1,0> Vector4;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,3,3,0> Matrix3;
    typedef Eigen::Matrix<double,4,4,0> Matrix4;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef Matrix6 ActionMatrix_t;
    typedef Vector3 Angular_t;
    typedef Vector3 Linear_t;
    typedef Eigen::Quaternion<double,0> Quaternion_t;
    typedef SE3Tpl<double,0> SE3;
    typedef ForceTpl<double,0> Force;
    typedef MotionTpl<double,0> Motion;
    typedef Symmetric3Tpl<double,0> Symmetric3;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  };

  struct BiasZero : public MotionBase< BiasZero >
  {
    SPATIAL_TYPEDEF_NO_TEMPLATE(BiasZero);
    operator Motion () const { return Motion::Zero(); }
  }; // struct BiasZero

const Motion & operator+( const Motion& v, const BiasZero&) { return v; }
const Motion & operator+ ( const BiasZero&,const Motion& v) { return v; }

} // namespace se3

#endif // ifndef __se3_motion_hpp__
