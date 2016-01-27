//
// Copyright (c) 2015 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#define MOTION_SPECIFIC_TYPEDEF \
typedef typename Eigen::VectorBlock<const Vector6,3> ConstLinear_t; \
typedef ConstLinear_t ConstAngular_t;

namespace se3
{


  template< class Derived>
  class MotionBase
  {
  protected:
    typedef Derived  Derived_t;
    SPATIAL_TYPEDEF_TEMPLATE(Derived_t);
    MOTION_SPECIFIC_TYPEDEF
    
  public:
    Derived_t & derived() { return *static_cast<Derived_t*>(this); }
    const Derived_t& derived() const { return *static_cast<const Derived_t*>(this); }

    ConstAngular_t angular() const  { return static_cast<const Derived_t*>(this)->angular_impl(); }
    ConstLinear_t linear() const  { return static_cast<const Derived_t*>(this)->linear_impl(); }
    Angular_t angular()  { return static_cast<Derived_t*>(this)->angular_impl(); }
    Linear_t linear()   { return static_cast<Derived_t*>(this)->linear_impl(); }
    
    template<typename D>
    void angular(const Eigen::MatrixBase<D> & w) { static_cast< Derived_t*>(this)->angular_impl(w); }
    template<typename D>
    void linear(const Eigen::MatrixBase<D> & v) { static_cast< Derived_t*>(this)->linear_impl(v); }

    const Vector6 & toVector() const { return derived().toVector_impl(); }
    Vector6 & toVector() { return derived().toVector_impl(); }
    operator Vector6 () const { return toVector(); }

    ActionMatrix_t toActionMatrix() const { return derived().toActionMatrix_impl(); }
    operator Matrix6 () const { return toActionMatrix(); }

    bool operator== (const Derived_t & other) const {return derived().isEqual(other);}
    Derived_t operator-() const { return derived().__minus__(); }
    Derived_t operator+(const Derived_t & v2) const { return derived().__plus__(v2); }
    Derived_t operator-(const Derived_t & v2) const { return derived().__minus__(v2); }
    Derived_t & operator+=(const Derived_t & v2) { return derived().__pequ__(v2); }

    Derived_t se3Action(const SE3 & m) const { return derived().se3Action_impl(m); }
    Derived_t se3ActionInverse(const SE3 & m) const { return derived().se3ActionInverse_impl(m); }
    
    Scalar_t dot(const Force & f) const { return static_cast<Derived_t*>(this)->dot(f); }

    void disp(std::ostream & os) const { derived().disp_impl(os); }
    friend std::ostream & operator << (std::ostream & os, const MotionBase<Derived_t> & mv)
    {
      mv.disp(os);
      return os;
    }

  }; // class MotionBase


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
    typedef typename Eigen::VectorBlock<Vector6,3> Linear_t;
    typedef Linear_t Angular_t;
    typedef Eigen::Quaternion<T,U> Quaternion_t;
    typedef SE3Tpl<T,U> SE3;
    typedef ForceTpl<T,U> Force;
    typedef MotionTpl<T,U> Motion;
    typedef Symmetric3Tpl<T,U> Symmetric3;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits MotionTpl


  template<typename _Scalar, int _Options>
  class MotionTpl : public MotionBase< MotionTpl< _Scalar, _Options > >
  {
  public:
    SPATIAL_TYPEDEF_TEMPLATE(MotionTpl);
    MOTION_SPECIFIC_TYPEDEF
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
  public:
    // Constructors
    MotionTpl() : data() {}

    template<typename v1,typename v2>
    MotionTpl(const Eigen::MatrixBase<v1> & v, const Eigen::MatrixBase<v2> & w)
    {
      data << v, w;
    }

    template<typename v6>
    explicit MotionTpl(const Eigen::MatrixBase<v6> & v)
    : data(v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(v6);
      assert( v.size() == 6 );
    }


    template<typename S2,int O2>
    explicit MotionTpl(const MotionTpl<S2,O2> & clone)
    : data(clone.toVector())
    {}

    // initializers
    static MotionTpl Zero()   { return MotionTpl(Vector6::Zero());   }
    static MotionTpl Random() { return MotionTpl(Vector6::Random()); }

    MotionTpl & setZero () { data.setZero (); return *this; }
    MotionTpl & setRandom () { data.setRandom (); return *this; }

    const Vector6 & toVector_impl() const { return data; }
    Vector6 & toVector_impl() { return data; }

    ActionMatrix_t toActionMatrix_impl () const
    {
      ActionMatrix_t X;
      X.block <3,3> (ANGULAR, ANGULAR) = X.block <3,3> (LINEAR, LINEAR) = skew (angular_impl());
      X.block <3,3> (LINEAR, ANGULAR) = skew (linear_impl());
      X.block <3,3> (ANGULAR, LINEAR).setZero ();

      return X;
    }

    // Getters
    ConstAngular_t angular_impl() const { return data.template segment<3> (ANGULAR); }
    ConstLinear_t linear_impl()  const { return data.template segment<3> (LINEAR); }
    Angular_t angular_impl() { return data.template segment<3> (ANGULAR); }
    Linear_t linear_impl()  { return data.template segment<3> (LINEAR); }
    
    template<typename D>
    void angular_impl(const Eigen::MatrixBase<D> & w) { data.template segment<3> (ANGULAR)=w; }
    template<typename D>
    void linear_impl(const Eigen::MatrixBase<D> & v) { data.template segment<3> (LINEAR)=v; }

    bool isEqual (const MotionTpl & other) const { return data == other.data; }
    
    // Arithmetic operators
    template<typename S2, int O2>
    MotionTpl & operator= (const MotionTpl<S2,O2> & other)
    {
      data = other.toVector();
      return *this;
    }
    
    template<typename V6>
    MotionTpl & operator=(const Eigen::MatrixBase<V6> & v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(V6); assert(v.size() == 6);
      data = v;
      return *this;
    }

    MotionTpl __minus__() const { return MotionTpl(-data); }
    MotionTpl __plus__(const MotionTpl & v2) const { return MotionTpl(data + v2.data); }
    MotionTpl __minus__(const MotionTpl & v2) const { return MotionTpl(data - v2.data); }
    MotionTpl& __pequ__(const MotionTpl & v2) { data += v2.data; return *this; }
    
    Scalar_t dot(const Force & f) const { return data.dot(f.toVector()); }

    MotionTpl cross(const MotionTpl& v2) const
    {
      return MotionTpl( linear_impl().cross(v2.angular_impl())+angular_impl().cross(v2.linear_impl()),
                        angular_impl().cross(v2.angular_impl()) );
    }

    Force cross(const Force& phi) const
    {
      return Force( angular_impl().cross(phi.linear_impl()),
                    angular_impl().cross(phi.angular_impl())+linear_impl().cross(phi.linear_impl()) );
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

//    /** \brief Compute the classical acceleration of point according to the spatial velocity and spatial acceleration of the frame centered on this point
//     */
//    static inline Vector3 computeLinearClassicalAcceleration (const MotionTpl & spatial_velocity, const MotionTpl & spatial_acceleration)
//    {
//      return spatial_acceleration.linear () + spatial_velocity.angular ().cross (spatial_velocity.linear ());
//    }
//
//    /**
//      \brief Compute the spatial motion quantity of the parallel frame translated by translation_vector
//     */
//    MotionTpl translate (const Vector3 & translation_vector) const
//    {
//      return MotionTpl (linear() + angular().cross (translation_vector), angular());
//    }

  protected:
    Vector6 data;

  }; // class MotionTpl

  template<typename S,int O>
  MotionTpl<S,O> operator^( const MotionTpl<S,O> &m1, const MotionTpl<S,O> &m2 ) { return m1.cross(m2); }
  template<typename S,int O>
  ForceTpl<S,O> operator^( const MotionTpl<S,O> &m, const ForceTpl<S,O> &f ) { return m.cross(f); }

  typedef MotionTpl<double,0> Motion;


  ///////////////   BiasZero  ///////////////
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
    typedef const Vector3 ConstAngular_t;
    typedef Vector3 Linear_t;
    typedef const Vector3 ConstLinear_t;
    typedef Eigen::Quaternion<double,0> Quaternion_t;
    typedef SE3Tpl<double,0> SE3;
    typedef ForceTpl<double,0> Force;
    typedef MotionTpl<double,0> Motion;
    typedef Symmetric3Tpl<double,0> Symmetric3;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits BiasZero

  struct BiasZero : public MotionBase< BiasZero >
  {
    SPATIAL_TYPEDEF_NO_TEMPLATE(BiasZero);
    operator Motion () const { return Motion::Zero(); }
  }; // struct BiasZero

inline const Motion & operator+( const Motion& v, const BiasZero&) { return v; }
inline const Motion & operator+ ( const BiasZero&,const Motion& v) { return v; }

} // namespace se3

#endif // ifndef __se3_motion_hpp__
