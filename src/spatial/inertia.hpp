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

#ifndef __se3_inertia_hpp__
#define __se3_inertia_hpp__

#include "pinocchio/spatial/symmetric3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"



namespace se3
{

  template< class Derived>
  class InertiaBase
  {
  protected:

    typedef Derived  Derived_t;
    SPATIAL_TYPEDEF_TEMPLATE(Derived_t);

  public:
    Derived_t & derived() { return *static_cast<Derived_t*>(this); }
    const Derived_t& derived() const { return *static_cast<const Derived_t*>(this); }

    Scalar_t           mass()    const { return static_cast<const Derived_t*>(this)->mass(); }
    const Vector3 &    lever()   const { return static_cast<const Derived_t*>(this)->lever(); }
    const Symmetric3 & inertia() const { return static_cast<const Derived_t*>(this)->inertia(); }

    Matrix6 matrix() const
    {
      return derived().matrix_impl();
    }

    operator Matrix6 () const { return matrix(); }


    Derived_t& operator= (const Derived_t& clone){return derived().__equl__(clone);}
    Derived_t& operator== (const Derived_t& other){return derived().isEqual(other);}
    Derived_t& operator+= (const Derived_t & Yb) { return derived().__pequ__(Yb); }
    Derived_t operator+(const Derived_t & Yb) const { return derived().__plus__(Yb); }
    Force operator*(const Motion & v) const    { return derived().__mult__(v); }

    /// aI = aXb.act(bI)
    Derived_t se3Action(const SE3 & M) const
    {
      return derived().se3Action_impl(M);
    }

    /// bI = aXb.actInv(aI)
    Derived_t se3ActionInverse(const SE3 & M) const
    {
      return derived().se3ActionInverse_impl(M);
    }

    void disp(std::ostream & os) const
    {
      static_cast<const Derived_t*>(this)->disp_impl(os);
    }

    friend std::ostream & operator << (std::ostream & os,const InertiaBase<Derived_t> & X)
    { 
      X.disp(os);
      return os;
    }

  }; // class InertiaBase


  template<typename T, int U>
  struct traits< InertiaTpl<T, U> >
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
  }; // traits InertiaTpl

  template<typename _Scalar, int _Options>
  class InertiaTpl : public InertiaBase< InertiaTpl< _Scalar, _Options > >
  {
  public:
    friend class InertiaBase< InertiaTpl< _Scalar, _Options > >;
    SPATIAL_TYPEDEF_TEMPLATE(InertiaTpl);
    
  public:
    // Constructors
    InertiaTpl() : m(), c(), I() {}

    InertiaTpl(Scalar_t m_, 
     const Vector3 &c_, 
     const Matrix3 &I_)
    : m(m_),
    c(c_),
    I(I_)
    {

    }

    InertiaTpl(Scalar_t _m, 
     const Vector3 &_c, 
     const Symmetric3 &_I)
    : m(_m),
    c(_c),
    I(_I)
    {

    }
    InertiaTpl(const InertiaTpl & clone)  // Clone constructor for std::vector 
    : m(clone.m),
    c(clone.c),
    I(clone.I)    
    {

    }

    template<typename S2,int O2>
    InertiaTpl( const InertiaTpl<S2,O2> & clone )
    : m(clone.mass()),
    c(clone.lever()),
    I(clone.inertia().matrix())
    {

    }



    // Initializers
    static InertiaTpl Zero() 
    {
      return InertiaTpl(0., 
                        Vector3::Zero(), 
                        Symmetric3::Zero());
    }

    static InertiaTpl Identity() 
    {
      return InertiaTpl(1., 
                        Vector3::Zero(), 
                        Symmetric3::Identity());
    }

    static InertiaTpl Random()
    {
        // We have to shoot "I" definite positive and not only symmetric.
      return InertiaTpl(Eigen::internal::random<Scalar_t>()+1,
                        Vector3::Random(),
                        Symmetric3::RandomPositive());
    }


    Matrix6 matrix_impl() const
    {
      Matrix6 M;
      const Matrix3 & c_cross = (skew(c));
      M.template block<3,3>(LINEAR, LINEAR ).setZero ();
      M.template block<3,3>(LINEAR, LINEAR ).diagonal ().fill (m);
      M.template block<3,3>(ANGULAR,LINEAR ) = m * c_cross;
      M.template block<3,3>(LINEAR, ANGULAR) = -M.template block<3,3> (ANGULAR, LINEAR);
      M.template block<3,3>(ANGULAR,ANGULAR) = (Matrix3)(I - M.template block<3,3>(ANGULAR, LINEAR) * c_cross);

      return M;
    }

    // Arithmetic operators
    InertiaTpl& __equl__ (const InertiaTpl& clone)
    {
      m=clone.m; c=clone.c; I=clone.I;
      return *this;
    }

    // Requiered by std::vector boost::python bindings. 
    bool isEqual( const InertiaTpl& Y2 ) 
    { 
      return (m==Y2.m) && (c==Y2.c) && (I==Y2.I);
    }

    InertiaTpl __plus__(const InertiaTpl &Yb) const
    {
      /* Y_{a+b} = ( m_a+m_b,
       *             (m_a*c_a + m_b*c_b ) / (m_a + m_b),
       *             I_a + I_b - (m_a*m_b)/(m_a+m_b) * AB_x * AB_x )
       */

      const double & mab = m+Yb.m;
      const Vector3 & AB = (c-Yb.c).eval();
      return InertiaTpl( mab,
                         (m*c+Yb.m*Yb.c)/mab,
                         I+Yb.I - (m*Yb.m/mab)* typename Symmetric3::SkewSquare(AB));
    }

    InertiaTpl& __pequ__(const InertiaTpl &Yb)
    {
      const InertiaTpl& Ya = *this;
      const double & mab = Ya.m+Yb.m;
      const Vector3 & AB = (Ya.c-Yb.c).eval();
      c *= m; c += Yb.m*Yb.c; c /= mab;
      I += Yb.I; I -= (Ya.m*Yb.m/mab)* typename Symmetric3::SkewSquare(AB);
      m  = mab;
      return *this;
    }

    Force __mult__(const Motion &v) const 
    {
      const Vector3 & mcxw = m*c.cross(v.angular());
      return Force( m*v.linear()-mcxw,
                    m*c.cross(v.linear()) + I*v.angular() - c.cross(mcxw) );
    }

    // Getters
    Scalar_t           mass()    const { return m; }
    const Vector3 &    lever()   const { return c; }
    const Symmetric3 & inertia() const { return I; }




    /// aI = aXb.act(bI)
    InertiaTpl se3Action_impl(const SE3 & M) const
    {
      /* The multiplication RIR' has a particular form that could be used, however it
       * does not seems to be more efficient, see http://stackoverflow.com/questions/
       * 13215467/eigen-best-way-to-evaluate-asa-transpose-and-store-the-result-in-a-symmetric .*/
       return InertiaTpl( m,
                          M.translation()+M.rotation()*c,
                          I.rotate(M.rotation()) );
     }

    ///bI = aXb.actInv(aI)
    InertiaTpl se3ActionInverse_impl(const SE3 & M) const
    {
      return InertiaTpl(m,
                        M.rotation().transpose()*(c-M.translation()),
                        I.rotate(M.rotation().transpose()) );
    }

    Force vxiv( const Motion& v ) const 
    {
      const Vector3 & mcxw = m*c.cross(v.angular());
      const Vector3 & mv_mcxw = m*v.linear()-mcxw;
      return Force( v.angular().cross(mv_mcxw),
                    v.angular().cross(c.cross(mv_mcxw)+I*v.angular())-v.linear().cross(mcxw) );
    }

    void disp_impl(std::ostream &os) const
    {
      os  << "m =" << m << ";\n"
      << "c = [\n" << c.transpose() << "]';\n"
      << "I = [\n" << (Matrix3)I << "];";
    }

  private:
    Scalar_t m;
    Vector3 c;
    Symmetric3 I;
    
  }; // class InertiaTpl

  typedef InertiaTpl<double,0> Inertia;
    
} // namespace se3

#endif // ifndef __se3_inertia_hpp__
