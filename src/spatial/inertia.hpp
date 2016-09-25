//
// Copyright (c) 2015-2016 CNRS
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#include <Eigen/Core>
#include <iostream>

#include "pinocchio/spatial/symmetric3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/skew.hpp"

/** \addtogroup Inertia_group 
 *
 *  This module represents a spatial inertia tensor of a rigid body. It defines the relationship between its velocity and momentum.
 *  It's a mapping from $M^6$ to  $F^6$ 
 *
 */

namespace se3
{

  /**
   * @brief     Base class for Inertia
   * 
   * @details   This class implements all the API for Intertia classes
   * @ingroup   Inertia_group
   *
   * @tparam     Derived  is the derived type, e.g an InertiaTpl 
   */
  template< class Derived>
  class InertiaBase
  {
  protected:

    typedef Derived  Derived_t;
    SPATIAL_TYPEDEF_TEMPLATE(Derived_t);

  public:
    Derived_t & derived() { return *static_cast<Derived_t*>(this); }
    const Derived_t & derived() const { return *static_cast<const Derived_t*>(this); }

    /**
     * @brief      Return the mass of the rigid body
     *
     * @return     The mass as a Scalar (double for the moment)
     */
    Scalar           mass()    const { return static_cast<const Derived_t*>(this)->mass(); }

    /// \copydoc InertiaBase::mass
    Scalar &         mass() { return static_cast<const Derived_t*>(this)->mass(); }

    /**
     * @brief      Return the lever from the point where is expressed the inertia matrix and the center of the joint to which to inertia is appended
     *
     * @return     The 3D vector representeing the lever
     */
    const Vector3 &    lever()   const { return static_cast<const Derived_t*>(this)->lever(); }

    /// @copydoc InertiaBase::lever
    Vector3 &          lever() { return static_cast<const Derived_t*>(this)->lever(); }
    
    /**
     * @brief      The Inertia matrix (3*3) as expressed in the center of mass of the body
     *
     * @return     The inertia matrix as a sparse Symmetric3
     */
    const Symmetric3 & inertia() const { return static_cast<const Derived_t*>(this)->inertia(); }
    /// @copydoc InertiaBase::inertia
    Symmetric3 &       inertia() { return static_cast<const Derived_t*>(this)->inertia(); }


    /**
     * @brief      The spatial inertia tensor expressed in the center of the supporting joint ( general form of the inertia matrix in Plücker coordinates ) 
     *
     * @return     The 6*6 matrix
     */
    Matrix6 matrix() const { return derived().matrix_impl(); }

    /**
     * @brief      C-style cast to 6*6 Matrix
     *
     * @sa         InertiaBase::matrix
     */
    operator Matrix6 () const { return matrix(); }

    Derived_t& operator= (const Derived_t& clone){return derived().__equl__(clone);}
    bool operator== (const Derived_t& other) const {return derived().isEqual(other);}
    Derived_t& operator+= (const Derived_t & Yb) { return derived().__pequ__(Yb); }
    Derived_t operator+(const Derived_t & Yb) const { return derived().__plus__(Yb); }
    Force operator*(const Motion & v) const    { return derived().__mult__(v); }

    Scalar vtiv(const Motion & v) const { return derived().vtiv_impl(v); }

    void setZero() { derived().setZero(); }
    void setIdentity() { derived().setIdentity(); }
    void setRandom() { derived().setRandom(); }

    /// aI = aXb.act(bI)
    Derived_t se3Action(const SE3 & M) const { return derived().se3Action_impl(M); }

    /// bI = aXb.actInv(aI)
    Derived_t se3ActionInverse(const SE3 & M) const { return derived().se3ActionInverse_impl(M); }

    void disp(std::ostream & os) const { static_cast<const Derived_t*>(this)->disp_impl(os); }
    friend std::ostream & operator << (std::ostream & os,const InertiaBase<Derived_t> & X)
    { 
      X.disp(os);
      return os;
    }

  }; // class InertiaBase


  template<typename T, int U>
  struct traits< InertiaTpl<T, U> >
  {
    typedef T Scalar;
    typedef Eigen::Matrix<T,3,1,U> Vector3;
    typedef Eigen::Matrix<T,4,1,U> Vector4;
    typedef Eigen::Matrix<T,6,1,U> Vector6;
    typedef Eigen::Matrix<T,3,3,U> Matrix3;
    typedef Eigen::Matrix<T,4,4,U> Matrix4;
    typedef Eigen::Matrix<T,6,6,U> Matrix6;
    typedef Matrix6 ActionMatrix_t;
    typedef Vector3 Angular_t;
    typedef Vector3 Linear_t;
    typedef const Vector3 ConstAngular_t;
    typedef const Vector3 ConstLinear_t;
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

  /**
   * @brief      Class for inertia tpl.
   *
   * @ingroup    Inertia_group
   * 
   * @tparam     _Scalar   type of scalar ( double or float)
   * @tparam     _Options  Eigen alignment's option
   */
  template<typename _Scalar, int _Options>
  class InertiaTpl : public InertiaBase< InertiaTpl< _Scalar, _Options > >
  {
  public:
    friend class InertiaBase< InertiaTpl< _Scalar, _Options > >;
    SPATIAL_TYPEDEF_TEMPLATE(InertiaTpl);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
  public:
    // Constructors
    InertiaTpl() : m(), c(), I() {}

    InertiaTpl(const Scalar m_, const Vector3 &c_, const Matrix3 &I_)
    : m(m_), c(c_), I(I_)
    {}
    
    InertiaTpl(const Matrix6 & I6)
    {
      assert((I6 - I6.transpose()).isMuchSmallerThan(I6));
      m = I6(LINEAR, LINEAR);
      const Matrix3 & mc_cross = I6.template block <3,3> (ANGULAR,LINEAR);
      c = unSkew(mc_cross);
      c /= m;
      
      Matrix3 I3 (mc_cross * mc_cross);
      I3 /= m;
      I3 += I6.template block<3,3>(ANGULAR,ANGULAR);
      I = Symmetric3(I3);
    }

    InertiaTpl(Scalar _m, 
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
    
    void setZero() { m = 0.; c.setZero(); I.setZero(); }

    static InertiaTpl Identity() 
    {
      return InertiaTpl(1., 
                        Vector3::Zero(), 
                        Symmetric3::Identity());
    }
    
    void setIdentity () { m = 1.; c.setZero(); I.setIdentity(); }

    static InertiaTpl Random()
    {
        // We have to shoot "I" definite positive and not only symmetric.
      return InertiaTpl(Eigen::internal::random<Scalar>()+1,
                        Vector3::Random(),
                        Symmetric3::RandomPositive());
    }

    static InertiaTpl FromEllipsoid(
        const Scalar m, const Scalar x, const Scalar y, const Scalar z)
    {
      Scalar a = m * (y*y + z*z) / 5;
      Scalar b = m * (x*x + z*z) / 5;
      Scalar c = m * (y*y + x*x) / 5;
      return InertiaTpl(m, Vector3::Zero(), Symmetric3(a, 0, b, 0, 0, c));
    }

    static InertiaTpl FromCylinder(
        const Scalar m, const Scalar r, const Scalar l)
    {
      Scalar a = m * (r*r / 4 + l*l / 12);
      Scalar c = m * (r*r / 2);
      return InertiaTpl(m, Vector3::Zero(), Symmetric3(a, 0, a, 0, 0, c));
    }

    static InertiaTpl FromBox(
        const Scalar m, const Scalar x, const Scalar y, const Scalar z)
    {
      Scalar a = m * (y*y + z*z) / 12;
      Scalar b = m * (x*x + z*z) / 12;
      Scalar c = m * (y*y + x*x) / 12;
      return InertiaTpl(m, Vector3::Zero(), Symmetric3(a, 0, b, 0, 0, c));
    }

    
    void setRandom()
    {
      m = static_cast<Scalar> (std::rand()) / static_cast<Scalar> (RAND_MAX);
      c.setRandom(); I.setRandom();
    }

    Matrix6 matrix_impl() const
    {
      Matrix6 M;
      const Matrix3 & c_cross = (skew(c));
      M.template block<3,3>(LINEAR, LINEAR ).setZero ();
      M.template block<3,3>(LINEAR, LINEAR ).diagonal ().fill (m);
      M.template block<3,3>(ANGULAR,LINEAR ) = m * c_cross;
      M.template block<3,3>(LINEAR, ANGULAR) = -M.template block<3,3> (ANGULAR, LINEAR);
      M.template block<3,3>(ANGULAR,ANGULAR) = I - M.template block<3,3>(ANGULAR, LINEAR) * c_cross;

      return M;
    }

    // Arithmetic operators
    InertiaTpl& __equl__ (const InertiaTpl& clone)
    {
      m=clone.m; c=clone.c; I=clone.I;
      return *this;
    }

    // Requiered by std::vector boost::python bindings. 
    bool isEqual( const InertiaTpl& Y2 ) const
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
      Force f;
      f.linear() = m*(v.linear() - c.cross(v.angular()));
      f.angular() = c.cross(f.linear()) + I*v.angular();
      return f;
    }
    
    Scalar vtiv_impl(const Motion & v) const
    {
      const Vector3 cxw (c.cross(v.angular()));
      Scalar res = m * (v.linear().squaredNorm() - 2.*v.linear().dot(cxw));
      const Vector3 mcxcxw (-m*c.cross(cxw));
      res += v.angular().dot(mcxcxw);
      res += I.vtiv(v.angular());
      
      return res;
    }

    // Getters
    Scalar           mass()    const { return m; }
    const Vector3 &    lever()   const { return c; }
    const Symmetric3 & inertia() const { return I; }
    
    Scalar &   mass()    { return m; }
    Vector3 &    lever()   { return c; }
    Symmetric3 & inertia() { return I; }

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

    void disp_impl(std::ostream & os) const
    {
      os  << "  m = " << m << "\n"
      << "  c = " << c.transpose() << "\n"
      << "  I = \n" << I.matrix() << "";
    }

  protected:
    Scalar m;
    Vector3 c;
    Symmetric3 I;
    
  }; // class InertiaTpl
    
} // namespace se3

#endif // ifndef __se3_inertia_hpp__
