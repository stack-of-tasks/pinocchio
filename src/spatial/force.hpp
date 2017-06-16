//
// Copyright (c) 2015 CNRS
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

#ifndef __se3_force_hpp__
#define __se3_force_hpp__

#include <Eigen/Core>
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/macros.hpp"
#include "pinocchio/spatial/se3.hpp"

/** \addtogroup Force_group Force
 *
 *  This module represents a spatial force, e.g. a spatial impulse or force associated to a body.
 *  The spatial force is the mathematical representation of \f$ se^{*}(3) \f$, the dual of \f$ se(3) \f$. 
 *  
 *
 */

namespace se3
{


  /**
   * @brief      Base interface for forces representation.
   * @details    The Class implements all 
   * \ingroup Force_group
   * 
   * @tparam     Derived  { description }
   */
  template< class Derived>
  class ForceBase
  {
  protected:

    typedef Derived  Derived_t;
    SPATIAL_TYPEDEF_TEMPLATE(Derived_t);
    
  public:
    Derived_t & derived() { return *static_cast<Derived_t*>(this); }
    const Derived_t& derived() const { return *static_cast<const Derived_t*>(this); }


    /**
     * @brief      Return the angular part of the force vector
     *
     * @return     The 3D vector associated to the angular part of the 6D force vector
     */
    ConstAngular_t angular() const { return derived().angular_impl(); }

    /**
     * @brief      Return the linear part of the force vector
     *
     * @return     The 3D vector associated to the linear part of the 6D force vector
     */
    ConstLinear_t linear() const { return derived().linear_impl(); }
    
    /// \copydoc ForceBase::angular
    Angular_t angular() { return derived().angular_impl(); }

    /// \copydoc ForceBase::linear
    Linear_t linear() { return derived().linear_impl(); }


    /**
     * @brief      Set the angular part of the force vector
     *
     * @param[in]  n
     */
    void angular(const Vector3 & n) { derived().angular_impl(n); }

    /**
     * @brief      Set the linear part of the force vector
     *
     * @param[in]  f
     */
    void linear(const Vector3 & f) { derived().linear_impl(f); }

    /**
     * @brief      Return the force
     *
     * @return     The 6D vector \f$ \phi \f$ such that 
     * \f{equation*}
     * \leftidx{^A}\phi = \begin{bmatrix} \leftidx{^A}f \\  \leftidx{^A}\tau \end{bmatrix}
     * \f}
     */
    const Vector6 & toVector() const { return derived().toVector_impl(); }

    /// \copydoc ForceBase::toVector
    Vector6 & toVector() { return derived().toVector_impl(); }
   
    /*
     * @brief C-style cast operator
     * \copydoc ForceBase::toVector
     */
    operator Vector6 () const { return toVector(); }
    
//    void disp(std::ostream & os) const
//    {
//      static_cast<const Derived_t*>(this)->disp_impl(os);
//    }


    /** \returns true if each coefficients of \c *this and \a other are all exactly equal.
     * \warning When using floating point scalar values you probably should rather use a
     *          fuzzy comparison such as isApprox()
     */
    bool operator== (const Derived_t & other) const {return derived().isEqual(other);}
    
    /** \returns true if at least one coefficient of \c *this and \a other does not match.
     */
    bool operator!=(const Derived_t & other) const { return !(*this == other); }
    
    /** \returns true if *this is approximately equal to other, within the precision given by prec.
     */
    bool isApprox(const Derived & other, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return derived().isApprox_impl(other, prec); }
    
    /** \brief Copies the Derived Force into *this
     *  \return a reference to *this
     */
    Derived_t & operator= (const Derived_t & other) { return derived().__equl__(other); }

    /** 
     * \brief Replaces *this by *this + other.
     * \return a reference to *this
     */
    Derived_t & operator+= (const Derived_t & phi) { return derived().__pequ__(phi); }
    
    /** 
     * \brief Replaces *this by *this - other.
     * \return a reference to *this
     */
    Derived_t & operator-= (const Derived_t & phi) { return derived().__mequ__(phi); }

    /** \return an expression of the sum of *this and other
     */
    Derived_t operator+(const Derived_t & phi) const { return derived().__plus__(phi); }

    /** \return an expression of *this scaled by the double factor a
     */
    Derived_t operator*(double a) const    { return derived().__mult__(a); }

    /** \return an expression of the opposite of *this
     */
    Derived_t operator-() const { return derived().__minus__(); }

    /** \return an expression of the difference of *this and phi
     */
    Derived_t operator-(const Derived_t & phi) const { return derived().__minus__(phi); }
    
    /** \return the dot product of *this with m     *
     */
    Scalar dot(const Motion & m) const { return static_cast<Derived_t*>(this)->dot(m); }


    /**
     * @brief      Transform from A to B coordinates the Force represented by *this such that
     *             \f{equation*}
     *             \leftidx{^B}f  =  \leftidx{^B}X_A^* * \leftidx{^A}f
     *             \f}
     * 
     * @param[in]  m     The rigid transformation \f$ \leftidx{^B}m_A \f$ whose coordinates transform for forces is
     *                   \leftidx{^B}X_A^*  
     *
     * @return     an expression of the force expressed in the new coordinates
     */
    Derived_t se3Action(const SE3 & m) const { return derived().se3Action_impl(m); }

    /**
     * @brief      Transform from B to A coordinates the Force represented by *this such that
     *             \f{equation*}
     *             \leftidx{^A}f  =  \leftidx{^A}X_B^* * \leftidx{^A}f
     *             \f}
     * 
     * @param[in]  m     The rigid transformation \f$ \leftidx{^B}m_A \f$ whose coordinates transform for forces is
     *                   \leftidx{^B}X_A^*  
     *
     * @return     an expression of the force expressed in the new coordinates
     */
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
    typedef T Scalar;
    typedef Eigen::Matrix<T,3,1,U> Vector3;
    typedef Eigen::Matrix<T,4,1,U> Vector4;
    typedef Eigen::Matrix<T,6,1,U> Vector6;
    typedef Eigen::Matrix<T,3,3,U> Matrix3;
    typedef Eigen::Matrix<T,4,4,U> Matrix4;
    typedef Eigen::Matrix<T,6,6,U> Matrix6;
    typedef typename Vector6::template FixedSegmentReturnType<3>::Type Linear_t;
    typedef typename Vector6::template FixedSegmentReturnType<3>::Type Angular_t;
    typedef typename Vector6::template ConstFixedSegmentReturnType<3>::Type ConstLinear_t;
    typedef typename Vector6::template ConstFixedSegmentReturnType<3>::Type ConstAngular_t;
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


  /**
   * @brief      Concreate Class representing a force
   *
   * \ingroup Force_group
   * @tparam     _Scalar   { description }
   * @tparam     _Options  { description }
   */
  template<typename _Scalar, int _Options>
  class ForceTpl : public ForceBase< ForceTpl< _Scalar, _Options > >
  {

  public:
    friend class ForceBase< ForceTpl< _Scalar, _Options > >;
    SPATIAL_TYPEDEF_TEMPLATE(ForceTpl);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ForceTpl() : data() {}

    template<typename F3,typename N3>
    ForceTpl(const Eigen::MatrixBase<F3> & f,const Eigen::MatrixBase<N3> & n)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(F3);
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(F3,3);
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(N3);
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(N3,3);
      data << f, n;
    }

    template<typename F6>
    explicit ForceTpl(const Eigen::MatrixBase<F6> & f)
    : data(f)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(F6);
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(F6,6);
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
      Vector3 Rf (m.rotation()*linear_impl());
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
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(F6);
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(F6,6);
      data = phi;
      return *this;
    }

    ForceTpl & __equl__(const ForceTpl & other) { data = other.data; return *this; }
    ForceTpl & __pequ__ (const ForceTpl & phi) { data += phi.data; return *this; }
    ForceTpl & __mequ__ (const ForceTpl & phi) { data -= phi.data; return *this; }
    ForceTpl __plus__(const ForceTpl & phi) const { return ForceTpl(data + phi.data); }
    ForceTpl __mult__(const double a) const { return ForceTpl(a*data); }
    ForceTpl __minus__() const { return ForceTpl(-data); }
    ForceTpl __minus__(const ForceTpl & phi) const { return ForceTpl(data - phi.data); }
    
    bool isApprox_impl(const ForceTpl & other, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return data.isApprox(other.data, prec); }

    /// \internal \copydoc ForceBase::angular
    ConstAngular_t angular_impl() const { return data.template segment<3> (ANGULAR); }
    /// \internal \copydoc ForceBase::angular
    Angular_t angular_impl() { return data.template segment<3> (ANGULAR); }
    /// \internal \copydoc ForceBase::angular(const Vector3 &)
    void angular_impl(const Vector3 & n) { data.template segment<3> (ANGULAR) = n; }
    /// \internal \copydoc ForceBase::linear
    ConstLinear_t linear_impl() const { return data.template segment<3> (LINEAR);}
    /// \internal \copydoc ForceBase::linear
    Linear_t linear_impl() { return data.template segment<3> (LINEAR);}
    /// \internal \copydoc ForceBase::linear(const Vector3 &)
    void linear_impl(const Vector3 & f) { data.template segment<3> (LINEAR) = f; }
    
    Scalar dot(const Motion & m) const { return data.dot(m.toVector()); }

  protected:
    Vector6 data;

  }; // class ForceTpl

} // namespace se3

#endif // ifndef __se3_force_hpp__

