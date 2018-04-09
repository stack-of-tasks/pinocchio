//
// Copyright (c) 2015-2018 CNRS
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

#ifndef __se3_force_base_hpp__
#define __se3_force_base_hpp__

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
  public:
    FORCE_TYPEDEF_TPL(Derived);
    
    Derived & derived() { return *static_cast<Derived*>(this); }
    const Derived& derived() const { return *static_cast<const Derived*>(this); }
    
    /**
     * @brief      Return the angular part of the force vector
     *
     * @return     The 3D vector associated to the angular part of the 6D force vector
     */
    ConstAngularType angular() const { return derived().angular_impl(); }
    
    /**
     * @brief      Return the linear part of the force vector
     *
     * @return     The 3D vector associated to the linear part of the 6D force vector
     */
    ConstLinearType linear() const { return derived().linear_impl(); }
    
    /// \copydoc ForceBase::angular
    AngularType angular() { return derived().angular_impl(); }
    
    /// \copydoc ForceBase::linear
    LinearType linear() { return derived().linear_impl(); }
    
    
    /**
     * @brief      Set the angular part of the force vector
     *
     * @tparam V3Like A vector 3 like type.
     *
     * @param[in]  n
     */
    template<typename V3Like>
    void angular(const Eigen::MatrixBase<V3Like> & n)
    { derived().angular_impl(n.derived()); }
    
    /**
     * @brief      Set the linear part of the force vector
     *
     * @tparam V3Like A vector 3 like type.
     *
     * @param[in]  f
     */
    template<typename V3Like>
    void linear(const Eigen::MatrixBase<V3Like> & f)
    { derived().linear_impl(f.derived()); }
    
    /**
     * @brief      Return the force as an Eigen vector.
     *
     * @return     The 6D vector \f$ \phi \f$ such that
     * \f{equation*}
     * {}^{A}\phi = \begin{bmatrix} {}^{A}f \\  {}^{A}\tau \end{bmatrix}
     * \f}
     */
    ToVectorConstReturnType toVector() const { return derived().toVector_impl(); }
    
    /// \copydoc ForceBase::toVector
    ToVectorReturnType toVector() { return derived().toVector_impl(); }
    
    /*
     * @brief C-style cast operator
     * \copydoc ForceBase::toVector
     */
    operator Vector6() const { return toVector(); }
    
    /** \returns true if each coefficients of \c *this and \a other are all exactly equal.
     * \warning When using floating point scalar values you probably should rather use a
     *          fuzzy comparison such as isApprox()
     */
    template<typename F2>
    bool operator==(const ForceBase<F2> & other) const {return derived().isEqual_impl(other.derived());}
    
    /** \returns true if at least one coefficient of \c *this and \a other does not match.
     */
    template<typename F2>
    bool operator!=(const ForceBase<F2> & other) const { return !(derived() == other.derived()); }
    
    /** \returns true if *this is approximately equal to other, within the precision given by prec.
     */
    bool isApprox(const Derived & other, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return derived().isApprox_impl(other, prec); }
    
    /** \brief Copies the Derived Force into *this
     *  \return a reference to *this
     */
    Derived & operator= (const Derived & other) { return derived().__equl__(other); }
    
    /**
     * \brief Replaces *this by *this + other.
     * \return a reference to *this
     */
    Derived & operator+= (const Derived & phi) { return derived().__pequ__(phi); }
    
    /**
     * \brief Replaces *this by *this - other.
     * \return a reference to *this
     */
    Derived & operator-= (const Derived & phi) { return derived().__mequ__(phi); }
    
    /** \return an expression of the sum of *this and other
     */
    Derived operator+(const Derived & phi) const { return derived().__plus__(phi); }
    
    /** \return an expression of *this scaled by the factor alpha
     */
    template<typename OtherScalar>
    ForcePlain operator*(const OtherScalar & alpha) const { return derived().__mult__(alpha); }
    
    /** \return an expression of *this divided by the factor alpha
     */
    template<typename OtherScalar>
    ForcePlain operator/(const OtherScalar & alpha) const { return derived().__div__(alpha); }
    
    /** \return an expression of the opposite of *this
     */
    Derived operator-() const { return derived().__opposite__(); }
    
    /** \return an expression of the difference of *this and phi
     */
    Derived operator-(const Derived & phi) const { return derived().__minus__(phi); }
    
    /** \return the dot product of *this with m     *
     */
    Scalar dot(const Motion & m) const { return derived().dot(m); }
    
    
    /**
     * @brief      Transform from A to B coordinates the Force represented by *this such that
     *             \f{equation*}
     *             {}^{B}f  =  {}^{B}X_A^* * {}^{A}f
     *             \f}
     *
     * @param[in]  m     The rigid transformation \f$ {}^{B}m_A \f$ whose coordinates transform for forces is
     *                   {}^{B}X_A^*
     *
     * @return     an expression of the force expressed in the new coordinates
     */
    template<typename S2, int O2>
    typename internal::SE3GroupAction<Derived>::ReturnType
    se3Action(const SE3Tpl<S2,O2> & m) const
    { return derived().se3Action_impl(m); }
    
    /**
     * @brief      Transform from B to A coordinates the Force represented by *this such that
     *             \f{equation*}
     *             {}^{A}f  =  {}^{A}X_B^* * {}^{A}f
     *             \f}
     *
     * @param[in]  m     The rigid transformation \f$ {}^{B}m_A \f$ whose coordinates transform for forces is
     *                   {}^{B}X_A^*
     *
     * @return     an expression of the force expressed in the new coordinates
     */
    template<typename S2, int O2>
    typename internal::SE3GroupAction<Derived>::ReturnType
    se3ActionInverse(const SE3Tpl<S2,O2> & m) const
    { return derived().se3ActionInverse_impl(m); }
    
    template<typename M1>
    typename internal::MotionAlgebraAction<Derived,M1>::ReturnType
    motionAction(const MotionDense<M1> & v) const
    {
      return derived().motionAction(v.derived());
    }
    
    void disp(std::ostream & os) const { derived().disp_impl(os); }
    friend std::ostream & operator << (std::ostream & os, const ForceBase<Derived> & X)
    {
      X.disp(os);
      return os;
    }
    
  }; // class ForceBase
  
} // namespace se3

#endif // ifndef __se3_force_base_hpp__
