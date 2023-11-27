//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_force_base_hpp__
#define __pinocchio_force_base_hpp__

namespace pinocchio
{
  /**
   * @brief      Base interface for forces representation.
   * @details    The Class implements all
   * \ingroup pinocchio_spatial
   *
   *  This class hierarchy represents a spatial force, e.g. a spatial impulse or force associated to a body.
   *  The spatial force is the mathematical representation of \f$ se^{*}(3) \f$, the dual of \f$ se(3) \f$.
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
    
    /** \returns true if the component of the linear and angular part of the Spatial Force are approximately equal to zero, within the precision given by prec.
     */
    bool isZero(const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return derived().isZero_impl(prec); }
    
    /** \brief Copies the Derived Force into *this
     *  \return a reference to *this
     */
    Derived & operator=(const ForceBase<Derived> & other)
    { return derived().setFrom(other.derived()); }
    
    /**
     * \brief Replaces *this by *this + other.
     * \return a reference to *this
     */
    Derived & operator+= (const ForceBase<Derived> & phi) { return derived().__pequ__(phi.derived()); }
    
    /**
     * \brief Replaces *this by *this - other.
     * \return a reference to *this
     */
    Derived & operator-= (const ForceBase<Derived> & phi) { return derived().__mequ__(phi.derived()); }
    
    /** \return an expression of the sum of *this and other
     */
    Derived operator+(const ForceBase<Derived> & phi) const { return derived().__plus__(phi.derived()); }
    
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
    Derived operator-(const ForceBase<Derived> & phi) const { return derived().__minus__(phi.derived()); }
    
    /** \return the dot product of *this with m     *
     */
    template<typename MotionDerived>
    Scalar dot(const MotionDense<MotionDerived> & m) const { return derived().dot(m.derived()); }
    
    
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
    typename SE3GroupAction<Derived>::ReturnType
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
    typename SE3GroupAction<Derived>::ReturnType
    se3ActionInverse(const SE3Tpl<S2,O2> & m) const
    { return derived().se3ActionInverse_impl(m); }
    
    template<typename M1>
    typename MotionAlgebraAction<Derived,M1>::ReturnType
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
  
} // namespace pinocchio

#endif // ifndef __pinocchio_force_base_hpp__
