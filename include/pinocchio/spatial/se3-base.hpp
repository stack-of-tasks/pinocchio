//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_se3_base_hpp__
#define __pinocchio_se3_base_hpp__

namespace pinocchio
{
  /** \brief Base class for rigid transformation.
   *
   * The rigid transform aMb can be seen in two ways:
   *
   * - given a point p expressed in frame B by its coordinate vector \f$ ^bp \f$, \f$ ^aM_b \f$
   * computes its coordinates in frame A by \f$ ^ap = {}^aM_b {}^bp \f$.
   * - \f$ ^aM_b \f$ displaces a solid S centered at frame A into the solid centered in
   * B. In particular, the origin of A is displaced at the origin of B:
   * \f$^aM_b {}^aA = {}^aB \f$.
   
   * The rigid displacement is stored as a rotation matrix and translation vector by:
   * \f$ ^aM_b x = {}^aR_b x + {}^aAB \f$
   * where \f$^aAB\f$ is the vector from origin A to origin B expressed in coordinates A.
   *
   * \cheatsheet \f$ {}^aM_c = {}^aM_b {}^bM_c \f$
   *
   * \ingroup pinocchio_spatial
   */
  template<class Derived>
  struct SE3Base
  {
    PINOCCHIO_SE3_TYPEDEF_TPL(Derived);
    
    Derived & derived() { return *static_cast<Derived*>(this); }
    const Derived& derived() const { return *static_cast<const Derived*>(this); }
    
    ConstAngularRef rotation() const  { return derived().rotation_impl(); }
    ConstLinearRef translation() const  { return derived().translation_impl(); }
    AngularRef rotation() { return derived().rotation_impl(); }
    LinearRef translation() { return derived().translation_impl(); }
    void rotation(const AngularType & R) { derived().rotation_impl(R); }
    void translation(const LinearType & t) { derived().translation_impl(t); }
    
    HomogeneousMatrixType toHomogeneousMatrix() const
    {
      return derived().toHomogeneousMatrix_impl();
    }
    operator HomogeneousMatrixType() const { return toHomogeneousMatrix(); }
    
    /**
     * @brief The action matrix \f$ {}^aX_b \f$ of \f$ {}^aM_b \f$.
     *
     * With \f$ {}^aM_b = \left( \begin{array}{cc} R & t \\ 0 & 1 \\ \end{array} \right) \f$,
     * \f[
     * {}^aX_b = \left( \begin{array}{cc} R & \hat{t} R \\ 0 & R \\ \end{array} \right)
     * \f]
     *
     * \cheatsheet \f$ {}^a\nu_c = {}^aX_b {}^b\nu_c \f$
     */
    ActionMatrixType toActionMatrix() const
    {
      return derived().toActionMatrix_impl();
    }
    operator ActionMatrixType() const { return toActionMatrix(); }
    
    /**
     * @brief The action matrix \f$ {}^bX_a \f$ of \f$ {}^aM_b \f$.
     * \sa toActionMatrix()
     */
    ActionMatrixType toActionMatrixInverse() const
    {
      return derived().toActionMatrixInverse_impl();
    }
    
    ActionMatrixType toDualActionMatrix() const
    { return derived().toDualActionMatrix_impl(); }
    
    void disp(std::ostream & os) const
    {
      static_cast<const Derived*>(this)->disp_impl(os);
    }
    
    typename SE3GroupAction<Derived>::ReturnType
    operator*(const Derived & m2) const
    { return derived().__mult__(m2); }
    
    /// ay = aXb.act(by)
    template<typename D>
    typename SE3GroupAction<D>::ReturnType
    act(const D & d) const
    {
      return derived().act_impl(d);
    }
    
    /// by = aXb.actInv(ay)
    template<typename D> typename SE3GroupAction<D>::ReturnType
    actInv(const D & d) const
    {
      return derived().actInv_impl(d);
    }
    
    bool operator==(const Derived & other) const
    { return derived().isEqual(other); }
    
    bool operator!=(const Derived & other) const
    { return !(*this == other); }
    
    bool isApprox(const Derived & other, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return derived().isApprox_impl(other, prec);
    }
    
    friend std::ostream & operator <<(std::ostream & os,const SE3Base<Derived> & X)
    {
      X.disp(os);
      return os;
    }
    
    ///
    /// \returns true if *this is approximately equal to the identity placement, within the precision given by prec.
    ///
    bool isIdentity(const typename traits<Derived>::Scalar & prec = Eigen::NumTraits<typename traits<Derived>::Scalar>::dummy_precision()) const
    {
      return derived().isIdentity(prec);
    }
   
    ///
    /// \returns true if the rotational part of *this is a rotation matrix (normalized columns), within the precision given by prec.
    ///
    bool isNormalized(const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return derived().isNormalized(prec);
    }
    
    ///
    /// \brief Normalize *this in such a way the rotation part of *this lies on SO(3).
    ///
    void normalize()
    {
      derived().normalize();
    }
    
    ///
    /// \returns a Normalized version of *this, in such a way the rotation part of the returned transformation lies on SO(3).
    ///
    PlainType normalized() const
    {
      derived().normalized();
    }
    
  }; // struct SE3Base
  
} // namespace pinocchio

#endif // ifndef __pinocchio_se3_base_hpp__
