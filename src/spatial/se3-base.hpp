//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_se3_base_hpp__
#define __pinocchio_se3_base_hpp__

namespace pinocchio
{
  /** The rigid transform aMb can be seen in two ways:
   *
   * - given a point p expressed in frame B by its coordinate vector Bp, aMb
   * computes its coordinates in frame A by Ap = aMb Bp.
   * - aMb displaces a solid S centered at frame A into the solid centered in
   * B. In particular, the origin of A is displaced at the origin of B: $^aM_b
   * ^aA = ^aB$.
   
   * The rigid displacement is stored as a rotation matrix and translation vector by:
   * aMb (x) =  aRb*x + aAB
   * where aAB is the vector from origin A to origin B expressed in coordinates A.
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
    
    ActionMatrixType toActionMatrix() const
    {
      return derived().toActionMatrix_impl();
    }
    operator ActionMatrixType() const { return toActionMatrix(); }
    
    ActionMatrixType toDualActionMatrix() const
    { return derived().toDualActionMatrix_impl(); }
    
    void disp(std::ostream & os) const
    {
      static_cast<const Derived*>(this)->disp_impl(os);
    }
    
    typename internal::SE3GroupAction<Derived>::ReturnType
    operator*(const Derived & m2) const
    { return derived().__mult__(m2); }
    
    /// ay = aXb.act(by)
    template<typename D>
    typename internal::SE3GroupAction<D>::ReturnType
    act(const D & d) const
    {
      return derived().act_impl(d);
    }
    
    /// by = aXb.actInv(ay)
    template<typename D> typename internal::SE3GroupAction<D>::ReturnType
    actInv(const D & d) const
    {
      return derived().actInv_impl(d);
    }
    
    bool operator==(const Derived & other) const
    { return derived().__equal__(other); }
    
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
    
  }; // struct SE3Base
  
} // namespace pinocchio

#endif // ifndef __pinocchio_se3_base_hpp__
