//
// Copyright (c) 2015-2019 CNRS, INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_multibody_constraint_base_hpp__
#define __pinocchio_multibody_constraint_base_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/act-on-set.hpp"

// S   : v   \in M^6              -> v_J \in lie(Q) ~= R^nv
// S^T : f_J \in lie(Q)^* ~= R^nv -> f    \in F^6

#define PINOCCHIO_CONSTRAINT_TYPEDEF_GENERIC(DERIVED,TYPENAME) \
  typedef TYPENAME traits<DERIVED>::Scalar Scalar; \
  typedef TYPENAME traits<DERIVED>::JointMotion JointMotion;  \
  typedef TYPENAME traits<DERIVED>::JointForce JointForce;  \
  typedef TYPENAME traits<DERIVED>::DenseBase DenseBase; \
  typedef TYPENAME traits<DERIVED>::MatrixReturnType MatrixReturnType; \
  typedef TYPENAME traits<DERIVED>::ConstMatrixReturnType ConstMatrixReturnType; \
  enum { LINEAR = traits<DERIVED>::LINEAR, ANGULAR = traits<DERIVED>::ANGULAR };

#define PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(DERIVED) PINOCCHIO_CONSTRAINT_TYPEDEF_GENERIC(DERIVED,typename)
#define PINOCCHIO_CONSTRAINT_TYPEDEF(DERIVED) PINOCCHIO_CONSTRAINT_TYPEDEF_GENERIC(DERIVED,PINOCCHIO_EMPTY_ARG)

namespace pinocchio
{
  
  template<class Derived>
  class ConstraintBase
  {
  protected:
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(Derived)

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Derived & derived() { return *static_cast<Derived*>(this); }
    const Derived & derived() const { return *static_cast<const Derived*>(this); }

    template<typename VectorLike>
    JointMotion operator*(const Eigen::MatrixBase<VectorLike> & vj) const
    { return derived().__mult__(vj); }

    MatrixReturnType matrix() { return derived().matrix_impl(); }
    ConstMatrixReturnType matrix() const  { return derived().matrix_impl(); }
    
    int nv() const { return derived().nv_impl(); }
    
    template<class OtherDerived>
    bool isApprox(const ConstraintBase<OtherDerived> & other,
                  const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return matrix().isApprox(other.matrix(),prec); }
    
    void disp(std::ostream & os) const { derived().disp_impl(os); }
    friend std::ostream & operator << (std::ostream & os,const ConstraintBase<Derived> & X)
    {
      X.disp(os);
      return os;
    }
    
    template<typename MotionDerived>
    DenseBase motionAction(const MotionDense<MotionDerived> & v) const
    { return derived().motionAction(v); }

  }; // class ConstraintBase

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_constraint_base_hpp__
