//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_constraint_hpp__
#define __pinocchio_constraint_hpp__


#include "pinocchio/macros.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/act-on-set.hpp"


// S   : v   \in M^6              -> v_J \in lie(Q) ~= R^nv
// S^T : f_J \in lie(Q)^* ~= R^nv -> f    \in F^6


namespace pinocchio
{
  template<int _Dim, typename _Scalar, int _Options=0> class ConstraintTpl;

  template<class Derived>
  class ConstraintBase
  {
  protected:
    typedef typename traits<Derived>::Scalar Scalar;
    typedef typename traits<Derived>::JointMotion JointMotion;
    typedef typename traits<Derived>::JointForce JointForce;
    typedef typename traits<Derived>::DenseBase DenseBase;
    typedef typename traits<Derived>::MatrixReturnType MatrixReturnType;
    typedef typename traits<Derived>::ConstMatrixReturnType ConstMatrixReturnType;

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
    
    void disp(std::ostream & os) const { static_cast<const Derived*>(this)->disp_impl(os); }
    friend std::ostream & operator << (std::ostream & os,const ConstraintBase<Derived> & X)
    {
      X.disp(os);
      return os;
    }
    
    template<typename MotionDerived>
    DenseBase motionAction(const MotionDense<MotionDerived> & v) const
    { return derived().motionAction(v); }

  }; // class ConstraintBase

  template<int _Dim, typename _Scalar, int _Options>
  struct traits< ConstraintTpl<_Dim, _Scalar, _Options> >
  {
    typedef _Scalar Scalar;
    enum {
      LINEAR = 0,
      ANGULAR = 3,
      Options = _Options,
      Dim = _Dim
    };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,4,1,Options> Vector4;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
    typedef Eigen::Matrix<Scalar,4,4,Options> Matrix4;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef Matrix3 Angular_t;
    typedef Vector3 Linear_t;
    typedef const Matrix3 ConstAngular_t;
    typedef const Vector3 ConstLinear_t;
    typedef Matrix6 ActionMatrix_t;
    typedef Eigen::Quaternion<Scalar,Options> Quaternion_t;
    typedef SE3Tpl<Scalar,Options> SE3;
    typedef ForceTpl<Scalar,Options> Force;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef Symmetric3Tpl<Scalar,Options> Symmetric3;
    
    typedef MotionTpl<Scalar,Options> JointMotion;
    typedef Eigen::Matrix<Scalar,Dim,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,Dim,Options> DenseBase;
    
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(DenseBase) ConstMatrixReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_TYPE(DenseBase) MatrixReturnType;

  }; // traits ConstraintTpl

  namespace internal
  {  
    template<int Dim, typename Scalar, int Options>
    struct SE3GroupAction< ConstraintTpl<Dim,Scalar,Options> >
    { typedef Eigen::Matrix<Scalar,6,Dim> ReturnType; };
    
    template<int Dim, typename Scalar, int Options, typename MotionDerived>
    struct MotionAlgebraAction< ConstraintTpl<Dim,Scalar,Options>, MotionDerived >
    { typedef Eigen::Matrix<Scalar,6,Dim> ReturnType; };
  }

  template<int _Dim, typename _Scalar, int _Options>
  class ConstraintTpl
  : public ConstraintBase< ConstraintTpl<_Dim,_Scalar,_Options> >
  { 
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef ConstraintBase<ConstraintTpl> Base;

    friend class ConstraintBase<ConstraintTpl>;
    SPATIAL_TYPEDEF_TEMPLATE(ConstraintTpl);
    
    typedef typename Base::JointMotion JointMotion;
    typedef typename Base::JointForce JointForce;
    typedef typename Base::DenseBase DenseBase;
    typedef typename Base::ConstMatrixReturnType ConstMatrixReturnType;
    typedef typename Base::MatrixReturnType MatrixReturnType;
    
    enum { NV = _Dim, Options = _Options };
    
    using Base::nv;

  public:
    template<typename D>
    explicit ConstraintTpl(const Eigen::MatrixBase<D> & _S) : S(_S)
    {
      // There is currently a bug in Eigen/Core/util/StaticAssert.h in the use of the full namespace path
      // TODO
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DenseBase, D);
    }

    ConstraintTpl() : S() 
    {
      EIGEN_STATIC_ASSERT(_Dim!=Eigen::Dynamic,YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR)
    }
    
    // It is only valid for dynamics size
    explicit ConstraintTpl(const int dim) : S(6,dim)
    {
      EIGEN_STATIC_ASSERT(_Dim==Eigen::Dynamic,YOU_CALLED_A_FIXED_SIZE_METHOD_ON_A_DYNAMIC_SIZE_MATRIX_OR_VECTOR)
    }

    template<typename VectorLike>
    JointMotion __mult__(const Eigen::MatrixBase<VectorLike> & vj) const
    {
      return JointMotion(S*vj);
    }

    struct Transpose
    {
      const ConstraintTpl & ref;
      Transpose( const ConstraintTpl & ref ) : ref(ref) {}

      template<typename Derived>
      JointForce operator*(const ForceDense<Derived> & f) const
      { return (ref.S.transpose()*f.toVector()).eval(); }

      template<typename D>
      typename Eigen::Matrix<Scalar,NV,Eigen::Dynamic>
      operator*(const Eigen::MatrixBase<D> & F)
      {
        return (ref.S.transpose()*F).eval();
      }

    };
    
    Transpose transpose() const { return Transpose(*this); }

    MatrixReturnType matrix_impl() { return S; }
    ConstMatrixReturnType matrix_impl() const { return S; }

    int nv_impl() const { return (int)S.cols(); }

    template<typename S2,int O2>
    friend typename ConstraintTpl<_Dim,_Scalar,_Options>::DenseBase
    operator*(const InertiaTpl<S2,O2> & Y, const ConstraintTpl & S)
    {
      typedef typename ConstraintTpl::DenseBase ReturnType;
      ReturnType res(6,S.nv());
      motionSet::inertiaAction(Y,S.S,res);
      return res;
    }
    
    template<typename S2,int O2>
    friend Eigen::Matrix<_Scalar,6,_Dim>
    operator*(const Eigen::Matrix<S2,6,6,O2> & Ymatrix, const ConstraintTpl & S)
    {
      typedef Eigen::Matrix<_Scalar,6,_Dim> ReturnType;
      return ReturnType(Ymatrix*S.matrix());
      
    }

    DenseBase se3Action(const SE3 & m) const
    {
      DenseBase res(6,nv());
      motionSet::se3Action(m,S,res);
      return res;
    }
    
    DenseBase se3ActionInverse(const SE3 & m) const
    {
      DenseBase res(6,nv());
      motionSet::se3ActionInverse(m,S,res);
      return res;
    }
    
    template<typename MotionDerived>
    DenseBase motionAction(const MotionDense<MotionDerived> & v) const
    {
      DenseBase res(6,nv());
      motionSet::motionAction(v,S,res);
      return res;
    }

    void disp_impl(std::ostream & os) const { os << "S =\n" << S << std::endl;}
    
  protected:
    DenseBase S;
  }; // class ConstraintTpl

  typedef ConstraintTpl<1,double,0> Constraint1d;
  typedef ConstraintTpl<3,double,0> Constraint3d;
  typedef ConstraintTpl<6,double,0> Constraint6d;
  typedef ConstraintTpl<Eigen::Dynamic,double,0> ConstraintXd;

} // namespace pinocchio

#endif // ifndef __pinocchio_constraint_hpp__
