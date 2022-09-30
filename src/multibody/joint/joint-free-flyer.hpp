//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_joint_free_flyer_hpp__
#define __pinocchio_joint_free_flyer_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/quaternion.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options> struct ConstraintIdentityTpl;

  template<typename _Scalar, int _Options>
  struct traits< ConstraintIdentityTpl<_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
    typedef MotionTpl<Scalar,Options> JointMotion;
    typedef Eigen::Matrix<Scalar,6,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,6,Options> DenseBase;
    typedef typename Matrix6::IdentityReturnType ConstMatrixReturnType;
    typedef typename Matrix6::IdentityReturnType MatrixReturnType;
  }; // traits ConstraintRevolute


  template<typename _Scalar, int _Options>
  struct ConstraintIdentityTpl
  : ConstraintBase< ConstraintIdentityTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(ConstraintIdentityTpl)
    
    enum { NV = 6 };
    
    template<typename Vector6Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector6Like> & vj) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector6Like,6);
      return JointMotion(vj);
    }
    
    template<typename S1, int O1>
    typename SE3Tpl<S1,O1>::ActionMatrixType
    se3Action(const SE3Tpl<S1,O1> & m) const
    {
      return m.toActionMatrix();
      
    }
    
    template<typename S1, int O1>
    typename SE3Tpl<S1,O1>::ActionMatrixType
    se3ActionInverse(const SE3Tpl<S1,O1> & m) const
    {
      return m.toActionMatrixInverse();
    }
    
    int nv_impl() const { return NV; }
    
    struct TransposeConst
    {
      template<typename Derived>
      typename ForceDense<Derived>::ToVectorConstReturnType
      operator*(const ForceDense<Derived> & phi)
      {  return phi.toVector();  }
      
      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename MatrixDerived>
      typename PINOCCHIO_EIGEN_REF_CONST_TYPE(MatrixDerived)
      operator*(const Eigen::MatrixBase<MatrixDerived> & F)
      {
        return F.derived();
      }
    };
    
    TransposeConst transpose() const { return TransposeConst(); }
    MatrixReturnType matrix_impl() const { return DenseBase::Identity(); }
    
    template<typename MotionDerived>
    typename MotionDerived::ActionMatrixType
    motionAction(const MotionBase<MotionDerived> & v) const
    { return v.toActionMatrix(); }
    
    bool isEqual(const ConstraintIdentityTpl &) const { return true; }
    
  }; // struct ConstraintIdentityTpl
  
  template<typename Scalar, int Options, typename Vector6Like>
  MotionRef<const Vector6Like>
  operator*(const ConstraintIdentityTpl<Scalar,Options> &,
            const Eigen::MatrixBase<Vector6Like> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector6Like,6);
//    typedef typename ConstraintIdentityTpl<Scalar,Options>::Motion Motion;
    typedef MotionRef<const Vector6Like> Motion;
    return Motion(v.derived());
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline typename InertiaTpl<S1,O1>::Matrix6
  operator*(const InertiaTpl<S1,O1> & Y, const ConstraintIdentityTpl<S2,O2> &)
  {
    return Y.matrix();
  }
  
  /* [ABA] Y*S operator*/
  template<typename Matrix6Like, typename S2, int O2>
  inline typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Matrix6Like)
  operator*(const Eigen::MatrixBase<Matrix6Like> & Y, const ConstraintIdentityTpl<S2,O2> &)
  {
    return Y.derived();
  }
  
  template<typename S1, int O1>
  struct SE3GroupAction< ConstraintIdentityTpl<S1,O1> >
  { typedef typename SE3Tpl<S1,O1>::ActionMatrixType ReturnType; };
  
  template<typename S1, int O1, typename MotionDerived>
  struct MotionAlgebraAction< ConstraintIdentityTpl<S1,O1>,MotionDerived >
  { typedef typename SE3Tpl<S1,O1>::ActionMatrixType ReturnType; };

  template<typename Scalar, int Options> struct JointFreeFlyerTpl;

  template<typename _Scalar, int _Options>
  struct traits< JointFreeFlyerTpl<_Scalar,_Options> >
  {
    enum {
      NQ = 7,
      NV = 6
    };
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef JointDataFreeFlyerTpl<Scalar,Options> JointDataDerived;
    typedef JointModelFreeFlyerTpl<Scalar,Options> JointModelDerived;
    typedef ConstraintIdentityTpl<Scalar,Options> Constraint_t;
    typedef SE3Tpl<Scalar,Options> Transformation_t;
    typedef MotionTpl<Scalar,Options> Motion_t;
    typedef MotionZeroTpl<Scalar,Options> Bias_t;

    // [ABA]
    typedef Eigen::Matrix<Scalar,6,NV,Options> U_t;
    typedef Eigen::Matrix<Scalar,NV,NV,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> UD_t;
    
    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE

    typedef Eigen::Matrix<Scalar,NQ,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1,Options> TangentVector_t;
  };
  
  template<typename Scalar, int Options>
  struct traits< JointDataFreeFlyerTpl<Scalar,Options> >
  { typedef JointFreeFlyerTpl<Scalar,Options> JointDerived; };
  
  template<typename Scalar, int Options>
  struct traits< JointModelFreeFlyerTpl<Scalar,Options> >
  { typedef JointFreeFlyerTpl<Scalar,Options> JointDerived; };

  template<typename _Scalar, int _Options>
  struct JointDataFreeFlyerTpl : public JointDataBase< JointDataFreeFlyerTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointFreeFlyerTpl<_Scalar,_Options> JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(JointDerived);
    PINOCCHIO_JOINT_DATA_BASE_DEFAULT_ACCESSOR
    
    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;
    
    JointDataFreeFlyerTpl()
    : M(Transformation_t::Identity())
    , v(Motion_t::Zero())
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Identity())
    {}

    static std::string classname() { return std::string("JointDataFreeFlyer"); }
    std::string shortname() const { return classname(); }
    
  }; // struct JointDataFreeFlyerTpl

  PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelFreeFlyerTpl);
  template<typename _Scalar, int _Options>
  struct JointModelFreeFlyerTpl
  : public JointModelBase< JointModelFreeFlyerTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointFreeFlyerTpl<_Scalar,_Options> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    
    typedef JointModelBase<JointModelFreeFlyerTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;

    JointDataDerived createData() const { return JointDataDerived(); }
    
    const std::vector<bool> hasConfigurationLimit() const
    {
      return {true, true, true, false, false, false, false};
    }

    const std::vector<bool> hasConfigurationLimitInTangent() const
    {
      return {true, true, true, false, false, false};
    }

    template<typename ConfigVectorLike>
    inline void forwardKinematics(Transformation_t & M, const Eigen::MatrixBase<ConfigVectorLike> & q_joint) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,ConfigVectorLike);
      typedef typename Eigen::Quaternion<typename ConfigVectorLike::Scalar,PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorLike)::Options> Quaternion;
      typedef Eigen::Map<const Quaternion> ConstQuaternionMap;

      ConstQuaternionMap quat(q_joint.template tail<4>().data());
      //assert(math::fabs(quat.coeffs().squaredNorm()-1.) <= sqrt(Eigen::NumTraits<typename V::Scalar>::epsilon())); TODO: check validity of the rhs precision
      assert(math::fabs(static_cast<Scalar>(quat.coeffs().squaredNorm()-1)) <= 1e-4);

      M.rotation(quat.matrix());
      M.translation(q_joint.template head<3>());
    }
    
    template<typename Vector3Derived, typename QuaternionDerived>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<Vector3Derived> & trans,
              const typename Eigen::QuaternionBase<QuaternionDerived> & quat) const
    {
      data.M.translation(trans);
      data.M.rotation(quat.matrix());
    }
    
    template<typename ConfigVector>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & data,
              const typename Eigen::PlainObjectBase<ConfigVector> & qs) const
    {
      typedef typename Eigen::Quaternion<typename ConfigVector::Scalar,ConfigVector::Options> Quaternion;
      typedef Eigen::Map<const Quaternion> ConstQuaternionMap;
      
      typename ConfigVector::template ConstFixedSegmentReturnType<NQ>::Type q = qs.template segment<NQ>(idx_q());
      ConstQuaternionMap quat(q.template tail<4>().data());
      
      calc(data,q.template head<3>(),quat);
    }
    
    template<typename ConfigVector>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      typedef typename Eigen::Quaternion<Scalar,Options> Quaternion;
      
      typename ConfigVector::template ConstFixedSegmentReturnType<NQ>::Type q = qs.template segment<NQ>(idx_q());
      const Quaternion quat(q.template tail<4>());
      
      calc(data,q.template head<3>(),quat);
    }
    
    template<typename ConfigVector, typename TangentVector>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data,qs.derived());
      
      data.v = vs.template segment<NV>(idx_v());
    }
    
    template<typename Matrix6Like>
    void calc_aba(JointDataDerived & data,
                  const Eigen::MatrixBase<Matrix6Like> & I,
                  const bool update_I) const
    {
      data.U = I;
      
      // compute inverse
//      data.Dinv.setIdentity();
//      I.llt().solveInPlace(data.Dinv);
      internal::PerformStYSInversion<Scalar>::run(I,data.Dinv);
      
      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,I).setZero();
    }

    static std::string classname() { return std::string("JointModelFreeFlyer"); }
    std::string shortname() const { return classname(); }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelFreeFlyerTpl<NewScalar,Options> cast() const
    {
      typedef JointModelFreeFlyerTpl<NewScalar,Options> ReturnType;
      ReturnType res;
      res.setIndexes(id(),idx_q(),idx_v());
      return res;
    }

  }; // struct JointModelFreeFlyerTpl

} // namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options>
  struct has_nothrow_constructor< ::pinocchio::JointModelFreeFlyerTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_copy< ::pinocchio::JointModelFreeFlyerTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_constructor< ::pinocchio::JointDataFreeFlyerTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_copy< ::pinocchio::JointDataFreeFlyerTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
}

#endif // ifndef __pinocchio_joint_free_flyer_hpp__
