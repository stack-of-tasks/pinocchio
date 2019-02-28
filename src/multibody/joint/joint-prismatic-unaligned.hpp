//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_joint_prismatic_unaligned_hpp__
#define __pinocchio_joint_prismatic_unaligned_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-translation.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options=0> struct MotionPrismaticUnalignedTpl;
  typedef MotionPrismaticUnalignedTpl<double> MotionPrismaticUnaligned;
  
  namespace internal
  {
    template<typename Scalar, int Options>
    struct SE3GroupAction< MotionPrismaticUnalignedTpl<Scalar,Options> >
    {
      typedef MotionTpl<Scalar,Options> ReturnType;
    };
    
    template<typename Scalar, int Options, typename MotionDerived>
    struct MotionAlgebraAction< MotionPrismaticUnalignedTpl<Scalar,Options>, MotionDerived>
    {
      typedef MotionTpl<Scalar,Options> ReturnType;
    };
  }
  
  template<typename _Scalar, int _Options>
  struct traits< MotionPrismaticUnalignedTpl<_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector6) ToVectorConstReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef Vector3 AngularType;
    typedef Vector3 LinearType;
    typedef const Vector3 ConstAngularType;
    typedef const Vector3 ConstLinearType;
    typedef Matrix6 ActionMatrixType;
    typedef MotionTpl<Scalar,Options> MotionPlain;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits MotionPrismaticUnalignedTpl

  template<typename _Scalar, int _Options>
  struct MotionPrismaticUnalignedTpl : MotionBase < MotionPrismaticUnalignedTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MOTION_TYPEDEF_TPL(MotionPrismaticUnalignedTpl);

    MotionPrismaticUnalignedTpl() {}
    
    template<typename Vector3Like, typename S2>
    MotionPrismaticUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis,
                                const S2 rate)
    : axis(axis), rate(rate)
    { EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3); }

//    operator MotionPlain() const
//    { return MotionPlain(axis*rate,MotionPlain::Vector3::Zero());}
    
    template<typename Derived>
    void addTo(MotionDense<Derived> & other) const
    {
      other.linear() += axis * rate;
    }
    
    template<typename Derived>
    void setTo(MotionDense<Derived> & other) const
    {
      other.linear().noalias() = axis*rate;
      other.angular().setZero();
    }

    template<typename S2, int O2, typename D2>
    void se3Action_impl(const SE3Tpl<S2,O2> & m, MotionDense<D2> & v) const
    {
      v.linear().noalias() = rate * (m.rotation() * axis); // TODO: check efficiency
      v.angular().setZero();
    }
    
    template<typename S2, int O2>
    MotionPlain se3Action_impl(const SE3Tpl<S2,O2> & m) const
    {
      MotionPlain res;
      se3Action_impl(m,res);
      return res;
    }
    
    template<typename S2, int O2, typename D2>
    void se3ActionInverse_impl(const SE3Tpl<S2,O2> & m, MotionDense<D2> & v) const
    {
      // Linear
      v.linear().noalias() = rate * (m.rotation().transpose() * axis);
      
      // Angular
      v.angular().setZero();
    }
    
    template<typename S2, int O2>
    MotionPlain se3ActionInverse_impl(const SE3Tpl<S2,O2> & m) const
    {
      MotionPlain res;
      se3ActionInverse_impl(m,res);
      return res;
    }
    
    template<typename M1, typename M2>
    void motionAction(const MotionDense<M1> & v, MotionDense<M2> & mout) const
    {
      // Linear
      mout.linear().noalias() = v.angular().cross(axis);
      mout.linear() *= rate;
      
      // Angular
      mout.angular().setZero();
    }
    
    template<typename M1>
    MotionPlain motionAction(const MotionDense<M1> & v) const
    {
      MotionPlain res;
      motionAction(v,res);
      return res;
    }
    
    // data
    Vector3 axis;
    Scalar rate;
  }; // struct MotionPrismaticUnalignedTpl

  template<typename Scalar, int Options, typename MotionDerived>
  inline typename MotionDerived::MotionPlain
  operator+(const MotionPrismaticUnalignedTpl<Scalar,Options> & m1, const MotionDense<MotionDerived> & m2)
  {
    typedef typename MotionDerived::MotionPlain ReturnType;
    return ReturnType(m1.rate*m1.axis + m2.linear(), m2.angular());
  }

  template<typename Scalar, int Options> struct ConstraintPrismaticUnaligned;
  
  template<typename _Scalar, int _Options>
  struct traits< ConstraintPrismaticUnaligned<_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
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
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
    typedef MotionPrismaticUnalignedTpl<Scalar,Options> JointMotion;
    typedef Eigen::Matrix<Scalar,1,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,1,Options> DenseBase;
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
  }; // traits ConstraintPrismaticUnaligned

  template<typename _Scalar, int _Options>
  struct ConstraintPrismaticUnaligned : ConstraintBase< ConstraintPrismaticUnaligned<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SPATIAL_TYPEDEF_TEMPLATE(ConstraintPrismaticUnaligned);
    enum { NV = 1, Options = _Options };
    
    typedef typename traits<ConstraintPrismaticUnaligned>::JointMotion JointMotion;
    typedef typename traits<ConstraintPrismaticUnaligned>::JointForce JointForce;
    typedef typename traits<ConstraintPrismaticUnaligned>::DenseBase DenseBase;
    
    ConstraintPrismaticUnaligned() {}
    
    template<typename Vector3Like>
    ConstraintPrismaticUnaligned(const Eigen::MatrixBase<Vector3Like> & axis)
    : axis(axis)
    { EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3); }

    template<typename Vector1Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector1Like> & v) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector1Like,1);
      return JointMotion(axis,v[0]);
    }
    
    template<typename S1, int O1>
    Vector6 se3Action(const SE3Tpl<S1,O1> & m) const
    {
      Vector6 res;
      res.template head<3>().noalias() = m.rotation()*axis;
      res.template tail<3>().setZero();
      return res;
    }
    
    int nv_impl() const { return NV; }
    
    struct TransposeConst
    {
      typedef typename traits<ConstraintPrismaticUnaligned>::Scalar Scalar;
      typedef typename traits<ConstraintPrismaticUnaligned>::Force Force;
      typedef typename traits<ConstraintPrismaticUnaligned>::Vector6 Vector6;
      
      const ConstraintPrismaticUnaligned & ref;
      TransposeConst(const ConstraintPrismaticUnaligned & ref) : ref(ref) {}
      
      template<typename Derived>
      Eigen::Matrix<
      typename PINOCCHIO_EIGEN_DOT_PRODUCT_RETURN_TYPE(Vector3,typename ForceDense<Derived>::ConstLinearType),
      1,1>
      operator* (const ForceDense<Derived> & f) const
      {
        typedef Eigen::Matrix<
        typename PINOCCHIO_EIGEN_DOT_PRODUCT_RETURN_TYPE(Vector3,typename ForceDense<Derived>::ConstLinearType),
        1,1> ReturnType;
        
        ReturnType res; res[0] = ref.axis.dot(f.linear());
        return res;
      }
      
      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename D>
      friend
#if EIGEN_VERSION_AT_LEAST(3,2,90)
      const Eigen::Product<
      Eigen::Transpose<const Vector3>,
      typename Eigen::MatrixBase<const D>::template NRowsBlockXpr<3>::Type
      >
#else
      const typename Eigen::ProductReturnType<
      Eigen::Transpose<const Vector3>,
      typename Eigen::MatrixBase<const D>::template NRowsBlockXpr<3>::Type
      >::Type
#endif
      operator* (const TransposeConst & tc, const Eigen::MatrixBase<D> & F)
      {
        EIGEN_STATIC_ASSERT(D::RowsAtCompileTime==6,THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE)
        /* Return ax.T * F[1:3,:] */
        return tc.ref.axis.transpose () * F.template topRows<3> ();
      }
      
    };
    TransposeConst transpose() const { return TransposeConst(*this); }
    
    
    /* CRBA joint operators
     *   - ForceSet::Block = ForceSet
     *   - ForceSet operator* (Inertia Y,Constraint S)
     *   - MatrixBase operator* (Constraint::Transpose S, ForceSet::Block)
     *   - SE3::act(ForceSet::Block)
     */
    DenseBase matrix_impl() const
    {
      DenseBase S;
      S << axis, Vector3::Zero();
      return S;
    }
    
    template<typename MotionDerived>
    DenseBase motionAction(const MotionDense<MotionDerived> & v) const
    {
      DenseBase res;
      res << v.angular().cross(axis), Vector3::Zero();
      
      return res;
    }
    
    // data
    Vector3 axis;
    
  }; // struct ConstraintPrismaticUnaligned


  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1,
            const MotionPrismaticUnalignedTpl<S2,O2> & m2)
  {
    return m2.motionAction(m1);
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline Eigen::Matrix<S1,6,1>
  operator*(const InertiaTpl<S1,O1> & Y, const ConstraintPrismaticUnaligned<S2,O2> & cpu)
  {
    typedef InertiaTpl<S1,O1> Inertia;
    /* YS = [ m -mcx ; mcx I-mcxcx ] [ v ; 0 ] = [ mv ; mcxv ] */
    const S1 & m                             = Y.mass();
    const typename Inertia::Vector3 & c      = Y.lever();
    
    Eigen::Matrix<S1,6,1> res;
    res.template head<3>().noalias() = m*cpu.axis;
    res.template tail<3>() = c.cross(res.template head<3>());
    return res;
  }
  
  /* [ABA] Y*S operator (Inertia Y,Constraint S) */
  template<typename M6, typename S2, int O2>
#if EIGEN_VERSION_AT_LEAST(3,2,90)
  const typename Eigen::Product<
  Eigen::Block<const M6,6,3>,
  typename ConstraintPrismaticUnaligned<S2,O2>::Vector3,
  Eigen::DefaultProduct>
#else
  const typename Eigen::ProductReturnType<
  Eigen::Block<const M6,6,3>,
  const typename ConstraintPrismaticUnaligned<S2,O2>::Vector3
  >::Type
#endif
  operator*(const Eigen::MatrixBase<M6> & Y, const ConstraintPrismaticUnaligned<S2,O2> & cpu)
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6,6,6);
    return Y.template block<6,3> (0,Inertia::LINEAR) * cpu.axis;
  }

  
  namespace internal
  {
    template<typename Scalar, int Options>
    struct SE3GroupAction< ConstraintPrismaticUnaligned<Scalar,Options> >
    { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
    
    template<typename Scalar, int Options, typename MotionDerived>
    struct MotionAlgebraAction< ConstraintPrismaticUnaligned<Scalar,Options>,MotionDerived >
    { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
  }
  
  template<typename Scalar, int Options> struct JointPrismaticUnalignedTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< JointPrismaticUnalignedTpl<_Scalar,_Options> >
  {
    enum {
      NQ = 1,
      NV = 1
    };
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef JointDataPrismaticUnalignedTpl<Scalar,Options> JointDataDerived;
    typedef JointModelPrismaticUnalignedTpl<Scalar,Options> JointModelDerived;
    typedef ConstraintPrismaticUnaligned<Scalar,Options> Constraint_t;
    typedef TransformTranslationTpl<Scalar,Options> Transformation_t;
    typedef MotionPrismaticUnalignedTpl<Scalar,Options> Motion_t;
    typedef BiasZeroTpl<Scalar,Options> Bias_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<Scalar,6,NV,Options> U_t;
    typedef Eigen::Matrix<Scalar,NV,NV,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> UD_t;
    
    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE
    
    typedef Eigen::Matrix<Scalar,NQ,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1,Options> TangentVector_t;
  };

  template<typename Scalar, int Options>
  struct traits< JointDataPrismaticUnalignedTpl<Scalar,Options> >
  { typedef JointPrismaticUnalignedTpl<Scalar,Options> JointDerived; };

  template<typename _Scalar, int _Options>
  struct JointDataPrismaticUnalignedTpl
  : public JointDataBase< JointDataPrismaticUnalignedTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointPrismaticUnalignedTpl<_Scalar,_Options> JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE;
    PINOCCHIO_JOINT_DATA_BASE_DEFAULT_ACCESSOR
    
    Transformation_t M;
    Constraint_t S;
    Motion_t v;
    Bias_t c;

    F_t F;
    
    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataPrismaticUnalignedTpl() {}
    
    template<typename Vector3Like>
    JointDataPrismaticUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : M()
    , S(axis)
    , v(axis,(Scalar)NAN)
    , U(), Dinv(), UDinv()
    {}

    static std::string classname() { return std::string("JointDataPrismaticUnaligned"); }
    std::string shortname() const { return classname(); }
    
  }; // struct JointDataPrismaticUnalignedTpl
  
  template<typename Scalar, int Options>
  struct traits< JointModelPrismaticUnalignedTpl<Scalar,Options> >
  { typedef JointPrismaticUnalignedTpl<Scalar,Options> JointDerived; };

  PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelPrismaticUnalignedTpl);
  template<typename _Scalar, int _Options>
  struct JointModelPrismaticUnalignedTpl
  : public JointModelBase< JointModelPrismaticUnalignedTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointPrismaticUnalignedTpl<_Scalar,_Options> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE;
    
    typedef JointModelBase<JointModelPrismaticUnalignedTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;
    
    typedef Eigen::Matrix<Scalar,3,1,_Options> Vector3;
    
    JointModelPrismaticUnalignedTpl() {}
    JointModelPrismaticUnalignedTpl(Scalar x, Scalar y, Scalar z)
    : axis(x,y,z)
    {
      axis.normalize();
      assert(axis.isUnitary() && "Translation axis is not unitary");
    }
    
    template<typename Vector3Like>
    JointModelPrismaticUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : axis(axis)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Vector3Like);
      assert(axis.isUnitary() && "Translation axis is not unitary");
    }

    JointDataDerived createData() const { return JointDataDerived(axis); }
    
    using Base::isEqual;
    bool isEqual(const JointModelPrismaticUnalignedTpl & other) const
    {
      return Base::isEqual(other) && axis == other.axis;
    }
    
    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      typedef typename ConfigVector::Scalar Scalar;
      const Scalar & q = qs[idx_q()];

      data.M.translation().noalias() = axis * q;
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data,qs.derived());
      
      typedef typename TangentVector::Scalar S2;
      const S2 & v = vs[idx_v()];
      data.v.rate = v;
    }
    
    template<typename Matrix6Like>
    void calc_aba(JointDataDerived & data, const Eigen::MatrixBase<Matrix6Like> & I, const bool update_I) const
    {
      data.U.noalias() = I.template block<6,3> (0,Inertia::LINEAR) * axis;
      data.Dinv[0] = Scalar(1)/axis.dot(data.U.template segment <3> (Inertia::LINEAR));
      data.UDinv.noalias() = data.U * data.Dinv;
      
      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,I) -= data.UDinv * data.U.transpose();
    }
    
    Scalar finiteDifferenceIncrement() const
    {
      using math::sqrt;
      return sqrt(Eigen::NumTraits<Scalar>::epsilon());
    }

    static std::string classname() { return std::string("JointModelPrismaticUnaligned"); }
    std::string shortname() const { return classname(); }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelPrismaticUnalignedTpl<NewScalar,Options> cast() const
    {
      typedef JointModelPrismaticUnalignedTpl<NewScalar,Options> ReturnType;
      ReturnType res(axis.template cast<NewScalar>());
      res.setIndexes(id(),idx_q(),idx_v());
      return res;
    }
    
    // data
    
    ///
    /// \brief 3d main axis of the joint.
    ///
    Vector3 axis;
  }; // struct JointModelPrismaticUnalignedTpl
  
} //namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options>
  struct has_nothrow_constructor< ::pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_copy< ::pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_constructor< ::pinocchio::JointDataPrismaticUnalignedTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_copy< ::pinocchio::JointDataPrismaticUnalignedTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
}


#endif // ifndef __pinocchio_joint_prismatic_unaligned_hpp__
