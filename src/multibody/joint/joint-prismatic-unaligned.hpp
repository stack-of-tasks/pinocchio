//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_joint_prismatic_unaligned_hpp__
#define __pinocchio_joint_prismatic_unaligned_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-translation.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/math/matrix.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options=0> struct MotionPrismaticUnalignedTpl;
  typedef MotionPrismaticUnalignedTpl<double> MotionPrismaticUnaligned;
  
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

  template<typename _Scalar, int _Options>
  struct traits< MotionPrismaticUnalignedTpl<_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,4,4,Options> Matrix4;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector6) ToVectorConstReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef Vector3 AngularType;
    typedef Vector3 LinearType;
    typedef const Vector3 ConstAngularType;
    typedef const Vector3 ConstLinearType;
    typedef Matrix6 ActionMatrixType;
    typedef Matrix4 HomogeneousMatrixType;
    typedef MotionTpl<Scalar,Options> MotionPlain;
    typedef MotionPlain PlainReturnType;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits MotionPrismaticUnalignedTpl

  template<typename _Scalar, int _Options>
  struct MotionPrismaticUnalignedTpl
  : MotionBase < MotionPrismaticUnalignedTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MOTION_TYPEDEF_TPL(MotionPrismaticUnalignedTpl);

    MotionPrismaticUnalignedTpl() {}
    
    template<typename Vector3Like, typename S2>
    MotionPrismaticUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis,
                                const S2 & v)
    : m_axis(axis), m_v(v)
    { EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3); }

    inline PlainReturnType plain() const
    {
      return PlainReturnType(m_axis*m_v,
                             PlainReturnType::Vector3::Zero());
    }
    
    template<typename OtherScalar>
    MotionPrismaticUnalignedTpl __mult__(const OtherScalar & alpha) const
    {
      return MotionPrismaticUnalignedTpl(m_axis,alpha*m_v);
    }
    
    template<typename Derived>
    void addTo(MotionDense<Derived> & other) const
    {
      other.linear() += m_axis * m_v;
    }
    
    template<typename Derived>
    void setTo(MotionDense<Derived> & other) const
    {
      other.linear().noalias() = m_axis*m_v;
      other.angular().setZero();
    }

    template<typename S2, int O2, typename D2>
    void se3Action_impl(const SE3Tpl<S2,O2> & m, MotionDense<D2> & v) const
    {
      v.linear().noalias() = m_v * (m.rotation() * m_axis); // TODO: check efficiency
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
      v.linear().noalias() = m_v * (m.rotation().transpose() * m_axis);
      
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
      mout.linear().noalias() = v.angular().cross(m_axis);
      mout.linear() *= m_v;
      
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
    
    bool isEqual_impl(const MotionPrismaticUnalignedTpl & other) const
    {
      return m_axis == other.m_axis && m_v == other.m_v;
    }
    
    const Scalar & linearRate() const { return m_v; }
    Scalar & linearRate() { return m_v; }
    
    const Vector3 & axis() const { return m_axis; }
    Vector3 & axis() { return m_axis; }
    
  protected:
    
    Vector3 m_axis;
    Scalar m_v;
  }; // struct MotionPrismaticUnalignedTpl

  template<typename Scalar, int Options, typename MotionDerived>
  inline typename MotionDerived::MotionPlain
  operator+(const MotionPrismaticUnalignedTpl<Scalar,Options> & m1, const MotionDense<MotionDerived> & m2)
  {
    typedef typename MotionDerived::MotionPlain ReturnType;
    return ReturnType(m1.linearRate() * m1.axis() + m2.linear(), m2.angular());
  }
  
  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1,
            const MotionPrismaticUnalignedTpl<S2,O2> & m2)
  {
    return m2.motionAction(m1);
  }

  template<typename Scalar, int Options> struct ConstraintPrismaticUnalignedTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< ConstraintPrismaticUnalignedTpl<_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
    typedef MotionPrismaticUnalignedTpl<Scalar,Options> JointMotion;
    typedef Eigen::Matrix<Scalar,1,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,1,Options> DenseBase;
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
    
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
  }; // traits ConstraintPrismaticUnalignedTpl
  
  template<typename Scalar, int Options>
  struct SE3GroupAction< ConstraintPrismaticUnalignedTpl<Scalar,Options> >
  { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
  
  template<typename Scalar, int Options, typename MotionDerived>
  struct MotionAlgebraAction< ConstraintPrismaticUnalignedTpl<Scalar,Options>,MotionDerived >
  { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };

  template<typename Scalar, int Options, typename ForceDerived>
  struct ConstraintForceOp< ConstraintPrismaticUnalignedTpl<Scalar,Options>, ForceDerived>
  {
    typedef typename traits< ConstraintPrismaticUnalignedTpl<Scalar,Options> >::Vector3 Vector3;
    typedef Eigen::Matrix<typename PINOCCHIO_EIGEN_DOT_PRODUCT_RETURN_TYPE(Vector3,typename ForceDense<ForceDerived>::ConstAngularType),1,1,Options> ReturnType;
  };
  
  template<typename Scalar, int Options, typename ForceSet>
  struct ConstraintForceSetOp< ConstraintPrismaticUnalignedTpl<Scalar,Options>, ForceSet>
  {
    typedef typename traits< ConstraintPrismaticUnalignedTpl<Scalar,Options> >::Vector3 Vector3;
    typedef typename MatrixMatrixProduct<Eigen::Transpose<const Vector3>,
    typename Eigen::MatrixBase<const ForceSet>::template NRowsBlockXpr<3>::Type
    >::type ReturnType;
  };

  template<typename _Scalar, int _Options>
  struct ConstraintPrismaticUnalignedTpl
  : ConstraintBase< ConstraintPrismaticUnalignedTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(ConstraintPrismaticUnalignedTpl)
    
    enum { NV = 1 };
    
    typedef typename traits<ConstraintPrismaticUnalignedTpl>::Vector3 Vector3;

    ConstraintPrismaticUnalignedTpl() {}
    
    template<typename Vector3Like>
    ConstraintPrismaticUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : m_axis(axis)
    { EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3); }

    template<typename Vector1Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector1Like> & v) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector1Like,1);
      return JointMotion(m_axis,v[0]);
    }
    
    template<typename S1, int O1>
    typename SE3GroupAction<ConstraintPrismaticUnalignedTpl>::ReturnType
    se3Action(const SE3Tpl<S1,O1> & m) const
    {
      typename SE3GroupAction<ConstraintPrismaticUnalignedTpl>::ReturnType res;
      MotionRef<DenseBase> v(res);
      v.linear().noalias() = m.rotation()*m_axis;
      v.angular().setZero();
      return res;
    }
    
    template<typename S1, int O1>
    typename SE3GroupAction<ConstraintPrismaticUnalignedTpl>::ReturnType
    se3ActionInverse(const SE3Tpl<S1,O1> & m) const
    {
      typename SE3GroupAction<ConstraintPrismaticUnalignedTpl>::ReturnType res;
      MotionRef<DenseBase> v(res);
      v.linear().noalias() = m.rotation().transpose()*m_axis;
      v.angular().setZero();
      return res;
    }
    
    int nv_impl() const { return NV; }
    
    struct TransposeConst
    {
      const ConstraintPrismaticUnalignedTpl & ref;
      TransposeConst(const ConstraintPrismaticUnalignedTpl & ref) : ref(ref) {}
      
      template<typename ForceDerived>
      typename ConstraintForceOp<ConstraintPrismaticUnalignedTpl,ForceDerived>::ReturnType
      operator* (const ForceDense<ForceDerived> & f) const
      {
        typedef typename ConstraintForceOp<ConstraintPrismaticUnalignedTpl,ForceDerived>::ReturnType ReturnType;
        ReturnType res;
        res[0] = ref.axis().dot(f.linear());
        return res;
      }
      
      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename ForceSet>
      typename ConstraintForceSetOp<ConstraintPrismaticUnalignedTpl,ForceSet>::ReturnType
      operator*(const Eigen::MatrixBase<ForceSet> & F)
      {
        EIGEN_STATIC_ASSERT(ForceSet::RowsAtCompileTime==6,THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE)
        /* Return ax.T * F[1:3,:] */
        return ref.axis().transpose() * F.template middleRows<3>(LINEAR);
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
      S.template segment<3>(LINEAR) = m_axis;
      S.template segment<3>(ANGULAR).setZero();
      return S;
    }
    
    template<typename MotionDerived>
    DenseBase motionAction(const MotionDense<MotionDerived> & v) const
    {
      DenseBase res;
      res.template segment<3>(LINEAR).noalias() = v.angular().cross(m_axis);
      res.template segment<3>(ANGULAR).setZero();
      
      return res;
    }
      
    const Vector3 & axis() const { return m_axis; }
    Vector3 & axis() { return m_axis; }
    
    bool isEqual(const ConstraintPrismaticUnalignedTpl & other) const
    {
      return m_axis == other.m_axis;
    }
    
  protected:
    
    Vector3 m_axis;
    
  }; // struct ConstraintPrismaticUnalignedTpl
  
  template<typename S1, int O1,typename S2, int O2>
  struct MultiplicationOp<InertiaTpl<S1,O1>, ConstraintPrismaticUnalignedTpl<S2,O2> >
  {
    typedef Eigen::Matrix<S2,6,1,O2> ReturnType;
  };
  
  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1,O1>, ConstraintPrismaticUnalignedTpl<S2,O2> >
    {
      typedef InertiaTpl<S1,O1> Inertia;
      typedef ConstraintPrismaticUnalignedTpl<S2,O2> Constraint;
      typedef typename MultiplicationOp<Inertia,Constraint>::ReturnType ReturnType;
      
      static inline ReturnType run(const Inertia & Y,
                                   const Constraint & cpu)
      {
        ReturnType res;
        /* YS = [ m -mcx ; mcx I-mcxcx ] [ 0 ; w ] = [ mcxw ; Iw -mcxcxw ] */
        const S1 & m                             = Y.mass();
        const typename Inertia::Vector3 & c      = Y.lever();
        
        res.template segment<3>(Constraint::LINEAR).noalias() = m*cpu.axis();
        res.template segment<3>(Constraint::ANGULAR).noalias() = c.cross(res.template segment<3>(Constraint::LINEAR));
        
        return res;
      }
    };
  } // namespace impl
  
  template<typename M6Like, typename Scalar, int Options>
  struct MultiplicationOp<Eigen::MatrixBase<M6Like>, ConstraintPrismaticUnalignedTpl<Scalar,Options> >
  {
    typedef typename SizeDepType<3>::ColsReturn<M6Like>::ConstType M6LikeCols;
    typedef typename Eigen::internal::remove_const<M6LikeCols>::type M6LikeColsNonConst;
    
    typedef ConstraintPrismaticUnalignedTpl<Scalar,Options> Constraint;
    typedef typename Constraint::Vector3 Vector3;
    typedef const typename MatrixMatrixProduct<M6LikeColsNonConst,Vector3>::type ReturnType;
  };
  
  /* [ABA] operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename M6Like, typename Scalar, int Options>
    struct LhsMultiplicationOp<Eigen::MatrixBase<M6Like>, ConstraintPrismaticUnalignedTpl<Scalar,Options> >
    {
      typedef ConstraintPrismaticUnalignedTpl<Scalar,Options> Constraint;
      typedef typename MultiplicationOp<Eigen::MatrixBase<M6Like>,Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Eigen::MatrixBase<M6Like> & Y,
                                   const Constraint & cru)
      {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6Like,6,6);
        return Y.derived().template middleCols<3>(Constraint::LINEAR) * cru.axis();
      }
    };
  } // namespace impl
  
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
    typedef ConstraintPrismaticUnalignedTpl<Scalar,Options> Constraint_t;
    typedef TransformTranslationTpl<Scalar,Options> Transformation_t;
    typedef MotionPrismaticUnalignedTpl<Scalar,Options> Motion_t;
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
  struct traits< JointDataPrismaticUnalignedTpl<Scalar,Options> >
  { typedef JointPrismaticUnalignedTpl<Scalar,Options> JointDerived; };

  template<typename _Scalar, int _Options>
  struct JointDataPrismaticUnalignedTpl
  : public JointDataBase< JointDataPrismaticUnalignedTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointPrismaticUnalignedTpl<_Scalar,_Options> JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(JointDerived);
    PINOCCHIO_JOINT_DATA_BASE_DEFAULT_ACCESSOR
    
    Transformation_t M;
    Constraint_t S;
    Motion_t v;
    Bias_t c;

    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataPrismaticUnalignedTpl()
    : M(Transformation_t::Vector3::Zero())
    , S(Constraint_t::Vector3::Zero())
    , v(Constraint_t::Vector3::Zero(),(Scalar)0)
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    {}
    
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
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    
    typedef JointModelBase<JointModelPrismaticUnalignedTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;
    
    typedef Eigen::Matrix<Scalar,3,1,_Options> Vector3;
    
    JointModelPrismaticUnalignedTpl() {}
    JointModelPrismaticUnalignedTpl(const Scalar & x,
                                    const Scalar & y,
                                    const Scalar & z)
    : axis(x,y,z)
    {
      axis.normalize();
      assert(isUnitary(axis) && "Translation axis is not unitary");
    }
    
    template<typename Vector3Like>
    JointModelPrismaticUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : axis(axis)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Vector3Like);
      assert(isUnitary(axis) && "Translation axis is not unitary");
    }

    JointDataDerived createData() const { return JointDataDerived(axis); }
    
    const std::vector<bool> hasConfigurationLimit() const
    {
      return {true};
    }
    
    const std::vector<bool> hasConfigurationLimitInTangent() const
    {
      return {true};
    }

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
      data.v.linearRate() = v;
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
