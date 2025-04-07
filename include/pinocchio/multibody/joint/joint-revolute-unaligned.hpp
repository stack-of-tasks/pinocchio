//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_multibody_joint_revolute_unaligned_hpp__
#define __pinocchio_multibody_joint_revolute_unaligned_hpp__

#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint-motion-subspace.hpp"
#include "pinocchio/spatial/inertia.hpp"

#include "pinocchio/math/matrix.hpp"
#include "pinocchio/math/rotation.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options = context::Options>
  struct MotionRevoluteUnalignedTpl;
  typedef MotionRevoluteUnalignedTpl<context::Scalar> MotionRevoluteUnaligned;

  template<typename Scalar, int Options>
  struct SE3GroupAction<MotionRevoluteUnalignedTpl<Scalar, Options>>
  {
    typedef MotionTpl<Scalar, Options> ReturnType;
  };

  template<typename Scalar, int Options, typename MotionDerived>
  struct MotionAlgebraAction<MotionRevoluteUnalignedTpl<Scalar, Options>, MotionDerived>
  {
    typedef MotionTpl<Scalar, Options> ReturnType;
  };

  template<typename _Scalar, int _Options>
  struct traits<MotionRevoluteUnalignedTpl<_Scalar, _Options>>
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;
    typedef Eigen::Matrix<Scalar, 6, 1, Options> Vector6;
    typedef Eigen::Matrix<Scalar, 4, 4, Options> Matrix4;
    typedef Eigen::Matrix<Scalar, 6, 6, Options> Matrix6;
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector6) ToVectorConstReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef Vector3 AngularType;
    typedef Vector3 LinearType;
    typedef const Vector3 ConstAngularType;
    typedef const Vector3 ConstLinearType;
    typedef Matrix6 ActionMatrixType;
    typedef Matrix4 HomogeneousMatrixType;
    typedef MotionTpl<Scalar, Options> MotionPlain;
    typedef MotionPlain PlainReturnType;
    enum
    {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits MotionRevoluteUnalignedTpl

  template<typename _Scalar, int _Options>
  struct MotionRevoluteUnalignedTpl : MotionBase<MotionRevoluteUnalignedTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MOTION_TYPEDEF_TPL(MotionRevoluteUnalignedTpl);

    MotionRevoluteUnalignedTpl()
    {
    }

    template<typename Vector3Like, typename OtherScalar>
    MotionRevoluteUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis, const OtherScalar & w)
    : m_axis(axis)
    , m_w(w)
    {
    }

    inline PlainReturnType plain() const
    {
      return PlainReturnType(PlainReturnType::Vector3::Zero(), m_axis * m_w);
    }

    template<typename OtherScalar>
    MotionRevoluteUnalignedTpl __mult__(const OtherScalar & alpha) const
    {
      return MotionRevoluteUnalignedTpl(m_axis, alpha * m_w);
    }

    template<typename MotionDerived>
    inline void addTo(MotionDense<MotionDerived> & v) const
    {
      v.angular() += m_axis * m_w;
    }

    template<typename Derived>
    void setTo(MotionDense<Derived> & other) const
    {
      other.linear().setZero();
      other.angular().noalias() = m_axis * m_w;
    }

    template<typename S2, int O2, typename D2>
    void se3Action_impl(const SE3Tpl<S2, O2> & m, MotionDense<D2> & v) const
    {
      // Angular
      v.angular().noalias() = m_w * m.rotation() * m_axis;

      // Linear
      v.linear().noalias() = m.translation().cross(v.angular());
    }

    template<typename S2, int O2>
    MotionPlain se3Action_impl(const SE3Tpl<S2, O2> & m) const
    {
      MotionPlain res;
      se3Action_impl(m, res);
      return res;
    }

    template<typename S2, int O2, typename D2>
    void se3ActionInverse_impl(const SE3Tpl<S2, O2> & m, MotionDense<D2> & v) const
    {
      // Linear
      // TODO: use v.angular() as temporary variable
      Vector3 v3_tmp;
      v3_tmp.noalias() = m_axis.cross(m.translation());
      v3_tmp *= m_w;
      v.linear().noalias() = m.rotation().transpose() * v3_tmp;

      // Angular
      v.angular().noalias() = m.rotation().transpose() * m_axis;
      v.angular() *= m_w;
    }

    template<typename S2, int O2>
    MotionPlain se3ActionInverse_impl(const SE3Tpl<S2, O2> & m) const
    {
      MotionPlain res;
      se3ActionInverse_impl(m, res);
      return res;
    }

    template<typename M1, typename M2>
    void motionAction(const MotionDense<M1> & v, MotionDense<M2> & mout) const
    {
      // Linear
      mout.linear().noalias() = v.linear().cross(m_axis);
      mout.linear() *= m_w;

      // Angular
      mout.angular().noalias() = v.angular().cross(m_axis);
      mout.angular() *= m_w;
    }

    template<typename M1>
    MotionPlain motionAction(const MotionDense<M1> & v) const
    {
      MotionPlain res;
      motionAction(v, res);
      return res;
    }

    bool isEqual_impl(const MotionRevoluteUnalignedTpl & other) const
    {
      return internal::comparison_eq(m_axis, other.m_axis)
             && internal::comparison_eq(m_w, other.m_w);
    }

    const Scalar & angularRate() const
    {
      return m_w;
    }
    Scalar & angularRate()
    {
      return m_w;
    }

    const Vector3 & axis() const
    {
      return m_axis;
    }
    Vector3 & axis()
    {
      return m_axis;
    }

  protected:
    Vector3 m_axis;
    Scalar m_w;

  }; // struct MotionRevoluteUnalignedTpl

  template<typename S1, int O1, typename MotionDerived>
  inline typename MotionDerived::MotionPlain
  operator+(const MotionRevoluteUnalignedTpl<S1, O1> & m1, const MotionDense<MotionDerived> & m2)
  {
    typename MotionDerived::MotionPlain res(m2);
    res += m1;
    return res;
  }

  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1, const MotionRevoluteUnalignedTpl<S2, O2> & m2)
  {
    return m2.motionAction(m1);
  }

  template<typename Scalar, int Options>
  struct JointMotionSubspaceRevoluteUnalignedTpl;

  template<typename _Scalar, int _Options>
  struct traits<JointMotionSubspaceRevoluteUnalignedTpl<_Scalar, _Options>>
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };
    enum
    {
      LINEAR = 0,
      ANGULAR = 3
    };

    typedef MotionRevoluteUnalignedTpl<Scalar, Options> JointMotion;
    typedef Eigen::Matrix<Scalar, 1, 1, Options> JointForce;
    typedef Eigen::Matrix<Scalar, 6, 1, Options> DenseBase;
    typedef Eigen::Matrix<Scalar, 1, 1, Options> ReducedSquaredMatrix;

    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;

    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;

    typedef typename ReducedSquaredMatrix::IdentityReturnType StDiagonalMatrixSOperationReturnType;
  }; // traits JointMotionSubspaceRevoluteUnalignedTpl

  template<typename Scalar, int Options>
  struct SE3GroupAction<JointMotionSubspaceRevoluteUnalignedTpl<Scalar, Options>>
  {
    typedef Eigen::Matrix<Scalar, 6, 1, Options> ReturnType;
  };

  template<typename Scalar, int Options, typename MotionDerived>
  struct MotionAlgebraAction<
    JointMotionSubspaceRevoluteUnalignedTpl<Scalar, Options>,
    MotionDerived>
  {
    typedef Eigen::Matrix<Scalar, 6, 1, Options> ReturnType;
  };

  template<typename Scalar, int Options, typename ForceDerived>
  struct ConstraintForceOp<JointMotionSubspaceRevoluteUnalignedTpl<Scalar, Options>, ForceDerived>
  {
    typedef
      typename traits<JointMotionSubspaceRevoluteUnalignedTpl<Scalar, Options>>::Vector3 Vector3;
    typedef Eigen::Matrix<
      typename PINOCCHIO_EIGEN_DOT_PRODUCT_RETURN_TYPE(
        Vector3, typename ForceDense<ForceDerived>::ConstAngularType),
      1,
      1,
      Options>
      ReturnType;
  };

  template<typename Scalar, int Options, typename ForceSet>
  struct ConstraintForceSetOp<JointMotionSubspaceRevoluteUnalignedTpl<Scalar, Options>, ForceSet>
  {
    typedef
      typename traits<JointMotionSubspaceRevoluteUnalignedTpl<Scalar, Options>>::Vector3 Vector3;
    typedef typename MatrixMatrixProduct<
      Eigen::Transpose<const Vector3>,
      typename Eigen::MatrixBase<const ForceSet>::template NRowsBlockXpr<3>::Type>::type ReturnType;
  };

  template<typename _Scalar, int _Options>
  struct JointMotionSubspaceRevoluteUnalignedTpl
  : JointMotionSubspaceBase<JointMotionSubspaceRevoluteUnalignedTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(JointMotionSubspaceRevoluteUnalignedTpl)

    enum
    {
      NV = 1
    };

    typedef typename traits<JointMotionSubspaceRevoluteUnalignedTpl>::Vector3 Vector3;

    JointMotionSubspaceRevoluteUnalignedTpl()
    {
    }

    template<typename Vector3Like>
    JointMotionSubspaceRevoluteUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : m_axis(axis)
    {
    }

    template<typename Vector1Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector1Like> & v) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector1Like, 1);
      return JointMotion(m_axis, v[0]);
    }

    template<typename S1, int O1>
    typename SE3GroupAction<JointMotionSubspaceRevoluteUnalignedTpl>::ReturnType
    se3Action(const SE3Tpl<S1, O1> & m) const
    {
      typedef
        typename SE3GroupAction<JointMotionSubspaceRevoluteUnalignedTpl>::ReturnType ReturnType;

      /* X*S = [ R pxR ; 0 R ] [ 0 ; a ] = [ px(Ra) ; Ra ] */
      ReturnType res;
      res.template segment<3>(ANGULAR).noalias() = m.rotation() * m_axis;
      res.template segment<3>(LINEAR).noalias() =
        m.translation().cross(res.template segment<3>(ANGULAR));
      return res;
    }

    template<typename S1, int O1>
    typename SE3GroupAction<JointMotionSubspaceRevoluteUnalignedTpl>::ReturnType
    se3ActionInverse(const SE3Tpl<S1, O1> & m) const
    {
      typedef
        typename SE3GroupAction<JointMotionSubspaceRevoluteUnalignedTpl>::ReturnType ReturnType;

      ReturnType res;
      res.template segment<3>(ANGULAR).noalias() = m.rotation().transpose() * m_axis;
      res.template segment<3>(LINEAR).noalias() =
        -m.rotation().transpose() * m.translation().cross(m_axis);
      return res;
    }

    int nv_impl() const
    {
      return NV;
    }

    struct TransposeConst
    : JointMotionSubspaceTransposeBase<JointMotionSubspaceRevoluteUnalignedTpl>
    {
      const JointMotionSubspaceRevoluteUnalignedTpl & ref;
      TransposeConst(const JointMotionSubspaceRevoluteUnalignedTpl & ref)
      : ref(ref)
      {
      }

      template<typename ForceDerived>
      typename ConstraintForceOp<JointMotionSubspaceRevoluteUnalignedTpl, ForceDerived>::ReturnType
      operator*(const ForceDense<ForceDerived> & f) const
      {
        typedef typename ConstraintForceOp<
          JointMotionSubspaceRevoluteUnalignedTpl, ForceDerived>::ReturnType ReturnType;
        ReturnType res;
        res[0] = ref.axis().dot(f.angular());
        return res;
      }

      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename ForceSet>
      typename ConstraintForceSetOp<JointMotionSubspaceRevoluteUnalignedTpl, ForceSet>::ReturnType
      operator*(const Eigen::MatrixBase<ForceSet> & F)
      {
        EIGEN_STATIC_ASSERT(
          ForceSet::RowsAtCompileTime == 6, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE)
        /* Return ax.T * F[3:end,:] */
        return ref.axis().transpose() * F.template middleRows<3>(ANGULAR);
      }
    };

    TransposeConst transpose() const
    {
      return TransposeConst(*this);
    }

    /* CRBA joint operators
     *   - ForceSet::Block = ForceSet
     *   - ForceSet operator* (Inertia Y,Constraint S)
     *   - MatrixBase operator* (Constraint::Transpose S, ForceSet::Block)
     *   - SE3::act(ForceSet::Block)
     */
    DenseBase matrix_impl() const
    {
      DenseBase S;
      S.template segment<3>(LINEAR).setZero();
      S.template segment<3>(ANGULAR) = m_axis;
      return S;
    }

    template<typename MotionDerived>
    typename MotionAlgebraAction<JointMotionSubspaceRevoluteUnalignedTpl, MotionDerived>::ReturnType
    motionAction(const MotionDense<MotionDerived> & m) const
    {
      const typename MotionDerived::ConstLinearType v = m.linear();
      const typename MotionDerived::ConstAngularType w = m.angular();

      DenseBase res;
      res.template segment<3>(LINEAR).noalias() = v.cross(m_axis);
      res.template segment<3>(ANGULAR).noalias() = w.cross(m_axis);

      return res;
    }

    const Vector3 & axis() const
    {
      return m_axis;
    }
    Vector3 & axis()
    {
      return m_axis;
    }

    bool isEqual(const JointMotionSubspaceRevoluteUnalignedTpl & other) const
    {
      return internal::comparison_eq(m_axis, other.m_axis);
    }

  protected:
    Vector3 m_axis;

  }; // struct JointMotionSubspaceRevoluteUnalignedTpl

  template<typename S1, int O1, typename S2, int O2>
  struct MultiplicationOp<InertiaTpl<S1, O1>, JointMotionSubspaceRevoluteUnalignedTpl<S2, O2>>
  {
    typedef Eigen::Matrix<S2, 6, 1, O2> ReturnType;
  };

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1, O1>, JointMotionSubspaceRevoluteUnalignedTpl<S2, O2>>
    {
      typedef InertiaTpl<S1, O1> Inertia;
      typedef JointMotionSubspaceRevoluteUnalignedTpl<S2, O2> Constraint;
      typedef typename MultiplicationOp<Inertia, Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Inertia & Y, const Constraint & cru)
      {
        ReturnType res;

        /* YS = [ m -mcx ; mcx I-mcxcx ] [ 0 ; w ] = [ mcxw ; Iw -mcxcxw ] */
        const typename Inertia::Scalar & m = Y.mass();
        const typename Inertia::Vector3 & c = Y.lever();
        const typename Inertia::Symmetric3 & I = Y.inertia();

        res.template segment<3>(Inertia::LINEAR) = -m * c.cross(cru.axis());
        res.template segment<3>(Inertia::ANGULAR).noalias() = I * cru.axis();
        res.template segment<3>(Inertia::ANGULAR) +=
          c.cross(res.template segment<3>(Inertia::LINEAR));

        return res;
      }
    };
  } // namespace impl

  template<typename M6Like, typename Scalar, int Options>
  struct MultiplicationOp<
    Eigen::MatrixBase<M6Like>,
    JointMotionSubspaceRevoluteUnalignedTpl<Scalar, Options>>
  {
    typedef typename SizeDepType<3>::ColsReturn<M6Like>::ConstType M6LikeCols;
    typedef typename Eigen::internal::remove_const<M6LikeCols>::type M6LikeColsNonConst;

    typedef JointMotionSubspaceRevoluteUnalignedTpl<Scalar, Options> Constraint;
    typedef typename Constraint::Vector3 Vector3;
    typedef const typename MatrixMatrixProduct<M6LikeColsNonConst, Vector3>::type ReturnType;
  };

  /* [ABA] operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename M6Like, typename Scalar, int Options>
    struct LhsMultiplicationOp<
      Eigen::MatrixBase<M6Like>,
      JointMotionSubspaceRevoluteUnalignedTpl<Scalar, Options>>
    {
      typedef JointMotionSubspaceRevoluteUnalignedTpl<Scalar, Options> Constraint;
      typedef
        typename MultiplicationOp<Eigen::MatrixBase<M6Like>, Constraint>::ReturnType ReturnType;

      static inline ReturnType run(const Eigen::MatrixBase<M6Like> & Y, const Constraint & cru)
      {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6Like, 6, 6);
        return Y.derived().template middleCols<3>(Constraint::ANGULAR) * cru.axis();
      }
    };
  } // namespace impl

  template<typename Scalar, int Options>
  struct JointRevoluteUnalignedTpl;

  template<typename _Scalar, int _Options>
  struct traits<JointRevoluteUnalignedTpl<_Scalar, _Options>>
  {
    enum
    {
      NQ = 1,
      NV = 1,
      NVExtended = 1
    };
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };
    typedef JointDataRevoluteUnalignedTpl<Scalar, Options> JointDataDerived;
    typedef JointModelRevoluteUnalignedTpl<Scalar, Options> JointModelDerived;
    typedef JointMotionSubspaceRevoluteUnalignedTpl<Scalar, Options> Constraint_t;
    typedef SE3Tpl<Scalar, Options> Transformation_t;
    typedef MotionRevoluteUnalignedTpl<Scalar, Options> Motion_t;
    typedef MotionZeroTpl<Scalar, Options> Bias_t;

    // [ABA]
    typedef Eigen::Matrix<Scalar, 6, NV, Options> U_t;
    typedef Eigen::Matrix<Scalar, NV, NV, Options> D_t;
    typedef Eigen::Matrix<Scalar, 6, NV, Options> UD_t;

    typedef Eigen::Matrix<Scalar, NQ, 1, Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar, NV, 1, Options> TangentVector_t;

    typedef boost::mpl::true_ is_mimicable_t;

    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE
  };

  template<typename _Scalar, int _Options>
  struct traits<JointDataRevoluteUnalignedTpl<_Scalar, _Options>>
  {
    typedef JointRevoluteUnalignedTpl<_Scalar, _Options> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options>
  struct traits<JointModelRevoluteUnalignedTpl<_Scalar, _Options>>
  {
    typedef JointRevoluteUnalignedTpl<_Scalar, _Options> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options>
  struct JointDataRevoluteUnalignedTpl
  : public JointDataBase<JointDataRevoluteUnalignedTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointRevoluteUnalignedTpl<_Scalar, _Options> JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(JointDerived);
    PINOCCHIO_JOINT_DATA_BASE_DEFAULT_ACCESSOR

    ConfigVector_t joint_q;
    TangentVector_t joint_v;

    Transformation_t M;
    Constraint_t S;
    Motion_t v;
    Bias_t c;

    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;
    D_t StU;

    JointDataRevoluteUnalignedTpl()
    : joint_q(ConfigVector_t::Zero())
    , joint_v(TangentVector_t::Zero())
    , M(Transformation_t::Identity())
    , S(Constraint_t::Vector3::Zero())
    , v(Constraint_t::Vector3::Zero(), (Scalar)0)
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    , StU(D_t::Zero())
    {
    }

    template<typename Vector3Like>
    JointDataRevoluteUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : joint_q(ConfigVector_t::Zero())
    , joint_v(TangentVector_t::Zero())
    , M(Transformation_t::Identity())
    , S(axis)
    , v(axis, (Scalar)NAN)
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    , StU(D_t::Zero())
    {
    }

    static std::string classname()
    {
      return std::string("JointDataRevoluteUnaligned");
    }
    std::string shortname() const
    {
      return classname();
    }

  }; // struct JointDataRevoluteUnalignedTpl

  PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelRevoluteUnalignedTpl);
  template<typename _Scalar, int _Options>
  struct JointModelRevoluteUnalignedTpl
  : public JointModelBase<JointModelRevoluteUnalignedTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointRevoluteUnalignedTpl<_Scalar, _Options> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    typedef Eigen::Matrix<Scalar, 3, 1, _Options> Vector3;

    typedef JointModelBase<JointModelRevoluteUnalignedTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::idx_vExtended;
    using Base::setIndexes;

    JointModelRevoluteUnalignedTpl()
    : axis(Vector3::UnitX())
    {
    }

    JointModelRevoluteUnalignedTpl(const Scalar & x, const Scalar & y, const Scalar & z)
    : axis(x, y, z)
    {
      normalize(axis);
      assert(isUnitary(axis) && "Rotation axis is not unitary");
    }

    template<typename Vector3Like>
    JointModelRevoluteUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : axis(axis)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Vector3Like);
      assert(isUnitary(axis) && "Rotation axis is not unitary");
    }

    JointDataDerived createData() const
    {
      return JointDataDerived(axis);
    }

    using Base::isEqual;
    bool isEqual(const JointModelRevoluteUnalignedTpl & other) const
    {
      return Base::isEqual(other) && internal::comparison_eq(axis, other.axis);
    }

    const std::vector<bool> hasConfigurationLimit() const
    {
      return {true};
    }

    const std::vector<bool> hasConfigurationLimitInTangent() const
    {
      return {true};
    }

    template<typename ConfigVector>
    void calc(JointDataDerived & data, const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      data.joint_q[0] = qs[idx_q()];

      toRotationMatrix(axis, data.joint_q[0], data.M.rotation());
    }

    template<typename TangentVector>
    void
    calc(JointDataDerived & data, const Blank, const typename Eigen::MatrixBase<TangentVector> & vs)
      const
    {
      data.v.angularRate() = static_cast<Scalar>(vs[idx_v()]);
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(
      JointDataDerived & data,
      const typename Eigen::MatrixBase<ConfigVector> & qs,
      const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data, qs.derived());

      data.v.angularRate() = static_cast<Scalar>(vs[idx_v()]);
    }

    template<typename VectorLike, typename Matrix6Like>
    void calc_aba(
      JointDataDerived & data,
      const Eigen::MatrixBase<VectorLike> & armature,
      const Eigen::MatrixBase<Matrix6Like> & I,
      const bool update_I) const
    {
      data.U.noalias() = I.template middleCols<3>(Motion::ANGULAR) * axis;
      data.Dinv[0] =
        Scalar(1) / (axis.dot(data.U.template segment<3>(Motion::ANGULAR)) + armature[0]);
      data.UDinv.noalias() = data.U * data.Dinv;

      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like, I).noalias() -= data.UDinv * data.U.transpose();
    }

    static std::string classname()
    {
      return std::string("JointModelRevoluteUnaligned");
    }
    std::string shortname() const
    {
      return classname();
    }

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelRevoluteUnalignedTpl<NewScalar, Options> cast() const
    {
      typedef JointModelRevoluteUnalignedTpl<NewScalar, Options> ReturnType;
      ReturnType res(axis.template cast<NewScalar>());
      res.setIndexes(id(), idx_q(), idx_v(), idx_vExtended());
      return res;
    }

    // data

    ///
    /// \brief 3d main axis of the joint.
    ///
    Vector3 axis;
  }; // struct JointModelRevoluteUnalignedTpl

  template<typename Scalar, int Options>
  struct ConfigVectorAffineTransform<JointRevoluteUnalignedTpl<Scalar, Options>>
  {
    typedef LinearAffineTransform Type;
  };
} // namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options>
  struct has_nothrow_constructor<::pinocchio::JointModelRevoluteUnalignedTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_copy<::pinocchio::JointModelRevoluteUnalignedTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_constructor<::pinocchio::JointDataRevoluteUnalignedTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_copy<::pinocchio::JointDataRevoluteUnalignedTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };
} // namespace boost

#endif // ifndef __pinocchio_multibody_joint_revolute_unaligned_hpp__
