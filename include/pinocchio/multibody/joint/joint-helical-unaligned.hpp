//
// Copyright (c) 2022-2023 INRIA
//

#ifndef __pinocchio_multibody_joint_helical_unaligned_hpp__
#define __pinocchio_multibody_joint_helical_unaligned_hpp__

#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint-motion-subspace.hpp"
#include "pinocchio/spatial/inertia.hpp"

#include "pinocchio/math/matrix.hpp"
#include "pinocchio/math/rotation.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options>
  struct MotionHelicalUnalignedTpl;

  template<typename Scalar, int Options>
  struct SE3GroupAction<MotionHelicalUnalignedTpl<Scalar, Options>>
  {
    typedef MotionTpl<Scalar, Options> ReturnType;
  };

  template<typename Scalar, int Options, typename MotionDerived>
  struct MotionAlgebraAction<MotionHelicalUnalignedTpl<Scalar, Options>, MotionDerived>
  {
    typedef MotionTpl<Scalar, Options> ReturnType;
  };

  template<typename _Scalar, int _Options>
  struct traits<MotionHelicalUnalignedTpl<_Scalar, _Options>>
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
    typedef MotionTpl<Scalar, Options> MotionPlain;
    typedef MotionPlain PlainReturnType;
    typedef Matrix4 HomogeneousMatrixType;
    enum
    {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits MotionHelicalUnalignedTpl

  template<typename _Scalar, int _Options>
  struct MotionHelicalUnalignedTpl : MotionBase<MotionHelicalUnalignedTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MOTION_TYPEDEF_TPL(MotionHelicalUnalignedTpl);

    MotionHelicalUnalignedTpl()
    {
    }

    template<typename Vector3Like, typename OtherScalar>
    MotionHelicalUnalignedTpl(
      const Eigen::MatrixBase<Vector3Like> & axis, const OtherScalar & w, const OtherScalar & v)
    : m_axis(axis)
    , m_w(w)
    , m_v(v)
    {
    }

    inline PlainReturnType plain() const
    {
      return PlainReturnType(m_axis * m_v, m_axis * m_w);
    }

    template<typename OtherScalar>
    MotionHelicalUnalignedTpl __mult__(const OtherScalar & alpha) const
    {
      return MotionHelicalUnalignedTpl(m_axis, alpha * m_w, alpha * m_v);
    }

    template<typename MotionDerived>
    void setTo(MotionDense<MotionDerived> & m) const
    {
      for (Eigen::DenseIndex k = 0; k < 3; ++k)
      {
        m.angular().noalias() = m_axis * m_w;
        m.linear().noalias() = m_axis * m_v;
      }
    }

    template<typename MotionDerived>
    inline void addTo(MotionDense<MotionDerived> & v) const
    {
      v.angular() += m_axis * m_w;
      v.linear() += m_axis * m_v;
    }

    template<typename S2, int O2, typename D2>
    inline void se3Action_impl(const SE3Tpl<S2, O2> & m, MotionDense<D2> & v) const
    {
      v.angular().noalias() = m_w * m.rotation() * m_axis;
      v.linear().noalias() = m.translation().cross(v.angular()) + m_v * m.rotation() * m_axis;
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
      v.angular().noalias() = m_axis.cross(m.translation());
      v.angular() *= m_w;
      v.linear().noalias() =
        m.rotation().transpose() * v.angular() + m_v * (m.rotation().transpose() * m_axis);

      // Angular
      v.angular().noalias() = m.rotation().transpose() * m_axis * m_w;
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
      mout.angular().noalias() = v.angular().cross(m_axis);
      mout.angular() *= m_v;
      mout.linear() += mout.angular();

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

    Scalar & angularRate()
    {
      return m_w;
    }
    const Scalar & angularRate() const
    {
      return m_w;
    }

    Scalar & linearRate()
    {
      return m_v;
    }
    const Scalar & linearRate() const
    {
      return m_v;
    }

    Vector3 & axis()
    {
      return m_axis;
    }
    const Vector3 & axis() const
    {
      return m_axis;
    }

    bool isEqual_impl(const MotionHelicalUnalignedTpl & other) const
    {
      return internal::comparison_eq(m_axis, other.m_axis)
             && internal::comparison_eq(m_w, other.m_w) && internal::comparison_eq(m_v, other.m_v);
    }

  protected:
    Vector3 m_axis;
    Scalar m_w, m_v;

  }; // struct MotionHelicalUnalignedTpl

  template<typename S1, int O1, typename MotionDerived>
  typename MotionDerived::MotionPlain
  operator+(const MotionHelicalUnalignedTpl<S1, O1> & m1, const MotionDense<MotionDerived> & m2)
  {
    typename MotionDerived::MotionPlain res(m2);
    res += m1;
    return res;
  }

  template<typename MotionDerived, typename S2, int O2>
  EIGEN_STRONG_INLINE typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1, const MotionHelicalUnalignedTpl<S2, O2> & m2)
  {
    return m2.motionAction(m1);
  }

  template<typename Scalar, int Options>
  struct JointMotionSubspaceHelicalUnalignedTpl;

  template<typename Scalar, int Options>
  struct SE3GroupAction<JointMotionSubspaceHelicalUnalignedTpl<Scalar, Options>>
  {
    typedef Eigen::Matrix<Scalar, 6, 1, Options> ReturnType;
  };

  template<typename Scalar, int Options, typename MotionDerived>
  struct MotionAlgebraAction<JointMotionSubspaceHelicalUnalignedTpl<Scalar, Options>, MotionDerived>
  {
    typedef Eigen::Matrix<Scalar, 6, 1, Options> ReturnType;
  };

  template<typename Scalar, int Options, typename ForceDerived>
  struct ConstraintForceOp<JointMotionSubspaceHelicalUnalignedTpl<Scalar, Options>, ForceDerived>
  {
    typedef typename Eigen::Matrix<Scalar, 1, 1> ReturnType;
  };

  template<typename Scalar, int Options, typename ForceSet>
  struct ConstraintForceSetOp<JointMotionSubspaceHelicalUnalignedTpl<Scalar, Options>, ForceSet>
  {
    typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> ReturnType;
  };

  template<typename _Scalar, int _Options>
  struct traits<JointMotionSubspaceHelicalUnalignedTpl<_Scalar, _Options>>
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

    typedef MotionHelicalUnalignedTpl<Scalar, Options> JointMotion;
    typedef Eigen::Matrix<Scalar, 1, 1, Options> JointForce;
    typedef Eigen::Matrix<Scalar, 6, 1, Options> DenseBase;
    typedef Eigen::Matrix<Scalar, 1, 1, Options> ReducedSquaredMatrix;

    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;

    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;

    typedef typename ReducedSquaredMatrix::IdentityReturnType StDiagonalMatrixSOperationReturnType;
  }; // traits JointMotionSubspaceHelicalUnalignedTpl

  template<typename _Scalar, int _Options>
  struct JointMotionSubspaceHelicalUnalignedTpl
  : JointMotionSubspaceBase<JointMotionSubspaceHelicalUnalignedTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(JointMotionSubspaceHelicalUnalignedTpl)
    enum
    {
      NV = 1
    };

    JointMotionSubspaceHelicalUnalignedTpl()
    {
    }

    typedef typename traits<JointMotionSubspaceHelicalUnalignedTpl>::Vector3 Vector3;

    template<typename Vector3Like>
    JointMotionSubspaceHelicalUnalignedTpl(
      const Eigen::MatrixBase<Vector3Like> & axis, const Scalar & h)
    : m_axis(axis)
    , m_pitch(h)
    {
    }

    template<typename Vector1Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector1Like> & v) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector1Like, 1);
      assert(v.size() == 1);
      return JointMotion(m_axis, v[0], Scalar(v[0] * m_pitch));
    }

    template<typename S1, int O1>
    typename SE3GroupAction<JointMotionSubspaceHelicalUnalignedTpl>::ReturnType
    se3Action(const SE3Tpl<S1, O1> & m) const
    {
      typedef
        typename SE3GroupAction<JointMotionSubspaceHelicalUnalignedTpl>::ReturnType ReturnType;
      ReturnType res;
      res.template segment<3>(ANGULAR).noalias() = m.rotation() * m_axis;
      res.template segment<3>(LINEAR).noalias() =
        m.translation().cross(res.template segment<3>(ANGULAR)) + m_pitch * (m.rotation() * m_axis);
      return res;
    }

    template<typename S1, int O1>
    typename SE3GroupAction<JointMotionSubspaceHelicalUnalignedTpl>::ReturnType
    se3ActionInverse(const SE3Tpl<S1, O1> & m) const
    {
      typedef
        typename SE3GroupAction<JointMotionSubspaceHelicalUnalignedTpl>::ReturnType ReturnType;

      ReturnType res;
      res.template segment<3>(ANGULAR).noalias() = m.rotation().transpose() * m_axis;
      res.template segment<3>(LINEAR).noalias() =
        -m.rotation().transpose() * m.translation().cross(m_axis)
        + m.rotation().transpose() * m_axis * m_pitch;

      return res;
    }

    int nv_impl() const
    {
      return NV;
    }

    struct TransposeConst : JointMotionSubspaceTransposeBase<JointMotionSubspaceHelicalUnalignedTpl>
    {
      const JointMotionSubspaceHelicalUnalignedTpl & ref;
      TransposeConst(const JointMotionSubspaceHelicalUnalignedTpl & ref)
      : ref(ref)
      {
      }

      template<typename ForceDerived>
      typename ConstraintForceOp<JointMotionSubspaceHelicalUnalignedTpl, ForceDerived>::ReturnType
      operator*(const ForceDense<ForceDerived> & f) const
      {
        typedef typename ConstraintForceOp<
          JointMotionSubspaceHelicalUnalignedTpl, ForceDerived>::ReturnType ReturnType;
        ReturnType res;
        res[0] = ref.axis().dot(f.angular()) + ref.axis().dot(f.linear()) * ref.m_pitch;
        return res;
      }

      /// [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block)
      template<typename Derived>
      typename ConstraintForceSetOp<JointMotionSubspaceHelicalUnalignedTpl, Derived>::ReturnType
      operator*(const Eigen::MatrixBase<Derived> & F) const
      {
        assert(F.rows() == 6);
        return (
          ref.axis().transpose() * F.template middleRows<3>(ANGULAR)
          + (ref.axis().transpose() * F.template middleRows<3>(LINEAR) * ref.m_pitch));
      }
    }; // struct TransposeConst

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
      S.template segment<3>(LINEAR) = m_axis * m_pitch;
      S.template segment<3>(ANGULAR) = m_axis;
      return S;
    }

    template<typename MotionDerived>
    typename MotionAlgebraAction<JointMotionSubspaceHelicalUnalignedTpl, MotionDerived>::ReturnType
    motionAction(const MotionDense<MotionDerived> & m) const
    {
      const typename MotionDerived::ConstLinearType v = m.linear();
      const typename MotionDerived::ConstAngularType w = m.angular();

      DenseBase res;
      res.template segment<3>(LINEAR).noalias() = v.cross(m_axis);
      res.template segment<3>(ANGULAR).noalias() = w.cross(m_axis * m_pitch);
      res.template segment<3>(LINEAR).noalias() += res.template segment<3>(ANGULAR);
      res.template segment<3>(ANGULAR).noalias() = w.cross(m_axis);

      return res;
    }

    bool isEqual(const JointMotionSubspaceHelicalUnalignedTpl & other) const
    {
      return internal::comparison_eq(m_axis, other.m_axis)
             && internal::comparison_eq(m_pitch, other.m_pitch);
    }

    Vector3 & axis()
    {
      return m_axis;
    }
    const Vector3 & axis() const
    {
      return m_axis;
    }

    Scalar & h()
    {
      return m_pitch;
    }
    const Scalar & h() const
    {
      return m_pitch;
    }

    Vector3 m_axis;
    Scalar m_pitch;

  }; // struct JointMotionSubspaceHelicalUnalignedTpl

  template<typename _Scalar, int _Options>
  Eigen::Matrix<_Scalar, 1, 1, _Options> operator*(
    const typename JointMotionSubspaceHelicalUnalignedTpl<_Scalar, _Options>::TransposeConst &
      S_transpose,
    const JointMotionSubspaceHelicalUnalignedTpl<_Scalar, _Options> & S)
  {
    Eigen::Matrix<_Scalar, 1, 1, _Options> res;
    res(0) = (S_transpose.ref.axis() * S_transpose.ref.h()).dot(S.axis() * S.h())
             + (S_transpose.ref.axis().dot(S.axis()));
    return res;
  }

  template<typename S1, int O1, typename S2, int O2>
  struct MultiplicationOp<InertiaTpl<S1, O1>, JointMotionSubspaceHelicalUnalignedTpl<S2, O2>>
  {
    typedef Eigen::Matrix<S2, 6, 1, O2> ReturnType;
  };

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1, O1>, JointMotionSubspaceHelicalUnalignedTpl<S2, O2>>
    {
      typedef InertiaTpl<S1, O1> Inertia;
      typedef JointMotionSubspaceHelicalUnalignedTpl<S2, O2> Constraint;
      typedef typename JointMotionSubspaceHelicalUnalignedTpl<S2, O2>::Vector3 Vector3;
      typedef typename MultiplicationOp<Inertia, Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Inertia & Y, const Constraint & cru)
      {
        ReturnType res;
        /* YS = [ m -mcx ; mcx I-mcxcx ] [ h ; w ] = [mh-mcxw ; mcxh+Iw-mcxcxw ] */

        const S2 & m_pitch = cru.h();
        const typename Inertia::Scalar & m = Y.mass();
        const typename Inertia::Vector3 & c = Y.lever();
        const typename Inertia::Symmetric3 & I = Y.inertia();

        res.template segment<3>(Inertia::LINEAR) = -m * c.cross(cru.axis());
        res.template segment<3>(Inertia::ANGULAR).noalias() = I * cru.axis();
        res.template segment<3>(Inertia::ANGULAR) += c.cross(cru.axis()) * m * m_pitch;
        res.template segment<3>(Inertia::ANGULAR) +=
          c.cross(res.template segment<3>(Inertia::LINEAR));
        res.template segment<3>(Inertia::LINEAR) += m * m_pitch * cru.axis();
        return res;
      }
    };
  } // namespace impl

  template<typename M6Like, typename Scalar, int Options>
  struct MultiplicationOp<
    Eigen::MatrixBase<M6Like>,
    JointMotionSubspaceHelicalUnalignedTpl<Scalar, Options>>
  {
    typedef const Eigen::Matrix<Scalar, 6, 1> ReturnType;
  };

  /* [ABA] operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename M6Like, typename Scalar, int Options>
    struct LhsMultiplicationOp<
      Eigen::MatrixBase<M6Like>,
      JointMotionSubspaceHelicalUnalignedTpl<Scalar, Options>>
    {
      typedef JointMotionSubspaceHelicalUnalignedTpl<Scalar, Options> Constraint;
      typedef Eigen::Matrix<Scalar, 6, 1> ReturnType;
      static inline ReturnType run(const Eigen::MatrixBase<M6Like> & Y, const Constraint & cru)
      {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6Like, 6, 6);
        return Y.derived().template middleCols<3>(Constraint::ANGULAR) * cru.axis()
               + Y.derived().template middleCols<3>(Constraint::LINEAR) * cru.axis() * cru.h();
      }
    };
  } // namespace impl

  template<typename Scalar, int Options>
  struct JointHelicalUnalignedTpl;

  template<typename _Scalar, int _Options>
  struct traits<JointHelicalUnalignedTpl<_Scalar, _Options>>
  {
    enum
    {
      NQ = 1,
      NV = 1
    };
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };
    typedef JointDataHelicalUnalignedTpl<Scalar, Options> JointDataDerived;
    typedef JointModelHelicalUnalignedTpl<Scalar, Options> JointModelDerived;
    typedef JointMotionSubspaceHelicalUnalignedTpl<Scalar, Options> Constraint_t;
    typedef SE3Tpl<Scalar, Options> Transformation_t;
    typedef MotionHelicalUnalignedTpl<Scalar, Options> Motion_t;
    typedef MotionZeroTpl<Scalar, Options> Bias_t;

    // [ABA]
    typedef Eigen::Matrix<Scalar, 6, NV, Options> U_t;
    typedef Eigen::Matrix<Scalar, NV, NV, Options> D_t;
    typedef Eigen::Matrix<Scalar, 6, NV, Options> UD_t;

    typedef Eigen::Matrix<Scalar, NQ, 1, Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar, NV, 1, Options> TangentVector_t;

    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE
  };

  template<typename _Scalar, int _Options>
  struct traits<JointDataHelicalUnalignedTpl<_Scalar, _Options>>
  {
    typedef JointHelicalUnalignedTpl<_Scalar, _Options> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options>
  struct traits<JointModelHelicalUnalignedTpl<_Scalar, _Options>>
  {
    typedef JointHelicalUnalignedTpl<_Scalar, _Options> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options>
  struct JointDataHelicalUnalignedTpl
  : public JointDataBase<JointDataHelicalUnalignedTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointHelicalUnalignedTpl<_Scalar, _Options> JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(JointDerived);
    PINOCCHIO_JOINT_DATA_BASE_DEFAULT_ACCESSOR

    ConfigVector_t joint_q;
    TangentVector_t joint_v;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;
    D_t StU;

    JointDataHelicalUnalignedTpl()
    : joint_q(ConfigVector_t::Zero())
    , joint_v(TangentVector_t::Zero())
    , S(Constraint_t::Vector3::Zero(), (Scalar)0)
    , M(Transformation_t::Identity())
    , v(Constraint_t::Vector3::Zero(), (Scalar)0, (Scalar)0)
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    , StU(D_t::Zero())
    {
    }

    template<typename Vector3Like>
    JointDataHelicalUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : joint_q(ConfigVector_t::Zero())
    , joint_v(TangentVector_t::Zero())
    , S(axis, (Scalar)0)
    , M(Transformation_t::Identity())
    , v(axis, (Scalar)0, (Scalar)0)
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    , StU(D_t::Zero())
    {
    }

    static std::string classname()
    {
      return std::string("JointDataHelicalUnaligned");
    }
    std::string shortname() const
    {
      return classname();
    }

  }; // struct JointDataHelicalUnalignedTpl

  PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelHelicalUnalignedTpl);
  template<typename _Scalar, int _Options>
  struct JointModelHelicalUnalignedTpl
  : public JointModelBase<JointModelHelicalUnalignedTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointHelicalUnalignedTpl<_Scalar, _Options> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    typedef Eigen::Matrix<Scalar, 3, 1, _Options> Vector3;

    typedef JointModelBase<JointModelHelicalUnalignedTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;

    JointModelHelicalUnalignedTpl()
    {
    }

    template<typename Vector3Like>
    JointModelHelicalUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : axis(axis)
    , m_pitch((Scalar)0)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Vector3Like);
      assert(isUnitary(axis) && "Rotation axis is not unitary");
    }

    JointModelHelicalUnalignedTpl(
      const Scalar & x, const Scalar & y, const Scalar & z, const Scalar & h)
    : axis(x, y, z)
    , m_pitch(h)
    {
      normalize(axis);
      assert(isUnitary(axis) && "Rotation axis is not unitary");
    }

    template<typename Vector3Like>
    JointModelHelicalUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis, const Scalar & h)
    : axis(axis)
    , m_pitch(h)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Vector3Like);
      assert(isUnitary(axis) && "Rotation axis is not unitary");
    }

    const std::vector<bool> hasConfigurationLimit() const
    {
      return {true, true};
    }

    const std::vector<bool> hasConfigurationLimitInTangent() const
    {
      return {true, true};
    }

    JointDataDerived createData() const
    {
      return JointDataDerived();
    }

    using Base::isEqual;
    bool isEqual(const JointModelHelicalUnalignedTpl & other) const
    {
      return Base::isEqual(other) && internal::comparison_eq(axis, other.axis)
             && internal::comparison_eq(m_pitch, other.m_pitch);
    }

    template<typename ConfigVector>
    void calc(JointDataDerived & data, const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      data.joint_q[0] = qs[idx_q()];

      toRotationMatrix(axis, data.joint_q[0], data.M.rotation());
      data.M.translation() = axis * data.joint_q[0] * m_pitch;
      data.S.h() = m_pitch;
      data.S.axis() = axis;
    }

    template<typename TangentVector>
    void
    calc(JointDataDerived & data, const Blank, const typename Eigen::MatrixBase<TangentVector> & vs)
      const
    {
      data.v.angularRate() = static_cast<Scalar>(vs[idx_v()]);
      data.v.axis() = axis;
      data.v.linearRate() = static_cast<Scalar>(vs[idx_v()] * m_pitch);
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(
      JointDataDerived & data,
      const typename Eigen::MatrixBase<ConfigVector> & qs,
      const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data, qs.derived());
      data.v.angularRate() = static_cast<Scalar>(vs[idx_v()]);
      data.v.axis() = axis;
      data.v.linearRate() = static_cast<Scalar>(vs[idx_v()] * m_pitch);
    }

    template<typename VectorLike, typename Matrix6Like>
    void calc_aba(
      JointDataDerived & data,
      const Eigen::MatrixBase<VectorLike> & armature,
      const Eigen::MatrixBase<Matrix6Like> & I,
      const bool update_I) const
    {
      data.U.noalias() = I.template middleCols<3>(Motion::ANGULAR) * axis
                         + m_pitch * I.template middleCols<3>(Motion::LINEAR) * axis;
      data.StU[0] = axis.dot(data.U.template segment<3>(Motion::ANGULAR))
                    + m_pitch * axis.dot(data.U.template segment<3>(Motion::LINEAR)) + armature[0];
      data.Dinv[0] = Scalar(1) / data.StU[0];
      data.UDinv.noalias() = data.U * data.Dinv;
      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like, I).noalias() -= data.UDinv * data.U.transpose();
    }

    static std::string classname()
    {
      return std::string("JointModelHelicalUnaligned");
    }
    std::string shortname() const
    {
      return classname();
    }

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelHelicalUnalignedTpl<NewScalar, Options> cast() const
    {
      typedef JointModelHelicalUnalignedTpl<NewScalar, Options> ReturnType;
      ReturnType res(axis.template cast<NewScalar>(), ScalarCast<NewScalar, Scalar>::cast(m_pitch));
      res.setIndexes(id(), idx_q(), idx_v());
      return res;
    }

    Vector3 axis;
    Scalar m_pitch;

  }; // struct JointModelHelicalUnalignedTpl

} // namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options>
  struct has_nothrow_constructor<::pinocchio::JointModelHelicalUnalignedTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_copy<::pinocchio::JointModelHelicalUnalignedTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_constructor<::pinocchio::JointDataHelicalUnalignedTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_copy<::pinocchio::JointDataHelicalUnalignedTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };
} // namespace boost

#endif // ifndef __pinocchio_multibody_joint_helical_unaligned_hpp__
