//
// Copyright (c) 2023 INRIA
//

#ifndef __pinocchio_multibody_joint_universal_hpp__
#define __pinocchio_multibody_joint_universal_hpp__

#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-spherical.hpp"
#include "pinocchio/multibody/joint-motion-subspace.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/utils/check.hpp"

#include "pinocchio/math/matrix.hpp"
#include "pinocchio/math/rotation.hpp"

namespace pinocchio
{
  template<typename Scalar, int Options>
  struct JointMotionSubspaceUniversalTpl;

  template<typename _Scalar, int _Options>
  struct traits<JointMotionSubspaceUniversalTpl<_Scalar, _Options>>
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

    typedef MotionSphericalTpl<Scalar, Options> JointMotion;
    typedef Eigen::Matrix<Scalar, 2, 1, Options> JointForce;
    typedef Eigen::Matrix<Scalar, 6, 2, Options> DenseBase;
    typedef Eigen::Matrix<Scalar, 3, 3, Options> ReducedSquaredMatrix;

    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;

    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;
    typedef Eigen::Matrix<Scalar, 3, 2, Options> Matrix32;
    typedef Eigen::Matrix<Scalar, 2, 2, Options> Matrix2;

    typedef Matrix2 StDiagonalMatrixSOperationReturnType;
  }; // traits JointMotionSubspaceUniversalTpl

  template<typename Scalar, int Options>
  struct SE3GroupAction<JointMotionSubspaceUniversalTpl<Scalar, Options>>
  {
    typedef Eigen::Matrix<Scalar, 6, 2, Options> ReturnType;
  };

  template<typename Scalar, int Options, typename MotionDerived>
  struct MotionAlgebraAction<JointMotionSubspaceUniversalTpl<Scalar, Options>, MotionDerived>
  {
    typedef Eigen::Matrix<Scalar, 6, 2, Options> ReturnType;
  };

  template<typename Scalar, int Options, typename ForceDerived>
  struct ConstraintForceOp<JointMotionSubspaceUniversalTpl<Scalar, Options>, ForceDerived>
  {
    typedef typename traits<JointMotionSubspaceUniversalTpl<Scalar, Options>>::Vector3 Vector3;
    typedef Eigen::Matrix<
      typename PINOCCHIO_EIGEN_DOT_PRODUCT_RETURN_TYPE(
        Vector3, typename ForceDense<ForceDerived>::ConstAngularType),
      2,
      1,
      Options>
      ReturnType;
  };

  template<typename Scalar, int Options, typename ForceSet>
  struct ConstraintForceSetOp<JointMotionSubspaceUniversalTpl<Scalar, Options>, ForceSet>
  {
    typedef typename traits<JointMotionSubspaceUniversalTpl<Scalar, Options>>::Matrix32 Matrix32;
    typedef typename MatrixMatrixProduct<
      Eigen::Transpose<const Matrix32>,
      typename Eigen::MatrixBase<const ForceSet>::template NRowsBlockXpr<3>::Type>::type ReturnType;
  };

  template<typename _Scalar, int _Options>
  struct JointMotionSubspaceUniversalTpl
  : JointMotionSubspaceBase<JointMotionSubspaceUniversalTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(JointMotionSubspaceUniversalTpl)

    enum
    {
      NV = 2
    };

    typedef typename traits<JointMotionSubspaceUniversalTpl>::Vector3 Vector3;
    typedef typename traits<JointMotionSubspaceUniversalTpl>::Matrix32 Matrix32;

    JointMotionSubspaceUniversalTpl()
    {
    }

    template<typename Matrix32Like>
    JointMotionSubspaceUniversalTpl(const Eigen::MatrixBase<Matrix32Like> & subspace)
    : m_S(subspace)
    {
    }

    template<typename Vector3Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector3Like> & v) const
    {
      // EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3);
      return JointMotion(m_S * v);
    }

    template<typename S1, int O1>
    typename SE3GroupAction<JointMotionSubspaceUniversalTpl>::ReturnType
    se3Action(const SE3Tpl<S1, O1> & m) const
    {
      typedef typename SE3GroupAction<JointMotionSubspaceUniversalTpl>::ReturnType ReturnType;

      ReturnType res;
      res.template middleRows<3>(ANGULAR).noalias() = m.rotation() * m_S;
      cross(
        m.translation(), res.template middleRows<3>(Motion::ANGULAR),
        res.template middleRows<3>(LINEAR));
      return res;
    }

    template<typename S1, int O1>
    typename SE3GroupAction<JointMotionSubspaceUniversalTpl>::ReturnType
    se3ActionInverse(const SE3Tpl<S1, O1> & m) const
    {
      typedef typename SE3GroupAction<JointMotionSubspaceUniversalTpl>::ReturnType ReturnType;

      ReturnType res;
      cross(m.translation(), m_S, res.template middleRows<3>(ANGULAR));
      res.template middleRows<3>(LINEAR).noalias() =
        -m.rotation().transpose() * res.template middleRows<3>(ANGULAR);

      // ANGULAR
      res.template middleRows<3>(ANGULAR).noalias() = m.rotation().transpose() * m_S;
      return res;
    }

    int nv_impl() const
    {
      return NV;
    }

    struct TransposeConst : JointMotionSubspaceTransposeBase<JointMotionSubspaceUniversalTpl>
    {
      const JointMotionSubspaceUniversalTpl & ref;
      TransposeConst(const JointMotionSubspaceUniversalTpl & ref)
      : ref(ref)
      {
      }

      template<typename ForceDerived>
      typename ConstraintForceOp<JointMotionSubspaceUniversalTpl, ForceDerived>::ReturnType
      operator*(const ForceDense<ForceDerived> & f) const
      {
        return ref.m_S.transpose() * f.angular();
      }

      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename ForceSet>
      typename ConstraintForceSetOp<JointMotionSubspaceUniversalTpl, ForceSet>::ReturnType
      operator*(const Eigen::MatrixBase<ForceSet> & F)
      {
        EIGEN_STATIC_ASSERT(
          ForceSet::RowsAtCompileTime == 6, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE)
        /* Return ax.T * F[3:end,:] */
        return ref.m_S.transpose() * F.template middleRows<3>(ANGULAR);
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
      S.template middleRows<3>(LINEAR).setZero();
      S.template middleRows<3>(ANGULAR) = m_S;
      return S;
    }

    template<typename MotionDerived>
    typename MotionAlgebraAction<JointMotionSubspaceUniversalTpl, MotionDerived>::ReturnType
    motionAction(const MotionDense<MotionDerived> & m) const
    {
      const typename MotionDerived::ConstLinearType v = m.linear();
      const typename MotionDerived::ConstAngularType w = m.angular();

      DenseBase res;
      cross(v, m_S, res.template middleRows<3>(LINEAR));
      cross(w, m_S, res.template middleRows<3>(ANGULAR));
      return res;
    }

    const Matrix32 & angularSubspace() const
    {
      return m_S;
    }
    Matrix32 & angularSubspace()
    {
      return m_S;
    }

    bool isEqual(const JointMotionSubspaceUniversalTpl & other) const
    {
      return internal::comparison_eq(m_S, other.m_S);
    }

  protected:
    Matrix32 m_S;

  }; // struct JointMotionSubspaceUniversalTpl

  namespace details
  {
    template<typename Scalar, int Options>
    struct StDiagonalMatrixSOperation<JointMotionSubspaceUniversalTpl<Scalar, Options>>
    {
      typedef JointMotionSubspaceUniversalTpl<Scalar, Options> Constraint;
      typedef typename traits<Constraint>::StDiagonalMatrixSOperationReturnType ReturnType;

      static ReturnType run(const JointMotionSubspaceBase<Constraint> & constraint)
      {
        return constraint.matrix().transpose() * constraint.matrix();
      }
    };
  } // namespace details

  template<typename S1, int O1, typename S2, int O2>
  struct MultiplicationOp<InertiaTpl<S1, O1>, JointMotionSubspaceUniversalTpl<S2, O2>>
  {
    typedef Eigen::Matrix<S2, 6, 2, O2> ReturnType;
  };

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1, O1>, JointMotionSubspaceUniversalTpl<S2, O2>>
    {
      typedef InertiaTpl<S1, O1> Inertia;
      typedef JointMotionSubspaceUniversalTpl<S2, O2> Constraint;
      typedef typename MultiplicationOp<Inertia, Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Inertia & Y, const Constraint & cru)
      {
        typedef typename InertiaTpl<S1, O1>::Symmetric3 Symmetric3;
        Eigen::Matrix<S1, 6, 3, O1> M;
        alphaSkew(-Y.mass(), Y.lever(), M.template middleRows<3>(Constraint::LINEAR));
        M.template middleRows<3>(Constraint::ANGULAR) =
          (Y.inertia() - typename Symmetric3::AlphaSkewSquare(Y.mass(), Y.lever())).matrix();

        return (M * cru.angularSubspace()).eval();
      }
    };
  } // namespace impl

  template<typename M6Like, typename Scalar, int Options>
  struct MultiplicationOp<
    Eigen::MatrixBase<M6Like>,
    JointMotionSubspaceUniversalTpl<Scalar, Options>>
  {
    typedef typename SizeDepType<3>::ColsReturn<M6Like>::ConstType M6LikeCols;
    typedef typename Eigen::internal::remove_const<M6LikeCols>::type M6LikeColsNonConst;

    typedef JointMotionSubspaceUniversalTpl<Scalar, Options> Constraint;
    typedef typename Constraint::Matrix32 Matrix32;
    typedef const typename MatrixMatrixProduct<M6LikeColsNonConst, Matrix32>::type ReturnType;
  };

  /* [ABA] operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename M6Like, typename Scalar, int Options>
    struct LhsMultiplicationOp<
      Eigen::MatrixBase<M6Like>,
      JointMotionSubspaceUniversalTpl<Scalar, Options>>
    {
      typedef JointMotionSubspaceUniversalTpl<Scalar, Options> Constraint;
      typedef
        typename MultiplicationOp<Eigen::MatrixBase<M6Like>, Constraint>::ReturnType ReturnType;

      static inline ReturnType run(const Eigen::MatrixBase<M6Like> & Y, const Constraint & cru)
      {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6Like, 6, 6);
        return Y.derived().template middleCols<3>(Constraint::ANGULAR) * cru.angularSubspace();
      }
    };
  } // namespace impl

  template<typename Scalar, int Options>
  struct JointUniversalTpl;

  template<typename _Scalar, int _Options>
  struct traits<JointUniversalTpl<_Scalar, _Options>>
  {
    enum
    {
      NQ = 2,
      NV = 2
    };
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };
    typedef JointDataUniversalTpl<Scalar, Options> JointDataDerived;
    typedef JointModelUniversalTpl<Scalar, Options> JointModelDerived;
    typedef JointMotionSubspaceUniversalTpl<Scalar, Options> Constraint_t;
    typedef SE3Tpl<Scalar, Options> Transformation_t;
    typedef MotionSphericalTpl<Scalar, Options> Motion_t;
    typedef MotionSphericalTpl<Scalar, Options> Bias_t;

    // [ABA]
    typedef Eigen::Matrix<Scalar, 6, NV, Options> U_t;
    typedef Eigen::Matrix<Scalar, NV, NV, Options> D_t;
    typedef Eigen::Matrix<Scalar, 6, NV, Options> UD_t;

    typedef Eigen::Matrix<Scalar, NQ, 1, Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar, NV, 1, Options> TangentVector_t;

    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE
  };

  template<typename _Scalar, int _Options>
  struct traits<JointDataUniversalTpl<_Scalar, _Options>>
  {
    typedef JointUniversalTpl<_Scalar, _Options> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options>
  struct traits<JointModelUniversalTpl<_Scalar, _Options>>
  {
    typedef JointUniversalTpl<_Scalar, _Options> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options>
  struct JointDataUniversalTpl : public JointDataBase<JointDataUniversalTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointUniversalTpl<_Scalar, _Options> JointDerived;
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

    JointDataUniversalTpl()
    : joint_q(ConfigVector_t::Zero())
    , joint_v(TangentVector_t::Zero())
    , M(Transformation_t::Identity())
    , S(Constraint_t::Matrix32::Zero())
    , v(Motion_t::Vector3::Zero())
    , c(Bias_t::Vector3::Zero())
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    , StU(D_t::Zero())
    {
    }

    static std::string classname()
    {
      return std::string("JointDataUniversal");
    }
    std::string shortname() const
    {
      return classname();
    }

  }; // struct JointDataUniversalTpl

  PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelUniversalTpl);
  template<typename _Scalar, int _Options>
  struct JointModelUniversalTpl : public JointModelBase<JointModelUniversalTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointUniversalTpl<_Scalar, _Options> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    typedef Eigen::Matrix<Scalar, 3, 1, _Options> Vector3;
    typedef Eigen::Matrix<Scalar, 3, 3, _Options> Matrix3;

    typedef JointModelBase<JointModelUniversalTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;

    JointModelUniversalTpl()
    {
    }

    JointModelUniversalTpl(
      const Scalar & x1,
      const Scalar & y1,
      const Scalar & z1,
      const Scalar & x2,
      const Scalar & y2,
      const Scalar & z2)
    : axis1(x1, y1, z1)
    , axis2(x2, y2, z2)
    {
      assert(isUnitary(axis1) && "First Rotation axis is not unitary");
      assert(isUnitary(axis2) && "Second Rotation axis is not unitary");
      assert(check_expression_if_real<Scalar>(axis1.dot(axis2) == 0) && "Axii are not orthogonal");
    }

    template<typename Vector3Like>
    JointModelUniversalTpl(
      const Eigen::MatrixBase<Vector3Like> & axis1_, const Eigen::MatrixBase<Vector3Like> & axis2_)
    : axis1(axis1_)
    , axis2(axis2_)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Vector3Like);
      assert(isUnitary(axis1) && "First Rotation axis is not unitary");
      assert(isUnitary(axis2) && "Second Rotation axis is not unitary");
      assert(
        check_expression_if_real<Scalar>(axis1.dot(axis2) == Scalar(0))
        && "Axii are not orthogonal");
    }

    JointDataDerived createData() const
    {
      return JointDataDerived();
    }

    const std::vector<bool> hasConfigurationLimit() const
    {
      return {true, true};
    }

    const std::vector<bool> hasConfigurationLimitInTangent() const
    {
      return {true, true};
    }

    using Base::isEqual;
    bool isEqual(const JointModelUniversalTpl & other) const
    {
      return Base::isEqual(other) && internal::comparison_eq(axis1, other.axis1)
             && internal::comparison_eq(axis2, other.axis2);
    }

    template<typename ConfigVector>
    void calc(JointDataDerived & data, const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      data.joint_q = qs.template segment<NQ>(idx_q());
      Scalar c0, s0;
      SINCOS(data.joint_q(0), &s0, &c0);
      Scalar c1, s1;
      SINCOS(data.joint_q(1), &s1, &c1);

      Matrix3 rot1, rot2;
      toRotationMatrix(axis1, c0, s0, rot1);
      toRotationMatrix(axis2, c1, s1, rot2);
      data.M.rotation() = rot1 * rot2;

      data.S.angularSubspace() << rot2.coeffRef(0, 0) * axis1.x() + rot2.coeffRef(1, 0) * axis1.y()
                                    + rot2.coeffRef(2, 0) * axis1.z(),
        axis2.x(),
        rot2.coeffRef(0, 1) * axis1.x() + rot2.coeffRef(1, 1) * axis1.y()
          + rot2.coeffRef(2, 1) * axis1.z(),
        axis2.y(),
        rot2.coeffRef(0, 2) * axis1.x() + rot2.coeffRef(1, 2) * axis1.y()
          + rot2.coeffRef(2, 2) * axis1.z(),
        axis2.z();
    }

    template<typename TangentVector>
    void
    calc(JointDataDerived & data, const Blank, const typename Eigen::MatrixBase<TangentVector> & vs)
      const
    {
      data.joint_v = vs.template segment<NV>(idx_v());
      data.v().noalias() = data.S.angularSubspace() * data.joint_v;

      // TODO: need to be done
      //    #define q_dot data.joint_v
      //      Scalar tmp;
      //      tmp = (-s1+axis2.x()*axis2.x()*s1)*axis1.x() +
      //      (axis2.x()*axis2.y()*s1+axis2.z()*c1)*axis1.y() +
      //      (axis2.x()*axis2.z()*s1-axis2.y()*c1)*axis1.z(); data.c()(0) = tmp *
      //      q_dot(1)*q_dot(0); tmp = (axis2.x()*axis2.y()*s1-axis2.z()*c1)*axis1.x() +
      //      (-s1+axis2.y()*axis2.y()*s1)*axis1.y() +
      //      (axis2.y()*axis2.z()*s1+axis2.x()*c1)*axis1.z(); data.c()(1) = tmp *
      //      q_dot(1)*q_dot(0); tmp = (axis2.z()*axis2.x()*s1+axis2.y()*c1)*axis1.x() +
      //      (axis2.y()*axis2.z()*s1-axis2.x()*c1)*axis1.y() +
      //      (-s1+axis2.z()*axis2.z()*s1)*axis1.z(); data.c()(2) = tmp * q_dot(1)*q_dot(0);
      //    #undef q_dot
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(
      JointDataDerived & data,
      const typename Eigen::MatrixBase<ConfigVector> & qs,
      const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      data.joint_q = qs.template segment<NQ>(idx_q());

      Scalar c0, s0;
      SINCOS(data.joint_q(0), &s0, &c0);
      Scalar c1, s1;
      SINCOS(data.joint_q(1), &s1, &c1);

      Matrix3 rot1, rot2;
      toRotationMatrix(axis1, c0, s0, rot1);
      toRotationMatrix(axis2, c1, s1, rot2);
      data.M.rotation() = rot1 * rot2;

      data.S.angularSubspace() << rot2.coeffRef(0, 0) * axis1.x() + rot2.coeffRef(1, 0) * axis1.y()
                                    + rot2.coeffRef(2, 0) * axis1.z(),
        axis2.x(),
        rot2.coeffRef(0, 1) * axis1.x() + rot2.coeffRef(1, 1) * axis1.y()
          + rot2.coeffRef(2, 1) * axis1.z(),
        axis2.y(),
        rot2.coeffRef(0, 2) * axis1.x() + rot2.coeffRef(1, 2) * axis1.y()
          + rot2.coeffRef(2, 2) * axis1.z(),
        axis2.z();

      data.joint_v = vs.template segment<NV>(idx_v());
      data.v().noalias() = data.S.angularSubspace() * data.joint_v;

#define q_dot data.joint_v
      Scalar tmp;
      tmp = (-s1 + axis2.x() * axis2.x() * s1) * axis1.x()
            + (axis2.x() * axis2.y() * s1 + axis2.z() * c1) * axis1.y()
            + (axis2.x() * axis2.z() * s1 - axis2.y() * c1) * axis1.z();
      data.c()(0) = tmp * q_dot(1) * q_dot(0);
      tmp = (axis2.x() * axis2.y() * s1 - axis2.z() * c1) * axis1.x()
            + (-s1 + axis2.y() * axis2.y() * s1) * axis1.y()
            + (axis2.y() * axis2.z() * s1 + axis2.x() * c1) * axis1.z();
      data.c()(1) = tmp * q_dot(1) * q_dot(0);
      tmp = (axis2.z() * axis2.x() * s1 + axis2.y() * c1) * axis1.x()
            + (axis2.y() * axis2.z() * s1 - axis2.x() * c1) * axis1.y()
            + (-s1 + axis2.z() * axis2.z() * s1) * axis1.z();
      data.c()(2) = tmp * q_dot(1) * q_dot(0);
#undef q_dot
    }

    template<typename VectorLike, typename Matrix6Like>
    void calc_aba(
      JointDataDerived & data,
      const Eigen::MatrixBase<VectorLike> & armature,
      const Eigen::MatrixBase<Matrix6Like> & I,
      const bool update_I) const
    {
      data.U.noalias() = I.template middleCols<3>(Motion::ANGULAR) * data.S.angularSubspace();
      data.StU.noalias() =
        data.S.angularSubspace().transpose() * data.U.template middleRows<3>(Motion::ANGULAR);
      data.StU.diagonal() += armature;
      internal::PerformStYSInversion<Scalar>::run(data.StU, data.Dinv);

      data.UDinv.noalias() = data.U * data.Dinv;

      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like, I).noalias() -= data.UDinv * data.U.transpose();
    }

    static std::string classname()
    {
      return std::string("JointModelUniversal");
    }
    std::string shortname() const
    {
      return classname();
    }

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelUniversalTpl<NewScalar, Options> cast() const
    {
      typedef JointModelUniversalTpl<NewScalar, Options> ReturnType;
      ReturnType res(axis1.template cast<NewScalar>(), axis2.template cast<NewScalar>());
      res.setIndexes(id(), idx_q(), idx_v());
      return res;
    }

    /// \brief 3d main axii of the joint.
    ///
    Vector3 axis1;
    Vector3 axis2;
  }; // struct JointModelUniversalTpl

} // namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options>
  struct has_nothrow_constructor<::pinocchio::JointModelUniversalTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_copy<::pinocchio::JointModelUniversalTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_constructor<::pinocchio::JointDataUniversalTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_copy<::pinocchio::JointDataUniversalTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };
} // namespace boost

#endif // ifndef __pinocchio_multibody_joint_universal_hpp__
