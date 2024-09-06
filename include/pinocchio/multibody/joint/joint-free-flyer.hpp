//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_multibody_joint_free_flyer_hpp__
#define __pinocchio_multibody_joint_free_flyer_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint-motion-subspace.hpp"
#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/quaternion.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options>
  struct JointMotionSubspaceIdentityTpl;

  template<typename _Scalar, int _Options>
  struct traits<JointMotionSubspaceIdentityTpl<_Scalar, _Options>>
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };
    typedef Eigen::Matrix<Scalar, 6, 6, Options> Matrix6;
    enum
    {
      LINEAR = 0,
      ANGULAR = 3
    };
    typedef MotionTpl<Scalar, Options> JointMotion;
    typedef Eigen::Matrix<Scalar, 6, 1, Options> JointForce;
    typedef Eigen::Matrix<Scalar, 6, 6, Options> DenseBase;
    typedef Eigen::Matrix<Scalar, 6, 6, Options> ReducedSquaredMatrix;

    typedef typename Matrix6::IdentityReturnType ConstMatrixReturnType;
    typedef typename Matrix6::IdentityReturnType MatrixReturnType;
    typedef typename Matrix6::IdentityReturnType StDiagonalMatrixSOperationReturnType;
  }; // traits ConstraintRevolute

  template<typename _Scalar, int _Options>
  struct JointMotionSubspaceIdentityTpl
  : JointMotionSubspaceBase<JointMotionSubspaceIdentityTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(JointMotionSubspaceIdentityTpl)

    enum
    {
      NV = 6
    };

    template<typename Vector6Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector6Like> & vj) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector6Like, 6);
      return JointMotion(vj);
    }

    template<typename S1, int O1>
    typename SE3Tpl<S1, O1>::ActionMatrixType se3Action(const SE3Tpl<S1, O1> & m) const
    {
      return m.toActionMatrix();
    }

    template<typename S1, int O1>
    typename SE3Tpl<S1, O1>::ActionMatrixType se3ActionInverse(const SE3Tpl<S1, O1> & m) const
    {
      return m.toActionMatrixInverse();
    }

    int nv_impl() const
    {
      return NV;
    }

    struct TransposeConst : JointMotionSubspaceTransposeBase<JointMotionSubspaceIdentityTpl>
    {
      template<typename Derived>
      typename ForceDense<Derived>::ToVectorConstReturnType
      operator*(const ForceDense<Derived> & phi)
      {
        return phi.toVector();
      }

      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename MatrixDerived>
      typename PINOCCHIO_EIGEN_REF_CONST_TYPE(MatrixDerived)
      operator*(const Eigen::MatrixBase<MatrixDerived> & F)
      {
        return F.derived();
      }
    };

    TransposeConst transpose() const
    {
      return TransposeConst();
    }
    MatrixReturnType matrix_impl() const
    {
      return DenseBase::Identity();
    }

    template<typename MotionDerived>
    typename MotionDerived::ActionMatrixType motionAction(const MotionBase<MotionDerived> & v) const
    {
      return v.toActionMatrix();
    }

    bool isEqual(const JointMotionSubspaceIdentityTpl &) const
    {
      return true;
    }

  }; // struct JointMotionSubspaceIdentityTpl

  template<typename Scalar, int Options, typename Vector6Like>
  MotionRef<const Vector6Like> operator*(
    const JointMotionSubspaceIdentityTpl<Scalar, Options> &,
    const Eigen::MatrixBase<Vector6Like> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector6Like, 6);
    //    typedef typename JointMotionSubspaceIdentityTpl<Scalar,Options>::Motion Motion;
    typedef MotionRef<const Vector6Like> Motion;
    return Motion(v.derived());
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline typename InertiaTpl<S1, O1>::Matrix6
  operator*(const InertiaTpl<S1, O1> & Y, const JointMotionSubspaceIdentityTpl<S2, O2> &)
  {
    return Y.matrix();
  }

  /* [ABA] Y*S operator*/
  template<typename Matrix6Like, typename S2, int O2>
  inline typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Matrix6Like) operator*(
    const Eigen::MatrixBase<Matrix6Like> & Y, const JointMotionSubspaceIdentityTpl<S2, O2> &)
  {
    return Y.derived();
  }

  template<typename S1, int O1>
  struct SE3GroupAction<JointMotionSubspaceIdentityTpl<S1, O1>>
  {
    typedef typename SE3Tpl<S1, O1>::ActionMatrixType ReturnType;
  };

  template<typename S1, int O1, typename MotionDerived>
  struct MotionAlgebraAction<JointMotionSubspaceIdentityTpl<S1, O1>, MotionDerived>
  {
    typedef typename SE3Tpl<S1, O1>::ActionMatrixType ReturnType;
  };

  template<typename Scalar, int Options>
  struct JointFreeFlyerTpl;

  template<typename _Scalar, int _Options>
  struct traits<JointFreeFlyerTpl<_Scalar, _Options>>
  {
    enum
    {
      NQ = 7,
      NV = 6
    };
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };
    typedef JointDataFreeFlyerTpl<Scalar, Options> JointDataDerived;
    typedef JointModelFreeFlyerTpl<Scalar, Options> JointModelDerived;
    typedef JointMotionSubspaceIdentityTpl<Scalar, Options> Constraint_t;
    typedef SE3Tpl<Scalar, Options> Transformation_t;
    typedef MotionTpl<Scalar, Options> Motion_t;
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
  struct traits<JointDataFreeFlyerTpl<_Scalar, _Options>>
  {
    typedef JointFreeFlyerTpl<_Scalar, _Options> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options>
  struct traits<JointModelFreeFlyerTpl<_Scalar, _Options>>
  {
    typedef JointFreeFlyerTpl<_Scalar, _Options> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options>
  struct JointDataFreeFlyerTpl : public JointDataBase<JointDataFreeFlyerTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointFreeFlyerTpl<_Scalar, _Options> JointDerived;
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

    JointDataFreeFlyerTpl()
    : joint_q(ConfigVector_t::Zero())
    , joint_v(TangentVector_t::Zero())
    , M(Transformation_t::Identity())
    , v(Motion_t::Zero())
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Identity())
    , StU(D_t::Zero())
    {
      joint_q[6] = Scalar(1);
    }

    static std::string classname()
    {
      return std::string("JointDataFreeFlyer");
    }
    std::string shortname() const
    {
      return classname();
    }

  }; // struct JointDataFreeFlyerTpl

  /// @brief Free-flyer joint in \f$SE(3)\f$.
  ///
  /// A free-flyer joint adds seven coordinates to the configuration space.
  /// Given a configuration vector `q`:
  ///
  /// - `q[idx_q:idx_q + 3]` are the translation coordinates, in meters,
  ///   representing the position of the child frame in the parent frame.
  /// - `q[idx_q + 3:idx_q + 7]` is a unit quaternion representing the rotation
  ///   from the child frame to the parent frame, with quaternion coordinates
  ///   ordered as (x, y, z, w).
  ///
  /// Likewise, a free-flyer joint adds six coordinates to the tangent space.
  /// Let's consider a tangent vector `v`, say, a velocity vector. Following
  /// Featherstone's convention, all our tangent vectors are body rather than
  /// spatial vectors:
  ///
  /// - `v[idx_v:idx_v + 3]` is the linear velocity, in meters / second,
  ///   corresponding to the linear velocity of the child frame with respect
  ///   to the parent frame, expressed in the child frame (body linear velocity
  ///   of the child frame).
  /// - `v[idx_v + 3:idx_v + 6]` is the angular velocity, in radians / second,
  ///   corresponding to the angular velocity from the child frame to the
  ///   parent frame, expressed in the child frame (body angular velocity of the
  ///   child frame).
  PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelFreeFlyerTpl);
  template<typename _Scalar, int _Options>
  struct JointModelFreeFlyerTpl : public JointModelBase<JointModelFreeFlyerTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointFreeFlyerTpl<_Scalar, _Options> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);

    typedef JointModelBase<JointModelFreeFlyerTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;

    JointDataDerived createData() const
    {
      return JointDataDerived();
    }

    const std::vector<bool> hasConfigurationLimit() const
    {
      return {true, true, true, false, false, false, false};
    }

    const std::vector<bool> hasConfigurationLimitInTangent() const
    {
      return {true, true, true, false, false, false};
    }

    template<typename ConfigVectorLike>
    inline void forwardKinematics(
      Transformation_t & M, const Eigen::MatrixBase<ConfigVectorLike> & q_joint) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t, ConfigVectorLike);
      typedef typename Eigen::Quaternion<
        typename ConfigVectorLike::Scalar, PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorLike)::Options>
        Quaternion;
      typedef Eigen::Map<const Quaternion> ConstQuaternionMap;

      ConstQuaternionMap quat(q_joint.template tail<4>().data());
      // assert(math::fabs(quat.coeffs().squaredNorm()-1.) <= sqrt(Eigen::NumTraits<typename
      // V::Scalar>::epsilon())); TODO: check validity of the rhs precision
      assert(math::fabs(static_cast<Scalar>(quat.coeffs().squaredNorm() - 1)) <= 1e-4);

      M.rotation(quat.matrix());
      M.translation(q_joint.template head<3>());
    }

    template<typename Vector3Derived, typename QuaternionDerived>
    EIGEN_DONT_INLINE void calc(
      JointDataDerived & data,
      const typename Eigen::MatrixBase<Vector3Derived> & trans,
      const typename Eigen::QuaternionBase<QuaternionDerived> & quat) const
    {
      data.M.translation(trans);
      data.M.rotation(quat.matrix());
    }

    template<typename ConfigVector>
    EIGEN_DONT_INLINE void
    calc(JointDataDerived & data, const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      typedef typename Eigen::Quaternion<Scalar, Options> Quaternion;
      typedef Eigen::Map<const Quaternion> ConstQuaternionMap;

      data.joint_q = qs.template segment<NQ>(idx_q());
      ConstQuaternionMap quat(data.joint_q.template tail<4>().data());

      calc(data, data.joint_q.template head<3>(), quat);
    }

    template<typename TangentVector>
    EIGEN_DONT_INLINE void
    calc(JointDataDerived & data, const Blank, const typename Eigen::MatrixBase<TangentVector> & vs)
      const
    {
      data.joint_v = vs.template segment<NV>(idx_v());
      data.v = data.joint_v;
    }

    template<typename ConfigVector, typename TangentVector>
    EIGEN_DONT_INLINE void calc(
      JointDataDerived & data,
      const typename Eigen::MatrixBase<ConfigVector> & qs,
      const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data, qs.derived());

      data.joint_v = vs.template segment<NV>(idx_v());
      data.v = data.joint_v;
    }

    template<typename VectorLike, typename Matrix6Like>
    void calc_aba(
      JointDataDerived & data,
      const Eigen::MatrixBase<VectorLike> & armature,
      const Eigen::MatrixBase<Matrix6Like> & I,
      const bool update_I) const
    {
      data.U = I;
      data.StU = I;
      data.StU.diagonal() += armature;

      internal::PerformStYSInversion<Scalar>::run(data.StU, data.Dinv);
      data.UDinv.noalias() = I * data.Dinv;

      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like, I).noalias() -= data.UDinv * data.U.transpose();
    }

    static std::string classname()
    {
      return std::string("JointModelFreeFlyer");
    }
    std::string shortname() const
    {
      return classname();
    }

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelFreeFlyerTpl<NewScalar, Options> cast() const
    {
      typedef JointModelFreeFlyerTpl<NewScalar, Options> ReturnType;
      ReturnType res;
      res.setIndexes(id(), idx_q(), idx_v());
      return res;
    }

  }; // struct JointModelFreeFlyerTpl

} // namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options>
  struct has_nothrow_constructor<::pinocchio::JointModelFreeFlyerTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_copy<::pinocchio::JointModelFreeFlyerTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_constructor<::pinocchio::JointDataFreeFlyerTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_copy<::pinocchio::JointDataFreeFlyerTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };
} // namespace boost

#endif // ifndef __pinocchio_multibody_joint_free_flyer_hpp__
