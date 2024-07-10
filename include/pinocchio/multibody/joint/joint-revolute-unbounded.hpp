//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_multibody_joint_revolute_unbounded_hpp__
#define __pinocchio_multibody_joint_revolute_unbounded_hpp__

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-revolute.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, int axis>
  struct JointRevoluteUnboundedTpl;

  template<typename _Scalar, int _Options, int axis>
  struct traits<JointRevoluteUnboundedTpl<_Scalar, _Options, axis>>
  {
    enum
    {
      NQ = 2,
      NV = 1
    };
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };
    typedef JointDataRevoluteUnboundedTpl<Scalar, Options, axis> JointDataDerived;
    typedef JointModelRevoluteUnboundedTpl<Scalar, Options, axis> JointModelDerived;
    typedef JointMotionSubspaceRevoluteTpl<Scalar, Options, axis> Constraint_t;
    typedef TransformRevoluteTpl<Scalar, Options, axis> Transformation_t;
    typedef MotionRevoluteTpl<Scalar, Options, axis> Motion_t;
    typedef MotionZeroTpl<Scalar, Options> Bias_t;

    // [ABA]
    typedef Eigen::Matrix<Scalar, 6, NV, Options> U_t;
    typedef Eigen::Matrix<Scalar, NV, NV, Options> D_t;
    typedef Eigen::Matrix<Scalar, 6, NV, Options> UD_t;

    typedef Eigen::Matrix<Scalar, NQ, 1, Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar, NV, 1, Options> TangentVector_t;

    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE
  };

  template<typename _Scalar, int _Options, int axis>
  struct traits<JointDataRevoluteUnboundedTpl<_Scalar, _Options, axis>>
  {
    typedef JointRevoluteUnboundedTpl<_Scalar, _Options, axis> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options, int axis>
  struct traits<JointModelRevoluteUnboundedTpl<_Scalar, _Options, axis>>
  {
    typedef JointRevoluteUnboundedTpl<_Scalar, _Options, axis> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options, int axis>
  struct JointDataRevoluteUnboundedTpl
  : public JointDataBase<JointDataRevoluteUnboundedTpl<_Scalar, _Options, axis>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointRevoluteUnboundedTpl<_Scalar, _Options, axis> JointDerived;
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

    JointDataRevoluteUnboundedTpl()
    : joint_q(Scalar(1), Scalar(0))
    , joint_v(TangentVector_t::Zero())
    , M((Scalar)0, (Scalar)1)
    , v((Scalar)0)
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    , StU(D_t::Zero())
    {
    }

    static std::string classname()
    {
      return std::string("JointDataRUB") + axisLabel<axis>();
    }
    std::string shortname() const
    {
      return classname();
    }

  }; // struct JointDataRevoluteUnbounded

  template<typename NewScalar, typename Scalar, int Options, int axis>
  struct CastType<NewScalar, JointModelRevoluteUnboundedTpl<Scalar, Options, axis>>
  {
    typedef JointModelRevoluteUnboundedTpl<NewScalar, Options, axis> type;
  };

  template<typename _Scalar, int _Options, int axis>
  struct JointModelRevoluteUnboundedTpl
  : public JointModelBase<JointModelRevoluteUnboundedTpl<_Scalar, _Options, axis>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointRevoluteUnboundedTpl<_Scalar, _Options, axis> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    typedef JointRevoluteTpl<Scalar, _Options, axis> JointDerivedBase;

    typedef JointModelBase<JointModelRevoluteUnboundedTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;

    typedef Eigen::Matrix<Scalar, 3, 1, _Options> Vector3;

    JointDataDerived createData() const
    {
      return JointDataDerived();
    }

    const std::vector<bool> hasConfigurationLimit() const
    {
      return {false, false};
    }

    const std::vector<bool> hasConfigurationLimitInTangent() const
    {
      return {false};
    }

    template<typename ConfigVector>
    void calc(JointDataDerived & data, const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      data.joint_q = qs.template segment<NQ>(idx_q());

      const Scalar & ca = data.joint_q[0];
      const Scalar & sa = data.joint_q[1];

      data.M.setValues(sa, ca);
    }

    template<typename TangentVector>
    void
    calc(JointDataDerived & data, const Blank, const typename Eigen::MatrixBase<TangentVector> & vs)
      const
    {
      data.joint_v[0] = vs[idx_v()];
      data.v.angularRate() = data.joint_v[0];
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(
      JointDataDerived & data,
      const typename Eigen::MatrixBase<ConfigVector> & qs,
      const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data, qs.derived());
      data.joint_v[0] = vs[idx_v()];
      data.v.angularRate() = data.joint_v[0];
    }

    template<typename VectorLike, typename Matrix6Like>
    void calc_aba(
      JointDataDerived & data,
      const Eigen::MatrixBase<VectorLike> & armature,
      const Eigen::MatrixBase<Matrix6Like> & I,
      const bool update_I) const
    {
      data.U = I.col(Inertia::ANGULAR + axis);
      data.Dinv[0] =
        (Scalar)(1) / (I(Inertia::ANGULAR + axis, Inertia::ANGULAR + axis) + armature[0]);
      data.UDinv.noalias() = data.U * data.Dinv[0];

      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like, I).noalias() -= data.UDinv * data.U.transpose();
    }

    static std::string classname()
    {
      return std::string("JointModelRUB") + axisLabel<axis>();
    }
    std::string shortname() const
    {
      return classname();
    }

    Vector3 getMotionAxis() const
    {
      switch (axis)
      {
      case 0:
        return Vector3::UnitX();
      case 1:
        return Vector3::UnitY();
      case 2:
        return Vector3::UnitZ();
      default:
        assert(false && "must never happen");
        break;
      }
    }

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelRevoluteUnboundedTpl<NewScalar, Options, axis> cast() const
    {
      typedef JointModelRevoluteUnboundedTpl<NewScalar, Options, axis> ReturnType;
      ReturnType res;
      res.setIndexes(id(), idx_q(), idx_v());
      return res;
    }

  }; // struct JointModelRevoluteUnboundedTpl

  struct UnboundedRevoluteAffineTransform
  {
    template<typename ConfigVectorIn, typename Scalar, typename ConfigVectorOut>
    static void run(
      const Eigen::MatrixBase<ConfigVectorIn> & q,
      const Scalar & scaling,
      const Scalar & offset,
      const Eigen::MatrixBase<ConfigVectorOut> & dest)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(ConfigVectorIn, 2);
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(ConfigVectorOut, 2);

      const typename ConfigVectorIn::Scalar & ca = q(0);
      const typename ConfigVectorIn::Scalar & sa = q(1);

      const typename ConfigVectorIn::Scalar & theta = math::atan2(sa, ca);
      const typename ConfigVectorIn::Scalar & theta_transform = scaling * theta + offset;

      ConfigVectorOut & dest_ = PINOCCHIO_EIGEN_CONST_CAST(ConfigVectorOut, dest);
      SINCOS(theta_transform, &dest_.coeffRef(1), &dest_.coeffRef(0));
    }
  };

  template<typename Scalar, int Options, int axis>
  struct ConfigVectorAffineTransform<JointRevoluteUnboundedTpl<Scalar, Options, axis>>
  {
    typedef UnboundedRevoluteAffineTransform Type;
  };

  typedef JointRevoluteUnboundedTpl<context::Scalar, context::Options, 0> JointRUBX;
  typedef JointDataRevoluteUnboundedTpl<context::Scalar, context::Options, 0> JointDataRUBX;
  typedef JointModelRevoluteUnboundedTpl<context::Scalar, context::Options, 0> JointModelRUBX;

  typedef JointRevoluteUnboundedTpl<context::Scalar, context::Options, 1> JointRUBY;
  typedef JointDataRevoluteUnboundedTpl<context::Scalar, context::Options, 1> JointDataRUBY;
  typedef JointModelRevoluteUnboundedTpl<context::Scalar, context::Options, 1> JointModelRUBY;

  typedef JointRevoluteUnboundedTpl<context::Scalar, context::Options, 2> JointRUBZ;
  typedef JointDataRevoluteUnboundedTpl<context::Scalar, context::Options, 2> JointDataRUBZ;
  typedef JointModelRevoluteUnboundedTpl<context::Scalar, context::Options, 2> JointModelRUBZ;

} // namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_constructor<::pinocchio::JointModelRevoluteUnboundedTpl<Scalar, Options, axis>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options, int axis>
  struct has_nothrow_copy<::pinocchio::JointModelRevoluteUnboundedTpl<Scalar, Options, axis>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options, int axis>
  struct has_nothrow_constructor<::pinocchio::JointDataRevoluteUnboundedTpl<Scalar, Options, axis>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options, int axis>
  struct has_nothrow_copy<::pinocchio::JointDataRevoluteUnboundedTpl<Scalar, Options, axis>>
  : public integral_constant<bool, true>
  {
  };
} // namespace boost

#endif // ifndef __pinocchio_multibody_joint_revolute_unbounded_hpp__
