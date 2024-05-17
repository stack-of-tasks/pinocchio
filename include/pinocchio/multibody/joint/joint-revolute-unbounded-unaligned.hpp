//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_multibody_joint_revolute_unbounded_unaligned_hpp__
#define __pinocchio_multibody_joint_revolute_unbounded_unaligned_hpp__

#include "pinocchio/fwd.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/math/rotation.hpp"
#include "pinocchio/math/matrix.hpp"

#include "pinocchio/multibody/joint/joint-revolute-unaligned.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options = 0>
  struct JointRevoluteUnboundedUnalignedTpl;

  template<typename _Scalar, int _Options>
  struct traits<JointRevoluteUnboundedUnalignedTpl<_Scalar, _Options>>
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

    typedef Eigen::Matrix<Scalar, NQ, 1, Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar, NV, 1, Options> TangentVector_t;

    typedef JointDataRevoluteUnboundedUnalignedTpl<Scalar, Options> JointDataDerived;
    typedef JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options> JointModelDerived;
    typedef JointMotionSubspaceRevoluteUnalignedTpl<Scalar, Options> Constraint_t;
    typedef SE3Tpl<Scalar, Options> Transformation_t;
    typedef MotionRevoluteUnalignedTpl<Scalar, Options> Motion_t;
    typedef MotionZeroTpl<Scalar, Options> Bias_t;
    typedef Eigen::Matrix<Scalar, 6, NV, Options> F_t;

    // [ABA]
    typedef Eigen::Matrix<Scalar, 6, NV, Options> U_t;
    typedef Eigen::Matrix<Scalar, NV, NV, Options> D_t;
    typedef Eigen::Matrix<Scalar, 6, NV, Options> UD_t;

    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE
  };

  template<typename _Scalar, int _Options>
  struct traits<JointDataRevoluteUnboundedUnalignedTpl<_Scalar, _Options>>
  {
    typedef JointRevoluteUnboundedUnalignedTpl<_Scalar, _Options> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options>
  struct traits<JointModelRevoluteUnboundedUnalignedTpl<_Scalar, _Options>>
  {
    typedef JointRevoluteUnboundedUnalignedTpl<_Scalar, _Options> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options>
  struct JointDataRevoluteUnboundedUnalignedTpl
  : public JointDataBase<JointDataRevoluteUnboundedUnalignedTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointRevoluteUnboundedUnalignedTpl<_Scalar, _Options> JointDerived;
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

    JointDataRevoluteUnboundedUnalignedTpl()
    : joint_q(Scalar(1), Scalar(0))
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
    JointDataRevoluteUnboundedUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : joint_q(Scalar(1), Scalar(0))
    , joint_v(TangentVector_t::Zero())
    , M(Transformation_t::Identity())
    , S(axis)
    , v(axis, (Scalar)0)
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    , StU(D_t::Zero())
    {
    }

    static std::string classname()
    {
      return std::string("JointDataRevoluteUnboundedUnalignedTpl");
    }
    std::string shortname() const
    {
      return classname();
    }

  }; // struct JointDataRevoluteUnboundedUnalignedTpl

  PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelRevoluteUnboundedUnalignedTpl);

  template<typename _Scalar, int _Options>
  struct JointModelRevoluteUnboundedUnalignedTpl
  : public JointModelBase<JointModelRevoluteUnboundedUnalignedTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointRevoluteUnboundedUnalignedTpl<_Scalar, _Options> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;

    typedef JointModelBase<JointModelRevoluteUnboundedUnalignedTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;

    JointModelRevoluteUnboundedUnalignedTpl()
    : axis(Vector3::UnitX())
    {
    }

    JointModelRevoluteUnboundedUnalignedTpl(const Scalar & x, const Scalar & y, const Scalar & z)
    : axis(x, y, z)
    {
      normalize(axis);
      assert(isUnitary(axis) && "Rotation axis is not unitary");
    }

    template<typename Vector3Like>
    JointModelRevoluteUnboundedUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : axis(axis)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Vector3Like);
      assert(isUnitary(axis) && "Rotation axis is not unitary");
    }

    JointDataDerived createData() const
    {
      return JointDataDerived(axis);
    }

    const std::vector<bool> hasConfigurationLimit() const
    {
      return {false, false};
    }

    const std::vector<bool> hasConfigurationLimitInTangent() const
    {
      return {false};
    }

    using Base::isEqual;
    bool isEqual(const JointModelRevoluteUnboundedUnalignedTpl & other) const
    {
      return Base::isEqual(other) && internal::comparison_eq(axis, other.axis);
    }

    template<typename ConfigVector>
    void calc(JointDataDerived & data, const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      data.joint_q = qs.template segment<NQ>(idx_q());

      const Scalar & ca = data.joint_q(0);
      const Scalar & sa = data.joint_q(1);

      toRotationMatrix(axis, ca, sa, data.M.rotation());
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
      data.U.noalias() = I.template middleCols<3>(Motion::ANGULAR) * axis;
      data.Dinv[0] =
        Scalar(1) / (axis.dot(data.U.template segment<3>(Motion::ANGULAR)) + armature[0]);
      data.UDinv.noalias() = data.U * data.Dinv;

      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like, I).noalias() -= data.UDinv * data.U.transpose();
    }

    static std::string classname()
    {
      return std::string("JointModelRevoluteUnboundedUnaligned");
    }
    std::string shortname() const
    {
      return classname();
    }

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelRevoluteUnboundedUnalignedTpl<NewScalar, Options> cast() const
    {
      typedef JointModelRevoluteUnboundedUnalignedTpl<NewScalar, Options> ReturnType;
      ReturnType res(axis.template cast<NewScalar>());
      res.setIndexes(id(), idx_q(), idx_v());
      return res;
    }

    // data
    ///
    /// \brief axis of rotation of the joint.
    ///
    Vector3 axis;
  }; // struct JointModelRevoluteUnboundedUnalignedTpl

} // namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options>
  struct has_nothrow_constructor<
    ::pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_copy<::pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_constructor<
    ::pinocchio::JointDataRevoluteUnboundedUnalignedTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_copy<::pinocchio::JointDataRevoluteUnboundedUnalignedTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };
} // namespace boost

#endif // ifndef __pinocchio_multibody_joint_revolute_unbounded_unaligned_hpp__
