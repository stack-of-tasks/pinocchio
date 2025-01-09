//
// Copyright (c) 2015-2021 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_spatial_se3_tpl_hpp__
#define __pinocchio_spatial_se3_tpl_hpp__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3-base.hpp"

#include "pinocchio/math/quaternion.hpp"
#include "pinocchio/math/rotation.hpp"
#include "pinocchio/spatial/cartesian-axis.hpp"

#include <Eigen/Geometry>

namespace pinocchio
{
  template<typename _Scalar, int _Options>
  struct traits<SE3Tpl<_Scalar, _Options>>
  {
    enum
    {
      Options = _Options,
      LINEAR = 0,
      ANGULAR = 3
    };
    typedef _Scalar Scalar;
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;
    typedef Eigen::Matrix<Scalar, 4, 1, Options> Vector4;
    typedef Eigen::Matrix<Scalar, 6, 1, Options> Vector6;
    typedef Eigen::Matrix<Scalar, 3, 3, Options> Matrix3;
    typedef Eigen::Matrix<Scalar, 4, 4, Options> Matrix4;
    typedef Eigen::Matrix<Scalar, 6, 6, Options> Matrix6;
    typedef Matrix3 AngularType;
    typedef typename PINOCCHIO_EIGEN_REF_TYPE(Matrix3) AngularRef;
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Matrix3) ConstAngularRef;
    typedef Vector3 LinearType;
    typedef typename PINOCCHIO_EIGEN_REF_TYPE(Vector3) LinearRef;
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector3) ConstLinearRef;
    typedef Matrix6 ActionMatrixType;
    typedef Matrix4 HomogeneousMatrixType;
    typedef SE3Tpl<Scalar, Options> PlainType;
  }; // traits SE3Tpl

  template<typename _Scalar, int _Options>
  struct SE3Tpl : public SE3Base<SE3Tpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PINOCCHIO_SE3_TYPEDEF_TPL(SE3Tpl);
    typedef SE3Base<SE3Tpl<_Scalar, _Options>> Base;
    typedef Eigen::Quaternion<Scalar, Options> Quaternion;
    typedef typename traits<SE3Tpl>::Vector3 Vector3;
    typedef typename traits<SE3Tpl>::Matrix3 Matrix3;
    typedef typename traits<SE3Tpl>::Matrix4 Matrix4;
    typedef typename traits<SE3Tpl>::Vector4 Vector4;
    typedef typename traits<SE3Tpl>::Matrix6 Matrix6;

    using Base::rotation;
    using Base::translation;

    SE3Tpl()
    : rot()
    , trans() {};

    template<typename QuaternionLike, typename Vector3Like>
    SE3Tpl(
      const Eigen::QuaternionBase<QuaternionLike> & quat,
      const Eigen::MatrixBase<Vector3Like> & trans)
    : rot(quat.matrix())
    , trans(trans)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like, 3)
    }

    template<typename Matrix3Like, typename Vector3Like>
    SE3Tpl(const Eigen::MatrixBase<Matrix3Like> & R, const Eigen::MatrixBase<Vector3Like> & trans)
    : rot(R)
    , trans(trans){EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like, 3)
                     EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3Like, 3, 3)}

    SE3Tpl(const SE3Tpl & other)
    {
      *this = other;
    }

    template<typename S2, int O2>
    explicit SE3Tpl(const SE3Tpl<S2, O2> & other)
    {
      *this = other.template cast<Scalar>();
    }

    template<typename Matrix4Like>
    explicit SE3Tpl(const Eigen::MatrixBase<Matrix4Like> & m)
    : rot(m.template block<3, 3>(LINEAR, LINEAR))
    , trans(m.template block<3, 1>(LINEAR, ANGULAR))
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix4Like, 4, 4);
    }

    explicit SE3Tpl(int)
    : rot(AngularType::Identity())
    , trans(LinearType::Zero())
    {
    }

    template<int O2>
    SE3Tpl(const SE3Tpl<Scalar, O2> & clone)
    : rot(clone.rotation())
    , trans(clone.translation())
    {
    }

    template<int O2>
    SE3Tpl & operator=(const SE3Tpl<Scalar, O2> & other)
    {
      rot = other.rotation();
      trans = other.translation();
      return *this;
    }

    ///
    /// \brief Copy assignment operator.
    ///
    /// \param[in] other SE3 to copy
    ///
    SE3Tpl & operator=(const SE3Tpl & other)
    {
      rot = other.rotation();
      trans = other.translation();
      return *this;
    }

    static SE3Tpl Identity()
    {
      return SE3Tpl(1);
    }

    SE3Tpl & setIdentity()
    {
      rot.setIdentity();
      trans.setZero();
      return *this;
    }

    /// aXb = bXa.inverse()
    SE3Tpl inverse() const
    {
      return SE3Tpl(rot.transpose(), -rot.transpose() * trans);
    }

    static SE3Tpl Random()
    {
      return SE3Tpl().setRandom();
    }

    SE3Tpl & setRandom()
    {
      Quaternion q;
      quaternion::uniformRandom(q);
      rot = q.matrix();
      trans.setRandom();

      return *this;
    }

    HomogeneousMatrixType toHomogeneousMatrix_impl() const
    {
      HomogeneousMatrixType M;
      M.template block<3, 3>(LINEAR, LINEAR) = rot;
      M.template block<3, 1>(LINEAR, ANGULAR) = trans;
      M.template block<1, 3>(ANGULAR, LINEAR).setZero();
      M(3, 3) = 1;
      return M;
    }

    /// Vb.toVector() = bXa.toMatrix() * Va.toVector()
    template<typename Matrix6Like>
    void toActionMatrix_impl(const Eigen::MatrixBase<Matrix6Like> & action_matrix) const
    {
      typedef Eigen::Block<Matrix6Like, 3, 3> Block3;

      Matrix6Like & M = action_matrix.const_cast_derived();
      M.template block<3, 3>(ANGULAR, ANGULAR) = M.template block<3, 3>(LINEAR, LINEAR) = rot;
      M.template block<3, 3>(ANGULAR, LINEAR).setZero();
      Block3 B = M.template block<3, 3>(LINEAR, ANGULAR);

      B.col(0) = trans.cross(rot.col(0));
      B.col(1) = trans.cross(rot.col(1));
      B.col(2) = trans.cross(rot.col(2));
    }

    ActionMatrixType toActionMatrix_impl() const
    {
      ActionMatrixType res;
      toActionMatrix_impl(res);
      return res;
    }

    template<typename Matrix6Like>
    void
    toActionMatrixInverse_impl(const Eigen::MatrixBase<Matrix6Like> & action_matrix_inverse) const
    {
      typedef Eigen::Block<Matrix6Like, 3, 3> Block3;

      Matrix6Like & M = action_matrix_inverse.const_cast_derived();
      M.template block<3, 3>(ANGULAR, ANGULAR) = M.template block<3, 3>(LINEAR, LINEAR) =
        rot.transpose();
      Block3 C = M.template block<3, 3>(ANGULAR, LINEAR); // used as temporary
      Block3 B = M.template block<3, 3>(LINEAR, ANGULAR);

#define PINOCCHIO_INTERNAL_COMPUTATION(axis_id, v3_in, v3_out, R, res)                             \
  CartesianAxis<axis_id>::cross(v3_in, v3_out);                                                    \
  res.col(axis_id).noalias() = R.transpose() * v3_out;

      PINOCCHIO_INTERNAL_COMPUTATION(0, trans, C.col(0), rot, B);
      PINOCCHIO_INTERNAL_COMPUTATION(1, trans, C.col(0), rot, B);
      PINOCCHIO_INTERNAL_COMPUTATION(2, trans, C.col(0), rot, B);

#undef PINOCCHIO_INTERNAL_COMPUTATION

      C.setZero();
    }

    ActionMatrixType toActionMatrixInverse_impl() const
    {
      ActionMatrixType res;
      toActionMatrixInverse_impl(res);
      return res;
    }

    template<typename Matrix6Like>
    void toDualActionMatrix_impl(const Eigen::MatrixBase<Matrix6Like> & dual_action_matrix) const
    {
      typedef Eigen::Block<Matrix6Like, 3, 3> Block3;

      Matrix6Like & M = dual_action_matrix.const_cast_derived();
      M.template block<3, 3>(ANGULAR, ANGULAR) = M.template block<3, 3>(LINEAR, LINEAR) = rot;
      M.template block<3, 3>(LINEAR, ANGULAR).setZero();
      Block3 B = M.template block<3, 3>(ANGULAR, LINEAR);

      B.col(0) = trans.cross(rot.col(0));
      B.col(1) = trans.cross(rot.col(1));
      B.col(2) = trans.cross(rot.col(2));
    }

    ActionMatrixType toDualActionMatrix_impl() const
    {
      ActionMatrixType res;
      toDualActionMatrix_impl(res);
      return res;
    }

    void disp_impl(std::ostream & os) const
    {
      os << "  R =\n" << rot << std::endl << "  p = " << trans.transpose() << std::endl;
    }

    /// --- GROUP ACTIONS ON M6, F6 and I6 ---

    /// ay = aXb.act(by)
    template<typename D>
    typename SE3GroupAction<D>::ReturnType act_impl(const D & d) const
    {
      return d.se3Action(*this);
    }

    /// by = aXb.actInv(ay)
    template<typename D>
    typename SE3GroupAction<D>::ReturnType actInv_impl(const D & d) const
    {
      return d.se3ActionInverse(*this);
    }

    template<typename EigenDerived>
    typename EigenDerived::PlainObject
    actOnEigenObject(const Eigen::MatrixBase<EigenDerived> & p) const
    {
      return (rotation() * p + translation()).eval();
    }

    template<typename MapDerived>
    Vector3 actOnEigenObject(const Eigen::MapBase<MapDerived> & p) const
    {
      return Vector3(rotation() * p + translation());
    }

    template<typename EigenDerived>
    typename EigenDerived::PlainObject
    actInvOnEigenObject(const Eigen::MatrixBase<EigenDerived> & p) const
    {
      return (rotation().transpose() * (p - translation())).eval();
    }

    template<typename MapDerived>
    Vector3 actInvOnEigenObject(const Eigen::MapBase<MapDerived> & p) const
    {
      return Vector3(rotation().transpose() * (p - translation()));
    }

    Vector3 act_impl(const Vector3 & p) const
    {
      return Vector3(rotation() * p + translation());
    }

    Vector3 actInv_impl(const Vector3 & p) const
    {
      return Vector3(rotation().transpose() * (p - translation()));
    }

    template<int O2>
    SE3Tpl act_impl(const SE3Tpl<Scalar, O2> & m2) const
    {
      return SE3Tpl(rot * m2.rotation(), translation() + rotation() * m2.translation());
    }

    template<int O2>
    SE3Tpl actInv_impl(const SE3Tpl<Scalar, O2> & m2) const
    {
      return SE3Tpl(
        rot.transpose() * m2.rotation(), rot.transpose() * (m2.translation() - translation()));
    }

    template<int O2>
    SE3Tpl __mult__(const SE3Tpl<Scalar, O2> & m2) const
    {
      return this->act_impl(m2);
    }

    template<int O2>
    bool isEqual(const SE3Tpl<Scalar, O2> & m2) const
    {
      return (rotation() == m2.rotation() && translation() == m2.translation());
    }

    template<int O2>
    bool isApprox_impl(
      const SE3Tpl<Scalar, O2> & m2,
      const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return rotation().isApprox(m2.rotation(), prec)
             && translation().isApprox(m2.translation(), prec);
    }

    bool isIdentity(const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return rotation().isIdentity(prec) && translation().isZero(prec);
    }

    ConstAngularRef rotation_impl() const
    {
      return rot;
    }
    AngularRef rotation_impl()
    {
      return rot;
    }
    void rotation_impl(const AngularType & R)
    {
      rot = R;
    }
    ConstLinearRef translation_impl() const
    {
      return trans;
    }
    LinearRef translation_impl()
    {
      return trans;
    }
    void translation_impl(const LinearType & p)
    {
      trans = p;
    }

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    SE3Tpl<NewScalar, Options> cast() const
    {
      typedef SE3Tpl<NewScalar, Options> ReturnType;
      ReturnType res(rot.template cast<NewScalar>(), trans.template cast<NewScalar>());

      // During the cast, it may appear that the matrix is not normalized correctly.
      // Force the normalization of the rotation part of the matrix.
      internal::cast_call_normalize_method<SE3Tpl, NewScalar, Scalar>::run(res);
      return res;
    }

    bool isNormalized(const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return isUnitary(rot, prec);
    }

    void normalize()
    {
      rot = orthogonalProjection(rot);
    }

    PlainType normalized() const
    {
      PlainType res(*this);
      res.normalize();
      return res;
    }

    ///
    /// \brief Linear interpolation on the SE3 manifold.
    ///
    /// \param[in] A Initial transformation.
    /// \param[in] B Target transformation.
    /// \param[in] alpha Interpolation factor in [0 ... 1].
    ///
    /// \returns An interpolated transformation between A and B.
    ///
    /// \note This is similar to the SLERP operation which acts initially for rotation but applied
    /// here to rigid transformation.
    ///
    template<typename OtherScalar>
    static SE3Tpl Interpolate(const SE3Tpl & A, const SE3Tpl & B, const OtherScalar & alpha);

  protected:
    AngularType rot;
    LinearType trans;

  }; // class SE3Tpl

  namespace internal
  {
    template<typename Scalar, int Options>
    struct cast_call_normalize_method<SE3Tpl<Scalar, Options>, Scalar, Scalar>
    {
      template<typename T>
      static void run(T &)
      {
      }
    };

    template<typename Scalar, int Options, typename NewScalar>
    struct cast_call_normalize_method<SE3Tpl<Scalar, Options>, NewScalar, Scalar>
    {
      template<typename T>
      static void run(T & self)
      {
        if (
          pinocchio::cast<NewScalar>(Eigen::NumTraits<Scalar>::epsilon())
          > Eigen::NumTraits<NewScalar>::epsilon())
          self.normalize();
      }
    };

  } // namespace internal

  template<typename _Scalar, int _Options>
  struct traits<SE3TplExpr<_Scalar, _Options>>
  {
    enum
    {
      Options = 0
    };
    typedef _Scalar Scalar;
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;
    typedef Eigen::Matrix<Scalar, 3, 3, Options> Matrix3;
    typedef Matrix3 AngularType;
    typedef Matrix3 & AngularRef;
    typedef const Matrix3 & ConstAngularRef;
    typedef Vector3 LinearType;
    typedef Vector3 & LinearRef;
    typedef const Vector3 & ConstLinearRef;
    typedef SE3Tpl<Scalar, Options> PlainType;
  };

  template<typename _Scalar, int _Options>
  struct SE3TplExpr : SE3ExprBase<SE3TplExpr<_Scalar, _Options>>
  {
    PINOCCHIO_SE3_EXPR_TYPEDEF_TPL(SE3TplExpr);

    SE3TplExpr(PlainType & se3)
    : se3(se3)
    {
    }

    ConstAngularRef rotation_impl() const
    {
      return se3.rotation();
    }
    ConstLinearRef translation_impl() const
    {
      return se3.translation();
    }
    AngularRef rotation_impl()
    {
      return se3.rotation();
    }
    LinearRef translation_impl()
    {
      return se3.translation();
    }

    template<typename OtherDerived>
    SE3TplExpr & operator=(const SE3ExprBase<OtherDerived> & other)
    {
      return SE3ExprBase<SE3TplExpr>::operator=(other);
    }

    PlainType & se3;
  };

  template<typename _Scalar, int _Options>
  struct traits<SE3TplConstExpr<_Scalar, _Options>>
  {
    enum
    {
      Options = 0
    };
    typedef _Scalar Scalar;
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;
    typedef Eigen::Matrix<Scalar, 3, 3, Options> Matrix3;
    typedef Matrix3 AngularType;
    typedef Matrix3 & AngularRef;
    typedef const Matrix3 & ConstAngularRef;
    typedef Vector3 LinearType;
    typedef Vector3 & LinearRef;
    typedef const Vector3 & ConstLinearRef;
    typedef SE3Tpl<Scalar, Options> PlainType;
  };

  template<typename _Scalar, int _Options>
  struct SE3TplConstExpr : SE3ExprBase<SE3TplConstExpr<_Scalar, _Options>>
  {
    PINOCCHIO_SE3_EXPR_TYPEDEF_TPL(SE3TplConstExpr);

    SE3TplConstExpr(const PlainType & se3)
    : se3(se3)
    {
    }

    ConstAngularRef rotation_impl() const
    {
      return se3.rotation();
    }
    ConstLinearRef translation_impl() const
    {
      return se3.translation();
    }

    const PlainType & se3;
  };

} // namespace pinocchio

#endif // ifndef __pinocchio_spatial_se3_tpl_hpp__
