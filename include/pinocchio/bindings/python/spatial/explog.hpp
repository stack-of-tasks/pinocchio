//
// Copyright (c) 2015-2021 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_python_explog_hpp__
#define __pinocchio_python_explog_hpp__

#include "pinocchio/spatial/explog.hpp"

namespace pinocchio
{
  namespace python
  {

    template<typename Vector3Like>
    Eigen::
      Matrix<typename Vector3Like::Scalar, 3, 3, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
      exp3_proxy(const Vector3Like & v)
    {
      return exp3(v);
    }

    template<typename Vector3Like>
    Eigen::
      Matrix<typename Vector3Like::Scalar, 4, 1, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
      exp3_proxy_quat(const Vector3Like & v)
    {
      typedef typename Vector3Like::Scalar Scalar;
      typedef Eigen::Quaternion<Scalar, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
        Quaternion_t;
      typedef Eigen::Map<Quaternion_t> QuaternionMap_t;
      typedef Eigen::Matrix<Scalar, 4, 1, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
        ReturnType;
      ReturnType res;
      QuaternionMap_t quat_out(res.derived().data());
      quaternion::exp3(v, quat_out);
      return res;
    }

    template<typename Vector3Like>
    Eigen::
      Matrix<typename Vector3Like::Scalar, 3, 3, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
      Jexp3_proxy(const Vector3Like & v)
    {
      typedef Eigen::Matrix<
        typename Vector3Like::Scalar, 3, 3, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
        ReturnType;
      ReturnType res;
      Jexp3(v, res);
      return res;
    }

    template<typename Matrix3Like>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like) Jlog3_proxy(const Matrix3Like & M)
    {
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like) ReturnType;
      ReturnType res;
      Jlog3(M, res);
      return res;
    }

    template<typename Matrix3Like, typename Vector3Like>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)
      Hlog3_proxy(const Matrix3Like & M, const Vector3Like & v)
    {
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like) ReturnType;
      ReturnType res;
      Hlog3(M, v, res);
      return res;
    }

    template<typename Scalar, int Options>
    SE3Tpl<Scalar, Options> exp6_proxy(const MotionTpl<Scalar, Options> & v)
    {
      return exp6(v);
    }

    template<typename Vector6Like>
    SE3Tpl<typename Vector6Like::Scalar, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector6Like)::Options>
    exp6_proxy(const Vector6Like & vec6)
    {
      return exp6(vec6);
    }

    template<typename Vector6Like>
    Eigen::
      Matrix<typename Vector6Like::Scalar, 7, 1, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector6Like)::Options>
      exp6_proxy_quatvec(const Vector6Like & vec6)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector6Like, 6);
      return quaternion::exp6(vec6); // use quaternion-exp6 overload
    }

    template<typename Scalar, int Options>
    typename SE3Tpl<Scalar, Options>::Matrix6 Jlog6_proxy(const SE3Tpl<Scalar, Options> & M)
    {
      typedef typename SE3Tpl<Scalar, Options>::Matrix6 ReturnType;
      ReturnType res;
      Jlog6(M, res);
      return res;
    }

    template<typename Scalar, int Options>
    typename MotionTpl<Scalar, Options>::Matrix6 Jexp6_proxy(const MotionTpl<Scalar, Options> & v)
    {
      typedef typename MotionTpl<Scalar, Options>::Matrix6 ReturnType;
      ReturnType res;
      Jexp6(v, res);
      return res;
    }

    template<typename Vector6Like>
    Eigen::
      Matrix<typename Vector6Like::Scalar, 6, 6, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector6Like)::Options>
      Jexp6_proxy(const Vector6Like & vec6)
    {
      typedef MotionRef<const Vector6Like> Motion;
      Motion v(vec6);
      typedef typename Motion::Matrix6 ReturnType;
      ReturnType res;
      Jexp6(v, res);
      return res;
    }

    template<typename Matrix3Like>
    Eigen::
      Matrix<typename Matrix3Like::Scalar, 3, 1, PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)::Options>
      log3_proxy(const Matrix3Like & R)
    {
      return log3(R);
    }

    template<typename Matrix3Like, typename Matrix1Like>
    Eigen::
      Matrix<typename Matrix3Like::Scalar, 3, 1, PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)::Options>
      log3_proxy(const Matrix3Like & R, Eigen::Ref<Matrix1Like> theta)
    {
      return log3(R, theta.coeffRef(0, 0));
    }

    template<typename Matrix3Like, typename Scalar>
    Eigen::
      Matrix<typename Matrix3Like::Scalar, 3, 1, PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)::Options>
      log3_proxy_fix(const Matrix3Like & R, Scalar & theta)
    {
      return log3(R, theta);
    }

    template<typename QuaternionLike>
    Eigen::Matrix<
      typename QuaternionLike::Scalar,
      3,
      1,
      PINOCCHIO_EIGEN_PLAIN_TYPE(typename QuaternionLike::Vector3)::Options>
    log3_proxy(const QuaternionLike & quat)
    {
      return quaternion::log3(quat);
    }

    template<typename Vector4Like>
    Eigen::
      Matrix<typename Vector4Like::Scalar, 3, 1, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector4Like)::Options>
      log3_proxy_quatvec(const Vector4Like & v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector4Like, 4);
      typedef typename Vector4Like::Scalar Scalar;
      typedef Eigen::Quaternion<Scalar, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector4Like)::Options>
        Quaternion_t;
      typedef Eigen::Map<const Quaternion_t> ConstQuaternionMap_t;

      ConstQuaternionMap_t q(v.derived().data());
      assert(quaternion::isNormalized(
        q, typename Vector4Like::RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      return quaternion::log3(q);
    }

    template<typename Vector4Like, typename Matrix1Like>
    Eigen::
      Matrix<typename Vector4Like::Scalar, 3, 1, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector4Like)::Options>
      log3_proxy_quatvec(const Vector4Like & v, Eigen::Ref<Matrix1Like> theta)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector4Like, 4);
      typedef typename Vector4Like::Scalar Scalar;
      typedef Eigen::Quaternion<Scalar, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector4Like)::Options>
        Quaternion_t;
      typedef Eigen::Map<const Quaternion_t> ConstQuaternionMap_t;

      ConstQuaternionMap_t q(v.derived().data());
      assert(quaternion::isNormalized(
        q, typename Vector4Like::RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));

      return quaternion::log3(q, theta.coeffRef(0, 0));
    }

    template<typename Vector4Like, typename _Scalar>
    Eigen::
      Matrix<typename Vector4Like::Scalar, 3, 1, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector4Like)::Options>
      log3_proxy_quatvec_fix(const Vector4Like & v, _Scalar & theta)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector4Like, 4);
      typedef typename Vector4Like::Scalar Scalar;
      typedef Eigen::Quaternion<Scalar, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector4Like)::Options>
        Quaternion_t;
      typedef Eigen::Map<const Quaternion_t> ConstQuaternionMap_t;

      ConstQuaternionMap_t q(v.derived().data());
      assert(quaternion::isNormalized(
        q, typename Vector4Like::RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));

      return quaternion::log3(q, theta);
    }

    template<typename Matrix4Like>
    MotionTpl<typename Matrix4Like::Scalar, PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix4Like)::Options>
    log6_proxy(const Matrix4Like & homegenous_matrix)
    {
      return log6(homegenous_matrix);
    }

    template<typename Vector7Like>
    MotionTpl<typename Vector7Like::Scalar, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector7Like)::Options>
    log6_proxy_quatvec(const Vector7Like & q)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector7Like, 7);
      typedef typename Vector7Like::Scalar Scalar;
      enum
      {
        Options = PINOCCHIO_EIGEN_PLAIN_TYPE(Vector7Like)::Options
      };
      typedef Eigen::Quaternion<Scalar, Options> Quaternion;
      typedef Eigen::Map<const Quaternion, Options> ConstQuaternionMap;
      typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;

      const Vector3 v(q.derived().template head<3>());
      ConstQuaternionMap quat(q.derived().template tail<4>().data());

      return log6(quat, v);
    }
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_explog_hpp__
