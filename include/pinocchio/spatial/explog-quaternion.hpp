//
// Copyright (c) 2018-2021 CNRS INRIA
//

#ifndef __pinocchio_spatial_explog_quaternion_hpp__
#define __pinocchio_spatial_explog_quaternion_hpp__

#include "pinocchio/math/quaternion.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/utils/static-if.hpp"

namespace pinocchio
{
  namespace quaternion
  {

    ///
    /// \brief Exp: so3 -> SO3 (quaternion)
    ///
    /// \returns the integral of the velocity vector as a quaternion.
    ///
    /// \param[in] v The angular velocity vector.
    /// \param[out] qout The quaternion where the result is stored.
    ///
    template<typename Vector3Like, typename QuaternionLike>
    void
    exp3(const Eigen::MatrixBase<Vector3Like> & v, Eigen::QuaternionBase<QuaternionLike> & quat_out)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Vector3Like);
      assert(v.size() == 3);

      typedef typename Vector3Like::Scalar Scalar;
      enum
      {
        Options = PINOCCHIO_EIGEN_PLAIN_TYPE(typename QuaternionLike::Coefficients)::Options
      };
      typedef Eigen::Quaternion<typename QuaternionLike::Scalar, Options> QuaternionPlain;
      const Scalar eps = Eigen::NumTraits<Scalar>::epsilon();

      const Scalar t2 = v.squaredNorm();
      const Scalar t = math::sqrt(t2 + eps * eps);

      static const Scalar ts_prec =
        TaylorSeriesExpansion<Scalar>::template precision<3>(); // Precision for the Taylor series
                                                                // expansion.

      Eigen::AngleAxis<Scalar> aa(t, v / t);
      QuaternionPlain quat_then(aa);

      // order 4 Taylor expansion in theta / (order 2 in t2)
      QuaternionPlain quat_else;
      const Scalar t2_2 = t2 / 4; // theta/2 squared
      quat_else.vec() =
        Scalar(0.5) * (Scalar(1) - t2_2 / Scalar(6) + t2_2 * t2_2 / Scalar(120)) * v;
      quat_else.w() = Scalar(1) - t2_2 / 2 + t2_2 * t2_2 / 24;

      using ::pinocchio::internal::if_then_else;
      for (Eigen::DenseIndex k = 0; k < 4; ++k)
      {
        quat_out.coeffs().coeffRef(k) = if_then_else(
          ::pinocchio::internal::GT, t2, ts_prec, quat_then.coeffs().coeffRef(k),
          quat_else.coeffs().coeffRef(k));
      }
    }

    /// \brief Exp: so3 -> SO3 (quaternion)
    ///
    /// \returns the integral of the velocity vector as a quaternion.
    ///
    /// \param[in] v The angular velocity vector.
    ///
    template<typename Vector3Like>
    Eigen::
      Quaternion<typename Vector3Like::Scalar, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
      exp3(const Eigen::MatrixBase<Vector3Like> & v)
    {
      typedef Eigen::Quaternion<
        typename Vector3Like::Scalar, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
        ReturnType;
      ReturnType res;
      exp3(v, res);
      return res;
    }

    /// \brief The se3 -> SE3 exponential map, using quaternions to represent the output rotation.
    ///
    /// \returns the integral of the twist motion over unit time.
    ///
    /// \param[in] motion the spatial motion.
    /// \param[out] q the output transform in \f$\mathbb{R}^3 x S^3\f$.
    template<typename MotionDerived, typename Config_t>
    void exp6(const MotionDense<MotionDerived> & motion, Eigen::MatrixBase<Config_t> & qout)
    {
      enum
      {
        Options = PINOCCHIO_EIGEN_PLAIN_TYPE(Config_t)::Options
      };
      typedef typename Config_t::Scalar Scalar;
      typedef typename MotionDerived::Vector3 Vector3;
      typedef Eigen::Quaternion<Scalar, Options> Quaternion_t;
      const Scalar eps = Eigen::NumTraits<Scalar>::epsilon();

      const typename MotionDerived::ConstAngularType & w = motion.angular();
      const typename MotionDerived::ConstLinearType & v = motion.linear();

      const Scalar t2 = w.squaredNorm() + eps * eps;
      const Scalar t = math::sqrt(t2);

      Scalar ct, st;
      SINCOS(t, &st, &ct);

      const Scalar inv_t2 = Scalar(1) / t2;
      const Scalar ts_prec =
        TaylorSeriesExpansion<Scalar>::template precision<3>(); // Taylor expansion precision

      using ::pinocchio::internal::if_then_else;
      using ::pinocchio::internal::LT;

      const Scalar alpha_wxv = if_then_else(
        LT, t, ts_prec,
        Scalar(0.5) - t2 / Scalar(24), // then: use Taylor expansion
        (Scalar(1) - ct) * inv_t2      // else
      );

      const Scalar alpha_w2 = if_then_else(
        LT, t, ts_prec, Scalar(1) / Scalar(6) - t2 / Scalar(120), (t - st) * inv_t2 / t);

      // linear part
      Eigen::Map<Vector3> trans_(qout.derived().template head<3>().data());
      trans_.noalias() = v + alpha_wxv * w.cross(v) + alpha_w2 * w.cross(w.cross(v));

      // quaternion part
      typedef Eigen::Map<Quaternion_t> QuaternionMap_t;
      QuaternionMap_t quat_(qout.derived().template tail<4>().data());
      exp3(w, quat_);
    }

    /// \brief The se3 -> SE3 exponential map, using quaternions to represent the output rotation.
    ///
    /// \returns the integral of the twist motion over unit time.
    ///
    /// \param[in] motion the spatial motion.
    template<typename MotionDerived>
    Eigen::Matrix<
      typename MotionDerived::Scalar,
      7,
      1,
      PINOCCHIO_EIGEN_PLAIN_TYPE(typename MotionDerived::Vector3)::Options>
    exp6(const MotionDense<MotionDerived> & motion)
    {
      typedef typename MotionDerived::Scalar Scalar;
      enum
      {
        Options = PINOCCHIO_EIGEN_PLAIN_TYPE(typename MotionDerived::Vector3)::Options
      };
      typedef Eigen::Matrix<Scalar, 7, 1, Options> ReturnType;

      ReturnType qout;
      exp6(motion, qout);
      return qout;
    }

    /// \brief The se3 -> SE3 exponential map, using quaternions to represent the output rotation.
    ///
    /// \returns the integral of the spatial velocity over unit time.
    ///
    /// \param[in] vec6 the vector representing the spatial velocity.
    /// \param[out] qout the output transform in R^3 x S^3.
    template<typename Vector6Like, typename Config_t>
    void exp6(const Eigen::MatrixBase<Vector6Like> & vec6, Eigen::MatrixBase<Config_t> & qout)
    {
      MotionRef<const Vector6Like> nu(vec6.derived());
      ::pinocchio::quaternion::exp6(nu, qout);
    }

    /// \brief The se3 -> SE3 exponential map, using quaternions to represent the output rotation.
    ///
    /// \returns the integral of the spatial velocity over unit time.
    ///
    /// \param[in] vec6 the vector representing the spatial velocity.
    template<typename Vector6Like>
    Eigen::
      Matrix<typename Vector6Like::Scalar, 7, 1, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector6Like)::Options>
      exp6(const Eigen::MatrixBase<Vector6Like> & vec6)
    {
      typedef typename Vector6Like::Scalar Scalar;
      enum
      {
        Options = PINOCCHIO_EIGEN_PLAIN_TYPE(Vector6Like)::Options
      };
      typedef Eigen::Matrix<Scalar, 7, 1, Options> ReturnType;

      ReturnType qout;
      ::pinocchio::quaternion::exp6(vec6, qout);
      return qout;
    }

    /// \brief Same as \ref log3 but with a unit quaternion as input.
    ///
    /// \param[in] quat the unit quaternion.
    /// \param[out] theta the angle value (resuling from compurations).
    ///
    /// \return The angular velocity vector associated to the rotation matrix.
    ///
    template<typename QuaternionLike>
    Eigen::Matrix<
      typename QuaternionLike::Scalar,
      3,
      1,
      PINOCCHIO_EIGEN_PLAIN_TYPE(typename QuaternionLike::Vector3)::Options>
    log3(
      const Eigen::QuaternionBase<QuaternionLike> & quat, typename QuaternionLike::Scalar & theta)
    {
      typedef typename QuaternionLike::Scalar Scalar;
      enum
      {
        Options = PINOCCHIO_EIGEN_PLAIN_TYPE(typename QuaternionLike::Vector3)::Options
      };
      typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;

      Vector3 res;
      const Scalar norm_squared = quat.vec().squaredNorm();

      static const Scalar eps = Eigen::NumTraits<Scalar>::epsilon();
      static const Scalar ts_prec = TaylorSeriesExpansion<Scalar>::template precision<2>();
      const Scalar norm = math::sqrt(norm_squared + eps * eps);

      using ::pinocchio::internal::GE;
      using ::pinocchio::internal::if_then_else;
      using ::pinocchio::internal::LT;

      const Scalar pos_neg = if_then_else(GE, quat.w(), Scalar(0), Scalar(+1), Scalar(-1));

      Eigen::Quaternion<Scalar, Options> quat_pos;
      quat_pos.w() = pos_neg * quat.w();
      quat_pos.vec() = pos_neg * quat.vec();

      const Scalar theta_2 = math::atan2(norm, quat_pos.w()); // in [0,pi]
      const Scalar y_x = norm / quat_pos.w();                 // nonnegative
      const Scalar y_x_sq = norm_squared / (quat_pos.w() * quat_pos.w());

      theta = if_then_else(
        LT, norm_squared, ts_prec, Scalar(2.) * (Scalar(1) - y_x_sq / Scalar(3)) * y_x,
        Scalar(2.) * theta_2);

      const Scalar th2_2 = theta * theta / Scalar(4);
      const Scalar inv_sinc = if_then_else(
        LT, norm_squared, ts_prec,
        Scalar(2) * (Scalar(1) + th2_2 / Scalar(6) + Scalar(7) / Scalar(360) * th2_2 * th2_2),
        theta / math::sin(theta_2));

      for (Eigen::DenseIndex k = 0; k < 3; ++k)
      {
        // res[k] = if_then_else(LT, norm_squared, ts_prec,
        //                       Scalar(2) * (Scalar(1) + y_x_sq / Scalar(6) - y_x_sq*y_x_sq /
        //                       Scalar(9)) * pos_neg * quat.vec()[k], inv_sinc * pos_neg *
        //                       quat.vec()[k]);
        res[k] = inv_sinc * quat_pos.vec()[k];
      }
      return res;
    }

    ///
    /// \brief Log: SO3 -> so3.
    ///
    /// Pseudo-inverse of log from \f$ SO3 -> { v \in so3, ||v|| \le pi } \f$.
    ///
    /// \param[in] quat The unit quaternion representing a certain rotation.
    ///
    /// \return The angular velocity vector associated to the quaternion.
    ///
    template<typename QuaternionLike>
    Eigen::Matrix<
      typename QuaternionLike::Scalar,
      3,
      1,
      PINOCCHIO_EIGEN_PLAIN_TYPE(typename QuaternionLike::Vector3)::Options>
    log3(const Eigen::QuaternionBase<QuaternionLike> & quat)
    {
      typename QuaternionLike::Scalar theta;
      return log3(quat.derived(), theta);
    }

    ///
    /// \brief Derivative of \f$ q = \exp{\mathbf{v} + \delta\mathbf{v}} \f$ where \f$
    /// \delta\mathbf{v} \f$
    ///        is a small perturbation of \f$ \mathbf{v} \f$ at identity.
    ///
    /// \returns The Jacobian of the quaternion components variation.
    ///
    template<typename Vector3Like, typename Matrix43Like>
    void Jexp3CoeffWise(
      const Eigen::MatrixBase<Vector3Like> & v, const Eigen::MatrixBase<Matrix43Like> & Jexp)
    {
      //      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix43Like,4,3);
      assert(Jexp.rows() == 4 && Jexp.cols() == 3 && "Jexp does have the right size.");
      Matrix43Like & Jout = PINOCCHIO_EIGEN_CONST_CAST(Matrix43Like, Jexp);

      typedef typename Vector3Like::Scalar Scalar;

      const Scalar n2 = v.squaredNorm();
      const Scalar n = math::sqrt(n2);
      const Scalar theta = Scalar(0.5) * n;
      const Scalar theta2 = Scalar(0.25) * n2;

      if (n2 > math::sqrt(Eigen::NumTraits<Scalar>::epsilon()))
      {
        Scalar c, s;
        SINCOS(theta, &s, &c);
        Jout.template topRows<3>().noalias() =
          ((Scalar(0.5) / n2) * (c - 2 * s / n)) * v * v.transpose();
        Jout.template topRows<3>().diagonal().array() += s / n;
        Jout.template bottomRows<1>().noalias() = -s / (2 * n) * v.transpose();
      }
      else
      {
        Jout.template topRows<3>().noalias() =
          (-Scalar(1) / Scalar(12) + n2 / Scalar(480)) * v * v.transpose();
        Jout.template topRows<3>().diagonal().array() += Scalar(0.5) * (1 - theta2 / 6);
        Jout.template bottomRows<1>().noalias() =
          (Scalar(-0.25) * (Scalar(1) - theta2 / 6)) * v.transpose();
      }
    }

    ///
    ///  \brief Computes the Jacobian of log3 operator for a unit quaternion.
    ///
    /// \param[in] quat A unit quaternion representing the input rotation.
    /// \param[out] Jlog The resulting Jacobian of the log operator.
    ///
    template<typename QuaternionLike, typename Matrix3Like>
    void Jlog3(
      const Eigen::QuaternionBase<QuaternionLike> & quat,
      const Eigen::MatrixBase<Matrix3Like> & Jlog)
    {
      typedef typename QuaternionLike::Scalar Scalar;
      typedef Eigen::Matrix<
        Scalar, 3, 1, PINOCCHIO_EIGEN_PLAIN_TYPE(typename QuaternionLike::Coefficients)::Options>
        Vector3;

      Scalar t;
      Vector3 w(log3(quat, t));
      pinocchio::Jlog3(t, w, PINOCCHIO_EIGEN_CONST_CAST(Matrix3Like, Jlog));
    }
  } // namespace quaternion
} // namespace pinocchio

#endif // ifndef __pinocchio_spatial_explog_quaternion_hpp__
