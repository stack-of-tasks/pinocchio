//
// Copyright (c) 2015-2017 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __spatial_explog_hpp__
# define __spatial_explog_hpp__

# include <Eigen/Geometry>

# include "pinocchio/macros.hpp"
# include "pinocchio/math/fwd.hpp"
# include "pinocchio/math/sincos.hpp"
# include "pinocchio/spatial/motion.hpp"
# include "pinocchio/spatial/skew.hpp"
# include "pinocchio/spatial/se3.hpp"

namespace se3
{
  /// \brief Exp: so3 -> SO3.
  ///
  /// Return the integral of the input angular velocity during time 1.
  ///
  /// \param[in] v The angular velocity vector.
  ///
  /// \return The rotational matrix associated to the integration of the angular velocity during time 1.
  ///
  template <typename D> Eigen::Matrix<typename D::Scalar,3,3,Eigen::internal::traits<D>::Options>
  exp3(const Eigen::MatrixBase<D> & v)
  {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE (Eigen::MatrixBase<D>,
                                          Eigen::Vector3f);
    assert (v.size () == 3);
    typename D::Scalar nv = v.norm();
    if (nv > 1e-14)
      return Eigen::AngleAxis<typename D::Scalar>(nv, v / nv).matrix();
    else
      return Eigen::Matrix<typename D::Scalar,3,3, Eigen::internal::traits<D>::Options>::Identity();
  }

  /// \brief Same as \ref log3
  ///
  /// \param[in] theta the angle value
  ///
  /// \return The angular velocity vector associated to the rotation matrix.
  ///
  template <typename D> Eigen::Matrix<typename D::Scalar,3,1,Eigen::internal::traits<D>::Options>
  log3(const Eigen::MatrixBase<D> & R, typename D::Scalar& theta)
  {
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(D, Eigen::Matrix3d);
    typedef typename D::Scalar Scalar;
    typedef Eigen::Matrix<Scalar,3,1,Eigen::internal::traits<D>::Options> Vector3;

    Vector3 value;
    const Scalar tr = R.trace();
    if (tr > 3)       theta = 0; // acos((3-1)/2)
    else if (tr < -1) theta = PI; // acos((-1-1)/2)
    else              theta = acos ((tr - 1)/2);
    assert (theta == theta); // theta != NaN
    // From runs of hpp-constraints/tests/logarithm.cc: 1e-6 is too small.
    if (theta < PI - 1e-2) {
      const Scalar t = ((theta > 1e-6)? theta / sin(theta) : 1) / 2;
      value(0) = t * (R (2, 1) - R (1, 2));
      value(1) = t * (R (0, 2) - R (2, 0));
      value(2) = t * (R (1, 0) - R (0, 1));
    } else {
      // 1e-2: A low value is not required since the computation is
      // using explicit formula. However, the precision of this method
      // is the square root of the precision with the antisymmetric
      // method (Nominal case).
      const Scalar cphi = cos(theta - PI);
      const Scalar beta  = theta*theta / ( 1 + cphi );
      Vector3 tmp ((R.diagonal().array() + cphi) * beta);
      value(0) = (R (2, 1) > R (1, 2) ? 1 : -1) * (tmp[0] > 0 ? sqrt(tmp[0]) : 0);
      value(1) = (R (0, 2) > R (2, 0) ? 1 : -1) * (tmp[1] > 0 ? sqrt(tmp[1]) : 0);
      value(2) = (R (1, 0) > R (0, 1) ? 1 : -1) * (tmp[2] > 0 ? sqrt(tmp[2]) : 0);
    }
    return value;
  }

  /// \brief Log: SO3 -> so3.
  ///
  /// Pseudo-inverse of log from \f$ SO3 -> { v \in so3, ||v|| \le pi } \f$.
  ///
  /// \param[in] R The rotation matrix.
  ///
  /// \return The angular velocity vector associated to the rotation matrix.
  ///
  template <typename D> Eigen::Matrix<typename D::Scalar,3,1,Eigen::internal::traits<D>::Options>
  log3(const Eigen::MatrixBase<D> & R)
  {
    typename D::Scalar theta;
    return log3 (R.derived(), theta);
  }

  template <typename Scalar, typename _Vector3, typename _Matrix3>
  void Jlog3 (const Scalar& theta,
              const Eigen::MatrixBase<_Vector3>& log,
              const Eigen::MatrixBase<_Matrix3>& Jlog)
  {
    _Matrix3& Jout = const_cast<_Matrix3&> (Jlog.derived());
    if (theta < 1e-6)
      Jout.setIdentity ();
    else {
      // Jlog = alpha I
      Scalar ct,st; SINCOS (theta,&st,&ct);
      const Scalar st_1mct = st/(1-ct);

      Jout.setZero ();
      Jout.diagonal().setConstant (theta*st_1mct);

      // Jlog += r_{\times}/2
      Jout(0,1) = -log(2); Jout(1,0) =  log(2);
      Jout(0,2) =  log(1); Jout(2,0) = -log(1);
      Jout(1,2) = -log(0); Jout(2,1) =  log(0);
      Jout /= 2;

      const Scalar alpha = 1/(theta*theta) - st_1mct/(2*theta);
      Jout.noalias() += alpha * log * log.transpose ();
    }
  }

  template <typename _D, typename _Matrix3>
  void Jlog3 (const Eigen::MatrixBase<_D> & R,
              const Eigen::MatrixBase<_Matrix3>& Jlog)
  {
    typedef typename _D::Scalar Scalar;
    typedef Eigen::Matrix<Scalar,3,1> Vector3;

    Scalar t;
    Vector3 w(log3(R, t));
    Jlog3 (t, w, Jlog);
  }

  /// \brief Exp: se3 -> SE3.
  ///
  /// Return the integral of the input twist during time 1.
  ///
  /// \param[in] nu The input twist.
  ///
  /// \return The rigid transformation associated to the integration of the twist during time 1.
  ///
  template <typename _Scalar, int _Options> SE3Tpl<_Scalar, _Options>
  exp6(const MotionTpl<_Scalar,_Options> & nu)
  {
    typedef _Scalar Scalar;
    typedef typename MotionTpl<Scalar,_Options>::Vector3 Vector3;
    typedef typename MotionTpl<Scalar,_Options>::Matrix3 Matrix3;

    const Vector3 & w = nu.angular();
    const Vector3 & v = nu.linear();
    Scalar t = w.norm();
    if (t > 1e-15)
    {
      const double inv_t = 1./t;
      Matrix3 S(alphaSkew(inv_t, w));
      double ct,st; SINCOS (t,&st,&ct);
      Matrix3 V((1. - ct) * inv_t * S + (1. - st * inv_t) * S * S);
      Vector3 p(v + V * v);
      return SE3Tpl<_Scalar, _Options>(exp3(w), p);
    }
    else
    {
      return SE3Tpl<_Scalar, _Options>(Matrix3::Identity(), v);
    }
  }

  /// \brief Exp: se3 -> SE3.
  ///
  /// Return the integral of the input spatial velocity during time 1.
  ///
  /// \param[in] v The twist represented by a vector.
  ///
  /// \return The rigid transformation associated to the integration of the twist vector during time 1..
  ///
  template <typename D> SE3Tpl<typename D::Scalar, Eigen::internal::traits<D>::Options>
  exp6(const Eigen::MatrixBase<D> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,6);
    MotionTpl<typename D::Scalar,Eigen::internal::traits<D>::Options> nu(v);
    return exp6(nu);
  }

  /// \brief Log: SE3 -> se3.
  ///
  /// Pseudo-inverse of exp from SE3 -> { v,w \in se3, ||w|| < 2pi }.
  ///
  /// \param[in] M The rigid transformation.
  ///
  /// \return The twist associated to the rigid transformation during time 1.
  ///
  template <typename _Scalar, int _Options>
  MotionTpl<_Scalar,_Options>
  log6(const SE3Tpl<_Scalar, _Options> & M)
  {
    typedef _Scalar Scalar;
    typedef typename SE3Tpl<Scalar,_Options>::Vector3 Vector3;
    typedef typename SE3Tpl<Scalar,_Options>::Matrix3 Matrix3;

    const Matrix3 & R = M.rotation();
    const Vector3 & p = M.translation();
    Scalar t;
    Vector3 w(log3(R, t));
    const Scalar t2 = t*t;
    Scalar alpha, beta;
    if (std::fabs(t) < 1e-4) {
      alpha = 1 - t2/12 - t2*t2/720;
      beta = 1./12 + t2/720;
    } else {
      Scalar st,ct; SINCOS (t, &st, &ct);
      alpha = t*st/(2*(1-ct));
      beta = 1/t2 - st/(2*t*(1-ct));
    }
    return MotionTpl<_Scalar,_Options>(
        alpha * p - alphaSkew(0.5, w) * p + beta * w.dot(p) * w,
        w);
  }

  /// \brief Log: SE3 -> se3.
  ///
  /// Pseudo-inverse of exp from SE3 -> { v,w \in se3, ||w|| < 2pi }.
  ///
  /// \param[in] R The rigid transformation represented as an homogenous matrix.
  ///
  /// \return The twist associated to the rigid transformation during time 1.
  ///
  template <typename D> MotionTpl<typename D::Scalar,Eigen::internal::traits<D>::Options>
  log6(const Eigen::MatrixBase<D> & M)
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(D, 4, 4);
    typedef typename SE3Tpl<typename D::Scalar,D::Options>::Vector3 Vector3;
    typedef typename SE3Tpl<typename D::Scalar,D::Options>::Matrix3 Matrix3;

    Matrix3 rot(M.template block<3,3>(0,0));
    Vector3 trans(M.template block<3,1>(0,3));
    SE3Tpl<typename D::Scalar,Eigen::internal::traits<D>::Options> m(rot, trans);
    return log6(m);
  }

  template <typename _Scalar, int _Options, typename D>
  void Jlog6 (const SE3Tpl<_Scalar, _Options> & M,
      const Eigen::MatrixBase<D>& Jlog)
  {
    typedef _Scalar Scalar;
    typedef typename SE3Tpl<Scalar,_Options>::Vector3 Vector3;
    typedef typename SE3Tpl<Scalar,_Options>::Matrix3 Matrix3;
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(D, 6, 6);
    D& value = const_cast<D&> (Jlog.derived());

    const Matrix3 & R = M.rotation();
    const Vector3 & p = M.translation();
    Scalar t;
    Vector3 w(log3(R, t));

    Matrix3 J3;
    Jlog3 (t, w, J3);

    const Scalar t2 = t*t;
    Scalar alpha, beta, beta_dot_over_theta;
    if (fabs (t) < 1e-2) {
      alpha = 1 - t2/12 - t2*t2/720;
      beta = 1./12 + t2/720;
      beta_dot_over_theta = 1. / 360.;
    } else {
      Scalar st,ct; SINCOS (t, &st, &ct);
      alpha = t*st/(2*(1-ct));
      beta = 1/(t2) - st/(2*t*(1-ct));
      beta_dot_over_theta = -2/(t2*t2) +
        (t + st) / (2*t2*t*(1-ct));
    }

    Matrix3 V (
        alpha * Matrix3::Identity ()
        - alphaSkew (.5, w) +
        beta * w * w.transpose ());

    Scalar wTp (w.dot (p));

    Matrix3 J ((alphaSkew(.5, p) +
          (beta_dot_over_theta*wTp)*w*w.transpose ()
          - (t2*beta_dot_over_theta+2*beta)*p*w.transpose ()
          + wTp * beta * Matrix3::Identity ()
          + beta * w*p.transpose ()
          ) * J3);

    value << V * R          , J,
             Matrix3::Zero(), J3;
  }
} // namespace se3

#endif //#ifndef __spatial_explog_hpp__
