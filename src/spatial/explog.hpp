//
// Copyright (c) 2015-2018 CNRS
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
#define __spatial_explog_hpp__

#include <Eigen/Geometry>

#include "pinocchio/macros.hpp"
#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/skew.hpp"
#include "pinocchio/spatial/se3.hpp"

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
  template<typename Vector3Like>
  typename Eigen::Matrix<typename Vector3Like::Scalar,3,3,EIGEN_PLAIN_TYPE(Vector3Like)::Options>
  exp3(const Eigen::MatrixBase<Vector3Like> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Vector3Like);
    assert(v.size() == 3);
    
    typedef typename Vector3Like::Scalar Scalar;
    typedef typename EIGEN_PLAIN_TYPE(Vector3Like) Vector3LikePlain;
    typedef Eigen::Matrix<Scalar,3,3,Vector3LikePlain::Options> Matrix3;
    
    const Scalar t2 = v.squaredNorm();
    
    const Scalar t = std::sqrt(t2);
    if(t > Eigen::NumTraits<Scalar>::dummy_precision())
    {
      Scalar ct,st; SINCOS(t,&st,&ct);
      const Scalar alpha_vxvx = (1 - ct)/t2;
      const Scalar alpha_vx = (st)/t;
      Matrix3 res(alpha_vxvx * v * v.transpose());
      res.coeffRef(0,1) -= alpha_vx * v[2]; res.coeffRef(1,0) += alpha_vx * v[2];
      res.coeffRef(0,2) += alpha_vx * v[1]; res.coeffRef(2,0) -= alpha_vx * v[1];
      res.coeffRef(1,2) -= alpha_vx * v[0]; res.coeffRef(2,1) += alpha_vx * v[0];
      res.diagonal().array() += ct;
      
      return res;
    }
    else
    {
      const Scalar alpha_vxvx = Scalar(1)/Scalar(2) - t2/24;
      const Scalar alpha_vx = Scalar(1) - t2/6;
      Matrix3 res(alpha_vxvx * v * v.transpose());
      res.coeffRef(0,1) -= alpha_vx * v[2]; res.coeffRef(1,0) += alpha_vx * v[2];
      res.coeffRef(0,2) += alpha_vx * v[1]; res.coeffRef(2,0) -= alpha_vx * v[1];
      res.coeffRef(1,2) -= alpha_vx * v[0]; res.coeffRef(2,1) += alpha_vx * v[0];
      res.diagonal().array() += Scalar(1) - t2/2;
      
      return res;
    }
  }

  /// \brief Same as \ref log3
  ///
  /// \param[in] R the rotation matrix.
  /// \param[out] theta the angle value.
  ///
  /// \return The angular velocity vector associated to the rotation matrix.
  ///
  template<typename Matrix3Like, typename S2>
  Eigen::Matrix<typename Matrix3Like::Scalar,3,1,Eigen::internal::traits<Matrix3Like>::Options>
  log3(const Eigen::MatrixBase<Matrix3Like> & R, S2 & theta)
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3Like,3,3);
    typedef typename Matrix3Like::Scalar Scalar;
    typedef Eigen::Matrix<Scalar,3,1,Eigen::internal::traits<Matrix3Like>::Options> Vector3;

    const Scalar PI_value = PI<Scalar>();
    
    Vector3 res;
    const Scalar tr = R.trace();
    if (tr > Scalar(3))       theta = 0; // acos((3-1)/2)
    else if (tr < Scalar(-1)) theta = PI_value; // acos((-1-1)/2)
    else                      theta = acos((tr - Scalar(1))/Scalar(2));
    assert(theta == theta && "theta contains some NaN"); // theta != NaN
    
    // From runs of hpp-constraints/tests/logarithm.cc: 1e-6 is too small.
    if (theta < PI_value - 1e-2)
    {
      const Scalar t = ((theta > 1e-6)? theta / sin(theta) : Scalar(1)) / Scalar(2);
      res(0) = t * (R (2, 1) - R (1, 2));
      res(1) = t * (R (0, 2) - R (2, 0));
      res(2) = t * (R (1, 0) - R (0, 1));
    }
    else
    {
      // 1e-2: A low value is not required since the computation is
      // using explicit formula. However, the precision of this method
      // is the square root of the precision with the antisymmetric
      // method (Nominal case).
      const Scalar cphi = cos(theta - PI_value);
      const Scalar beta  = theta*theta / ( Scalar(1) + cphi );
      Vector3 tmp((R.diagonal().array() + cphi) * beta);
      res(0) = (R (2, 1) > R (1, 2) ? Scalar(1) : Scalar(-1)) * (tmp[0] > Scalar(0) ? sqrt(tmp[0]) : Scalar(0));
      res(1) = (R (0, 2) > R (2, 0) ? Scalar(1) : Scalar(-1)) * (tmp[1] > Scalar(0) ? sqrt(tmp[1]) : Scalar(0));
      res(2) = (R (1, 0) > R (0, 1) ? Scalar(1) : Scalar(-1)) * (tmp[2] > Scalar(0) ? sqrt(tmp[2]) : Scalar(0));
    }
    
    return res;
  }

  /// \brief Log: SO3 -> so3.
  ///
  /// Pseudo-inverse of log from \f$ SO3 -> { v \in so3, ||v|| \le pi } \f$.
  ///
  /// \param[in] R The rotation matrix.
  ///
  /// \return The angular velocity vector associated to the rotation matrix.
  ///
  template<typename Matrix3Like>
  Eigen::Matrix<typename Matrix3Like::Scalar,3,1,Eigen::internal::traits<Matrix3Like>::Options>
  log3(const Eigen::MatrixBase<Matrix3Like> & R)
  {
    typename Matrix3Like::Scalar theta;
    return log3(R.derived(), theta);
  }

  /// \brief Derivative of \f$ \exp{r} \f$
  /// \f[
  ///     \frac{\sin{||r||}}{||r||}                       I_3
  ///   - \frac{1-\cos{||r||}}{||r||^2}                   \left[ r \right]_x
  ///   + \frac{1}{||n||^2} (1-\frac{\sin{||r||}}{||r||}) r r^T
  /// \f]
  template<typename Vector3Like, typename Matrix3Like>
  void Jexp3(const Eigen::MatrixBase<Vector3Like> & r,
             const Eigen::MatrixBase<Matrix3Like> & Jexp)
  {
    Matrix3Like & Jout = const_cast<Matrix3Like &>(Jexp.derived());
    typedef typename Matrix3Like::Scalar Scalar;

    Scalar n = r.norm(),a,b,c;
    
    if (n < 1e-6) {
      Scalar n2 = n;

      a =   Scalar(1)           - n/Scalar(6)   + n2/Scalar(120);
      b = - Scalar(1)/Scalar(2) + n/Scalar(24)  - n2/Scalar(720);
      c =   Scalar(1)/Scalar(6) - n/Scalar(120) + n2/Scalar(5040);
    } else
    {
      Scalar n_inv = Scalar(1.)/n;
      Scalar n2_inv = n_inv * n_inv;
      Scalar cn,sn; SINCOS(n,&sn,&cn);

      a = sn*n_inv;
      b = - (1-cn)*n2_inv;
      c = n2_inv * (1 - a);
    }

    Jout.setZero ();
    Jout.diagonal().setConstant(a);

    Jout(0,1) = -b*r[2]; Jout(1,0) = -Jout(0,1);
    Jout(0,2) =  b*r[1]; Jout(2,0) = -Jout(0,2);
    Jout(1,2) = -b*r[0]; Jout(2,1) = -Jout(1,2);

    Jout.noalias() += c * r * r.transpose();
  }

  template<typename Scalar, typename Vector3Like, typename Matrix3Like>
  void Jlog3(const Scalar & theta,
             const Eigen::MatrixBase<Vector3Like> & log,
             const Eigen::MatrixBase<Matrix3Like> & Jlog)
  {
    Matrix3Like & Jout = const_cast<Matrix3Like &>(Jlog.derived());
    typedef typename Matrix3Like::Scalar Scalar3;
    
    if (theta < 1e-6)
      Jout.setIdentity();
    else
    {
      // Jlog = alpha I
      Scalar ct,st; SINCOS(theta,&st,&ct);
      const Scalar st_1mct = st/(Scalar(1)-ct);

      Jout.setZero ();
      Jout.diagonal().setConstant (theta*st_1mct);

      // Jlog += r_{\times}/2
      Jout(0,1) = -log(2); Jout(1,0) =  log(2);
      Jout(0,2) =  log(1); Jout(2,0) = -log(1);
      Jout(1,2) = -log(0); Jout(2,1) =  log(0);
      Jout /= Scalar3(2);

      const Scalar alpha = Scalar(1)/(theta*theta) - st_1mct/(Scalar(2)*theta);
      Jout += alpha * log * log.transpose();
    }
  }

  template<typename Matrix3Like1, typename Matrix3Like2>
  void Jlog3(const Eigen::MatrixBase<Matrix3Like1> & R,
             const Eigen::MatrixBase<Matrix3Like2> & Jlog)
  {
    typedef typename Matrix3Like1::Scalar Scalar;
    typedef Eigen::Matrix<Scalar,3,1,Eigen::internal::traits<Matrix3Like1>::Options> Vector3;

    Scalar t;
    Vector3 w(log3(R,t));
    Jlog3(t,w,Jlog);
  }

  /// \brief Exp: se3 -> SE3.
  ///
  /// Return the integral of the input twist during time 1.
  ///
  /// \param[in] nu The input twist.
  ///
  /// \return The rigid transformation associated to the integration of the twist during time 1.
  ///
  template<typename MotionDerived>
  SE3Tpl<typename MotionDerived::Scalar, Eigen::internal::traits<typename MotionDerived::Vector3>::Options>
  exp6(const MotionDense<MotionDerived> & nu)
  {
    typedef typename MotionDerived::Scalar Scalar;
    enum { Options = Eigen::internal::traits<typename MotionDerived::Vector3>::Options };

    typedef SE3Tpl<Scalar,Options> SE3;
    
    const typename MotionDerived::ConstAngularType & w = nu.angular();
    const typename MotionDerived::ConstLinearType & v = nu.linear();
    
    const Scalar t2 = w.squaredNorm();
    
    SE3 res;
    typename SE3::Linear_t & trans = res.translation();
    typename SE3::Angular_t & rot = res.rotation();
    
    const Scalar t = std::sqrt(t2);
    if(t > 1e-4)
    {
      Scalar ct,st; SINCOS(t,&st,&ct);

      const Scalar inv_t2 = Scalar(1)/t2;
      const Scalar alpha_wxv = (Scalar(1) - ct)*inv_t2;
      const Scalar alpha_v = (st)/t;
      const Scalar alpha_w = (Scalar(1) - alpha_v)*inv_t2 * w.dot(v);
      
      // Linear
      trans.noalias() = (alpha_v*v + alpha_w*w + alpha_wxv*w.cross(v));
      
      // Rotational
      rot.noalias() = alpha_wxv * w * w.transpose();
      rot.coeffRef(0,1) -= alpha_v * w[2]; rot.coeffRef(1,0) += alpha_v * w[2];
      rot.coeffRef(0,2) += alpha_v * w[1]; rot.coeffRef(2,0) -= alpha_v * w[1];
      rot.coeffRef(1,2) -= alpha_v * w[0]; rot.coeffRef(2,1) += alpha_v * w[0];
      rot.diagonal().array() += ct;
    }
    else
    {
      const Scalar alpha_wxv = Scalar(1)/Scalar(2) - t2/24;
      const Scalar alpha_v = Scalar(1) - t2/6;
      const Scalar alpha_w = (Scalar(1)/Scalar(6) - t2/120) * w.dot(v);
      
      // Linear
      trans.noalias() = (alpha_v*v + alpha_w*w + alpha_wxv*w.cross(v));
      
      // Rotational
      rot.noalias() = alpha_wxv * w * w.transpose();
      rot.coeffRef(0,1) -= alpha_v * w[2]; rot.coeffRef(1,0) += alpha_v * w[2];
      rot.coeffRef(0,2) += alpha_v * w[1]; rot.coeffRef(2,0) -= alpha_v * w[1];
      rot.coeffRef(1,2) -= alpha_v * w[0]; rot.coeffRef(2,1) += alpha_v * w[0];
      rot.diagonal().array() += Scalar(1) - t2/2;
    }
    
    return res;
  }

  /// \brief Exp: se3 -> SE3.
  ///
  /// Return the integral of the input spatial velocity during time 1.
  ///
  /// \param[in] v The twist represented by a vector.
  ///
  /// \return The rigid transformation associated to the integration of the twist vector during time 1..
  ///
  template<typename Vector6Like>
  SE3Tpl<typename Vector6Like::Scalar, Eigen::internal::traits<Vector6Like>::Options>
  exp6(const Eigen::MatrixBase<Vector6Like> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector6Like,6);
    MotionRef<Vector6Like> nu(v);
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
  template <typename Scalar, int Options>
  MotionTpl<Scalar,Options>
  log6(const SE3Tpl<Scalar,Options> & M)
  {
    typedef SE3Tpl<Scalar,Options> SE3;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef typename SE3::Vector3 Vector3;

    const typename SE3::ConstAngular_t & R = M.rotation();
    const typename SE3::ConstLinear_t & p = M.translation();
    
    Scalar t;
    Vector3 w(log3(R,t));
    const Scalar t2 = t*t;
    Scalar alpha, beta;
    if (std::fabs(t) < 1e-4)
    {
      alpha = Scalar(1) - t2/Scalar(12) - t2*t2/Scalar(720);
      beta = Scalar(1)/Scalar(12) + t2/Scalar(720);
    } else
    {
      Scalar st,ct; SINCOS(t,&st,&ct);
      alpha = t*st/(Scalar(2)*(Scalar(1)-ct));
      beta = Scalar(1)/t2 - st/(Scalar(2)*t*(Scalar(1)-ct));
    }
    
    return Motion(alpha * p - alphaSkew(0.5, w) * p + beta * w.dot(p) * w,
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
  template<typename D>
  MotionTpl<typename D::Scalar,Eigen::internal::traits<D>::Options>
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

  /// \brief Derivative of exp6
  /// Computed as the inverse of Jlog6
  template<typename MotionDerived, typename Matrix6Like>
  void Jexp6(const MotionDense<MotionDerived>     & nu,
             const Eigen::MatrixBase<Matrix6Like> & Jexp)
  {
    typedef typename MotionDerived::Scalar Scalar;
    typedef typename MotionDerived::Vector3 Vector3;
    typedef Eigen::Matrix<Scalar, 3, 3, Vector3::Options> Matrix3;
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix6Like,6,6);
    Matrix6Like & Jout = const_cast<Matrix6Like &> (Jexp.derived());

    const typename MotionDerived::ConstLinearType  & v = nu.linear();
    const typename MotionDerived::ConstAngularType & w = nu.angular();
    const Scalar t = w.norm();

    // Matrix3 J3;
    // Jexp3(w, J3);
    Jexp3(w, Jout.template bottomRightCorner<3,3>());
    Jout.template topLeftCorner<3,3>() = Jout.template bottomRightCorner<3,3>();

    const Scalar t2 = t*t;
    Scalar beta, beta_dot_over_theta;
    if (t < 1e-4) {
      beta                = Scalar(1)/Scalar(12) + t2/Scalar(720);
      beta_dot_over_theta = Scalar(1)/Scalar(360);
    } else {
      const Scalar tinv = Scalar(1)/t,
                   t2inv = tinv*tinv;
      Scalar st,ct; SINCOS (t, &st, &ct);
      const Scalar inv_2_2ct = Scalar(1)/(Scalar(2)*(Scalar(1)-ct));

      beta = t2inv - st*tinv*inv_2_2ct;
      beta_dot_over_theta = -Scalar(2)*t2inv*t2inv +
        (Scalar(1) + st*tinv) * t2inv * inv_2_2ct;
    }

    Vector3 p (Jout.template topLeftCorner<3,3>().transpose() * v);
    Scalar wTp (w.dot (p));
    Matrix3 J (alphaSkew(.5, p) +
          (beta_dot_over_theta*wTp)                *w*w.transpose()
          - (t2*beta_dot_over_theta+Scalar(2)*beta)*p*w.transpose()
          + wTp * beta                             * Matrix3::Identity()
          + beta                                   *w*p.transpose());

    Jout.template topRightCorner<3,3>().noalias() =
      - Jout.template topLeftCorner<3,3>() * J;
    Jout.template bottomLeftCorner<3,3>().setZero();
  }

  /** \brief Derivative of log6
   *  \f[
   *  \left(\begin{array}{cc}
   *  Jlog3(R) & J * Jlog3(R) \\
   *      0    &     Jlog3(R) \\
   *  \end{array}\right)
   *  where
   *  \f[
   *  \begin{eqnarray}
   *  J &=& 
   *  \frac{1}{2}[\mathbf{p}]_{\times} + \dot{\beta} (\normr) \frac{\rot^T\mathbf{p}}{\normr}\rot\rot^T
   *  - (\normr\dot{\beta} (\normr) + 2 \beta(\normr)) \mathbf{p}\rot^T\right.\\
   *  &&\left. + \rot^T\mathbf{p}\beta (\normr)I_3 + \beta (\normr)\rot\mathbf{p}^T
   *  \end{eqnarray}
   *  \f]
   *  and
   *  \f[ \beta(x)=\left(\frac{1}{x^2} - \frac{\sin x}{2x(1-\cos x)}\right) \f]
   */
  template<typename Scalar, int Options, typename Matrix6Like>
  void Jlog6(const SE3Tpl<Scalar, Options> & M,
             const Eigen::MatrixBase<Matrix6Like> & Jlog)
  {
    typedef SE3Tpl<Scalar,Options> SE3;
    typedef typename SE3::Vector3 Vector3;
    typedef typename SE3::Matrix3 Matrix3;
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix6Like,6,6);
    Matrix6Like & value = const_cast<Matrix6Like &> (Jlog.derived());

    const typename SE3::ConstAngular_t & R = M.rotation();
    const typename SE3::ConstLinear_t & p = M.translation();
    
    Scalar t;
    Vector3 w(log3(R,t));

    Matrix3 J3;
    Jlog3(t, w, J3);

    const Scalar t2 = t*t;
    Scalar beta, beta_dot_over_theta;
    if (t < 1e-4)
    {
      beta                = Scalar(1)/Scalar(12) + t2/Scalar(720);
      beta_dot_over_theta = Scalar(1)/Scalar(360);
    }
    else
    {
      const Scalar tinv = Scalar(1)/t,
                   t2inv = tinv*tinv;
      Scalar st,ct; SINCOS (t, &st, &ct);
      const Scalar inv_2_2ct = Scalar(1)/(Scalar(2)*(Scalar(1)-ct));

      beta = t2inv - st*tinv*inv_2_2ct;
      beta_dot_over_theta = -Scalar(2)*t2inv*t2inv +
        (Scalar(1) + st*tinv) * t2inv * inv_2_2ct;
    }

    Scalar wTp (w.dot (p));

    Matrix3 J ((alphaSkew(.5, p) +
          (beta_dot_over_theta*wTp)*w*w.transpose()
          - (t2*beta_dot_over_theta+Scalar(2)*beta)*p*w.transpose()
          + wTp * beta * Matrix3::Identity()
          + beta * w*p.transpose()
          ) * J3);

    value << J3             , J,
             Matrix3::Zero(), J3;
  }
} // namespace se3

#endif //#ifndef __spatial_explog_hpp__
