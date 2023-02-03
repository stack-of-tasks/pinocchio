//
// Copyright (c) 2015-2023 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_spatial_explog_hpp__
#define __pinocchio_spatial_explog_hpp__

#include "pinocchio/fwd.hpp"
#include "pinocchio/utils/static-if.hpp"
#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/math/taylor-expansion.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/skew.hpp"
#include "pinocchio/spatial/se3.hpp"

#include <Eigen/Geometry>

#include "pinocchio/spatial/log.hpp"

namespace pinocchio
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
  typename Eigen::Matrix<typename Vector3Like::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
  exp3(const Eigen::MatrixBase<Vector3Like> & v)
  {
    PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE (Vector3Like, v, 3, 1);

    typedef typename Vector3Like::Scalar Scalar;
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like) Vector3LikePlain;
    typedef Eigen::Matrix<Scalar,3,3,Vector3LikePlain::Options> Matrix3;
    
    const Scalar t2 = v.squaredNorm();
    
    const Scalar t = math::sqrt(t2);
    Scalar ct,st; SINCOS(t,&st,&ct);

    const Scalar alpha_vxvx = internal::if_then_else(internal::GT, t, TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                                     (1 - ct)/t2,
                                                     Scalar(1)/Scalar(2) - t2/24);
    const Scalar alpha_vx = internal::if_then_else(internal::GT, t, TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                                   (st)/t,
                                                   Scalar(1) - t2/6);
    Matrix3 res(alpha_vxvx * v * v.transpose());
    res.coeffRef(0,1) -= alpha_vx * v[2]; res.coeffRef(1,0) += alpha_vx * v[2];
    res.coeffRef(0,2) += alpha_vx * v[1]; res.coeffRef(2,0) -= alpha_vx * v[1];
    res.coeffRef(1,2) -= alpha_vx * v[0]; res.coeffRef(2,1) += alpha_vx * v[0];

    ct = internal::if_then_else(internal::GT, t, TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                ct,
                                Scalar(1) - t2/2);
    res.diagonal().array() += ct;   
      
    return res;
  }
  
  /// \brief Same as \ref log3
  ///
  /// \param[in] R the rotation matrix.
  /// \param[out] theta the angle value.
  ///
  /// \return The angular velocity vector associated to the rotation matrix.
  ///
  template<typename Matrix3Like>
  Eigen::Matrix<typename Matrix3Like::Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)::Options>
  log3(const Eigen::MatrixBase<Matrix3Like> & R,
       typename Matrix3Like::Scalar & theta)
  {
    typedef typename Matrix3Like::Scalar Scalar;
    typedef Eigen::Matrix<Scalar,3,1,
                          PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)::Options> Vector3;
    Vector3 res;
    log3_impl<Scalar>::run(R, theta, res);
    return res;
  }

  /// \brief Log: SO(3)-> so(3).
  ///
  /// Pseudo-inverse of log from \f$ SO3 -> { v \in so3, ||v|| \le pi } \f$.
  ///
  /// \param[in] R The rotation matrix.
  ///
  /// \return The angular velocity vector associated to the rotation matrix.
  ///
  template<typename Matrix3Like>
  Eigen::Matrix<typename Matrix3Like::Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)::Options>
  log3(const Eigen::MatrixBase<Matrix3Like> & R)
  {
    typename Matrix3Like::Scalar theta;
    return log3(R.derived(),theta);
  }

  ///
  /// \brief Derivative of \f$ \exp{r} \f$
  /// \f[
  ///     \frac{\sin{||r||}}{||r||}                       I_3
  ///   - \frac{1-\cos{||r||}}{||r||^2}                   \left[ r \right]_x
  ///   + \frac{1}{||n||^2} (1-\frac{\sin{||r||}}{||r||}) r r^T
  /// \f]
  ///
  template<AssignmentOperatorType op, typename Vector3Like, typename Matrix3Like>
  void Jexp3(const Eigen::MatrixBase<Vector3Like> & r,
             const Eigen::MatrixBase<Matrix3Like> & Jexp)
  {
    PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE (Vector3Like, r   , 3, 1);
    PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE (Matrix3Like, Jexp, 3, 3);

    Matrix3Like & Jout = PINOCCHIO_EIGEN_CONST_CAST(Matrix3Like,Jexp);
    typedef typename Matrix3Like::Scalar Scalar;

    const Scalar n2 = r.squaredNorm();
    const Scalar n = math::sqrt(n2);
    const Scalar n_inv = Scalar(1)/n;
    const Scalar n2_inv = n_inv * n_inv;
    Scalar cn,sn; SINCOS(n,&sn,&cn);

    const Scalar a = internal::if_then_else(internal::LT, n, TaylorSeriesExpansion<Scalar>::template precision<3>(), 
                                            Scalar(1) - n2/Scalar(6),
                                            sn*n_inv);
    const Scalar b = internal::if_then_else(internal::LT, n, TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                            - Scalar(1)/Scalar(2) - n2/Scalar(24),
                                            - (1-cn)*n2_inv);
    const Scalar c = internal::if_then_else(internal::LT, n, TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                            Scalar(1)/Scalar(6) - n2/Scalar(120),
                                            n2_inv * (1 - a));

    switch(op)
      {
      case SETTO:
        Jout.diagonal().setConstant(a);
        Jout(0,1) = -b*r[2]; Jout(1,0) = -Jout(0,1);
        Jout(0,2) =  b*r[1]; Jout(2,0) = -Jout(0,2);
        Jout(1,2) = -b*r[0]; Jout(2,1) = -Jout(1,2); 
        Jout.noalias() += c * r * r.transpose();
        break;
      case ADDTO:
        Jout.diagonal().array() += a;
        Jout(0,1) += -b*r[2]; Jout(1,0) += b*r[2];
        Jout(0,2) +=  b*r[1]; Jout(2,0) += -b*r[1];
        Jout(1,2) += -b*r[0]; Jout(2,1) += b*r[0]; 
        Jout.noalias() += c * r * r.transpose();
        break;
      case RMTO:
        Jout.diagonal().array() -= a;
        Jout(0,1) -= -b*r[2]; Jout(1,0) -= b*r[2];
        Jout(0,2) -=  b*r[1]; Jout(2,0) -= -b*r[1];
        Jout(1,2) -= -b*r[0]; Jout(2,1) -= b*r[0]; 
        Jout.noalias() -= c * r * r.transpose();
        break;
      default:
        assert(false && "Wrong Op requesed value");
        break;
      }
  }

  ///
  /// \brief Derivative of \f$ \exp{r} \f$
  /// \f[
  ///     \frac{\sin{||r||}}{||r||}                       I_3
  ///   - \frac{1-\cos{||r||}}{||r||^2}                   \left[ r \right]_x
  ///   + \frac{1}{||n||^2} (1-\frac{\sin{||r||}}{||r||}) r r^T
  /// \f]
  ///
  template<typename Vector3Like, typename Matrix3Like>
  void Jexp3(const Eigen::MatrixBase<Vector3Like> & r,
             const Eigen::MatrixBase<Matrix3Like> & Jexp)
  {
    Jexp3<SETTO>(r, Jexp);
  }
  
  /** \brief Derivative of log3
   *
   * This function is the right derivative of @ref log3, that is, for \f$R \in
   * SO(3)\f$ and \f$\omega t in \mathfrak{so}(3)\f$, it provides the linear
   * approximation:
   *
   * \f[
   * \log_3(R \oplus \omega t) = \log_3(R \exp_3(\omega t)) \approx \log_3(R) + \text{Jlog3}(R) \omega t
   * \f]
   *
   *  \param[in] theta the angle value.
   *  \param[in] log the output of log3.
   *  \param[out] Jlog the jacobian
   *
   * Equivalently, \f$\text{Jlog3}\f$ is the right Jacobian of \f$\log_3\f$:
   *
   * \f[
   * \text{Jlog3}(R) = \frac{\partial \log_3(R)}{\partial R}
   * \f]
   *
   * Note that this is the right Jacobian: \f$\text{Jlog3}(R) : T_{R} SO(3) \to T_{\log_6(R)} \mathfrak{so}(3)\f$.
   * (By convention, calculations in Pinocchio always perform right differentiation,
   * i.e., Jacobians are in local coordinates (also known as body coordinates), unless otherwise specified.)
   *
   * If we denote by \f$\theta = \log_3(R)\f$ and \f$\log = \log_3(R,
   * \theta)\f$, then \f$\text{Jlog} = \text{Jlog}_3(R)\f$ can be calculated as:
   *
   *  \f[
   *  \begin{align*}
   *  \text{Jlog} & = \frac{\theta \sin(\theta)}{2 (1 - \cos(\theta))} I_3
   *             + \frac{1}{2} \widehat{\log}
   *             + \left(\frac{1}{\theta^2} - \frac{\sin(\theta)}{2\theta(1-\cos(\theta))}\right) \log \log^T \\
   *             & = I_3
   *             + \frac{1}{2} \widehat{\log}
   *             + \left(\frac{1}{\theta^2} - \frac{1 + \cos \theta}{2 \theta \sin \theta}\right) \widehat{\log}^2
   *  \end{align*}
   *  \f]
   *
   *  where \f$\widehat{v}\f$ denotes the skew-symmetric matrix obtained from the 3D vector \f$v\f$.
   *
   *  \note The inputs must be such that \f$ \theta = \Vert \log \Vert \f$.
   */
  template<typename Scalar, typename Vector3Like, typename Matrix3Like>
  void Jlog3(const Scalar & theta,
             const Eigen::MatrixBase<Vector3Like> & log,
             const Eigen::MatrixBase<Matrix3Like> & Jlog)
  {
    Jlog3_impl<Scalar>::run(theta, log,
                            PINOCCHIO_EIGEN_CONST_CAST(Matrix3Like,Jlog));
  }
  
  /** \brief Derivative of log3
   *
   *  \param[in] R the rotation matrix.
   *  \param[out] Jlog the jacobian
   *
   *  Equivalent to
   *  \code
   *  double theta;
   *  Vector3 log = pinocchio::log3 (R, theta);
   *  pinocchio::Jlog3 (theta, log, Jlog);
   *  \endcode
   */
  template<typename Matrix3Like1, typename Matrix3Like2>
  void Jlog3(const Eigen::MatrixBase<Matrix3Like1> & R,
             const Eigen::MatrixBase<Matrix3Like2> & Jlog)
  {
    typedef typename Matrix3Like1::Scalar Scalar;
    typedef Eigen::Matrix<Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like1)::Options> Vector3;

    Scalar t;
    Vector3 w(log3(R,t));
    Jlog3(t,w,PINOCCHIO_EIGEN_CONST_CAST(Matrix3Like2,Jlog));
  }

  template<typename Scalar, typename Vector3Like1, typename Vector3Like2, typename Matrix3Like>
  void Hlog3(const Scalar & theta,
             const Eigen::MatrixBase<Vector3Like1> & log,
             const Eigen::MatrixBase<Vector3Like2> & v,
             const Eigen::MatrixBase<Matrix3Like> & vt_Hlog)
  {
    typedef Eigen::Matrix<Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)::Options> Vector3;
    Matrix3Like & vt_Hlog_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3Like,vt_Hlog);

    // theta = (log^T * log)^.5
    // dt/dl = .5 * 2 * log^T * (log^T * log)^-.5
    //       = log^T / theta
    // dt_dl = log / theta
    Scalar ctheta,stheta; SINCOS(theta,&stheta,&ctheta);

    Scalar denom = .5 / (1-ctheta),
           a = theta * stheta * denom,
           da_dt = (stheta - theta) * denom, // da / dtheta
           b = ( 1 - a ) / (theta*theta),
           //db_dt = - (2 * (1 - a) / theta + da_dt ) / theta**2; // db / dtheta
           db_dt = - (2 / theta - (theta + stheta) * denom) / (theta*theta); // db / dtheta

    // Compute dl_dv_v = Jlog * v
    // Jlog = a I3 + .5 [ log ] + b * log * log^T
    // if v == log, then Jlog * v == v
    Vector3 dl_dv_v (a*v + .5*log.cross(v) + b*log*log.transpose()*v);

    Scalar dt_dv_v = log.dot(dl_dv_v) / theta;

    // Derivative of b * log * log^T
    vt_Hlog_.noalias() = db_dt * dt_dv_v * log * log.transpose();
    vt_Hlog_.noalias() += b * dl_dv_v * log.transpose();
    vt_Hlog_.noalias() += b * log * dl_dv_v.transpose();
    // Derivative of .5 * [ log ]
    addSkew(.5 * dl_dv_v, vt_Hlog_);
    // Derivative of a * I3
    vt_Hlog_.diagonal().array() += da_dt * dt_dv_v;
  }

  /** \brief Second order derivative of log3
   *
   *  This computes \f$ v^T H_{log} \f$.
   *
   *  \param[in] R the rotation matrix.
   *  \param[in] v the 3D vector.
   *  \param[out] vt_Hlog the product of the Hessian with the input vector
   */
  template<typename Matrix3Like1, typename Vector3Like, typename Matrix3Like2>
  void Hlog3(const Eigen::MatrixBase<Matrix3Like1> & R,
             const Eigen::MatrixBase<Vector3Like> & v,
             const Eigen::MatrixBase<Matrix3Like2> & vt_Hlog)
  {
    typedef typename Matrix3Like1::Scalar Scalar;
    typedef Eigen::Matrix<Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like1)::Options> Vector3;

    Scalar t;
    Vector3 w(log3(R,t));
    Hlog3(t,w,v,PINOCCHIO_EIGEN_CONST_CAST(Matrix3Like2,vt_Hlog));
  }
  
  ///
  /// \brief Exp: se3 -> SE3.
  ///
  /// Return the integral of the input twist during time 1.
  ///
  /// \param[in] nu The input twist.
  ///
  /// \return The rigid transformation associated to the integration of the twist during time 1.
  ///
  template<typename MotionDerived>
  SE3Tpl<typename MotionDerived::Scalar,PINOCCHIO_EIGEN_PLAIN_TYPE(typename MotionDerived::Vector3)::Options>
  exp6(const MotionDense<MotionDerived> & nu)
  {
    typedef typename MotionDerived::Scalar Scalar;
    enum { Options = PINOCCHIO_EIGEN_PLAIN_TYPE(typename MotionDerived::Vector3)::Options };

    typedef SE3Tpl<Scalar,Options> SE3;
    
    SE3 res;
    typename SE3::LinearType & trans = res.translation();
    typename SE3::AngularType & rot = res.rotation();
    
    const typename MotionDerived::ConstAngularType & w = nu.angular();
    const typename MotionDerived::ConstLinearType & v = nu.linear();
    
    Scalar alpha_wxv, alpha_v, alpha_w, diagonal_term;
    const Scalar t2 = w.squaredNorm();
    const Scalar t = math::sqrt(t2);
    Scalar ct,st; SINCOS(t,&st,&ct);
    const Scalar inv_t2 = Scalar(1)/t2;
    
    alpha_wxv = internal::if_then_else(internal::LT, t, TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                       Scalar(1)/Scalar(2) - t2/24,
                                       (Scalar(1) - ct)*inv_t2);
    
    alpha_v = internal::if_then_else(internal::LT, t, TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                     Scalar(1) - t2/6,
                                     (st)/t);
    
    alpha_w = internal::if_then_else(internal::LT, t, TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                     (Scalar(1)/Scalar(6) - t2/120),
                                     (Scalar(1) - alpha_v)*inv_t2);
    
    diagonal_term = internal::if_then_else(internal::LT, t, TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                           Scalar(1) - t2/2,
                                           ct);
    
    // Linear
    trans.noalias() = (alpha_v*v + (alpha_w*w.dot(v))*w + alpha_wxv*w.cross(v));
    
    // Rotational
    rot.noalias() = alpha_wxv * w * w.transpose();
    rot.coeffRef(0,1) -= alpha_v * w[2]; rot.coeffRef(1,0) += alpha_v * w[2];
    rot.coeffRef(0,2) += alpha_v * w[1]; rot.coeffRef(2,0) -= alpha_v * w[1];
    rot.coeffRef(1,2) -= alpha_v * w[0]; rot.coeffRef(2,1) += alpha_v * w[0];
    rot.diagonal().array() += diagonal_term;
    
    return res;
  }

  /// \brief Exp: se3 -> SE3.
  ///
  /// Return the integral of the input spatial velocity during time 1.
  ///
  /// \param[in] v The twist represented by a vector.
  ///
  /// \return The rigid transformation associated to the integration of the twist vector during time 1.
  ///
  template<typename Vector6Like>
  SE3Tpl<typename Vector6Like::Scalar,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector6Like)::Options>
  exp6(const Eigen::MatrixBase<Vector6Like> & v)
  {
    PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE (Vector6Like, v, 6, 1);

    MotionRef<const Vector6Like> nu(v.derived());
    return exp6(nu);
  }

  /// \brief Log: SE3 -> se3.
  ///
  /// Pseudo-inverse of exp from \f$ SE3 \to { v,\omega \in \mathfrak{se}(3), ||\omega|| < 2\pi } \f$.
  ///
  /// \param[in] M The rigid transformation.
  ///
  /// \return The twist associated to the rigid transformation during time 1.
  ///
  template<typename Scalar, int Options>
  MotionTpl<Scalar,Options>
  log6(const SE3Tpl<Scalar,Options> & M)
  {
    typedef MotionTpl<Scalar,Options> Motion;
    Motion mout;
    log6_impl<Scalar>::run(M, mout);
    return mout;
  }

  /// \brief Log: SE3 -> se3.
  ///
  /// Pseudo-inverse of exp from \f$ SE3 \to { v,\omega \in \mathfrak{se}(3), ||\omega|| < 2\pi } \f$.
  ///
  /// \param[in] M The rigid transformation represented as an homogenous matrix.
  ///
  /// \return The twist associated to the rigid transformation during time 1.
  ///
  template<typename Matrix4Like>
  MotionTpl<typename Matrix4Like::Scalar,Eigen::internal::traits<Matrix4Like>::Options>
  log6(const Eigen::MatrixBase<Matrix4Like> & M)
  {
    PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix4Like, M, 4, 4);
    
    typedef typename Matrix4Like::Scalar Scalar;
    enum {Options = Eigen::internal::traits<Matrix4Like>::Options};
    typedef MotionTpl<Scalar,Options> Motion;
    typedef SE3Tpl<Scalar,Options> SE3;
    
    SE3 m(M);
    Motion mout;
    log6_impl<Scalar>::run(m, mout);
    return mout;
  }
 
  /// \brief Derivative of exp6
  /// Computed as the inverse of Jlog6
  template<AssignmentOperatorType op, typename MotionDerived, typename Matrix6Like>
  void Jexp6(const MotionDense<MotionDerived>     & nu,
             const Eigen::MatrixBase<Matrix6Like> & Jexp)
  {
    PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE (Matrix6Like, Jexp, 6, 6);

    typedef typename MotionDerived::Scalar Scalar;
    typedef typename MotionDerived::Vector3 Vector3;
    typedef Eigen::Matrix<Scalar, 3, 3, Vector3::Options> Matrix3;
    Matrix6Like & Jout = PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,Jexp);

    const typename MotionDerived::ConstLinearType  & v = nu.linear();
    const typename MotionDerived::ConstAngularType & w = nu.angular();
    const Scalar t2 = w.squaredNorm();
    const Scalar t = math::sqrt(t2);

    const Scalar tinv = Scalar(1)/t,
                 t2inv = tinv*tinv;
    Scalar st,ct; SINCOS (t, &st, &ct);
    const Scalar inv_2_2ct = Scalar(1)/(Scalar(2)*(Scalar(1)-ct));
    
    
    const Scalar beta = internal::if_then_else(internal::LT, t, TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                               Scalar(1)/Scalar(12) + t2/Scalar(720),
                                               t2inv - st*tinv*inv_2_2ct);
    
    const Scalar beta_dot_over_theta = internal::if_then_else(internal::LT, t, TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                                              Scalar(1)/Scalar(360),
                                                              -Scalar(2)*t2inv*t2inv + (Scalar(1) + st*tinv) * t2inv * inv_2_2ct);

    switch(op)
      {
      case SETTO:
      {
        Jexp3<SETTO>(w, Jout.template bottomRightCorner<3,3>());
        Jout.template topLeftCorner<3,3>() = Jout.template bottomRightCorner<3,3>();
        const Vector3 p = Jout.template topLeftCorner<3,3>().transpose() * v;
        const Scalar wTp (w.dot (p));
        const Matrix3 J (alphaSkew(.5, p) +
                         (beta_dot_over_theta*wTp)                *w*w.transpose()
                         - (t2*beta_dot_over_theta+Scalar(2)*beta)*p*w.transpose()
                         + wTp * beta                             * Matrix3::Identity()
                         + beta                                   *w*p.transpose());
        Jout.template topRightCorner<3,3>().noalias() =
          - Jout.template topLeftCorner<3,3>() * J;
        Jout.template bottomLeftCorner<3,3>().setZero();
        break;
      }
      case ADDTO:
      {
        Matrix3 Jtmp3;
        Jexp3<SETTO>(w, Jtmp3);
        Jout.template bottomRightCorner<3,3>() += Jtmp3;
        Jout.template topLeftCorner<3,3>() += Jtmp3;
        const Vector3 p = Jtmp3.transpose() * v;
        const Scalar wTp (w.dot (p));
        const Matrix3 J (alphaSkew(.5, p) +
                         (beta_dot_over_theta*wTp)                *w*w.transpose()
                         - (t2*beta_dot_over_theta+Scalar(2)*beta)*p*w.transpose()
                         + wTp * beta                             * Matrix3::Identity()
                         + beta                                   *w*p.transpose());
        Jout.template topRightCorner<3,3>().noalias() +=
          - Jtmp3 * J;
        break;
      }
      case RMTO:
      {
        Matrix3 Jtmp3;
        Jexp3<SETTO>(w, Jtmp3);
        Jout.template bottomRightCorner<3,3>() -= Jtmp3;
        Jout.template topLeftCorner<3,3>() -= Jtmp3;
        const Vector3 p = Jtmp3.transpose() * v;
        const Scalar wTp (w.dot (p));
        const Matrix3 J (alphaSkew(.5, p) +
                         (beta_dot_over_theta*wTp)                *w*w.transpose()
                         - (t2*beta_dot_over_theta+Scalar(2)*beta)*p*w.transpose()
                         + wTp * beta                             * Matrix3::Identity()
                         + beta                                   *w*p.transpose());
        Jout.template topRightCorner<3,3>().noalias() -=
          - Jtmp3 * J;
        break;
      }
      default:
        assert(false && "Wrong Op requesed value");
        break;
      }      
  }

  /// \brief Derivative of exp6
  /// Computed as the inverse of Jlog6
  template<typename MotionDerived, typename Matrix6Like>
  void Jexp6(const MotionDense<MotionDerived>     & nu,
             const Eigen::MatrixBase<Matrix6Like> & Jexp)
  {
    Jexp6<SETTO>(nu, Jexp);
  }

  /** \brief Derivative of log6
   *
   * This function is the right derivative of @ref log6, that is, for \f$M \in
   * SE(3)\f$ and \f$\xi in \mathfrak{se}(3)\f$, it provides the linear
   * approximation:
   *
   * \f[
   * \log_6(M \oplus \xi) = \log_6(M \exp_6(\xi)) \approx \log_6(M) + \text{Jlog6}(M) \xi
   * \f]
   *
   * Equivalently, \f$\text{Jlog6}\f$ is the right Jacobian of \f$\log_6\f$:
   *
   * \f[
   * \text{Jlog6}(M) = \frac{\partial \log_6(M)}{\partial M}
   * \f]
   *
   * Note that this is the right Jacobian: \f$\text{Jlog6}(M) : T_{M} SE(3) \to T_{\log_6(M)} \mathfrak{se}(3)\f$.
   * (By convention, calculations in Pinocchio always perform right differentiation,
   * i.e., Jacobians are in local coordinates (also known as body coordinates), unless otherwise specified.)
   *
   * Internally, it is calculated using the following formulas:
   *
   *  \f[
   *  \text{Jlog6}(M) =
   *  \left(\begin{array}{cc}
   *  \text{Jlog3}(R) & J * \text{Jlog3}(R) \\
   *            0     &     \text{Jlog3}(R) \\
   *  \end{array}\right)
   *  \f]
   *  where
   *  \f[ M =
   *  \left(\begin{array}{cc}
   *  \exp(\mathbf{r}) & \mathbf{p} \\
   *             0     & 1          \\
   *  \end{array}\right)
   *  \f]
   *  \f[
   *  \begin{eqnarray}
   *  J &=&
   *  \left.\frac{1}{2}[\mathbf{p}]_{\times} + \beta'(||r||) \frac{\mathbf{r}^T\mathbf{p}}{||r||}\mathbf{r}\mathbf{r}^T
   *  - (||r||\beta'(||r||) + 2 \beta(||r||)) \mathbf{p}\mathbf{r}^T\right.\\
   *  &&\left. + \mathbf{r}^T\mathbf{p}\beta(||r||)I_3 + \beta(||r||)\mathbf{r}\mathbf{p}^T\right.
   *  \end{eqnarray}
   *  \f]
   *  and
   *  \f[ \beta(x)=\left(\frac{1}{x^2} - \frac{\sin x}{2x(1-\cos x)}\right) \f]
   *
   *
   * \cheatsheet For \f$(A,B) \in SE(3)^2\f$, let \f$M_1(A, B) = A B\f$ and
   * \f$m_1 = \log_6(M_1) \f$. Then, we have the following partial (right)
   * Jacobians: \n
   *  - \f$ \frac{\partial m_1}{\partial A} = Jlog_6(M_1) Ad_B^{-1} \f$,
   *  - \f$ \frac{\partial m_1}{\partial B} = Jlog_6(M_1) \f$.
   *
   * \cheatsheet Let \f$A \in SE(3)\f$, \f$M_2(A) = A^{-1}\f$ and \f$m_2 =
   * \log_6(M_2)\f$. Then, we have the following partial (right) Jacobian: \n
   *  - \f$ \frac{\partial m_2}{\partial A} = - Jlog_6(M_2) Ad_A \f$.
   */
  template<typename Scalar, int Options, typename Matrix6Like>
  void Jlog6(const SE3Tpl<Scalar, Options> & M,
             const Eigen::MatrixBase<Matrix6Like> & Jlog)
  {
    Jlog6_impl<Scalar>::run(M,PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,Jlog));
  }
  
  template<typename Scalar, int Options>
  template<typename OtherScalar>
  SE3Tpl<Scalar,Options> SE3Tpl<Scalar,Options>::Interpolate(const SE3Tpl & A,
                                                             const SE3Tpl & B,
                                                             const OtherScalar & alpha)
  {
    typedef SE3Tpl<Scalar,Options> ReturnType;
    typedef MotionTpl<Scalar,Options> Motion;
    
    Motion dv = log6(A.actInv(B));
    ReturnType res = A * exp6(alpha*dv);
    return res;
  }

} // namespace pinocchio

#include "pinocchio/spatial/explog-quaternion.hpp"
#include "pinocchio/spatial/log.hxx"

#endif //#ifndef __pinocchio_spatial_explog_hpp__
