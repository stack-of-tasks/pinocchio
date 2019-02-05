//
// Copyright (c) 2018 CNRS, INRIA
//

#ifndef __pinocchio_spatial_explog_quaternion_hpp__
#define __pinocchio_spatial_explog_quaternion_hpp__

#include "pinocchio/math/quaternion.hpp"

namespace pinocchio
{
  namespace quaternion
  {
    
    ///
    /// \brief Exp: so3 -> SO3 (quaternion)
    ///
    /// \returns the integral of the velocity vector as a queternion.
    ///
    /// \param[in] v The angular velocity vector.
    /// \param[out] qout The quanternion where the result is stored.
    ///
    template<typename Vector3Like, typename QuaternionLike>
    void exp3(const Eigen::MatrixBase<Vector3Like> & v,
              Eigen::QuaternionBase<QuaternionLike> & quat_out)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Vector3Like);
      assert(v.size() == 3);
      
      typedef typename Vector3Like::Scalar Scalar;
      
      const Scalar t2 = v.squaredNorm();
      
      static const Scalar ts_prec = math::sqrt(Eigen::NumTraits<Scalar>::epsilon()); // Precision for the Taylor series expansion.
      if(t2 > ts_prec)
      {
        const Scalar t = math::sqrt(t2);
        Eigen::AngleAxis<Scalar> aa(t,v/t);
        quat_out = aa;
      }
      else
      {
        quat_out.vec().noalias() = (Scalar(1)/Scalar(2) - t2/48) * v;
        quat_out.w() = Scalar(1) - t2/8;
      }
      
    }
    
    ///
    /// \brief Exp: so3 -> SO3 (quaternion)
    ///
    /// \returns the integral of the velocity vector as a queternion.
    ///
    /// \param[in] v The angular velocity vector.
    ///
    template<typename Vector3Like>
    Eigen::Quaternion<typename Vector3Like::Scalar, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
    exp3(const Eigen::MatrixBase<Vector3Like> & v)
    {
      typedef Eigen::Quaternion<typename Vector3Like::Scalar, PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options> ReturnType;
      ReturnType res; exp3(v,res);
      return res;
    }
    
    ///
    /// \brief Same as \ref log3 but with a unit quaternion as input.
    ///
    /// \param[in] quat the unit quaternion.
    /// \param[out] theta the angle value (resuling from compurations).
    ///
    /// \return The angular velocity vector associated to the rotation matrix.
    ///
    template<typename QuaternionLike>
    Eigen::Matrix<typename QuaternionLike::Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(typename QuaternionLike::Vector3)::Options>
    log3(const Eigen::QuaternionBase<QuaternionLike> & quat,
         typename QuaternionLike::Scalar & theta)
    {
      typedef typename QuaternionLike::Scalar Scalar;
      typedef Eigen::Matrix<Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(typename QuaternionLike::Vector3)::Options> Vector3;
      
      Vector3 res;
      const Scalar norm_squared = quat.vec().squaredNorm();
      const Scalar norm = math::sqrt(norm_squared);
      static const Scalar ts_prec = math::sqrt(Eigen::NumTraits<Scalar>::epsilon());
      if(norm_squared < ts_prec)
      {
        const Scalar y_x = norm / quat.w();
        theta = (1 - y_x * y_x / 3) * y_x;
        res.noalias() = (Scalar(1) + norm_squared / (6 * quat.w() * quat.w())) * quat.vec();
      }
      else
      {
        static const Scalar PI_value = PI<Scalar>();
        Scalar theta_2;
        // Here, y is always positive
        if(quat.w() >= 0.) // x >= 0. in atan2(y,x)
        {
          theta_2 = math::atan2(norm,quat.w());
          theta = 2.*theta_2;
          res.noalias() = (theta / math::sin(theta_2)) * quat.vec();
        }
        else
        { // We take here the oposite as we want to have theta in [-pi;pi];
          theta_2 = PI_value - math::atan2(norm,quat.w());
          theta = 2.*theta_2;
          res.noalias() = -(theta / math::sin(theta_2)) * quat.vec();
        }
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
    Eigen::Matrix<typename QuaternionLike::Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(typename QuaternionLike::Vector3)::Options>
    log3(const Eigen::QuaternionBase<QuaternionLike> & quat)
    {
      typename QuaternionLike::Scalar theta;
      return log3(quat.derived(),theta);
    }
    
    ///
    /// \brief Derivative of \f$ q = \exp{\mathbf{v} + \delta\mathbf{v}} \f$ where \f$ \delta\mathbf{v} \f$
    ///        is a small perturbation of \f$ \mathbf{v} \f$ at identity.
    ///
    /// \returns The Jacobian of the quaternion components variation.
    ///
    template<typename Vector3Like, typename Matrix43Like>
    void Jexp3CoeffWise(const Eigen::MatrixBase<Vector3Like> & v,
                        const Eigen::MatrixBase<Matrix43Like> & Jexp)
    {
//      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix43Like,4,3);
      assert(Jexp.rows() == 4 && Jexp.cols() == 3 && "Jexp does have the right size.");
      Matrix43Like & Jout = PINOCCHIO_EIGEN_CONST_CAST(Matrix43Like,Jexp);
      
      typedef typename Vector3Like::Scalar Scalar;
      
      const Scalar n2 = v.squaredNorm();
      const Scalar n = math::sqrt(n2);
      const Scalar theta = Scalar(0.5) * n;
      const Scalar theta2 = Scalar(0.25) * n2;
      
      if(n2 > math::sqrt(Eigen::NumTraits<Scalar>::epsilon()))
      {
        Scalar c, s;
        SINCOS(theta,&s,&c);
        Jout.template topRows<3>().noalias() = ((0.5/n2) * (c - 2*s/n)) * v * v.transpose();
        Jout.template topRows<3>().diagonal().array() += s/n;
        Jout.template bottomRows<1>().noalias() = -s/(2*n) * v.transpose();
      }
      else
      {
        Jout.template topRows<3>().noalias() =  (-1./12. + n2/480.) * v * v.transpose();
        Jout.template topRows<3>().diagonal().array() += Scalar(0.5) * (1 - theta2/6);
        Jout.template bottomRows<1>().noalias() = (Scalar(-0.25) * (1 - theta2/6)) * v.transpose();
        
      }
    }
    
    ///
    /// \brief Computes the Jacobian of log3 operator for a unit quaternion.
    ///
    /// \param[in] quat A unit quaternion representing the input rotation.
    /// \param[out] Jlog The resulting Jacobian of the log operator.
    ///
    template<typename QuaternionLike, typename Matrix3Like>
    void Jlog3(const Eigen::QuaternionBase<QuaternionLike> & quat,
               const Eigen::MatrixBase<Matrix3Like> & Jlog)
    {
      typedef typename QuaternionLike::Scalar Scalar;
      typedef Eigen::Matrix<Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(typename QuaternionLike::Coefficients)::Options> Vector3;
      
      Scalar t;
      Vector3 w(log3(quat,t));
      pinocchio::Jlog3(t,w,Jlog);
    }
  } // namespace quaternion
}

#endif // ifndef __pinocchio_spatial_explog_quaternion_hpp__
