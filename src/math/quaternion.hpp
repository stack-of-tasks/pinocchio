//
// Copyright (c) 2016,2018 CNRS, INRIA
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

#ifndef __math_quaternion_hpp__
#define __math_quaternion_hpp__

#include "pinocchio/math/fwd.hpp"

namespace se3
{
  namespace quaternion
  {
    ///
    /// \brief Compute the minimal angle between q1 and q2.
    ///
    /// \param[in] q1 input quaternion.
    /// \param[in] q2 input quaternion.
    ///
    /// \return angle between the two quaternions
    ///
    template<typename D1, typename D2>
    typename D1::Scalar
    angleBetweenQuaternions(const Eigen::QuaternionBase<D1> & q1,
                            const Eigen::QuaternionBase<D2> & q2)
    {
      typedef typename D1::Scalar Scalar;
      const Scalar innerprod = q1.dot(q2);
      Scalar theta = acos(innerprod);
      static const Scalar PI_value = PI<Scalar>();
      
      if(innerprod < 0)
        return PI_value - theta;
      
      return theta;
    }
    
    ///
    /// \brief Check if two quaternions define the same rotations.
    /// \note Two quaternions define the same rotation iff q1 == q2 OR q1 == -q2.
    ///
    /// \param[in] q1 input quaternion.
    /// \param[in] q2 input quaternion.
    ///
    /// \return Return true if the two input quaternions define the same rotation.
    ///
    template<typename D1, typename D2>
    bool defineSameRotation(const Eigen::QuaternionBase<D1> & q1,
                            const Eigen::QuaternionBase<D2> & q2,
                            const typename D1::RealScalar & prec = Eigen::NumTraits<typename D1::Scalar>::dummy_precision())
    {
      return (q1.coeffs().isApprox(q2.coeffs(), prec) || q1.coeffs().isApprox(-q2.coeffs(), prec) );
    }
    
    /// Approximately normalize by applying the first order limited development
    /// of the normalization function.
    ///
    /// Only additions and multiplications are required. Neither square root nor
    /// division are used (except a division by 2). Let \f$ \delta = ||q||^2 - 1 \f$.
    /// Using the following limited development:
    /// \f[ \frac{1}{||q||} = (1 + \delta)^{-\frac{1}{2}} = 1 - \frac{\delta}{2} + \mathcal{O}(\delta^2) \f]
    ///
    /// The output is
    /// \f[ q_{out} = q \times \frac{3 - ||q_{in}||^2}{2} \f]
    ///
    /// The output quaternion is guaranted to statisfy the following:
    /// \f[ | ||q_{out}|| - 1 | \le \frac{M}{2} ||q_{in}|| ( ||q_{in}||^2 - 1 )^2 \f]
    /// where \f$ M = \frac{3}{4} (1 - \epsilon)^{-\frac{5}{2}} \f$
    /// and \f$ \epsilon \f$ is the maximum tolerance of \f$ ||q_{in}||^2 - 1 \f$.
    ///
    /// \warning \f$ ||q||^2 - 1 \f$ should already be close to zero.
    ///
    /// \note See
    /// http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html#title3
    /// to know the reason why the argument is const.
    template<typename D>
    void firstOrderNormalize(const Eigen::QuaternionBase<D> & q)
    {
      typedef typename D::Scalar Scalar;
      const Scalar N2 = q.squaredNorm();
#ifndef NDEBUG
      const Scalar epsilon = sqrt(sqrt(Eigen::NumTraits<Scalar>::epsilon()));
      assert(math::fabs(N2-1.) <= epsilon);
#endif
      const Scalar alpha = ((Scalar)3 - N2) / Scalar(2);
      const_cast <Eigen::QuaternionBase<D> &> (q).coeffs() *= alpha;
#ifndef NDEBUG
      const Scalar M = Scalar(3) * math::pow(Scalar(1)-epsilon, ((Scalar)-Scalar(5))/Scalar(2)) / Scalar(4);
      assert(math::fabs(q.norm() - Scalar(1)) <=
             std::max(M * sqrt(N2) * (N2 - Scalar(1))*(N2 - Scalar(1)) / Scalar(2), Eigen::NumTraits<Scalar>::dummy_precision()));
#endif
    }
    
    /// Uniformly random quaternion sphere.
    template<typename Derived>
    void uniformRandom(const Eigen::QuaternionBase<Derived> & q)
    {
      typedef typename Derived::Scalar Scalar;

      // Rotational part
      const Scalar u1 = (Scalar)rand() / RAND_MAX;
      const Scalar u2 = (Scalar)rand() / RAND_MAX;
      const Scalar u3 = (Scalar)rand() / RAND_MAX;
      
      const Scalar mult1 = sqrt(Scalar(1)-u1);
      const Scalar mult2 = sqrt(u1);
      
      const Scalar PI_value = PI<Scalar>();
      Scalar s2,c2; SINCOS(Scalar(2)*PI_value*u2,&s2,&c2);
      Scalar s3,c3; SINCOS(Scalar(2)*PI_value*u3,&s3,&c3);
      
      EIGEN_CONST_CAST(Derived,q).w() = mult1 * s2;
      EIGEN_CONST_CAST(Derived,q).x() = mult1 * c2;
      EIGEN_CONST_CAST(Derived,q).y() = mult2 * s3;
      EIGEN_CONST_CAST(Derived,q).z() = mult2 * c3;
    }
    
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
    Eigen::Quaternion<typename Vector3Like::Scalar, EIGEN_PLAIN_TYPE(Vector3Like)::Options>
    exp3(const Eigen::MatrixBase<Vector3Like> & v)
    {
      typedef Eigen::Quaternion<typename Vector3Like::Scalar, EIGEN_PLAIN_TYPE(Vector3Like)::Options> ReturnType;
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
    Eigen::Matrix<typename QuaternionLike::Scalar,3,1,EIGEN_PLAIN_TYPE(typename QuaternionLike::Vector3)::Options>
    log3(const Eigen::QuaternionBase<QuaternionLike> & quat,
         typename QuaternionLike::Scalar & theta)
    {
      typedef typename QuaternionLike::Scalar Scalar;
      typedef Eigen::Matrix<Scalar,3,1,EIGEN_PLAIN_TYPE(typename QuaternionLike::Vector3)::Options> Vector3;
      
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
    Eigen::Matrix<typename QuaternionLike::Scalar,3,1,EIGEN_PLAIN_TYPE(typename QuaternionLike::Vector3)::Options>
    log3(const Eigen::QuaternionBase<QuaternionLike> & quat)
    {
      typename QuaternionLike::Scalar theta;
      return log3(quat.derived(),theta);
    }
    
    ///
    /// \brief Derivative of \f$ q = \exp{\bm{v} + \delta\bm{v}} \f$ where \f$ \delta\bm{v} \f$
    ///        is a small perturbation of \f$ \bm{v} \f$ at identity.
    ///
    /// \returns The Jacobian of the quaternion components variation.
    ///
    template<typename Vector3Like, typename Matrix43Like>
    void Jexp3CoeffWise(const Eigen::MatrixBase<Vector3Like> & v,
                        const Eigen::MatrixBase<Matrix43Like> & Jexp)
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix43Like,4,3);
      Matrix43Like & Jout = EIGEN_CONST_CAST(Matrix43Like,Jexp);
      
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
    
  } // namespace quaternion
  
  /// Deprecated functions. They are now in the se3::pinocchio namespace
  
  /// This function has been replaced by quaternion::angleBetweenQuaternions.
  template<typename D1, typename D2>
  typename D1::Scalar PINOCCHIO_DEPRECATED
  angleBetweenQuaternions(const Eigen::QuaternionBase<D1> & q1,
                          const Eigen::QuaternionBase<D2> & q2)
  {
    return quaternion::angleBetweenQuaternions(q1,q2);
  }
  
  /// This function has been replaced by quaternion::defineSameRotation.
  template<typename D1, typename D2>
  bool PINOCCHIO_DEPRECATED
  defineSameRotation(const Eigen::QuaternionBase<D1> & q1,
                     const Eigen::QuaternionBase<D2> & q2,
                     const typename D1::RealScalar & prec
                     = Eigen::NumTraits<typename D1::Scalar>::dummy_precision())
  {
    return quaternion::defineSameRotation(q1,q2,prec);
  }
  
  /// This function has been replaced by quaternion::firstOrderNormalize.
  template<typename D>
  void PINOCCHIO_DEPRECATED
  firstOrderNormalize(const Eigen::QuaternionBase<D> & q)
  {
    quaternion::firstOrderNormalize(q);
  }
  
  /// This function has been replaced by quaternion::uniformRandom.
  template<typename D>
  void PINOCCHIO_DEPRECATED
  uniformRandom(const Eigen::QuaternionBase<D> & q)
  {
    quaternion::uniformRandom(q);
  }
}
#endif //#ifndef __math_quaternion_hpp__
