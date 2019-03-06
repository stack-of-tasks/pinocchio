//
// Copyright (c) 2016,2018 CNRS, INRIA
//

#ifndef __math_quaternion_hpp__
#define __math_quaternion_hpp__

#include "pinocchio/math/fwd.hpp"

namespace pinocchio
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
      
      PINOCCHIO_EIGEN_CONST_CAST(Derived,q).w() = mult1 * s2;
      PINOCCHIO_EIGEN_CONST_CAST(Derived,q).x() = mult1 * c2;
      PINOCCHIO_EIGEN_CONST_CAST(Derived,q).y() = mult2 * s3;
      PINOCCHIO_EIGEN_CONST_CAST(Derived,q).z() = mult2 * c3;
    }
    
  } // namespace quaternion
  
  // Deprecated functions. They are now in the pinocchio::quaternion namespace
  
  /// \deprecated This function has been replaced by quaternion::angleBetweenQuaternions.
  template<typename D1, typename D2>
  typename D1::Scalar PINOCCHIO_DEPRECATED
  angleBetweenQuaternions(const Eigen::QuaternionBase<D1> & q1,
                          const Eigen::QuaternionBase<D2> & q2)
  {
    return quaternion::angleBetweenQuaternions(q1,q2);
  }
  
  /// \deprecated This function has been replaced by quaternion::defineSameRotation.
  template<typename D1, typename D2>
  bool PINOCCHIO_DEPRECATED
  defineSameRotation(const Eigen::QuaternionBase<D1> & q1,
                     const Eigen::QuaternionBase<D2> & q2,
                     const typename D1::RealScalar & prec
                     = Eigen::NumTraits<typename D1::Scalar>::dummy_precision())
  {
    return quaternion::defineSameRotation(q1,q2,prec);
  }
  
  /// \deprecated This function has been replaced by quaternion::firstOrderNormalize.
  template<typename D>
  void PINOCCHIO_DEPRECATED
  firstOrderNormalize(const Eigen::QuaternionBase<D> & q)
  {
    quaternion::firstOrderNormalize(q);
  }
  
  /// \deprecated This function has been replaced by quaternion::uniformRandom.
  template<typename D>
  void PINOCCHIO_DEPRECATED
  uniformRandom(const Eigen::QuaternionBase<D> & q)
  {
    quaternion::uniformRandom(q);
  }
}
#endif //#ifndef __math_quaternion_hpp__
