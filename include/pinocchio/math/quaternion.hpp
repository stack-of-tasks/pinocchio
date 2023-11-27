//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_math_quaternion_hpp__
#define __pinocchio_math_quaternion_hpp__

#ifndef PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE
  #define PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE 1e-8
#endif

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/comparison-operators.hpp"
#include "pinocchio/math/matrix.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/utils/static-if.hpp"

#include <boost/type_traits.hpp>
#include <Eigen/Geometry>

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
      Scalar theta = math::acos(innerprod);
      static const Scalar PI_value = PI<Scalar>();

      theta = internal::if_then_else(internal::LT, innerprod, Scalar(0),
                                     PI_value - theta,
                                     theta);
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
      typedef apply_op_if<less_than_or_equal_to_op,is_floating_point<Scalar>::value,true> static_leq;
      assert(static_leq::op(math::fabs(static_cast<Scalar>(N2-Scalar(1))), epsilon));
#endif
      const Scalar alpha = ((Scalar)3 - N2) / Scalar(2);
      PINOCCHIO_EIGEN_CONST_CAST(D,q).coeffs() *= alpha;
#ifndef NDEBUG
      const Scalar M = Scalar(3) * math::pow(Scalar(1)-epsilon, ((Scalar)-Scalar(5))/Scalar(2)) / Scalar(4);
      assert(static_leq::op(math::fabs(static_cast<Scalar>(q.norm() - Scalar(1))),
                            math::max(M * sqrt(N2) * (N2 - Scalar(1))*(N2 - Scalar(1)) / Scalar(2), Eigen::NumTraits<Scalar>::dummy_precision())));
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
      
      static const Scalar PI_value = PI<Scalar>();
      Scalar s2,c2; SINCOS(Scalar(2)*PI_value*u2,&s2,&c2);
      Scalar s3,c3; SINCOS(Scalar(2)*PI_value*u3,&s3,&c3);
      
      PINOCCHIO_EIGEN_CONST_CAST(Derived,q).w() = mult1 * s2;
      PINOCCHIO_EIGEN_CONST_CAST(Derived,q).x() = mult1 * c2;
      PINOCCHIO_EIGEN_CONST_CAST(Derived,q).y() = mult2 * s3;
      PINOCCHIO_EIGEN_CONST_CAST(Derived,q).z() = mult2 * c3;
    }
    
    namespace internal
    {

      template<typename Scalar, bool value = is_floating_point<Scalar>::value>
      struct quaternionbase_assign_impl;
      
      template<Eigen::DenseIndex i>
      struct quaternionbase_assign_impl_if_t_negative
      {
        template<typename Scalar, typename Matrix3, typename QuaternionDerived>
        static inline void run(Scalar t,
                               Eigen::QuaternionBase<QuaternionDerived> & q,
                               const Matrix3 & mat)
        {
          using pinocchio::math::sqrt;
          
          Eigen::DenseIndex j = (i+1)%3;
          Eigen::DenseIndex k = (j+1)%3;
          
          t = sqrt(mat.coeff(i,i)-mat.coeff(j,j)-mat.coeff(k,k) + Scalar(1.0));
          q.coeffs().coeffRef(i) = Scalar(0.5) * t;
          t = Scalar(0.5)/t;
          q.w() = (mat.coeff(k,j)-mat.coeff(j,k))*t;
          q.coeffs().coeffRef(j) = (mat.coeff(j,i)+mat.coeff(i,j))*t;
          q.coeffs().coeffRef(k) = (mat.coeff(k,i)+mat.coeff(i,k))*t;
        }
      };
      
      struct quaternionbase_assign_impl_if_t_positive
      {
        template<typename Scalar, typename Matrix3, typename QuaternionDerived>
        static inline void run(Scalar t, 
                               Eigen::QuaternionBase<QuaternionDerived> & q,
                               const Matrix3 & mat)
        {
          using pinocchio::math::sqrt;
          
          t = sqrt(t + Scalar(1.0));
          q.w() = Scalar(0.5)*t;
          t = Scalar(0.5)/t;
          q.x() = (mat.coeff(2,1) - mat.coeff(1,2)) * t;
          q.y() = (mat.coeff(0,2) - mat.coeff(2,0)) * t;
          q.z() = (mat.coeff(1,0) - mat.coeff(0,1)) * t;
        }
      };
      
      template<typename Scalar>
      struct quaternionbase_assign_impl<Scalar, true>
      {
        template<typename Matrix3, typename QuaternionDerived>
        static inline void run(Eigen::QuaternionBase<QuaternionDerived> & q,
                               const Matrix3 & mat)
        {
          using pinocchio::math::sqrt;
          
          Scalar t = mat.trace();
          if (t > Scalar(0.))
            quaternionbase_assign_impl_if_t_positive::run(t,q,mat);
          else
          {
            Eigen::DenseIndex i = 0;
            if (mat.coeff(1,1) > mat.coeff(0,0))
              i = 1;
            if (mat.coeff(2,2) > mat.coeff(i,i))
              i = 2;
              
            if(i==0)
              quaternionbase_assign_impl_if_t_negative<0>::run(t,q,mat);
            else if(i==1)
              quaternionbase_assign_impl_if_t_negative<1>::run(t,q,mat);
            else
              quaternionbase_assign_impl_if_t_negative<2>::run(t,q,mat);
          }
        }
      };
      
    } // namespace internal
    
    template<typename D, typename Matrix3>
    void assignQuaternion(Eigen::QuaternionBase<D> & quat,
                          const Eigen::MatrixBase<Matrix3> & R)
    {
      internal::quaternionbase_assign_impl<typename Matrix3::Scalar>::run(PINOCCHIO_EIGEN_CONST_CAST(D,quat),
                                                                          R.derived());
    }

    ///
    /// \brief Check whether the input quaternion is Normalized within the given precision.
    ///
    /// \param[in] quat Input quaternion
    /// \param[in] prec Required precision
    ///
    /// \returns true if quat is normalized within the precision prec.
    ///
    template<typename Quaternion>
    inline bool isNormalized(const Eigen::QuaternionBase<Quaternion> & quat,
                             const typename Quaternion::Coefficients::RealScalar & prec =
                             Eigen::NumTraits< typename Quaternion::Coefficients::RealScalar >::dummy_precision())
    {
      return pinocchio::isNormalized(quat.coeffs(),prec);
    }
      
  } // namespace quaternion

}
#endif //#ifndef __pinocchio_math_quaternion_hpp__
