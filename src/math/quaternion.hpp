//
// Copyright (c) 2016 CNRS
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

#include <boost/math/constants/constants.hpp>

namespace se3
{
  ///
  /// \brief Compute the minimal angle between q1 and q2.
  ///
  /// \ingroup math_group
  /// 
  /// \param[in] q1 input quaternion.
  /// \param[in] q2 input quaternion.
  ///
  /// \return angle between the two quaternions
  ///
  template <typename D>
  typename D::Scalar angleBetweenQuaternions(const Eigen::QuaternionBase<D> & q1,
                                             const Eigen::QuaternionBase<D> & q2)
  {
    typedef typename D::Scalar Scalar;
    const Scalar innerprod = q1.dot(q2);
    Scalar theta = acos(innerprod);
    const Scalar PIs = boost::math::constants::pi<Scalar>();
    
    if (innerprod < 0)
      return PIs - theta;
    
    return theta;
  }
  
  ///
  /// \brief Check if two quaternions define the same rotations.
  /// \note Two quaternions define the same rotation iff q1 == q2 OR q1 == -q2.
  ///
  /// \ingroup math_group
  /// 
  /// \param[in] q1 input quaternion.
  /// \param[in] q2 input quaternion.
  ///
  /// \return Return true if the two input quaternions define the same rotation.
  ///
  template <typename Derived, typename otherDerived>
  bool defineSameRotation(const Eigen::QuaternionBase<Derived> & q1,
                          const Eigen::QuaternionBase<otherDerived> & q2)
  {
    return (q1.coeffs().isApprox(q2.coeffs()) || q1.coeffs().isApprox(-q2.coeffs()) );
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
  /// \ingroup math_group
  /// 
  /// \note See
  /// http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html#title3
  /// to know the reason why the argument is const.
  template <typename D>
  void firstOrderNormalize(const Eigen::QuaternionBase<D> & q)
  {
    typedef typename D::Scalar Scalar;
    const Scalar N2 = q.squaredNorm();
#ifndef NDEBUG
    const Scalar epsilon = sqrt(sqrt(Eigen::NumTraits<Scalar>::epsilon()));
    assert(std::fabs(N2-1.) <= epsilon);
#endif
    const Scalar alpha = ((Scalar)3 - N2) / 2;
    const_cast <Eigen::QuaternionBase<D> &> (q).coeffs() *= alpha;
#ifndef NDEBUG
    const Scalar M = 3 * std::pow((Scalar)1.-epsilon, ((Scalar)-5)/2) / 4;
    assert(std::fabs(q.norm() - 1) <=
        std::max(M * sqrt(N2) * (N2 - 1)*(N2 - 1) / 2, Eigen::NumTraits<Scalar>::epsilon()));
#endif
  }

  /// Uniformly random quaternion sphere.
  template <typename D>
  void uniformRandom (const Eigen::QuaternionBase<D> & q)
  {
    typedef typename D::Scalar Scalar;
    typedef Eigen::QuaternionBase<D> Base;

    // Rotational part
    const Scalar u1 = (Scalar)rand() / RAND_MAX;
    const Scalar u2 = (Scalar)rand() / RAND_MAX;
    const Scalar u3 = (Scalar)rand() / RAND_MAX;

    const Scalar mult1 = sqrt (1-u1);
    const Scalar mult2 = sqrt (u1);

    Scalar s2,c2; SINCOS(2.*PI*u2,&s2,&c2);
    Scalar s3,c3; SINCOS(2.*PI*u3,&s3,&c3);

    const_cast <Base &> (q).w() = mult1 * s2;
    const_cast <Base &> (q).x() = mult1 * c2;
    const_cast <Base &> (q).y() = mult2 * s3;
    const_cast <Base &> (q).z() = mult2 * c3;
  }
}
#endif //#ifndef __math_quaternion_hpp__
