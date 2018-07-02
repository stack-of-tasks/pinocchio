//
// Copyright (c) 2016-2018 CNRS
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

#ifndef __se3_special_orthogonal_operation_hpp__
#define __se3_special_orthogonal_operation_hpp__

#include <limits>

#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/multibody/liegroup/operation-base.hpp>

namespace se3
{
  template<int N> struct SpecialOrthogonalOperation {};
  template<int N> struct traits<SpecialOrthogonalOperation<N> > {};

  template <> struct traits<SpecialOrthogonalOperation<2> > {
    typedef double Scalar;
    enum {
      NQ = 2,
      NV = 1
    };
  };

  template <> struct traits<SpecialOrthogonalOperation<3> > {
    typedef double Scalar;
    enum {
      NQ = 4,
      NV = 3
    };
  };

  template<>
  struct SpecialOrthogonalOperation<2> : public LieGroupOperationBase <SpecialOrthogonalOperation<2> >
  {
    SE3_LIE_GROUP_PUBLIC_INTERFACE(SpecialOrthogonalOperation);
    typedef Eigen::Matrix<Scalar,2,2> Matrix2;

    static Scalar log (const Matrix2& R)
    {
      Scalar theta;
      const Scalar tr = R.trace();
      const bool pos = (R (1, 0) > 0);
      if (tr > 2)       theta = 0; // acos((3-1)/2)
      else if (tr < -2) theta = (pos ? PI : -PI); // acos((-1-1)/2)
      // Around 0, asin is numerically more stable than acos because
      // acos(x) = PI/2 - x and asin(x) = x (the precision of x is not lost in PI/2).
      else if (tr > 2 - 1e-2) theta = asin ((R(1,0) - R(0,1)) / 2);
      else              theta = (pos ? acos (tr/2) : -acos(tr/2));
      assert (theta == theta); // theta != NaN
      assert (fabs(theta - atan2 (R(1,0), R(0,0))) < 1e-6);
      return theta;
    }

    static Scalar Jlog (const Matrix2&)
    {
      return 1;
    }

    /// Get dimension of Lie Group vector representation
    ///
    /// For instance, for SO(3), the dimension of the vector representation is
    /// 4 (quaternion) while the dimension of the tangent space is 3.
    Index nq () const
    {
      return NQ;
    }
    /// Get dimension of Lie Group tangent space
    Index nv () const
    {
      return NV;
    }

    ConfigVector_t neutral () const
    {
      ConfigVector_t n; n.setZero (); n [0] = 1;
      return n;
    }

    std::string name () const
    {
      return std::string ("SO(2)");
    }

    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    static void difference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1,
                                const Eigen::MatrixBase<Tangent_t> & d)
    {
      Matrix2 R; // R0.transpose() * R1;
      R(0,0) = R(1,1) = q0.dot(q1);
      R(1,0) = q0(0) * q1(1) - q0(1) * q1(0);
      R(0,1) = - R(1,0);
      const_cast < Tangent_t& > (d.derived()) [0] = log (R);
    }

    template <class ConfigL_t, class ConfigR_t, class JacobianLOut_t, class JacobianROut_t>
    static void Jdifference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                 const Eigen::MatrixBase<ConfigR_t> & q1,
                                 const Eigen::MatrixBase<JacobianLOut_t>& J0,
                                 const Eigen::MatrixBase<JacobianROut_t>& J1)
    {
      Matrix2 R; // R0.transpose() * R1;
      R(0,0) = R(1,1) = q0.dot(q1);
      R(1,0) = q0(0) * q1(1) - q0(1) * q1(0);
      R(0,1) = - R(1,0);

      Scalar w (Jlog(R));
      const_cast< JacobianLOut_t& > (J0.derived()).coeffRef(0) = -w;
      const_cast< JacobianROut_t& > (J1.derived()).coeffRef(0) =  w;
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    static void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      ConfigOut_t& out = (const_cast< Eigen::MatrixBase<ConfigOut_t>& >(qout)).derived();

      const Scalar & ca = q(0);
      const Scalar & sa = q(1);
      const Scalar & omega = v(0);

      Scalar cosOmega,sinOmega; SINCOS(omega, &sinOmega, &cosOmega);
      // TODO check the cost of atan2 vs SINCOS

      out << cosOmega * ca - sinOmega * sa,
             sinOmega * ca + cosOmega * sa;
      const Scalar norm2 = q.squaredNorm();
      out *= (3 - norm2) / 2;
    }

    template <class ConfigIn_t, class Tangent_t, class JacobianLOut_t, class JacobianROut_t>
    static void Jintegrate_impl ( const Eigen::MatrixBase<ConfigIn_t> & /*q*/,
                                  const Eigen::MatrixBase<Tangent_t>  & /*v*/,
                                  const Eigen::MatrixBase<JacobianLOut_t>& Jq,
                                  const Eigen::MatrixBase<JacobianROut_t>& Jv)
    {
      JacobianLOut_t& Jqout = const_cast< JacobianLOut_t& >(Jq.derived());
      JacobianROut_t& Jvout = const_cast< JacobianROut_t& >(Jv.derived());
      Jqout(0) = 1;
      Jvout(0) = 1;
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    static void interpolate_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                 const Eigen::MatrixBase<ConfigR_t> & q1,
                                 const Scalar& u,
                                 const Eigen::MatrixBase<ConfigOut_t>& qout)
    {
      ConfigOut_t& out = (const_cast< Eigen::MatrixBase<ConfigOut_t>& >(qout)).derived();

      assert ( (q0.norm() - 1) < 1e-8 && "initial configuration not normalized");
      assert ( (q1.norm() - 1) < 1e-8 && "final configuration not normalized");
      Scalar cosTheta = q0.dot(q1);
      Scalar sinTheta = q0(0)*q1(1) - q0(1)*q1(0);
      Scalar theta = atan2(sinTheta, cosTheta);
      assert (fabs (sin (theta) - sinTheta) < 1e-8);

      if (fabs (theta) > 1e-6 && fabs (theta) < PI - 1e-6)
      {
        out = (sin ((1-u)*theta)/sinTheta) * q0
            + (sin (   u *theta)/sinTheta) * q1;
      }
      else if (fabs (theta) < 1e-6) // theta = 0
      {
        out = (1-u) * q0 + u * q1;
      }
      else // theta = +-PI
      {
        Scalar theta0 = atan2 (q0(1), q0(0));
        SINCOS(theta0,&out[1],&out[0]);
      }
    }

    // template <class ConfigL_t, class ConfigR_t>
    // static double squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                       // const Eigen::MatrixBase<ConfigR_t> & q1)
    
    template <class Config_t>
    static void normalize_impl (const Eigen::MatrixBase<Config_t>& qout)
    {
      Config_t& qout_ = (const_cast< Eigen::MatrixBase<Config_t>& >(qout)).derived();
      qout_.normalize();
    }

    template <class Config_t>
    void random_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      Config_t& out = (const_cast< Eigen::MatrixBase<Config_t>& >(qout)).derived();
      const Scalar angle = -PI + 2*PI * ((Scalar)rand())/RAND_MAX;
      SINCOS (angle, &out(1), &out(0));
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> &,
                                  const Eigen::MatrixBase<ConfigR_t> &,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout)
      const
    {
      random_impl(qout);
    }
  }; // struct SpecialOrthogonalOperation<2>

  template<>
  struct SpecialOrthogonalOperation<3> : public LieGroupOperationBase <SpecialOrthogonalOperation<3> >
  {
    SE3_LIE_GROUP_PUBLIC_INTERFACE(SpecialOrthogonalOperation);

    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
    typedef Eigen::Quaternion<Scalar> Quaternion_t;
    typedef Eigen::Map<      Quaternion_t> QuaternionMap_t;
    typedef Eigen::Map<const Quaternion_t> ConstQuaternionMap_t;

    /// Get dimension of Lie Group vector representation
    ///
    /// For instance, for SO(3), the dimension of the vector representation is
    /// 4 (quaternion) while the dimension of the tangent space is 3.
    Index nq () const
    {
      return NQ;
    }
    /// Get dimension of Lie Group tangent space
    Index nv () const
    {
      return NV;
    }

    ConfigVector_t neutral () const
    {
      ConfigVector_t n; n.setZero (); n [3] = 1;
      return n;
    }

    std::string name () const
    {
      return std::string ("SO(3)");
    }

    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    static void difference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1,
                                const Eigen::MatrixBase<Tangent_t> & d)
    {
      ConstQuaternionMap_t p0 (q0.derived().data());
      ConstQuaternionMap_t p1 (q1.derived().data());
      const_cast < Eigen::MatrixBase<Tangent_t>& > (d)
        = log3((p0.matrix().transpose() * p1.matrix()).eval());
    }

    template <class ConfigL_t, class ConfigR_t, class JacobianLOut_t, class JacobianROut_t>
    static void Jdifference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                 const Eigen::MatrixBase<ConfigR_t> & q1,
                                 const Eigen::MatrixBase<JacobianLOut_t>& J0,
                                 const Eigen::MatrixBase<JacobianROut_t>& J1)
    {
      ConstQuaternionMap_t p0 (q0.derived().data());
      ConstQuaternionMap_t p1 (q1.derived().data());
      Eigen::Matrix<Scalar, 3, 3> R = p0.matrix().transpose() * p1.matrix();

      Jlog3 (R, J1);

      JacobianLOut_t& J0v = const_cast< JacobianLOut_t& > (J0.derived());
      J0v.noalias() = - J1.derived() * R.transpose();
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    static void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      ConstQuaternionMap_t quat(q.derived().data());
      QuaternionMap_t quaternion_result (
          (const_cast< Eigen::MatrixBase<ConfigOut_t>& >(qout)).derived().data()
          );

      Quaternion_t pOmega(exp3(v));
      quaternion_result = quat * pOmega;
      firstOrderNormalize(quaternion_result);
    }

    template <class ConfigIn_t, class Tangent_t, class JacobianLOut_t, class JacobianROut_t>
    static void Jintegrate_impl ( const Eigen::MatrixBase<ConfigIn_t> & q,
                                  const Eigen::MatrixBase<Tangent_t>  & v,
                                  const Eigen::MatrixBase<JacobianLOut_t>& Jq,
                                  const Eigen::MatrixBase<JacobianROut_t>& Jv)
    {
      JacobianLOut_t& JqOut = const_cast< JacobianLOut_t& >(Jq.derived());
      JacobianROut_t& JvOut = const_cast< JacobianROut_t& >(Jv.derived());
      JqOut = exp3(v).transpose();
      Matrix3 R (ConstQuaternionMap_t(q.derived().data()).matrix());
      Scalar vn = v.norm(), cvn, svn; SINCOS(vn, &svn, &cvn);
      Scalar alpha = (2*(1-cvn) - vn * svn) / (2*vn*vn*(1-cvn));
      Matrix3 Jexp (Matrix3::Identity());
      Jexp.noalias() += alphaSkew(0.5, v);
      Jexp.noalias() += alphaSkew(alpha,v)*skew(v);
      JvOut = Jexp.inverse();
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    static void interpolate_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                 const Eigen::MatrixBase<ConfigR_t> & q1,
                                 const Scalar& u,
                                 const Eigen::MatrixBase<ConfigOut_t>& qout)
    {
      ConstQuaternionMap_t p0 (q0.derived().data());
      ConstQuaternionMap_t p1 (q1.derived().data());
      QuaternionMap_t quaternion_result (
          (const_cast< Eigen::MatrixBase<ConfigOut_t>& >(qout)).derived().data()
          );

      quaternion_result = p0.slerp(u, p1);
    }

    template <class ConfigL_t, class ConfigR_t>
    static double squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                       const Eigen::MatrixBase<ConfigR_t> & q1)
    {
      TangentVector_t t;
      difference_impl(q0, q1, t);
      return t.squaredNorm();
    }
    
    template <class Config_t>
    static void normalize_impl (const Eigen::MatrixBase<Config_t>& qout)
    {
      Config_t& qout_ = (const_cast< Eigen::MatrixBase<Config_t>& >(qout)).derived();
      qout_.normalize();
    }

    template <class Config_t>
    void random_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      QuaternionMap_t out (
          (const_cast< Eigen::MatrixBase<Config_t>& >(qout)).derived().data()
          );
      uniformRandom(out);
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> &,
                                  const Eigen::MatrixBase<ConfigR_t> &,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout)
      const
    {
      random_impl(qout);
    }

    template <class ConfigL_t, class ConfigR_t>
    static bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                         const Eigen::MatrixBase<ConfigR_t> & q1,
                                         const Scalar & prec)
    {
      ConstQuaternionMap_t quat1(q0.derived().data());
      ConstQuaternionMap_t quat2(q1.derived().data());

      return defineSameRotation(quat1,quat2,prec);
    }
  }; // struct SpecialOrthogonalOperation<3>
} // namespace se3

#endif // ifndef __se3_special_orthogonal_operation_hpp__
