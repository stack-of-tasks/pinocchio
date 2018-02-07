//
// Copyright (c) 2016-2017 CNRS
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

#ifndef __se3_special_euclidean_operation_hpp__
#define __se3_special_euclidean_operation_hpp__

#include <limits>

#include <pinocchio/macros.hpp>
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/liegroup/operation-base.hpp"

#include "pinocchio/multibody/liegroup/vector-space.hpp"
#include "pinocchio/multibody/liegroup/cartesian-product.hpp"
#include "pinocchio/multibody/liegroup/special-orthogonal.hpp"

namespace se3
{
  template<int N> struct SpecialEuclideanOperation {};
  template<int N> struct traits<SpecialEuclideanOperation<N> > {};

  template<> struct traits<SpecialEuclideanOperation<2> > {
    typedef double Scalar;
    enum {
      NQ = 4,
      NV = 3
    };
  };

  template<> struct traits<SpecialEuclideanOperation<3> > {
    typedef double Scalar;
    enum {
      NQ = 7,
      NV = 6
    };
  };

  template<>
  struct SpecialEuclideanOperation<2> : public LieGroupOperationBase <SpecialEuclideanOperation<2> >
  {
    typedef CartesianProductOperation <VectorSpaceOperation<2>, SpecialOrthogonalOperation<2> > R2crossSO2_t;

    SE3_LIE_GROUP_PUBLIC_INTERFACE(SpecialEuclideanOperation);
    typedef Eigen::Matrix<Scalar,2,2> Matrix2;
    typedef Eigen::Matrix<Scalar,2,1> Vector2;

    template <typename Tangent_t>
    static void exp (const Eigen::MatrixBase<Tangent_t>& v, Matrix2& R, Vector2& t)
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(TangentVector_t,Tangent_t);

      const Scalar & omega = v(2);
      Scalar cv,sv; SINCOS(omega, &sv, &cv);
      R << cv, -sv, sv, cv;

      if (std::fabs (omega) > 1e-14) {
        Vector2 vcross (-v(1), v(0));
        vcross /= omega;
        t = vcross - R * vcross;
      } else {
        t = v.template head<2>();
      }
    }

    template <typename Matrix_t>
    static void toInverseActionMatrix (const Matrix2& R, const Vector2& t, const Eigen::MatrixBase<Matrix_t>& M)
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix_t, 3, 3);
      Matrix_t& Mout = const_cast <Matrix_t&> (M.derived());
      Mout.template topLeftCorner<2,2>().noalias() = R.transpose();
      Vector2 tinv (R.transpose() * t);
      Mout.template topRightCorner<2,1>() << - tinv(1), tinv(0);
      Mout.template bottomLeftCorner<1,2>().setZero();
      Mout(2,2) = 1;
    }

    static Scalar log (const Matrix2& R)
    {
      Scalar theta;
      const Scalar tr = R.trace();
      const bool pos = (R (1, 0) > R (0, 1));
      if (tr > 2)       theta = 0; // acos((3-1)/2)
      else if (tr < -2) theta = (pos ? PI : -PI); // acos((-1-1)/2)
      // Around 0, asin is numerically more stable than acos because
      // acos(x) = PI/2 - x and asin(x) = x (the precision of x is not lost in PI/2).
      else if (tr > 2 - 1e-2) theta = asin ((R(1,0) - R(0,1)) / 2);
      else              theta = (pos ? acos (tr/2) : -acos(tr/2));
      assert (theta == theta); // theta != NaN
      return  theta;
    }

    template <typename Tangent_t>
    static void log (Matrix2& R, Vector2& p,
        const Eigen::MatrixBase<Tangent_t>& v)
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(TangentVector_t,Tangent_t);
      Tangent_t& vout = const_cast< Tangent_t& >(v.derived());

      Scalar t = log(R);
      const Scalar tabs = std::fabs(t);
      const Scalar t2 = t*t;
      Scalar alpha;
      if (tabs < 1e-4) {
        alpha = 1 - t2/12 - t2*t2/720;
      } else {
        Scalar st,ct; SINCOS (tabs, &st, &ct);
        alpha = tabs*st/(2*(1-ct));
      }

      Matrix2 sk; sk << 0, -t/2, t/2, 0;
      vout.template head<2>().noalias() = alpha * p - sk * p;
      vout(2) = t;
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
      ConfigVector_t n; n.setZero (); n [2] = 1;
      return n;
    }

    std::string name () const
    {
      return std::string ("SE(2)");
    }

    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    static void difference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1,
                                const Eigen::MatrixBase<Tangent_t> & d)
    {
      Matrix2 R0, R1; Vector2 t0, t1;
      forwardKinematics(R0, t0, q0);
      forwardKinematics(R1, t1, q1);
      Matrix2 R (R0.transpose() * R1);
      Vector2 t (R0.transpose() * (t1 - t0));

      log (R, t, d);
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    static void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      ConfigOut_t& out = const_cast< ConfigOut_t& >(qout.derived());

      Matrix2 R0, R;
      Vector2 t0, t;
      forwardKinematics(R0, t0, q);
      exp(v, R, t);

      out.template head<2>().noalias() = R0 * t + t0;
      out.template tail<2>().noalias() = R0 * R.col(0);
    }

    template <class Tangent_t, class JacobianOut_t>
    static void Jintegrate_impl(const Eigen::MatrixBase<Tangent_t>  & v,
                                const Eigen::MatrixBase<JacobianOut_t>& J)
    {
      JacobianOut_t& Jout = const_cast< JacobianOut_t& >(J.derived());

      Matrix2 R;
      Vector2 t;
      exp(v, R, t);

      toInverseActionMatrix (R, t, Jout);
    }

    // interpolate_impl use default implementation.
    // template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    // static void interpolate_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                 // const Eigen::MatrixBase<ConfigR_t> & q1,
                                 // const Scalar& u,
                                 // const Eigen::MatrixBase<ConfigOut_t>& qout)

    // template <class ConfigL_t, class ConfigR_t>
    // static double squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                       // const Eigen::MatrixBase<ConfigR_t> & q1)
    template <class Config_t>
    static void normalize_impl (const Eigen::MatrixBase<Config_t>& qout)
    {
      Config_t& qout_ = (const_cast< Eigen::MatrixBase<Config_t>& >(qout)).derived();
      qout_.template tail<2>().normalize();
    }

    template <class Config_t>
    void random_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      R2crossSO2_t().random(qout);
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & lower,
                                  const Eigen::MatrixBase<ConfigR_t> & upper,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout)
      const
    {
      R2crossSO2_t ().randomConfiguration(lower, upper, qout);
    }

    template <class ConfigL_t, class ConfigR_t>
    static bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                         const Eigen::MatrixBase<ConfigR_t> & q1,
                                         const Scalar & prec)
    {
      return R2crossSO2_t::isSameConfiguration(q0, q1, prec);
    }

    private:
    template<typename V>
    static void forwardKinematics(Matrix2 & R, Vector2 & t, const Eigen::MatrixBase<V>& q)
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,V);

      const double& c_theta = q(2),
                    s_theta = q(3);

      R << c_theta, -s_theta, s_theta, c_theta;
      t = q.template head<2>();
    }
  }; // struct SpecialEuclideanOperation<2>

  /// SE(3)
  template<>
  struct SpecialEuclideanOperation<3> : public LieGroupOperationBase <SpecialEuclideanOperation<3> >
  {
    typedef CartesianProductOperation <VectorSpaceOperation<3>, SpecialOrthogonalOperation<3> > R3crossSO3_t;

    SE3_LIE_GROUP_PUBLIC_INTERFACE(SpecialEuclideanOperation);

    typedef Eigen::Quaternion<Scalar> Quaternion_t;
    typedef Eigen::Map<      Quaternion_t> QuaternionMap_t;
    typedef Eigen::Map<const Quaternion_t> ConstQuaternionMap_t;
    typedef SE3 Transformation_t;

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
      ConfigVector_t n; n.setZero (); n [6] = 1;
      return n;
    }

    std::string name () const
    {
      return std::string ("SE(3)");
    }

    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    static void difference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1,
                                const Eigen::MatrixBase<Tangent_t> & d)
    {
      ConstQuaternionMap_t p0 (q0.derived().template tail<4>().data());
      ConstQuaternionMap_t p1 (q1.derived().template tail<4>().data());
      const_cast < Eigen::MatrixBase<Tangent_t>& > (d)
        = log6(  SE3(p0.matrix(), q0.derived().template head<3>()).inverse()
               * SE3(p1.matrix(), q1.derived().template head<3>())).toVector();
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    static void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      ConfigOut_t& out = (const_cast< Eigen::MatrixBase<ConfigOut_t>& >(qout)).derived();
      ConstQuaternionMap_t quat(q.derived().template tail<4>().data());
      QuaternionMap_t res_quat (out.template tail<4>().data());

      SE3 M0 (quat.matrix(), q.derived().template head<3>());
      SE3 M1 (M0 * exp6(Motion(v)));

      out.template head<3>() = M1.translation();
      res_quat = M1.rotation();
      // Norm of qs might be epsilon-different to 1, so M1.rotation might be epsilon-different to a rotation matrix.
      // It is then safer to re-normalized after converting M1.rotation to quaternion.
      firstOrderNormalize(res_quat);
    }

    template <class Tangent_t, class JacobianOut_t>
    static void Jintegrate_impl(const Eigen::MatrixBase<Tangent_t>  & v,
                                const Eigen::MatrixBase<JacobianOut_t>& J)
    {
      JacobianOut_t& Jout = const_cast< JacobianOut_t& >(J.derived());
      Jout = exp6(v).inverse().toActionMatrix();
    }

    // interpolate_impl use default implementation.
    // template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    // static void interpolate_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                 // const Eigen::MatrixBase<ConfigR_t> & q1,
                                 // const Scalar& u,
                                 // const Eigen::MatrixBase<ConfigOut_t>& qout)
    // {
    // }

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
      qout_.template tail<4>().normalize();
    }

    template <class Config_t>
    void random_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      R3crossSO3_t().random(qout);
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & lower,
                                  const Eigen::MatrixBase<ConfigR_t> & upper,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout)
      const
    {
      R3crossSO3_t ().randomConfiguration(lower, upper, qout);
    }

    template <class ConfigL_t, class ConfigR_t>
    static bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                         const Eigen::MatrixBase<ConfigR_t> & q1,
                                         const Scalar & prec)
    {
      return R3crossSO3_t::isSameConfiguration(q0, q1, prec);
    }
  }; // struct SpecialEuclideanOperation<3>

} // namespace se3

#endif // ifndef __se3_special_euclidean_operation_hpp__
