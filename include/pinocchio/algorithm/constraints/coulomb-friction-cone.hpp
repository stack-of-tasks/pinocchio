//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_constraints_coulomb_friction_cone_hpp__
#define __pinocchio_algorithm_constraints_coulomb_friction_cone_hpp__

#include "pinocchio/algorithm/constraints/fwd.hpp"
#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/comparison-operators.hpp"

namespace pinocchio
{

  template<typename Scalar>
  struct DualCoulombFrictionConeTpl;

  ///  \brief 3d Coulomb friction cone.
  template<typename _Scalar>
  struct CoulombFrictionConeTpl
  {
    typedef _Scalar Scalar;
    typedef DualCoulombFrictionConeTpl<Scalar> DualCone;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

    /// \brief Default constructor
    ///
    /// \param[in] mu Friction coefficient
    explicit CoulombFrictionConeTpl(const Scalar mu)
    : mu(mu)
    {
      assert(mu >= 0 && "mu must be positive");
    }

    /// \brief Copy constructor.
    CoulombFrictionConeTpl(const CoulombFrictionConeTpl & other) = default;

    /// \brief Copy operator
    CoulombFrictionConeTpl & operator=(const CoulombFrictionConeTpl & other) = default;

    /// \brief Comparison operator
    bool operator==(const CoulombFrictionConeTpl & other) const
    {
      return mu == other.mu;
    }

    /// \brief Difference  operator
    bool operator!=(const CoulombFrictionConeTpl & other) const
    {
      return !(*this == other);
    }

    /// \brief Check whether a vector x lies within the cone.
    ///
    /// \param[in] f vector to check (assimilated to a  force vector).
    ///
    template<typename Vector3Like>
    bool isInside(const Eigen::MatrixBase<Vector3Like> & f, const Scalar prec = Scalar(0)) const
    {
      assert(mu >= 0 && "mu must be positive");
      assert(prec >= 0 && "prec should be positive");
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like, 3);
      return f.template head<2>().norm() <= mu * f[2] + prec;
    }

    /// \brief Project a vector x onto the cone.
    ///
    /// \param[in] x a 3d vector to project.
    ///
    template<typename Vector3Like>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)
      project(const Eigen::MatrixBase<Vector3Like> & x) const
    {
      assert(mu >= 0 && "mu must be positive");
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like, 3);
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like) Vector3Plain;
      const Scalar & z = x[2];
      const Scalar mu_z = mu * z;

      const Eigen::Matrix<Scalar, 2, 1> t = x.template head<2>();
      const Scalar t_norm = t.norm();

      if (mu * t_norm <= -z)
        return Vector3Plain::Zero();
      else if (t_norm <= mu_z)
      {
        return x;
      }
      else
      {
        Vector3Plain res;
        res.template head<2>() = (mu / t_norm) * t;
        res[2] = 1;
        res.normalize();
        const Scalar scale = x.dot(res);
        res *= scale;
        return res;
      }
    }

    /// \brief Project a vector x onto the cone with a matric specified by the diagonal matrix R.
    ///
    /// \param[in] x a 3d vector to project.
    /// \param[in] R a 3d vector representing the diagonal of the weights matrix. The tangential
    /// components (the first two) of R should be equal.
    ///
    template<typename Vector3Like>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like) weightedProject(
      const Eigen::MatrixBase<Vector3Like> & x, const Eigen::MatrixBase<Vector3Like> & R) const
    {
      assert(mu >= 0 && "mu must be positive");
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like, 3);
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like) Vector3Plain;
      assert(R(2) > 0 && "R(2) must be strictly positive");
      Scalar weighted_mu = mu * math::sqrt(R(0) / R(2));
      CoulombFrictionConeTpl weighted_cone(weighted_mu);
      Vector3Plain R_sqrt = R.cwiseSqrt();
      Vector3Plain R_sqrt_times_x = (R_sqrt.array() * x.array()).matrix();
      Vector3Plain res = (weighted_cone.project(R_sqrt_times_x).array() / R_sqrt.array()).matrix();
      return res;
    }

    /// \brief Compute the complementary shift associted to the Coulomb friction cone for
    /// complementarity satisfaction in complementary problems.
    ///
    /// \param[in] v a dual vector.
    ///
    template<typename Vector3Like>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)
      computeNormalCorrection(const Eigen::MatrixBase<Vector3Like> & v) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like, 3);
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like) Vector3Plain;

      Vector3Plain res;
      res.template head<2>().setZero();
      res[2] = mu * v.template head<2>().norm();

      return res;
    }

    /// \brief Compute the radial projection associted to the Coulomb friction cone.
    ///
    /// \param[in] f a force vector.
    ///
    template<typename Vector3Like>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)
      computeRadialProjection(const Eigen::MatrixBase<Vector3Like> & f) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like, 3);
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like) Vector3Plain;

      Vector3Plain res;
      const auto & ft = f.template head<2>();
      const Scalar ft_norm = ft.norm();

      res[2] = math::max(Scalar(0), f[2]);
      const Scalar mu_fz = mu * res[2];
      if (ft_norm > mu_fz)
      {
        res.template head<2>() = Scalar(mu_fz / ft_norm) * ft;
      }
      else
        res.template head<2>() = ft;

      return res;
    }

    template<typename Vector3Like1, typename Vector3Like2>
    Scalar computeContactComplementarity(
      const Eigen::MatrixBase<Vector3Like1> & v, const Eigen::MatrixBase<Vector3Like2> & f) const
    {
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like1) Vector3Plain;
      return math::fabs(f.dot(Vector3Plain(v + computeNormalCorrection(v))));
    }

    template<typename Vector3Like1, typename Vector3Like2>
    Scalar computeConicComplementarity(
      const Eigen::MatrixBase<Vector3Like1> & v, const Eigen::MatrixBase<Vector3Like2> & f) const
    {
      return math::fabs(f.dot(v));
    }

    /// \brief Returns the dual cone associated to this.
    DualCone dual() const
    {
      return DualCone(mu);
    }

    /// \brief Returns the dimension of the cone.
    static int dim()
    {
      return 3;
    }

    /// \var Friction coefficient
    Scalar mu;
  }; // CoulombFrictionConeTpl

  ///  \brief Dual of the 3d Coulomb friction cone.
  template<typename _Scalar>
  struct DualCoulombFrictionConeTpl
  {
    typedef _Scalar Scalar;
    typedef CoulombFrictionConeTpl<Scalar> DualCone;

    /// \brief Default constructor
    ///
    /// \param[in] mu Friction coefficient
    explicit DualCoulombFrictionConeTpl(const Scalar mu)
    : mu(mu)
    {
      assert(mu >= 0 && "mu must be positive");
    }

    /// \brief Copy constructor.
    DualCoulombFrictionConeTpl(const DualCoulombFrictionConeTpl & other) = default;

    /// \brief Copy operator
    DualCoulombFrictionConeTpl & operator=(const DualCoulombFrictionConeTpl & other) = default;

    /// \brief Comparison operator
    bool operator==(const DualCoulombFrictionConeTpl & other) const
    {
      return mu == other.mu;
    }

    /// \brief Difference  operator
    bool operator!=(const DualCoulombFrictionConeTpl & other) const
    {
      return !(*this == other);
    }

    /// \brief Check whether a vector v lies within the cone.
    ///
    /// \param[in] v vector to check (assimilated to a linear velocity).
    ///
    template<typename Vector3Like>
    bool isInside(const Eigen::MatrixBase<Vector3Like> & v, const Scalar prec = Scalar(0)) const
    {
      assert(mu >= 0 && "mu must be positive");
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like, 3);
      return mu * v.template head<2>().norm() <= v[2] + prec;
    }

    /// \brief Project a vector x onto the cone
    template<typename Vector3Like>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)
      project(const Eigen::MatrixBase<Vector3Like> & x) const
    {
      assert(mu >= 0 && "mu must be positive");
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like, 3);
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like) Vector3Plain;
      const Scalar & z = x[2];

      const Eigen::Matrix<Scalar, 2, 1> t = x.template head<2>();
      const Scalar t_norm = t.norm();

      if (t_norm <= -mu * z)
        return Vector3Plain::Zero();
      else if (mu * t_norm <= z)
      {
        return x;
      }
      else
      {
        Vector3Plain res;
        res.template head<2>() = t;
        res[2] = mu * t_norm;
        res.normalize();
        const Scalar scale = x.dot(res);
        res *= scale;
        return res;
      }
    }

    /// \brief Returns the dimension of the cone.
    static int dim()
    {
      return 3;
    }

    /// \brief Returns the dual cone associated to this.    ///
    DualCone dual() const
    {
      return DualCone(mu);
    }

    /// \var Friction coefficient
    Scalar mu;

  }; // DualCoulombFrictionConeTpl

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_constraints_coulomb_friction_cone_hpp__
