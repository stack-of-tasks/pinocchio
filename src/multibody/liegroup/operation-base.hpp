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

#ifndef __se3_lie_group_operation_base_hpp__
#define __se3_lie_group_operation_base_hpp__

#include "pinocchio/spatial/fwd.hpp" // struct traits

#include <Eigen/Core>
#include <limits>

namespace se3
{
#ifdef __clang__

#define SE3_LIE_GROUP_TYPEDEF_ARG(prefix)              \
   typedef prefix traits<LieGroupDerived>::Scalar Scalar;    \
   enum {                  \
    NQ = traits<LieGroupDerived>::NQ,              \
    NV = traits<LieGroupDerived>::NV               \
  };                        \
  typedef prefix traits<LieGroupDerived>::ConfigVector_t ConfigVector_t;        \
  typedef prefix traits<LieGroupDerived>::TangentVector_t TangentVector_t

#define SE3_LIE_GROUP_TYPEDEF SE3_LIE_GROUP_TYPEDEF_ARG()
#define SE3_LIE_GROUP_TYPEDEF_TEMPLATE SE3_LIE_GROUP_TYPEDEF_ARG(typename)

#elif (__GNUC__ == 4) && (__GNUC_MINOR__ == 4) && (__GNUC_PATCHLEVEL__ == 2)

#define SE3_LIE_GROUP_TYPEDEF_NOARG()       \
  typedef int Index;            \
  typedef traits<LieGroupDerived>::Scalar Scalar;    \
  enum {              \
    NQ = traits<LieGroupDerived>::NQ,         \
    NV = traits<LieGroupDerived>::NV          \
  };                        \
  typedef traits<LieGroupDerived>::ConfigVector_t ConfigVector_t;        \
  typedef traits<LieGroupDerived>::TangentVector_t TangentVector_t

#define SE3_LIE_GROUP_TYPEDEF_ARG(prefix)         \
  typedef int Index;              \
  typedef prefix traits<LieGroupDerived>::Scalar Scalar;           \
  enum {                \
    NQ = traits<LieGroupDerived>::NQ,           \
    NV = traits<LieGroupDerived>::NV            \
  };                        \
  typedef prefix traits<LieGroupDerived>::ConfigVector_t ConfigVector_t;        \
  typedef prefix traits<LieGroupDerived>::TangentVector_t TangentVector_t

#define SE3_LIE_GROUP_TYPEDEF SE3_LIE_GROUP_TYPEDEF_NOARG()
#define SE3_LIE_GROUP_TYPEDEF_TEMPLATE SE3_LIE_GROUP_TYPEDEF_ARG(typename)

#else

#define SE3_LIE_GROUP_TYPEDEF_ARG()              \
  typedef int Index;                 \
  typedef typename traits<LieGroupDerived>::Scalar Scalar;    \
  enum {                   \
    NQ = traits<LieGroupDerived>::NQ,              \
    NV = traits<LieGroupDerived>::NV               \
  };                        \
  typedef typename traits<LieGroupDerived>::ConfigVector_t ConfigVector_t;        \
  typedef typename traits<LieGroupDerived>::TangentVector_t TangentVector_t

#define SE3_LIE_GROUP_TYPEDEF SE3_LIE_GROUP_TYPEDEF_ARG()
#define SE3_LIE_GROUP_TYPEDEF_TEMPLATE SE3_LIE_GROUP_TYPEDEF_ARG()

#endif

  template<typename Derived>
  struct LieGroupOperationBase
  {
    typedef Derived LieGroupDerived;
    SE3_LIE_GROUP_TYPEDEF_TEMPLATE;

    ///
    /// \brief Return the resolution of the finite differerence increment according to the Scalar type
    /// \remark Ideally, this function must depend on the value of q
    ///
    /// \returns The finite difference increment.
    ///
    // typename ConfigVector_t::Scalar finiteDifferenceIncrement() const
    // { return derived().finiteDifferenceIncrement(); }

    /**
     * @brief      Integrate joint's configuration for a tangent vector during one unit time
     *
     * @param[in]  q     initatial configuration  (size full model.nq)
     * @param[in]  v     joint velocity (size full model.nv)
     *
     * @return     The configuration integrated
     */
    template <class Config_t, class Tangent_t>
    static ConfigVector_t integrate(const Eigen::MatrixBase<Config_t>  & q,
                                   const Eigen::MatrixBase<Tangent_t> & v)
    {
      ConfigVector_t qout;
      integrate(q, v, qout);
      return qout;
    }

    template <class ConfigIn_t, class Tangent_t, class ConfigOut_t>
    static void integrate(const Eigen::MatrixBase<ConfigIn_t> & q,
                          const Eigen::MatrixBase<Tangent_t>  & v,
                          const Eigen::MatrixBase<ConfigOut_t>& qout)
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigIn_t , ConfigVector_t);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Tangent_t  , TangentVector_t);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigOut_t, ConfigVector_t);
      Derived::integrate_impl(q, v, qout);
    }


    /**
     * @brief      Interpolation between two joint's configurations
     *
     * @param[in]  q0    Initial configuration to interpolate
     * @param[in]  q1    Final configuration to interpolate
     * @param[in]  u     u in [0;1] position along the interpolation.
     *
     * @return     The interpolated configuration (q0 if u = 0, q1 if u = 1)
     */
    // ConfigVector_t interpolate(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1, double u) const
    // { return derived().interpolate_impl(q0, q1, u); }
    template <class ConfigL_t, class ConfigR_t>
    static ConfigVector_t interpolate(const Eigen::MatrixBase<ConfigL_t> & q0,
                                      const Eigen::MatrixBase<ConfigR_t> & q1,
                                      const Scalar& u)
    {
      ConfigVector_t qout;
      interpolate(q0, q1, u, qout);
      return qout;
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    static void interpolate(const Eigen::MatrixBase<ConfigL_t> & q0,
                            const Eigen::MatrixBase<ConfigR_t> & q1,
                            const Scalar& u,
                            const Eigen::MatrixBase<ConfigOut_t>& qout)
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigL_t  , ConfigVector_t);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigR_t  , ConfigVector_t);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigOut_t, ConfigVector_t);
      Derived::interpolate_impl(q0, q1, u, qout);
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    static void interpolate_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                 const Eigen::MatrixBase<ConfigR_t> & q1,
                                 const Scalar& u,
                                 const Eigen::MatrixBase<ConfigOut_t>& qout)
    {
      if     (u == 0) const_cast<Eigen::MatrixBase<ConfigOut_t>&>(qout) = q0;
      else if(u == 1) const_cast<Eigen::MatrixBase<ConfigOut_t>&>(qout) = q1;
      else integrate(q0, u * difference(q0, q1), qout);
    }

    /**
     * @brief      Generate a random joint configuration, normalizing quaternions when necessary.
     *
     * \warning    Do not take into account the joint limits. To shoot a configuration uniformingly
     *             depending on joint limits, see uniformySample
     *
     * @return     The joint configuration
     */
    static ConfigVector_t random()
    {
      ConfigVector_t qout;
      random(qout);
      return qout;
    }

    template <class Config_t>
    static void random (const Eigen::MatrixBase<Config_t>& qout)
    { return Derived::random_impl (qout); }

    /**
     * @brief      Generate a configuration vector uniformly sampled among
     *             provided limits.
     *
     * @param[in]  lower_pos_limit  lower joint limit
     * @param[in]  upper_pos_limit  upper joint limit
     *
     * @return     The joint configuration
     */
    template <class ConfigL_t, class ConfigR_t>
    static ConfigVector_t randomConfiguration(const Eigen::MatrixBase<ConfigL_t> & lower_pos_limit,
                                              const Eigen::MatrixBase<ConfigR_t> & upper_pos_limit)
    {
      ConfigVector_t qout;
      randomConfiguration(lower_pos_limit, upper_pos_limit, qout);
      return qout;
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    static void randomConfiguration(const Eigen::MatrixBase<ConfigL_t> & lower_pos_limit,
                                    const Eigen::MatrixBase<ConfigR_t> & upper_pos_limit,
                                    const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigL_t  , ConfigVector_t);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigR_t  , ConfigVector_t);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigOut_t, ConfigVector_t);
      Derived::randomConfiguration_impl(lower_pos_limit, upper_pos_limit, qout);
    }

    /**
     * @brief      the tangent vector that must be integrated during one unit time to go from q0 to q1
     *
     * @param[in]  q0    Initial configuration
     * @param[in]  q1    Wished configuration
     *
     * @return     The corresponding velocity
     */
    template <class ConfigL_t, class ConfigR_t>
    static TangentVector_t difference(const Eigen::MatrixBase<ConfigL_t> & q0,
                                      const Eigen::MatrixBase<ConfigR_t> & q1)
    {
      TangentVector_t diff;
      difference(q0, q1, diff);
      return diff;
    }

    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    static void difference(const Eigen::MatrixBase<ConfigL_t> & q0,
                           const Eigen::MatrixBase<ConfigR_t> & q1,
                           const Eigen::MatrixBase<Tangent_t>& d)
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigL_t, ConfigVector_t);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigR_t, ConfigVector_t);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Tangent_t, TangentVector_t);
      Derived::difference_impl(q0, q1, d);
    }

    /**
     * @brief      Squared distance between two configurations of the joint
     *
     * @param[in]  q0    Configuration 1
     * @param[in]  q1    Configuration 2
     *
     * @return     The corresponding distance
     */
    template <class ConfigL_t, class ConfigR_t>
    static double squaredDistance(const Eigen::MatrixBase<ConfigL_t> & q0,
                                  const Eigen::MatrixBase<ConfigR_t> & q1)
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigL_t, ConfigVector_t);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigR_t, ConfigVector_t);
      return Derived::squaredDistance_impl(q0, q1);
    }

    template <class ConfigL_t, class ConfigR_t>
    static double squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                       const Eigen::MatrixBase<ConfigR_t> & q1)
    {
      TangentVector_t t;
      difference(q0, q1, t);
      return t.squaredNorm();
    }

    /**
     * @brief      Distance between two configurations of the joint
     *
     * @param[in]  q0    Configuration 1
     * @param[in]  q1    Configuration 2
     *
     * @return     The corresponding distance
     */
    template <class ConfigL_t, class ConfigR_t>
    static double distance(const Eigen::MatrixBase<ConfigL_t> & q0,
                           const Eigen::MatrixBase<ConfigR_t> & q1)
    { return sqrt(squaredDistance(q0, q1)); }

    /**
     * @brief      Get neutral configuration of joint
     *
     * @return     The joint's neutral configuration
     */
    // ConfigVector_t neutralConfiguration() const
    // { return derived().neutralConfiguration_impl(); }

    /**
     * @brief      Normalize a configuration
     *
     * @param[in,out]  q     Configuration to normalize (size full model.nq)
     */
    // void normalize(Eigen::VectorXd & q) const
    // { return derived().normalize_impl(q); }

    /**
     * @brief      Default implementation of normalize
     */
    // void normalize_impl(Eigen::VectorXd &) const { }

    /**
     * @brief      Check if two configurations are equivalent within the given precision
     *
     * @param[in]  q0    Configuration 0
     * @param[in]  q1    Configuration 1
     */
    template <class ConfigL_t, class ConfigR_t>
    static bool isSameConfiguration(const Eigen::MatrixBase<ConfigL_t> & q0,
                                    const Eigen::MatrixBase<ConfigR_t> & q1,
                                    const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision())
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigL_t, ConfigVector_t);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigR_t, ConfigVector_t);
      return Derived::isSameConfiguration_impl(q0, q1, prec);
    }

    template <class ConfigL_t, class ConfigR_t>
    static bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                         const Eigen::MatrixBase<ConfigR_t> & q1,
                                         const Scalar & prec)
    { return q0.isApprox(q1, prec); }

  protected:
    /// Default constructor.
    ///
    /// Prevent the construction of derived class.
    LieGroupOperationBase() {}

    /// Copy constructor
    ///
    /// Prevent the copy of derived class.
    LieGroupOperationBase( const LieGroupOperationBase& clone) {}
  }; // struct LieGroupOperationBase

} // namespace se3

#endif // ifndef __se3_lie_group_operation_base_hpp__
