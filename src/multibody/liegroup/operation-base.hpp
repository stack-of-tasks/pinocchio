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

#ifndef __se3_lie_group_operation_base_hpp__
#define __se3_lie_group_operation_base_hpp__

#include "pinocchio/spatial/fwd.hpp" // struct traits

#include <Eigen/Core>
#include <limits>

namespace se3
{
#define SE3_LIE_GROUP_PUBLIC_INTERFACE(Derived)                                \
  typedef LieGroupOperationBase<Derived> Base;                                 \
  typedef Base::Index Index;                                                   \
  typedef traits<Derived>::Scalar Scalar;                                      \
  enum {                                                                       \
    NQ = Base::NQ,                                                             \
    NV = Base::NV                                                              \
  };                                                                           \
  typedef Base::ConfigVector_t ConfigVector_t;                                 \
  typedef Base::TangentVector_t TangentVector_t;                               \
  typedef Base::JacobianMatrix_t JacobianMatrix_t

#define SE3_LIE_GROUP_TPL_PUBLIC_INTERFACE(Derived)                            \
  typedef          LieGroupOperationBase<Derived> Base;                        \
  typedef typename Base::Index Index;                                          \
  typedef typename traits<Derived>::Scalar Scalar;                             \
  enum {                                                                       \
    NQ = Base::NQ,                                                             \
    NV = Base::NV                                                              \
  };                                                                           \
  typedef typename Base::ConfigVector_t ConfigVector_t;                        \
  typedef typename Base::TangentVector_t TangentVector_t;                      \
  typedef typename Base::JacobianMatrix_t JacobianMatrix_t

  template<typename Derived>
  struct LieGroupOperationBase
  {
    typedef Derived LieGroupDerived;
    typedef int Index;
    typedef typename traits<LieGroupDerived>::Scalar Scalar;
    enum {
      NQ = traits<LieGroupDerived>::NQ,
      NV = traits<LieGroupDerived>::NV
    };

    typedef Eigen::Matrix<Scalar,NQ,1> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1> TangentVector_t;
    typedef Eigen::Matrix<Scalar,NV,NV> JacobianMatrix_t;

    /// \name API with return value as argument
    /// \{

    /**
     * @brief      Integrate joint's configuration for a tangent vector during one unit time
     *
     * @param[in]  q     initatial configuration  (size full model.nq)
     * @param[in]  v     joint velocity (size full model.nv)
     *
     * @return     The configuration integrated
     */
    template <class ConfigIn_t, class Tangent_t, class ConfigOut_t>
    static void integrate(const Eigen::MatrixBase<ConfigIn_t> & q,
                          const Eigen::MatrixBase<Tangent_t>  & v,
                          const Eigen::MatrixBase<ConfigOut_t>& qout);

    /**
     * @brief      Computes the Jacobian of the Integrate operation.
     *             \f$ Jv * dv \approx (q + (v+dv)) - (q+v) \f$
     *
     * @param[in]  v     joint velocity (size full model.nv)
     *
     * @return     The Jacobian
     */
    template <class ConfigIn_t, class Tangent_t, class JacobianLOut_t, class JacobianROut_t>
    static void Jintegrate(const Eigen::MatrixBase<ConfigIn_t> & q,
                           const Eigen::MatrixBase<Tangent_t>  & v,
                           const Eigen::MatrixBase<JacobianLOut_t>& Jq,
                           const Eigen::MatrixBase<JacobianROut_t>& Jv);

    /**
     * @brief      Interpolation between two joint's configurations
     *
     * @param[in]  q0    Initial configuration to interpolate
     * @param[in]  q1    Final configuration to interpolate
     * @param[in]  u     u in [0;1] position along the interpolation.
     *
     * @return     The interpolated configuration (q0 if u = 0, q1 if u = 1)
     */
    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    static void interpolate(const Eigen::MatrixBase<ConfigL_t> & q0,
                            const Eigen::MatrixBase<ConfigR_t> & q1,
                            const Scalar& u,
                            const Eigen::MatrixBase<ConfigOut_t>& qout);
    
    /**
     * @brief      Normalize the joint configuration given as input. For instance, the quaternion must unitary.
     *
     * @return     The normalized joint configuration.
     */
    template <class Config_t>
    static void normalize (const Eigen::MatrixBase<Config_t>& qout);

    /**
     * @brief      Generate a random joint configuration, normalizing quaternions when necessary.
     *
     * \warning    Do not take into account the joint limits. To shoot a configuration uniformingly
     *             depending on joint limits, see uniformySample
     *
     * @return     The joint configuration
     */
    template <class Config_t>
    void random (const Eigen::MatrixBase<Config_t>& qout) const;

    /**
     * @brief      Generate a configuration vector uniformly sampled among
     *             provided limits.
     *
     * @param[in]  lower_pos_limit  lower joint limit
     * @param[in]  upper_pos_limit  upper joint limit
     *
     * @return     The joint configuration
     */
    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration
    (const Eigen::MatrixBase<ConfigL_t> & lower_pos_limit,
     const Eigen::MatrixBase<ConfigR_t> & upper_pos_limit,
     const Eigen::MatrixBase<ConfigOut_t> & qout) const;

    /**
     * @brief      the tangent vector that must be integrated during one unit time to go from q0 to q1
     *
     * @param[in]  q0    Initial configuration
     * @param[in]  q1    Wished configuration
     *
     * @return     The corresponding velocity
     */
    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    static void difference(const Eigen::MatrixBase<ConfigL_t> & q0,
                           const Eigen::MatrixBase<ConfigR_t> & q1,
                           const Eigen::MatrixBase<Tangent_t>& d);

    /**
     * @brief      Computes the Jacobian of the difference operation
     *
     * @param[in]  q0    Initial configuration
     * @param[in]  q1    Wished configuration
     *
     * @return     The Jacobian
     */
    template <class ConfigL_t, class ConfigR_t, class JacobianLOut_t, class JacobianROut_t>
    static void Jdifference(const Eigen::MatrixBase<ConfigL_t> & q0,
                            const Eigen::MatrixBase<ConfigR_t> & q1,
                            const Eigen::MatrixBase<JacobianLOut_t>& J0,
                            const Eigen::MatrixBase<JacobianROut_t>& J1);

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
                                  const Eigen::MatrixBase<ConfigR_t> & q1);

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
                           const Eigen::MatrixBase<ConfigR_t> & q1);

    /**
     * @brief      Check if two configurations are equivalent within the given precision
     *
     * @param[in]  q0    Configuration 0
     * @param[in]  q1    Configuration 1
     */
    template <class ConfigL_t, class ConfigR_t>
    static bool isSameConfiguration(const Eigen::MatrixBase<ConfigL_t> & q0,
                                    const Eigen::MatrixBase<ConfigR_t> & q1,
                                    const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision());
    /// \}

    /// \name API that allocates memory
    /// \{

    template <class Config_t, class Tangent_t>
    static ConfigVector_t integrate(const Eigen::MatrixBase<Config_t>  & q,
                                    const Eigen::MatrixBase<Tangent_t> & v);

    template <class ConfigL_t, class ConfigR_t>
    static ConfigVector_t interpolate(const Eigen::MatrixBase<ConfigL_t> & q0,
                                      const Eigen::MatrixBase<ConfigR_t> & q1,
                                      const Scalar& u);

    ConfigVector_t random() const;

    template <class ConfigL_t, class ConfigR_t>
    ConfigVector_t randomConfiguration
    (const Eigen::MatrixBase<ConfigL_t> & lower_pos_limit,
     const Eigen::MatrixBase<ConfigR_t> & upper_pos_limit) const;

    template <class ConfigL_t, class ConfigR_t>
    static TangentVector_t difference(const Eigen::MatrixBase<ConfigL_t> & q0,
                                      const Eigen::MatrixBase<ConfigR_t> & q1);
    /// \}


    /// \name Default implementations
    /// \{

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    static void interpolate_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                 const Eigen::MatrixBase<ConfigR_t> & q1,
                                 const Scalar& u,
                                 const Eigen::MatrixBase<ConfigOut_t>& qout);
    
    template <class Config_t>
    static void normalize_impl(const Eigen::MatrixBase<Config_t> & qout);

    template <class ConfigL_t, class ConfigR_t>
    static double squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                       const Eigen::MatrixBase<ConfigR_t> & q1);

    template <class ConfigL_t, class ConfigR_t>
    static bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                         const Eigen::MatrixBase<ConfigR_t> & q1,
                                         const Scalar & prec);
    /// Get dimension of Lie Group vector representation
    ///
    /// For instance, for SO(3), the dimension of the vector representation is
    /// 4 (quaternion) while the dimension of the tangent space is 3.
    Index nq () const;
    /// Get dimension of Lie Group tangent space
    Index nv () const;
    /// Get neutral element as a vector
    ConfigVector_t neutral () const;

    /// Get name of instance
    std::string name () const;

    Derived& derived ()
    {
      return static_cast <Derived&> (*this);
    }

    const Derived& derived () const
    {
      return static_cast <const Derived&> (*this);
    }
    /// \}

  protected:
    /// Default constructor.
    ///
    /// Prevent the construction of derived class.
    LieGroupOperationBase() {}

    /// Copy constructor
    ///
    /// Prevent the copy of derived class.
    LieGroupOperationBase( const LieGroupOperationBase & /*clone*/) {}
  }; // struct LieGroupOperationBase

} // namespace se3

#include "pinocchio/multibody/liegroup/operation-base.hxx"

#endif // ifndef __se3_lie_group_operation_base_hpp__
