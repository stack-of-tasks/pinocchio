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

#ifndef __se3_lie_group_operation_base_hpp__
#define __se3_lie_group_operation_base_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/spatial/fwd.hpp" // struct traits

#include <Eigen/Core>
#include <limits>

namespace se3
{
  
#define SE3_LIE_GROUP_PUBLIC_INTERFACE_GENERIC(Derived,TYPENAME)               \
  typedef          LieGroupBase<Derived> Base;                        \
  typedef TYPENAME Base::Index Index;                                          \
  typedef TYPENAME traits<Derived>::Scalar Scalar;                             \
  enum {                                                                       \
    NQ = Base::NQ,                                                             \
    NV = Base::NV                                                              \
  };                                                                           \
  typedef TYPENAME Base::ConfigVector_t ConfigVector_t;                        \
  typedef TYPENAME Base::TangentVector_t TangentVector_t;                      \
  typedef TYPENAME Base::JacobianMatrix_t JacobianMatrix_t
  
#define SE3_LIE_GROUP_PUBLIC_INTERFACE(Derived)                                \
SE3_LIE_GROUP_PUBLIC_INTERFACE_GENERIC(Derived,PINOCCHIO_MACRO_EMPTY_ARG)
  
#define SE3_LIE_GROUP_TPL_PUBLIC_INTERFACE(Derived)                            \
SE3_LIE_GROUP_PUBLIC_INTERFACE_GENERIC(Derived,typename)

  template<typename Derived>
  struct LieGroupBase
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
     * @brief      Integrate a joint's configuration with a tangent vector during one unit time duration
     *
     * @param[in]  q     the initial configuration.
     * @param[in]  v     the tangent velocity.
     *
     * @param[out] qout  the configuration integrated.
     */
    template <class ConfigIn_t, class Tangent_t, class ConfigOut_t>
    void integrate(const Eigen::MatrixBase<ConfigIn_t> & q,
                   const Eigen::MatrixBase<Tangent_t>  & v,
                   const Eigen::MatrixBase<ConfigOut_t>& qout) const;

    /**
     * @brief      Computes the Jacobian of the Integrate operation.
     *
     * @param[in]  q    configuration
     * @param[in]  v    tangent velocity.
     *
     * @param[out] J    the Jacobian of the Integrate operation wrt q.
     */
    template <class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrate_dq(const Eigen::MatrixBase<Config_t >  & q,
                       const Eigen::MatrixBase<Tangent_t>  & v,
                       const Eigen::MatrixBase<JacobianOut_t>& J) const;

    /**
     * @brief      Computes the Jacobian of the Integrate operation.
     *
     * @param[in]  q    configuration
     * @param[in]  v    tangent velocity.
     *
     * @param[out] J    the Jacobian of the Integrate operation wrt v.
     */
    template <class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrate_dv(const Eigen::MatrixBase<Config_t >  & q,
                       const Eigen::MatrixBase<Tangent_t>  & v,
                       const Eigen::MatrixBase<JacobianOut_t>& J) const;

    /**
     * @brief      Interpolation between two joint's configurations
     *
     * @param[in]  q0    the initial configuration to interpolate.
     * @param[in]  q1    the final configuration to interpolate.
     * @param[in]  u     in [0;1] the absicca along the interpolation.
     *
     * @param[out] qout  the interpolated configuration (q0 if u = 0, q1 if u = 1)
     */
    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void interpolate(const Eigen::MatrixBase<ConfigL_t> & q0,
                     const Eigen::MatrixBase<ConfigR_t> & q1,
                     const Scalar& u,
                     const Eigen::MatrixBase<ConfigOut_t>& qout) const;
    
    /**
     * @brief      Normalize the joint configuration given as input.
     *             For instance, the quaternion must be unitary.
     *
     * @param[out]     qout  the normalized joint configuration.
     */
    template <class Config_t>
    void normalize (const Eigen::MatrixBase<Config_t>& qout) const;

    /**
     * @brief      Generate a random joint configuration, normalizing quaternions when necessary.
     *
     * \warning    Do not take into account the joint limits. To shoot a configuration uniformingly
     *             depending on joint limits, see randomConfiguration.
     *
     * @param[out] qout  the random joint configuration.
     */
    template <class Config_t>
    void random (const Eigen::MatrixBase<Config_t>& qout) const;

    /**
     * @brief      Generate a configuration vector uniformly sampled among
     *             provided limits.
     *
     * @param[in]  lower_pos_limit  the lower joint limit vector.
     * @param[in]  upper_pos_limit  the upper joint limit vector.
     *
     * @param[out] qout             the random joint configuration in the two bounds.
     */
    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration
    (const Eigen::MatrixBase<ConfigL_t> & lower_pos_limit,
     const Eigen::MatrixBase<ConfigR_t> & upper_pos_limit,
     const Eigen::MatrixBase<ConfigOut_t> & qout) const;

    /**
     * @brief      Computes the tangent vector that must be integrated during one unit time to go from q0 to q1.
     *
     * @param[in]  q0    the initial configuration vector.
     * @param[in]  q1    the terminal configuration vector.
     *
     * @param[out] v     the corresponding velocity.
     */
    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    void difference(const Eigen::MatrixBase<ConfigL_t> & q0,
                    const Eigen::MatrixBase<ConfigR_t> & q1,
                    const Eigen::MatrixBase<Tangent_t>& v) const;

    /**
     * @brief      Computes the Jacobian of the difference operation.
     *
     * @param[in]  q0    the initial configuration vector.
     * @param[in]  q1    the terminal configuration vector.
     *
     * @param[out] J0    the Jacobian of the difference operation with respect to q0.
     * @param[out] J1    the Jacobian of the difference operation with respect to q1.
     */
    template <class ConfigL_t, class ConfigR_t, class JacobianLOut_t, class JacobianROut_t>
    void Jdifference(const Eigen::MatrixBase<ConfigL_t> & q0,
                     const Eigen::MatrixBase<ConfigR_t> & q1,
                     const Eigen::MatrixBase<JacobianLOut_t>& J0,
                     const Eigen::MatrixBase<JacobianROut_t>& J1) const;

    /**
     * @brief      Squared distance between two joint configurations.
     *
     * @param[in]  q0    the initial configuration vector.
     * @param[in]  q1    the terminal configuration vector.
     *
     * @param[out] d     the corresponding distance betwenn q0 and q1.
     */
    template <class ConfigL_t, class ConfigR_t>
    Scalar squaredDistance(const Eigen::MatrixBase<ConfigL_t> & q0,
                           const Eigen::MatrixBase<ConfigR_t> & q1) const;

    /**
     * @brief      Distance between two configurations of the joint
     *
     * @param[in]  q0    the initial configuration vector.
     * @param[in]  q1    the terminal configuration vector.
     *
     * @return     The corresponding distance.
     */
    template <class ConfigL_t, class ConfigR_t>
    Scalar distance(const Eigen::MatrixBase<ConfigL_t> & q0,
                    const Eigen::MatrixBase<ConfigR_t> & q1) const;

    /**
     * @brief      Check if two configurations are equivalent within the given precision.
     *
     * @param[in]  q0    Configuration 0
     * @param[in]  q1    Configuration 1
     */
    template <class ConfigL_t, class ConfigR_t>
    bool isSameConfiguration(const Eigen::MatrixBase<ConfigL_t> & q0,
                             const Eigen::MatrixBase<ConfigR_t> & q1,
                             const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const;
    /// \}

    /// \name API that allocates memory
    /// \{

    template <class Config_t, class Tangent_t>
    ConfigVector_t integrate(const Eigen::MatrixBase<Config_t>  & q,
                             const Eigen::MatrixBase<Tangent_t> & v) const ;

    template <class ConfigL_t, class ConfigR_t>
    ConfigVector_t interpolate(const Eigen::MatrixBase<ConfigL_t> & q0,
                               const Eigen::MatrixBase<ConfigR_t> & q1,
                               const Scalar& u) const;

    ConfigVector_t random() const;

    template <class ConfigL_t, class ConfigR_t>
    ConfigVector_t randomConfiguration
    (const Eigen::MatrixBase<ConfigL_t> & lower_pos_limit,
     const Eigen::MatrixBase<ConfigR_t> & upper_pos_limit) const;

    template <class ConfigL_t, class ConfigR_t>
    TangentVector_t difference(const Eigen::MatrixBase<ConfigL_t> & q0,
                               const Eigen::MatrixBase<ConfigR_t> & q1) const;
    /// \}


    /// \name Default implementations
    /// \{

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void interpolate_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                          const Eigen::MatrixBase<ConfigR_t> & q1,
                          const Scalar& u,
                          const Eigen::MatrixBase<ConfigOut_t>& qout) const;
    
    template <class Config_t>
    void normalize_impl(const Eigen::MatrixBase<Config_t> & qout) const;

    template <class ConfigL_t, class ConfigR_t>
    Scalar squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1) const;

    template <class ConfigL_t, class ConfigR_t>
    bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                  const Eigen::MatrixBase<ConfigR_t> & q1,
                                  const Scalar & prec) const;
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
    LieGroupBase() {}

    /// Copy constructor
    ///
    /// Prevent the copy of derived class.
    LieGroupBase( const LieGroupBase & /*clone*/) {}
  }; // struct LieGroupBase

} // namespace se3

#include "pinocchio/multibody/liegroup/operation-base.hxx"

#endif // ifndef __se3_lie_group_operation_base_hpp__
