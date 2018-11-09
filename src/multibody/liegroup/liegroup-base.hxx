//
// Copyright (c) 2016-2017 CNRS
//

#ifndef __pinocchio_lie_group_operation_base_hxx__
#define __pinocchio_lie_group_operation_base_hxx__

#include "pinocchio/macros.hpp"

namespace pinocchio {

  // --------------- API with return value as argument ---------------------- //

  template <class Derived>
  template <class ConfigIn_t, class Tangent_t, class ConfigOut_t>
  void LieGroupBase<Derived>
  ::integrate(const Eigen::MatrixBase<ConfigIn_t> & q,
              const Eigen::MatrixBase<Tangent_t>  & v,
              const Eigen::MatrixBase<ConfigOut_t>& qout) const
  {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigIn_t , ConfigVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Tangent_t  , TangentVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigOut_t, ConfigVector_t);
    derived().integrate_impl(q, v, qout);
  }
  
  template <class Derived>
  template<class Config_t, class Jacobian_t>
  void LieGroupBase<Derived>::
  integrateCoeffWiseJacobian(const Eigen::MatrixBase<Config_t >  & q,
                             const Eigen::MatrixBase<Jacobian_t> & J) const
  {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Config_t     , ConfigVector_t);

    derived().integrateCoeffWiseJacobian_impl(q,J);
    
  }

  template <class Derived>
  template<class Config_t, class Tangent_t, class JacobianOut_t>
  void LieGroupBase<Derived>::dIntegrate(const Eigen::MatrixBase<Config_t >  & q,
                                         const Eigen::MatrixBase<Tangent_t>  & v,
                                         const Eigen::MatrixBase<JacobianOut_t>& J,
                                         const ArgumentPosition arg) const
  {
    assert((arg==ARG0||arg==ARG1) && "arg should be either ARG0 or ARG1");
    
    switch (arg) {
      case ARG0:
        dIntegrate_dq(q,v,J); return;
      case ARG1:
        dIntegrate_dv(q,v,J); return;
      default: return;
    }
  }

  template <class Derived>
  template <class Config_t, class Tangent_t, class JacobianOut_t>
  void LieGroupBase<Derived>::dIntegrate_dq(
      const Eigen::MatrixBase<Config_t >  & q,
      const Eigen::MatrixBase<Tangent_t>  & v,
      const Eigen::MatrixBase<JacobianOut_t>& J) const
  {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Config_t     , ConfigVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Tangent_t    , TangentVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(JacobianOut_t, JacobianMatrix_t);
    derived().dIntegrate_dq_impl(q, v, J);
  }

  template <class Derived>
  template <class Config_t, class Tangent_t, class JacobianOut_t>
  void LieGroupBase<Derived>::dIntegrate_dv(
      const Eigen::MatrixBase<Config_t >  & q,
      const Eigen::MatrixBase<Tangent_t>  & v,
      const Eigen::MatrixBase<JacobianOut_t>& J) const
  {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Config_t     , ConfigVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Tangent_t    , TangentVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(JacobianOut_t, JacobianMatrix_t);
    derived().dIntegrate_dv_impl(q, v, J);
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
  template <class Derived>
  template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
  void LieGroupBase<Derived>::interpolate(
      const Eigen::MatrixBase<ConfigL_t> & q0,
      const Eigen::MatrixBase<ConfigR_t> & q1,
      const Scalar& u,
      const Eigen::MatrixBase<ConfigOut_t>& qout) const
  {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigL_t  , ConfigVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigR_t  , ConfigVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigOut_t, ConfigVector_t);
    Derived().interpolate_impl(q0, q1, u, qout);
  }

  template <class Derived>
  template <class Config_t>
  void LieGroupBase<Derived>::normalize
  (const Eigen::MatrixBase<Config_t>& qout) const
  {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Config_t, ConfigVector_t);
    return derived().normalize_impl (qout);
  }

  /**
   * @brief      Generate a random joint configuration, normalizing quaternions when necessary.
   *
   * \warning    Do not take into account the joint limits. To shoot a configuration uniformingly
   *             depending on joint limits, see randomConfiguration
   *
   * @return     The joint configuration
   */
  template <class Derived>
  template <class Config_t>
  void LieGroupBase<Derived>::random
  (const Eigen::MatrixBase<Config_t>& qout) const
  {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Config_t, ConfigVector_t);
    return derived().random_impl (qout);
  }

  template <class Derived>
  template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
  void LieGroupBase<Derived>::randomConfiguration(
      const Eigen::MatrixBase<ConfigL_t> & lower_pos_limit,
      const Eigen::MatrixBase<ConfigR_t> & upper_pos_limit,
      const Eigen::MatrixBase<ConfigOut_t> & qout) const
  {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigL_t  , ConfigVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigR_t  , ConfigVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigOut_t, ConfigVector_t);
    derived ().randomConfiguration_impl(lower_pos_limit, upper_pos_limit, qout);
  }

  template <class Derived>
  template <class ConfigL_t, class ConfigR_t, class Tangent_t>
  void LieGroupBase<Derived>::difference(
      const Eigen::MatrixBase<ConfigL_t> & q0,
      const Eigen::MatrixBase<ConfigR_t> & q1,
      const Eigen::MatrixBase<Tangent_t>& d) const
  {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigL_t, ConfigVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigR_t, ConfigVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Tangent_t, TangentVector_t);
    derived().difference_impl(q0, q1, d);
  }

  template <class Derived>
  template <class ConfigL_t, class ConfigR_t, class JacobianLOut_t, class JacobianROut_t>
  void LieGroupBase<Derived>::Jdifference(
      const Eigen::MatrixBase<ConfigL_t> & q0,
      const Eigen::MatrixBase<ConfigR_t> & q1,
      const Eigen::MatrixBase<JacobianLOut_t>& J0,
      const Eigen::MatrixBase<JacobianROut_t>& J1) const
  {
    derived().template dDifference<ARG0> (q0, q1, J0);
    derived().template dDifference<ARG1> (q0, q1, J1);
  }

  template <class Derived>
  template <ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianOut_t>
  void LieGroupBase<Derived>::dDifference(
      const Eigen::MatrixBase<ConfigL_t> & q0,
      const Eigen::MatrixBase<ConfigR_t> & q1,
      const Eigen::MatrixBase<JacobianOut_t>& J) const
  {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigL_t, ConfigVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigR_t, ConfigVector_t);
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(JacobianOut_t, JacobianMatrix_t);
    PINOCCHIO_STATIC_ASSERT(arg==ARG0||arg==ARG1, arg_SHOULD_BE_ARG0_OR_ARG1);
    derived().template dDifference_impl<arg> (q0, q1, J);
  }

  template <class Derived>
  template <class ConfigL_t, class ConfigR_t>
  typename LieGroupBase<Derived>::Scalar
  LieGroupBase<Derived>::squaredDistance(
      const Eigen::MatrixBase<ConfigL_t> & q0,
      const Eigen::MatrixBase<ConfigR_t> & q1) const
  {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigL_t, ConfigVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigR_t, ConfigVector_t);
    return derived().squaredDistance_impl(q0, q1);
  }

  template <class Derived>
  template <class ConfigL_t, class ConfigR_t>
  typename LieGroupBase<Derived>::Scalar
  LieGroupBase<Derived>::distance(
      const Eigen::MatrixBase<ConfigL_t> & q0,
      const Eigen::MatrixBase<ConfigR_t> & q1) const
  {
    return sqrt(squaredDistance(q0, q1));
  }

  template <class Derived>
  template <class ConfigL_t, class ConfigR_t>
  bool LieGroupBase<Derived>::isSameConfiguration(
      const Eigen::MatrixBase<ConfigL_t> & q0,
      const Eigen::MatrixBase<ConfigR_t> & q1,
      const Scalar & prec) const
  {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigL_t, ConfigVector_t);
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigR_t, ConfigVector_t);
    return derived().isSameConfiguration_impl(q0, q1, prec);
  }

  // ----------------- API that allocates memory ---------------------------- //


  template <class Derived>
  template <class Config_t, class Tangent_t>
  typename LieGroupBase<Derived>::ConfigVector_t
  LieGroupBase<Derived>::integrate(const Eigen::MatrixBase<Config_t>  & q,
                                            const Eigen::MatrixBase<Tangent_t> & v) const
  {
    ConfigVector_t qout;
    integrate(q, v, qout);
    return qout;
  }

  template <class Derived>
  template <class ConfigL_t, class ConfigR_t>
  typename LieGroupBase<Derived>::ConfigVector_t LieGroupBase<Derived>::interpolate(
      const Eigen::MatrixBase<ConfigL_t> & q0,
      const Eigen::MatrixBase<ConfigR_t> & q1,
      const Scalar& u) const
  {
    ConfigVector_t qout;
    interpolate(q0, q1, u, qout);
    return qout;
  }

  template <class Derived>
  typename LieGroupBase<Derived>::ConfigVector_t
  LieGroupBase<Derived>::random() const
  {
    ConfigVector_t qout;
    random(qout);
    return qout;
  }

  template <class Derived>
  template <class ConfigL_t, class ConfigR_t>
  typename LieGroupBase<Derived>::ConfigVector_t
  LieGroupBase<Derived>::randomConfiguration
  (const Eigen::MatrixBase<ConfigL_t> & lower_pos_limit,
   const Eigen::MatrixBase<ConfigR_t> & upper_pos_limit) const
  {
    ConfigVector_t qout;
    randomConfiguration(lower_pos_limit, upper_pos_limit, qout);
    return qout;
  }

  template <class Derived>
  template <class ConfigL_t, class ConfigR_t>
  typename LieGroupBase<Derived>::TangentVector_t LieGroupBase<Derived>::difference(
      const Eigen::MatrixBase<ConfigL_t> & q0,
      const Eigen::MatrixBase<ConfigR_t> & q1) const
  {
    TangentVector_t diff;
    difference(q0, q1, diff);
    return diff;
  }

  // ----------------- Default implementations ------------------------------ //
  template <class Derived>
  template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
  void LieGroupBase<Derived>::interpolate_impl(
      const Eigen::MatrixBase<ConfigL_t> & q0,
      const Eigen::MatrixBase<ConfigR_t> & q1,
      const Scalar& u,
      const Eigen::MatrixBase<ConfigOut_t>& qout) const
  {
    if     (u == 0) const_cast<Eigen::MatrixBase<ConfigOut_t>&>(qout) = q0;
    else if(u == 1) const_cast<Eigen::MatrixBase<ConfigOut_t>&>(qout) = q1;
    else 
    {
      TangentVector_t vdiff(u * difference(q0, q1));
      integrate(q0, vdiff, qout);
    }
  }

  template <class Derived>
  template <class ConfigL_t, class ConfigR_t>
  typename LieGroupBase<Derived>::Scalar
  LieGroupBase<Derived>::squaredDistance_impl(
      const Eigen::MatrixBase<ConfigL_t> & q0,
      const Eigen::MatrixBase<ConfigR_t> & q1) const
  {
    TangentVector_t t;
    difference(q0, q1, t);
    return t.squaredNorm();
  }

  template <class Derived>
  template <class ConfigL_t, class ConfigR_t>
  bool LieGroupBase<Derived>::isSameConfiguration_impl(
      const Eigen::MatrixBase<ConfigL_t> & q0,
      const Eigen::MatrixBase<ConfigR_t> & q1,
      const Scalar & prec) const
  {
    return q0.isApprox(q1, prec);
  }

  template <class Derived>
  typename LieGroupBase <Derived>::Index
  LieGroupBase <Derived>::nq () const
  {
    return derived().nq();
  }

  template <class Derived>
  typename LieGroupBase <Derived>::Index
  LieGroupBase <Derived>::nv () const
  {
    return derived().nv();
  }

  template <class Derived>
  typename LieGroupBase <Derived>::ConfigVector_t
  LieGroupBase <Derived>::neutral () const
  {
    return derived().neutral();
  }

  template <class Derived>
  std::string LieGroupBase <Derived>::name () const
  {
    return derived().name();
  }

} // namespace pinocchio

#endif // __pinocchio_lie_group_operation_base_hxx__
