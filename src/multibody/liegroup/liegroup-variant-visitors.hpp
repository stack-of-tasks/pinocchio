//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_lie_group_variant_visitor_hpp__
#define __pinocchio_lie_group_variant_visitor_hpp__

#include "pinocchio/multibody/liegroup/fwd.hpp"

namespace pinocchio
{
  
  /**
   * @brief      Visit a LieGroupVariant to get the dimension of
   *             the Lie group configuration space
   *
   * @param[in]  lg  the LieGroupVariant.
   *
   * @return     The dimension of the Lie group configuration space
   */
  template<typename LieGroupCollection>
  inline int nq(const LieGroupGenericTpl<LieGroupCollection> & lg);
  
  /**
   * @brief      Visit a LieGroupVariant to get the dimension of
   *             the Lie group tangent space
   *
   * @param[in]  lg  the LieGroupVariant.
   *
   * @return     The dimension of the Lie group tangent space
   */
  template<typename LieGroupCollection>
  inline int nv(const LieGroupGenericTpl<LieGroupCollection> & lg);
  
  /**
   * @brief      Visit a LieGroupVariant to get the name of it
   *
   * @param[in]  lg  the LieGroupVariant.
   *
   * @return     The Lie group name
   */
  template<typename LieGroupCollection>
  inline std::string name(const LieGroupGenericTpl<LieGroupCollection> & lg);
  
  /**
   * @brief      Visit a LieGroupVariant to get the neutral element of it
   *
   * @param[in]  lg  the LieGroupVariant.
   *
   * @return     The Lie group neutral element
   */
  template<typename LieGroupCollection>
  inline Eigen::Matrix<typename LieGroupCollection::Scalar,Eigen::Dynamic,1,LieGroupCollection::Options>
  neutral(const LieGroupGenericTpl<LieGroupCollection> & lg);
  
  /**
   * @brief      Visit a LieGroupVariant to call its integrate method
   *
   * @param[in]  lg  the LieGroupVariant.
   * @param[in]  q   the starting configuration.
   * @param[in]  v   the tangent velocity.
   *
   */
  template<typename LieGroupCollection, class ConfigIn_t, class Tangent_t, class ConfigOut_t>
  inline void integrate(const LieGroupGenericTpl<LieGroupCollection> & lg,
                        const Eigen::MatrixBase<ConfigIn_t> & q,
                        const Eigen::MatrixBase<Tangent_t>  & v,
                        const Eigen::MatrixBase<ConfigOut_t>& qout);

  template<typename LieGroupCollection, class Config_t>
  inline void random(const LieGroupGenericTpl<LieGroupCollection> & lg,
                     const Eigen::MatrixBase<Config_t> & qout);

  template<typename LieGroupCollection, class Config_t>
  inline void normalize(const LieGroupGenericTpl<LieGroupCollection> & lg,
                        const Eigen::MatrixBase<Config_t> & qout);

  template<typename LieGroupCollection, class ConfigL_t, class ConfigR_t>
  inline bool isSameConfiguration(const LieGroupGenericTpl<LieGroupCollection> & lg,
                          const Eigen::MatrixBase<ConfigL_t> & q0,
                          const Eigen::MatrixBase<ConfigR_t> & q1,
                          const typename ConfigL_t::Scalar& prec);

  template<typename LieGroupCollection, class ConfigL_t, class ConfigR_t>
  inline typename ConfigL_t::Scalar
  squaredDistance(const LieGroupGenericTpl<LieGroupCollection> & lg,
                  const Eigen::MatrixBase<ConfigL_t> & q0,
                  const Eigen::MatrixBase<ConfigR_t> & q1);

  template<typename LieGroupCollection, class ConfigL_t, class ConfigR_t>
  inline typename ConfigL_t::Scalar
  distance(const LieGroupGenericTpl<LieGroupCollection> & lg,
           const Eigen::MatrixBase<ConfigL_t> & q0,
           const Eigen::MatrixBase<ConfigR_t> & q1)
  {
    return std::sqrt(squaredDistance(lg, q0, q1));
  }
  
  template<typename LieGroupCollection, class ConfigL_t, class ConfigR_t, class Tangent_t>
  inline void difference(const LieGroupGenericTpl<LieGroupCollection> & lg,
                         const Eigen::MatrixBase<ConfigL_t> & q0,
                         const Eigen::MatrixBase<ConfigR_t> & q1,
                         const Eigen::MatrixBase<Tangent_t> & v);

  template<typename LieGroupCollection, class ConfigL_t, class ConfigR_t, class ConfigOut_t>
  inline void randomConfiguration(const LieGroupGenericTpl<LieGroupCollection> & lg,
                                  const Eigen::MatrixBase<ConfigL_t> & q0,
                                  const Eigen::MatrixBase<ConfigR_t> & q1,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout);

  template<typename LieGroupCollection, class ConfigL_t, class ConfigR_t, class ConfigOut_t>
  inline void interpolate(const LieGroupGenericTpl<LieGroupCollection> & lg,
                          const Eigen::MatrixBase<ConfigL_t> & q0,
                          const Eigen::MatrixBase<ConfigR_t> & q1,
                          const typename ConfigL_t::Scalar& u,
                          const Eigen::MatrixBase<ConfigOut_t> & qout);

  template<typename LieGroupCollection, class Config_t, class Tangent_t, class JacobianOut_t>
  void dIntegrate(const LieGroupGenericTpl<LieGroupCollection> & lg,
                  const Eigen::MatrixBase<Config_t >  & q,
                  const Eigen::MatrixBase<Tangent_t>  & v,
                  const Eigen::MatrixBase<JacobianOut_t> & J,
                  const ArgumentPosition arg,
                  const AssignmentOperatorType op = SETTO);

  template<typename LieGroupCollection, class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
  void dIntegrate(const LieGroupGenericTpl<LieGroupCollection> & lg,
                  const Eigen::MatrixBase<Config_t >  & q,
                  const Eigen::MatrixBase<Tangent_t>  & v,
                  const Eigen::MatrixBase<JacobianIn_t> & J_in,
                  int self,
                  const Eigen::MatrixBase<JacobianOut_t> & J_out,
                  const ArgumentPosition arg,
                  const AssignmentOperatorType op = SETTO);

  template<typename LieGroupCollection, class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
  void dIntegrate(const LieGroupGenericTpl<LieGroupCollection> & lg,
                  const Eigen::MatrixBase<Config_t >  & q,
                  const Eigen::MatrixBase<Tangent_t>  & v,
                  int self,
                  const Eigen::MatrixBase<JacobianIn_t> & J_in,
                  const Eigen::MatrixBase<JacobianOut_t> & J_out,
                  const ArgumentPosition arg,
                  const AssignmentOperatorType op = SETTO);

  template<typename LieGroupCollection, class ConfigL_t, class ConfigR_t, class JacobianOut_t>
  void dDifference(const LieGroupGenericTpl<LieGroupCollection> & lg,
                   const Eigen::MatrixBase<ConfigL_t> & q0,
                   const Eigen::MatrixBase<ConfigR_t> & q1,
                   const Eigen::MatrixBase<JacobianOut_t> & J,
                   const ArgumentPosition arg);

  template<typename LieGroupCollection, class ConfigL_t, class ConfigR_t, class JacobianIn_t, class JacobianOut_t>
  void dDifference(const LieGroupGenericTpl<LieGroupCollection> & lg,
                   const Eigen::MatrixBase<ConfigL_t> & q0,
                   const Eigen::MatrixBase<ConfigR_t> & q1,
                   const Eigen::MatrixBase<JacobianIn_t> & Jin,
                   int self,
                   const Eigen::MatrixBase<JacobianOut_t> & Jout,
                   const ArgumentPosition arg);

  template<typename LieGroupCollection, class ConfigL_t, class ConfigR_t, class JacobianIn_t, class JacobianOut_t>
  void dDifference(const LieGroupGenericTpl<LieGroupCollection> & lg,
                   const Eigen::MatrixBase<ConfigL_t> & q0,
                   const Eigen::MatrixBase<ConfigR_t> & q1,
                   int self,
                   const Eigen::MatrixBase<JacobianIn_t> & Jin,
                   const Eigen::MatrixBase<JacobianOut_t> & Jout,
                   const ArgumentPosition arg);

  template<typename LieGroupCollection, class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
  void dIntegrateTransport(const LieGroupGenericTpl<LieGroupCollection> & lg,
                           const Eigen::MatrixBase<Config_t > & q,
                           const Eigen::MatrixBase<Tangent_t> & v,
                           const Eigen::MatrixBase<JacobianIn_t> & J_in,
                           const Eigen::MatrixBase<JacobianOut_t> & J_out,
                           const ArgumentPosition arg);

  template<typename LieGroupCollection, class Config_t, class Tangent_t, class JacobianOut_t>
  void dIntegrateTransport(const LieGroupGenericTpl<LieGroupCollection> & lg,
                           const Eigen::MatrixBase<Config_t > & q,
                           const Eigen::MatrixBase<Tangent_t> & v,
                           const Eigen::MatrixBase<JacobianOut_t> & J,
                           const ArgumentPosition arg);
}

/// Details
#include "pinocchio/multibody/liegroup/liegroup-variant-visitors.hxx"

#endif // ifndef __pinocchio_lie_group_variant_visitor_hpp__
