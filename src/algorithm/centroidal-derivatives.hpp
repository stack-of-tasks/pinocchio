//
// Copyright (c) 2018 INRIA
//

#ifndef __pinocchio_algorithm_centroidal_derivatives_hpp__
#define __pinocchio_algorithm_centroidal_derivatives_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{
  
  ///
  /// \brief Computes the analytical derivatives of the centroidal dynamics with respect to
  ///        the joint configuration vector, velocity and acceleration.
  ///
  /// \details Computes the first order approximation of the centroidal dynamics time derivative
  ///          and corresponds to the following equation
  ///          \f$
  ///               d\dot{h_{g}} = \frac{\partial \dot{h_{g}}}{\partial \mathbf{q}} d\mathbf{q}
  ///                            + \frac{\partial \dot{h_{g}}}{\partial \mathbf{v}} d\mathbf{v}
  ///                            + \frac{\partial \dot{h_{g}}}{\partial \mathbf{a}} d\mathbf{a}
  ///          \f$
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint acceleration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] a The joint acceleration vector (dim model.nv).
  /// \param[out] dh_dq The partial derivative of the centroidal momentum with respect to the configuration vector (dim 6 x model.nv).
  /// \param[out] dhdot_dq The partial derivative of the centroidal dynamics with respect to the configuration vector (dim 6 x model.nv).
  /// \param[out] dhdot_dv The partial derivative of the centroidal dynamics with respect to the velocity vector (dim 6 x model.nv).
  /// \param[out] dhdot_da The partial derivative of the centroidal dynamics with respect to the acceleration vector (dim 6 x model.nv).
  ///
  /// \result It also computes the current centroidal dynamics and its time derivative.
  ///         For information, the centroidal momentum matrix is equivalent to dhdot_da.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
           typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2,
           typename Matrix6xLike0, typename Matrix6xLike1, typename Matrix6xLike2, typename Matrix6xLike3>
  inline void
  computeCentroidalDynamicsDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                       DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                       const Eigen::MatrixBase<ConfigVectorType> & q,
                                       const Eigen::MatrixBase<TangentVectorType1> & v,
                                       const Eigen::MatrixBase<TangentVectorType2> & a,
                                       const Eigen::MatrixBase<Matrix6xLike0> & dh_dq,
                                       const Eigen::MatrixBase<Matrix6xLike1> & dhdot_dq,
                                       const Eigen::MatrixBase<Matrix6xLike2> & dhdot_dv,
                                       const Eigen::MatrixBase<Matrix6xLike3> & dhdot_da);
  ///
  /// \brief Retrive the analytical derivatives of the centroidal dynamics from the RNEA derivatives.
  ///        pinocchio::computeRNEADerivatives should have been called first.
  ///
  /// \details Computes the first order approximation of the centroidal dynamics time derivative
  ///          and corresponds to the following equation
  ///          \f$
  ///               d\dot{h_{g}} = \frac{\partial \dot{h_{g}}}{\partial \mathbf{q}} d\mathbf{q}
  ///                            + \frac{\partial \dot{h_{g}}}{\partial \mathbf{v}} d\mathbf{v}
  ///                            + \frac{\partial \dot{h_{g}}}{\partial \mathbf{a}} d\mathbf{a}
  ///          \f$
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[out] dhdot_dq The partial derivative of the centroidal dynamics with respect to the configuration vector (dim 6 x model.nv).
  /// \param[out] dhdot_dv The partial derivative of the centroidal dynamics with respect to the velocity vector (dim 6 x model.nv).
  /// \param[out] dhdot_da The partial derivative of the centroidal dynamics with respect to the acceleration vector (dim 6 x model.nv).
  ///
  /// \result It also computes the current centroidal dynamics and its time derivative.
  ///         For information, the centroidal momentum matrix is equivalent to dhdot_da.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
           typename Matrix6xLike0,typename Matrix6xLike1, typename Matrix6xLike2, typename Matrix6xLike3>
  inline void
  getCentroidalDynamicsDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                   DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                   const Eigen::MatrixBase<Matrix6xLike1> & dh_dq,
                                   const Eigen::MatrixBase<Matrix6xLike1> & dhdot_dq,
                                   const Eigen::MatrixBase<Matrix6xLike2> & dhdot_dv,
                                   const Eigen::MatrixBase<Matrix6xLike3> & dhdot_da);
  
  
} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/centroidal-derivatives.hxx"

#endif // ifndef __pinocchio_algorithm_centroidal_derivatives_hpp__
