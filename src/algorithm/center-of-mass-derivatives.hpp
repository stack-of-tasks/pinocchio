//
// Copyright (c) 2019 CNRS
//

#ifndef __pinocchio_center_of_mass_derivatives_hpp__
#define __pinocchio_center_of_mass_derivatives_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{
  ///
  /// \brief Computes the partial derivatie of the center-of-mass velocity with respect to 
  ///        the joint configuration q.
  ///        You must first call computForwardKinematicsDerivatives and computeCenterOfMass(q,vq)
  ///        before calling this function.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam Matrix3xOut Matrix3x containing the partial derivatives of the CoM velocity with respect to the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[out] v_partial_dq Partial derivative of the CoM velocity w.r.t. \f$ q \f$.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
           typename Matrix3xOut>
  inline void getCenterOfMassVelocityDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                 DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                 const Eigen::MatrixBase<Matrix3xOut> & vcom_partial_dq);



} // namespace pinocchio 

#include "pinocchio/algorithm/center-of-mass-derivatives.hxx"

#endif // ifndef __pinocchio_center_of_mass_derivatives_hpp__
