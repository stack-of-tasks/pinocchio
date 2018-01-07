//
// Copyright (c) 2017-2018 CNRS
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

#ifndef __se3_rnea_derivatives_hpp__
#define __se3_rnea_derivatives_hpp__

#include "pinocchio/multibody/model.hpp"

namespace se3
{

  ///
  /// \brief Computes the derivative of the generalized gravity contribution
  ///        with respect to the joint configuration.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[out] gravity_partial_dq Partial derivative of the generalized gravity vector with respect to the joint configuration.
  ///
  /// \remark gravity_partial_dq must be first initialized with zeros (gravity_partial_dq.setZero).
  ///
  /// \sa se3::computeGeneralizedGravity
  ///
  inline void
  computeGeneralizedGravityDerivatives(const Model & model, Data & data,
                                       const Eigen::VectorXd & q,
                                       Eigen::MatrixXd & gravity_partial_dq);
 

} // namespace se3 

#include "pinocchio/algorithm/rnea-derivatives.hxx"

#endif // ifndef __se3_rnea_derivatives_hpp__
