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
  /// \brief Computes the generalized gravity contribution \f$ g(q) \f$ of the Lagrangian dynamics:
  /// <CENTER> \f$ \begin{eqnarray} M \ddot{q} + c(q, \dot{q}) + g(q) = \tau  \end{eqnarray} \f$ </CENTER> <BR>
  /// \note This function is equivalent to se3::rnea(model, data, q, 0, 0).
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The bias terms stored in data.g.
  ///
  inline void
  computeGeneralizedGravityDerivatives(const Model & model, Data & data,
                                       const Eigen::VectorXd & q,
                                       Eigen::MatrixXd & gravity_partial_dq);
 

} // namespace se3 

#include "pinocchio/algorithm/rnea-derivatives.hxx"

#endif // ifndef __se3_rnea_derivatives_hpp__
