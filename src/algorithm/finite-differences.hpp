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

#ifndef __se3_finite_differences_hpp__
#define __se3_finite_differences_hpp__

#include "pinocchio/multibody/model.hpp"

/** \addtogroup algo_finitediff
 *
 *  This module is related to finite differences
 *  
 *
 */

namespace se3 {
  
  ///
  /// \brief Computes the finite difference increments for each degree of freedom according to the current joint configuration.
  ///
  /// \ingroup algo_finitediff
  /// 
  /// \input[in] model The model of the kinematic tree.
  ///
  /// \returns The finite difference increments for each degree of freedom.
  ///
  inline Eigen::VectorXd finiteDifferenceIncrement(const Model & model);
}

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/finite-differences.hxx"

#endif // ifndef __se3_finite_differences_hpp__
