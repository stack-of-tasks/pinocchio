//
// Copyright (c) 2018 CNRS
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

#ifndef __se3_regressor_hpp__
#define __se3_regressor_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace se3
{
  
  namespace regressor
  {
    
    
    ///
    /// \brief Computes the static regressor that links the center of mass positions of all the links
    ///        to the center of mass of the complete model according to the current configuration of the robot.
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    /// \param[in] q The joint configuration vector (dim model.nq).
    ///
    /// \return The static regressor of the system.
    ///
    inline Data::Matrix3x &
    computeStaticRegressor(const Model & model,
                           Data & data,
                           const Eigen::VectorXd & q);
  }
  
} // namespace se3

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/regressor.hxx"

#endif // ifndef __se3_regressor_hpp__
