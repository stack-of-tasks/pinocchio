//
// Copyright (c) 2015-2016 CNRS
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

#ifndef __se3_kinematics_hpp__
#define __se3_kinematics_hpp__

#include "pinocchio/multibody/model.hpp"

namespace se3
{
    ///
    /// \brief Browse through the model tree structure with an empty step
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    ///
  inline void emptyForwardPass(const Model & model,
                               Data & data);
  
  inline void forwardKinematics(const Model & model,
                                Data & data,
                                const Eigen::VectorXd & q);

  inline void forwardKinematics(const Model & model,
                                Data & data,
                                const Eigen::VectorXd & q,
                                const Eigen::VectorXd & v);
  
  inline void forwardKinematics(const Model & model,
                                Data & data,
                                const Eigen::VectorXd & q,
                                const Eigen::VectorXd & v,
                                const Eigen::VectorXd & a);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/kinematics.hxx"


#endif // ifndef __se3_kinematics_hpp__
