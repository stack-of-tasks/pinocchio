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

/** \addtogroup algo_kinematics
 *
 *  This module is related to kinematics computations
 *  
 *
 */

namespace se3
{
  ///
  /// \brief Browse through the kinematic structure with a void step.
  ///
  /// \note This void step allows to quantify the time spent in the rollout.
  ///
  /// \ingroup algo_kinematics
  /// 
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  inline void emptyForwardPass(const Model & model,
                               Data & data);
  
  ///
  /// \brief Update the joint placement according to the current joint configuration.
  ///
  /// \ingroup algo_kinematics
  /// 
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  ///
  inline void forwardKinematics(const Model & model,
                                Data & data,
                                const Eigen::VectorXd & q);

  ///
  /// \brief Update the joint placement according to the current joint configuration and velocity.
  ///
  /// \ingroup algo_kinematics
  /// 
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  /// \param[in] v The joint velocity (vector dim model.nv).
  ///
  inline void forwardKinematics(const Model & model,
                                Data & data,
                                const Eigen::VectorXd & q,
                                const Eigen::VectorXd & v);
  ///
  /// \brief Update the joint placement according to the current joint configuration, velocity and acceleration.
  ///
  /// \ingroup algo_kinematics
  /// 
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  /// \param[in] v The joint velocity (vector dim model.nv).
  /// \param[in] a The joint acceleration (vector dim model.nv).
  ///
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
