//
// Copyright (c) 2016-2018 CNRS
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

#ifndef __se3_aba_hpp__
#define __se3_aba_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace se3
{
  ///
  /// \brief The Articulated-Body algorithm. It computes the forward dynamics, aka the joint accelerations given the current state and actuation of the model.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] tau The joint torque vector (dim model.nv).
  ///
  /// \return The current joint acceleration stored in data.ddq.
  ///
  inline const Eigen::VectorXd &
  aba(const Model & model,
      Data & data,
      const Eigen::VectorXd & q,
      const Eigen::VectorXd & v,
      const Eigen::VectorXd & tau);

  ///
  /// \brief The Articulated-Body algorithm. It computes the forward dynamics, aka the joint accelerations given the current state and actuation of the model.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] tau The joint torque vector (dim model.nv).
  /// \param[in] fext Vector of external forces expressed in the local frame of the joints (dim model.njoints)
  ///
  /// \return The current joint acceleration stored in data.ddq.
  ///
  inline const Eigen::VectorXd &
  aba(const Model & model,
      Data & data,
      const Eigen::VectorXd & q,
      const Eigen::VectorXd & v,
      const Eigen::VectorXd & tau,
      const container::aligned_vector<Force> & fext);
  
  ///
  /// \brief Computes the inverse of the joint space inertia matrix using Articulated Body formulation.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The inverse of the joint space inertia matrix stored in data.ddq.
  ///
  inline const Data::RowMatrixXd &
  computeMinverse(const Model & model,
                  Data & data,
                  const Eigen::VectorXd & q);


  DEFINE_ALGO_CHECKER(ABA);

} // namespace se3

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/aba.hxx"

#endif // ifndef __se3_aba_hpp__
