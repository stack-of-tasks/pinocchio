//
// Copyright (c) 2015-2018 CNRS
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

#ifndef __se3_centroidal_hpp__
#define __se3_centroidal_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace se3
{
  
  ///
  /// \brief Computes the Centroidal Momentum Matrix, the Composite Ridig Body Inertia as well as the centroidal momenta
  ///        according to the current joint configuration and velocity.
  ///
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  ///
  /// \return The Centroidal Momentum Matrix Ag.
  ///
  inline const Data::Matrix6x &
  ccrba(const Model & model,
        Data & data,
        const Eigen::VectorXd & q,
        const Eigen::VectorXd & v);
  
  ///
  /// \brief Computes the time derivative of the Centroidal Momentum Matrix according to the current configuration and velocity vectors.
  ///
  /// \note The computed terms allow to decomposed the spatial momentum variation as following: \f$ \dot{h} = A_g \ddot{q} + \dot{A_g}(q,\dot{q})\dot{q}\f$.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint configuration vector (dim model.nv).
  ///
  /// \return The Centroidal Momentum Matrix time derivative dAg
  ///
  inline const Data::Matrix6x &
  dccrba(const Model & model,
         Data & data,
         const Eigen::VectorXd & q,
         const Eigen::VectorXd & v);
  
} // namespace se3

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/centroidal.hxx"

#endif // ifndef __se3_centroidal_hpp__
