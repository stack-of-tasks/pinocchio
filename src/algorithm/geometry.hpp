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

#ifndef __se3_algo_geometry_hpp__
#define __se3_algo_geometry_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/multibody/geometry.hpp"

namespace se3
{


  ///
  /// \brief Apply a forward kinematics and update the placement of the geometry objects.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] geom The geometry model containing the collision objects.
  /// \param[out] geom_data The geometry data containing the placements of the collision objects. See oMg field in GeometryData.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  inline void updateGeometryPlacements(const Model & model,
                                       Data & data,
                                       const GeometryModel & geom,
                                       GeometryData & geom_data,
                                       const Eigen::VectorXd & q
                                       );
  
  ///
  /// \brief Update the placement of the geometry objects according to the current joint placements contained in data.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] geom The geometry model containing the collision objects.
  /// \param[out] geom_data The geometry data containing the placements of the collision objects. See oMg field in GeometryData.
  ///
  inline void updateGeometryPlacements(const Model & model,
                                       const Data & data,
                                       const GeometryModel & geom,
                                       GeometryData & geom_data
                                       );
#ifdef WITH_HPP_FCL
  inline bool computeCollisions(const Model & model,
                                Data & data,
                                const GeometryModel & model_geom,
                                GeometryData & data_geom,
                                const Eigen::VectorXd & q,
                                const bool stopAtFirstCollision = false
                                );

  inline bool computeCollisions(GeometryData & data_geom,
                                const bool stopAtFirstCollision = false
                                );

  /// Compute the distances of all collision pairs
  ///
  /// \param ComputeShortest default to true.
  /// \param data_geom
  /// \return When ComputeShortest is true, the index of the collision pair which has the shortest distance.
  ///         When ComputeShortest is false, the number of collision pairs.
  template <bool ComputeShortest>
  inline std::size_t computeDistances(GeometryData & data_geom);

  /// Compute the forward kinematics, update the goemetry placements and
  /// calls computeDistances(GeometryData&).
  template <bool ComputeShortest>
  inline std::size_t computeDistances(const Model & model,
                                      Data & data,
                                      const GeometryModel & model_geom,
                                      GeometryData & data_geom,
                                      const Eigen::VectorXd & q
                                      );

  inline void computeBodyRadius(const Model &         model,
                                const GeometryModel & geomModel,
                                GeometryData &        geomData);
#endif // WITH_HPP_FCL
} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/geometry.hxx"

#endif // ifndef __se3_algo_geometry_hpp__
