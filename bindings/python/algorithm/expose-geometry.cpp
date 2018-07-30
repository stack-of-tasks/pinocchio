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

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/geometry.hpp"

namespace se3
{
  namespace python
  {
    
    void exposeGeometryAlgo()
    {
      using namespace Eigen;
      
      bp::def("updateGeometryPlacements",
              &updateGeometryPlacements<JointCollectionDefault,VectorXd>,
              bp::args("model", "data", "geometry model", "geometry data", "Configuration vector q (size Model::nq)"),
              "Update the placement of the collision objects according to the current configuration."
              "The algorithm also updates the current placement of the joint in Data."
              );
      
      bp::def("updateGeometryPlacements",
              &updateGeometryPlacements<JointCollectionDefault>,
              bp::args("model", "data", "geometry model", "geometry data"),
              "Update the placement of the collision objects according to the current joint placement stored in data."
              );
      
#ifdef WITH_HPP_FCL       
      bp::def("computeCollision",computeCollision,
              bp::args("geometry model", "geometry data", "collision pair index"),
              "Check if the collision objects of a collision pair for a given Geometry Model and Data are in collision."
             "The collision pair is given by the two index of the collision objects."
              );

      bp::def("computeCollisions",
              (bool (*)(const GeometryModel &, GeometryData &, const bool))&computeCollisions,
              bp::args("geometry model","geometry data","stop at first collision"),
              "Determine if collision pairs are effectively in collision."
              );
      
      bp::def("computeCollisions",
              &computeCollisions<JointCollectionDefault,VectorXd>,
              bp::args("model","data","geometry model","geometry data","Configuration q (size Model::nq)", "bool"),
              "Update the geometry for a given configuration and"
              "determine if all collision pairs are effectively in collision or not."
              );
      
      bp::def("computeDistance",&computeDistance,
              bp::args("geometry model","geometry data", "pairIndex"),
              "Compute the distance between the two geometry objects of a given collision pair for a GeometryModel and associated GeometryData.",
              bp::return_value_policy<bp::return_by_value>()
//              bp::with_custodian_and_ward_postcall<0,1>()
              );

      bp::def("computeDistances",
              (std::size_t (*)(const GeometryModel &, GeometryData &))&computeDistances,
              bp::args("geometry model","geometry data"),
              "Compute the distance between each collision pair for a given GeometryModel and associated GeometryData."
              );
      
      bp::def("computeDistances",
              &computeDistances<JointCollectionDefault,VectorXd>,
              bp::args("model","data","geometry model","geometry data","Configuration q (size Model::nq)"),
              "Update the geometry for a given configuration and"
              "compute the distance between each collision pair"
              );
#endif // WITH_HPP_FCL      
    }
  } // namespace python
} // namespace se3
