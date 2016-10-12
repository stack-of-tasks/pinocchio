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
#include "pinocchio/bindings/python/multibody/geometry-object.hpp"
#include "pinocchio/bindings/python/multibody/geometry-model.hpp"
#include "pinocchio/bindings/python/multibody/geometry-data.hpp"

namespace se3
{
  namespace python
  {
    
    static void updateGeometryPlacements_proxy(const Model & model,
                                               DataHandler & data,
                                               const GeometryModelHandler & geom_model,
                                               GeometryDataHandler & geom_data,
                                               const Eigen::VectorXd & q
                                               )
    {
      return updateGeometryPlacements(model, *data, *geom_model, *geom_data, q);
    }

#ifdef WITH_HPP_FCL   

    static bool computeCollision_proxy(const GeometryModelHandler & model_geom,
                                       GeometryDataHandler & data_geom,
                                       const PairIndex & pairId)
    {
      return computeCollision(*model_geom, *data_geom, pairId);
    }

    static bool computeCollisions_proxy(const GeometryModelHandler & model_geom,
                                        GeometryDataHandler & data_geom,
                                        const bool stopAtFirstCollision)
    {
      return computeCollisions(*model_geom, *data_geom, stopAtFirstCollision);
    }
    
    static bool computeGeometryAndCollisions_proxy(const Model & model,
                                                   DataHandler & data,
                                                   const GeometryModelHandler & model_geom,
                                                   GeometryDataHandler & data_geom,
                                                   const Eigen::VectorXd & q,
                                                   const bool stopAtFirstCollision)
    {
      return computeCollisions(model,*data,*model_geom, *data_geom, q, stopAtFirstCollision);
    }
    
    static fcl::DistanceResult computeDistance_proxy(const GeometryModelHandler & model_geom,
                                                     GeometryDataHandler & data_geom,
                                                     const PairIndex & pairId)
    {
      return computeDistance(*model_geom, *data_geom, pairId);
    }

    static std::size_t computeDistances_proxy(const GeometryModelHandler & model_geom,
                                              GeometryDataHandler & data_geom)
    {
      return computeDistances(*model_geom, *data_geom);
    }
    
    static std::size_t computeGeometryAndDistances_proxy(const Model & model,
                                                         DataHandler & data,
                                                         const GeometryModelHandler & model_geom,
                                                         GeometryDataHandler & data_geom,
                                                         const Eigen::VectorXd & q
                                                         )
    {
      return computeDistances<true>(model, *data, *model_geom, *data_geom, q);
    }

#endif // WITH_HPP_FCL

    void exposeGeometryAlgo()
    {
      bp::def("updateGeometryPlacements",updateGeometryPlacements_proxy,
              bp::args("Model", "Data", "GeometryModel", "GeometryData", "Configuration q (size Model::nq)"),
              "Update the placement of the collision objects according to the current configuration."
              "The algorithm also updates the current placement of the joint in Data."
              );
      
#ifdef WITH_HPP_FCL       
      bp::def("computeCollision", computeCollision_proxy,
              bp::args("GoometryModel", "GeometryData", "pairIndex"),
              "Check if the collision objects of a collision pair for a given Geometry Model and Data are in collision."
             "The collision pair is given by the two index of the collision objects."
              );

      bp::def("computeCollisions",computeCollisions_proxy,
              bp::args("GeometryData","bool"),
              "Determine if collision pairs are effectively in collision."
              );
      
      bp::def("computeGeometryAndCollisions",computeGeometryAndCollisions_proxy,
              bp::args("Model","Data","GeometryModel","GeometryData","Configuration q (size Model::nq)", "bool"),
              "Update the geometry for a given configuration and"
              "determine if all collision pairs are effectively in collision or not."
              );
      
      bp::def("computeDistance",computeDistance_proxy,
              bp::args("GeometryModel","GeometryData", "pairIndex"),
              "Compute the distance between the two geometry objects of a given collision pair for a GeometryModel and associated GeometryData."
              );

      bp::def("computeDistances",computeDistances_proxy,
              bp::args("GeometryModel","GeometryData"),
              "Compute the distance between each collision pair for a given GeometryModel and associated GeometryData."
              );
      
      bp::def("computeGeometryAndDistances",computeGeometryAndDistances_proxy,
              bp::args("Model","Data","GeometryModel","GeometryData","Configuration q (size Model::nq)"),
              "Update the geometry for a given configuration and"
              "compute the distance between each collision pair"
              );
#endif // WITH_HPP_FCL      
    }
  } // namespace python
} // namespace se3
