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
#include "pinocchio/bindings/python/geometry-object.hpp"
#include "pinocchio/bindings/python/geometry-model.hpp"
#include "pinocchio/bindings/python/geometry-data.hpp"

namespace se3
{
  namespace python
  {
    
    static void updateGeometryPlacements_proxy(const ModelHandler & model,
                                               DataHandler & data,
                                               const GeometryModelHandler & geom_model,
                                               GeometryDataHandler & geom_data,
                                               const VectorXd_fx & q
                                               )
    {
      return updateGeometryPlacements(*model, *data, *geom_model, *geom_data, q);
    }
    
    static bool computeCollisions_proxy(GeometryDataHandler & data_geom,
                                        const bool stopAtFirstCollision)
    {
      return computeCollisions(*data_geom, stopAtFirstCollision);
    }
    
    static bool computeGeometryAndCollisions_proxy(const ModelHandler & model,
                                                   DataHandler & data,
                                                   const GeometryModelHandler & model_geom,
                                                   GeometryDataHandler & data_geom,
                                                   const VectorXd_fx & q,
                                                   const bool stopAtFirstCollision)
    {
      return computeCollisions(*model,*data,*model_geom, *data_geom, q, stopAtFirstCollision);
    }
    
    static void computeDistances_proxy(GeometryDataHandler & data_geom)
    {
      computeDistances(*data_geom);
    }
    
    static void computeGeometryAndDistances_proxy(const ModelHandler & model,
                                                  DataHandler & data,
                                                  const GeometryModelHandler & model_geom,
                                                  GeometryDataHandler & data_geom,
                                                  const Eigen::VectorXd & q
                                                  )
    {
      computeDistances(*model, *data, *model_geom, *data_geom, q);
    }
    
    void exposeGeometryAlgo()
    {
      bp::def("updateGeometryPlacements",updateGeometryPlacements_proxy,
              bp::args("Model", "Data", "GeometryModel", "GeometryData", "Configuration q (size Model::nq)"),
              "Update the placement of the collision objects according to the current configuration."
              "The algorithm also updates the current placement of the joint in Data."
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
      
      bp::def("computeDistances",computeDistances_proxy,
              bp::args("GeometryData"),
              "Compute the distance between each collision pair."
              );
      
      bp::def("computeGeometryAndDistances",computeGeometryAndDistances_proxy,
              bp::args("Model","Data","GeometryModel","GeometryData","Configuration q (size Model::nq)"),
              "Update the geometry for a given configuration and"
              "compute the distance between each collision pair"
              );
    }
  } // namespace python
} // namespace se3
