//
// Copyright (c) 2015-2022 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/geometry.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeGeometryAlgo()
    {
      using namespace Eigen;
      
      bp::def("updateGeometryPlacements",
              &updateGeometryPlacements<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model", "data", "geometry_model", "geometry_data", "q"),
              "Update the placement of the collision objects according to the current configuration.\n"
              "The algorithm also updates the current placement of the joint in Data."
              );
      
      bp::def("updateGeometryPlacements",
              &updateGeometryPlacements<double,0,JointCollectionDefaultTpl>,
              bp::args("model", "data", "geometry_model", "geometry_data"),
              "Update the placement of the collision objects according to the current joint placement stored in data."
              );

#ifdef PINOCCHIO_WITH_HPP_FCL       
      bp::def("computeCollision",computeCollision,
              bp::args("geometry_model", "geometry_data", "pair_index"),
              "Check if the collision objects of a collision pair for a given Geometry Model and Data are in collision.\n"
              "The collision pair is given by the two index of the collision objects."
              );

      bp::def("computeCollisions",
              (bool (*)(const GeometryModel &, GeometryData &, const bool))&computeCollisions,
              (bp::arg("geometry_model"),bp::arg("geometry_data"),bp::arg("stop_at_first_collision") = false),
              "Determine if all collision pairs are effectively in collision or not."
              );
      
      bp::def("computeCollisions",
              &computeCollisions<double,0,JointCollectionDefaultTpl,VectorXd>,
              (bp::arg("model"),bp::arg("data"),bp::arg("geometry_model"),bp::arg("geometry_data"),bp::arg("q"),bp::arg("stop_at_first_collision") = false),
              "Update the geometry for a given configuration and "
              "determine if all collision pairs are effectively in collision or not."
              );
      
      bp::def("computeDistance",&computeDistance,
              bp::args("geometry_model", "geometry_data", "pair_index"),
              "Compute the distance between the two geometry objects of a given collision pair for a GeometryModel and associated GeometryData.",
              bp::with_custodian_and_ward_postcall<0,2,bp::return_value_policy<bp::reference_existing_object> >()
              );

      bp::def("computeDistances",
              (std::size_t (*)(const GeometryModel &, GeometryData &))&computeDistances,
              bp::args("geometry_model","geometry_data"),
              "Compute the distance between each collision pair for a given GeometryModel and associated GeometryData."
              );
      
      bp::def("computeDistances",
              &computeDistances<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model","data","geometry_model","geometry_data","q"),
              "Update the geometry for a given configuration and "
              "compute the distance between each collision pair"
              );
      
      bp::def("computeBodyRadius",
              &computeBodyRadius<double,0,JointCollectionDefaultTpl>,
              bp::args("model","geometry_model","geometry_data"),
              "Compute the radius of the geometry volumes attached to every joints.");
      
      exposeBroadphase();
#endif // PINOCCHIO_WITH_HPP_FCL
      
    }
  } // namespace python
} // namespace pinocchio
