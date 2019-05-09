//
// Copyright (c) 2015-2016 CNRS
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
              bp::args("model", "data", "geometry model", "geometry data", "Configuration vector q (size Model::nq)"),
              "Update the placement of the collision objects according to the current configuration."
              "The algorithm also updates the current placement of the joint in Data."
              );
      
      bp::def("updateGeometryPlacements",
              &updateGeometryPlacements<double,0,JointCollectionDefaultTpl>,
              bp::args("model", "data", "geometry model", "geometry data"),
              "Update the placement of the collision objects according to the current joint placement stored in data."
              );

      bp::def("setGeometryMeshScales",
              (void (*)(GeometryModel &, const Vector3d &))&setGeometryMeshScales<Vector3d>,
              bp::args("geometry model", "scale"),
              "Set a mesh scaling vector to each GeometryObject contained in the the GeometryModel."
              );

      bp::def("setGeometryMeshScales",
              (void (*)(GeometryModel &, const double))&setGeometryMeshScales,
              bp::args("geometry model", "scale"),
              "Set an isotropic mesh scaling to each GeometryObject contained in the the GeometryModel."
              );

#ifdef PINOCCHIO_WITH_HPP_FCL       
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
              &computeCollisions<double,0,JointCollectionDefaultTpl,VectorXd>,
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
              &computeDistances<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model","data","geometry model","geometry data","Configuration q (size Model::nq)"),
              "Update the geometry for a given configuration and"
              "compute the distance between each collision pair"
              );
#endif // PINOCCHIO_WITH_HPP_FCL      
    }
  } // namespace python
} // namespace pinocchio
