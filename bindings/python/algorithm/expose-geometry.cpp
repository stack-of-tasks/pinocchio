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

      bp::def(
        "updateGeometryPlacements",
        &updateGeometryPlacements<double, 0, JointCollectionDefaultTpl, VectorXd>,
        bp::args("model", "data", "geometry_model", "geometry_data", "q"),
        "Update the placement of the collision objects according to the current configuration.\n"
        "The algorithm also updates the current placement of the joint in Data.");

      bp::def(
        "updateGeometryPlacements", &updateGeometryPlacements<double, 0, JointCollectionDefaultTpl>,
        bp::args("model", "data", "geometry_model", "geometry_data"),
        "Update the placement of the collision objects according to the current joint "
        "placement stored in data.");
    }
  } // namespace python
} // namespace pinocchio
