//
// Copyright (c) 2019 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include <boost/python.hpp>
#include "pinocchio/bindings/python/utils/namespace.hpp"
#include "pinocchio/math/rpy.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    Eigen::Matrix3d rpyToMatrix_proxy(const Eigen::Vector3d & rpy)
    {
      return pinocchio::rpy::rpyToMatrix(rpy);
    }

    Eigen::Vector3d matrixToRpy_proxy(const Eigen::Matrix3d & R)
    {
      return pinocchio::rpy::matrixToRpy(R);
    }

    void exposeRpy()
    {
      using namespace Eigen;
      using namespace pinocchio::rpy;

      {
        // using the rpy scope
        bp::scope current_scope = getOrCreatePythonNamespace("rpy");

        bp::def("rpyToMatrix",
                static_cast<Matrix3d (*)(const double, const double, const double)>(&rpyToMatrix),
                bp::args("roll", "pitch", "yaw"),
                "Given (r, p, y), the rotation is given as R = R_z(y)R_y(p)R_x(r),"
                " where R_a(theta) denotes the rotation of theta degrees axis a");

        bp::def("rpyToMatrix",
                &rpyToMatrix_proxy,
                bp::arg("rpy"),
                "Given (r, p, y), the rotation is given as R = R_z(y)R_y(p)R_x(r),"
                " where R_a(theta) denotes the rotation of theta degrees axis a");

        bp::def("matrixToRpy",
                &matrixToRpy_proxy,
                bp::arg("R"),
                "Given a rotation matrix R, the angles (r, p, y) are given so that R = R_z(y)R_y(p)R_x(r),"
                " where R_a(theta) denotes the rotation of theta degrees axis a."
                " The angles are guaranteed to be in the ranges: r in [-pi,pi],"
                " p in[-pi/2,pi/2], y in [-pi,pi]");
      }
      
    }
    
  } // namespace python
} // namespace pinocchio
