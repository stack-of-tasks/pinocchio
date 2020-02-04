//
// Copyright (c) 2019-2020 CNRS INRIA
//

#include <boost/python.hpp>

#include "pinocchio/bindings/python/fwd.hpp"
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

    Eigen::Matrix3d rotate(const std::string & axis, const double ang)
    {
      if(axis.length() != 1U)
          throw std::invalid_argument(std::string("Invalid axis: ").append(axis));
      Eigen::Vector3d u;
      u.setZero();
      const char axis_ = axis[0];
      switch(axis_)
      {
        case 'x': u[0] = 1.; break;
        case 'y': u[1] = 1.; break;
        case 'z': u[2] = 1.; break;
        default: throw std::invalid_argument(std::string("Invalid axis: ").append(1U,axis_));
      }

      return Eigen::AngleAxisd(ang, u).matrix();
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
                " where R_a(theta) denotes the rotation of theta radians axis a");

        bp::def("rpyToMatrix",
                &rpyToMatrix_proxy,
                bp::arg("rpy"),
                "Given (r, p, y), the rotation is given as R = R_z(y)R_y(p)R_x(r),"
                " where R_a(theta) denotes the rotation of theta radians axis a");

        bp::def("matrixToRpy",
                &matrixToRpy_proxy,
                bp::arg("R"),
                "Given a rotation matrix R, the angles (r, p, y) are given so that R = R_z(y)R_y(p)R_x(r),"
                " where R_a(theta) denotes the rotation of theta radians axis a."
                " The angles are guaranteed to be in the ranges: r in [-pi,pi],"
                " p in[-pi/2,pi/2], y in [-pi,pi]");

        bp::def("rotate",
                &rotate,
                bp::args("axis", "ang"),
                "Rotation matrix corresponding to a rotation about x, y or z"
                " e.g. R = rot('x', pi / 4): rotate pi/4 rad about x axis");
      }
      
    }
    
  } // namespace python
} // namespace pinocchio
