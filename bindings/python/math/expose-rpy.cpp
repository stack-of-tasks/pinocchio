//
// Copyright (c) 2019-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"
#include "pinocchio/math/rpy.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    context::Matrix3s rotate(const std::string & axis, const context::Scalar ang)
    {
      typedef context::Scalar Scalar;
      if (axis.length() != 1U)
        throw std::invalid_argument(std::string("Invalid axis: ").append(axis));
      context::Vector3s u;
      u.setZero();
      const char axis_ = axis[0];
      switch (axis_)
      {
      case 'x':
        u[0] = Scalar(1);
        break;
      case 'y':
        u[1] = Scalar(1);
        break;
      case 'z':
        u[2] = Scalar(1);
        break;
      default:
        throw std::invalid_argument(std::string("Invalid axis: ").append(1U, axis_));
      }

      return context::AngleAxis(ang, u).matrix();
    }

    void exposeRpy()
    {
      using namespace Eigen;
      using namespace pinocchio::rpy;

      {
        // using the rpy scope
        bp::scope current_scope = getOrCreatePythonNamespace("rpy");

        bp::def(
          "rpyToMatrix",
          static_cast<context::Matrix3s (*)(
            const context::Scalar &, const context::Scalar &, const context::Scalar &)>(
            &rpyToMatrix),
          bp::args("roll", "pitch", "yaw"),
          "Given (r, p, y), the rotation is given as R = R_z(y)R_y(p)R_x(r),"
          " where R_a(theta) denotes the rotation of theta radians axis a");

        bp::def(
          "rpyToMatrix",
          static_cast<context::Matrix3s (*)(const MatrixBase<context::Vector3s> &)>(&rpyToMatrix),
          bp::arg("rpy"),
          "Given (r, p, y), the rotation is given as R = R_z(y)R_y(p)R_x(r),"
          " where R_a(theta) denotes the rotation of theta radians axis a");

        bp::def(
          "matrixToRpy",
          //                &matrixToRpy<context::Matrix3s>, TODO: change internal routines to
          //                make them compliant with Autodiff Frameworks
          &matrixToRpy<Eigen::Matrix3d>, bp::arg("R"),
          "Given a rotation matrix R, the angles (r, p, y) are given so that R = "
          "R_z(y)R_y(p)R_x(r),"
          " where R_a(theta) denotes the rotation of theta radians axis a."
          " The angles are guaranteed to be in the ranges: r in [-pi,pi],"
          " p in[-pi/2,pi/2], y in [-pi,pi]");

        bp::def(
          "rotate", &rotate, bp::args("axis", "angle"),
          "Rotation matrix corresponding to a rotation about x, y or z"
          " e.g. R = rot('x', pi / 4): rotate pi/4 rad about x axis");

        bp::def(
          "computeRpyJacobian", &computeRpyJacobian<context::Vector3s>,
          (bp::arg("rpy"), bp::arg("reference_frame") = LOCAL),
          "Compute the Jacobian of the Roll-Pitch-Yaw conversion"
          " Given phi = (r, p, y) such that that R = R_z(y)R_y(p)R_x(r)"
          " and reference frame F (either LOCAL or WORLD),"
          " the Jacobian is such that omega_F = J_F(phi)phidot,"
          " where omega_F is the angular velocity expressed in frame F"
          " and J_F is the Jacobian computed with reference frame F"
          "\nParameters:\n"
          "\trpy Roll-Pitch-Yaw vector"
          "\treference_frame  Reference frame in which the angular velocity is expressed."
          " Notice LOCAL_WORLD_ALIGNED is equivalent to WORLD");

        bp::def(
          "computeRpyJacobianInverse", &computeRpyJacobianInverse<context::Vector3s>,
          (bp::arg("rpy"), bp::arg("reference_frame") = LOCAL),
          "Compute the inverse Jacobian of the Roll-Pitch-Yaw conversion"
          " Given phi = (r, p, y) such that that R = R_z(y)R_y(p)R_x(r)"
          " and reference frame F (either LOCAL or WORLD),"
          " the Jacobian is such that omega_F = J_F(phi)phidot,"
          " where omega_F is the angular velocity expressed in frame F"
          " and J_F is the Jacobian computed with reference frame F"
          "\nParameters:\n"
          "\trpy Roll-Pitch-Yaw vector"
          "\treference_frame  Reference frame in which the angular velocity is expressed."
          " Notice LOCAL_WORLD_ALIGNED is equivalent to WORLD");

        bp::def(
          "computeRpyJacobianTimeDerivative",
          &computeRpyJacobianTimeDerivative<context::Vector3s, context::Vector3s>,
          (bp::arg("rpy"), bp::arg("rpydot"), bp::arg("reference_frame") = LOCAL),
          "Compute the time derivative of the Jacobian of the Roll-Pitch-Yaw conversion"
          " Given phi = (r, p, y) such that that R = R_z(y)R_y(p)R_x(r)"
          " and reference frame F (either LOCAL or WORLD),"
          " the Jacobian is such that omega_F = J_F(phi)phidot,"
          " where omega_F is the angular velocity expressed in frame F"
          " and J_F is the Jacobian computed with reference frame F"
          "\nParameters:\n"
          "\trpy Roll-Pitch-Yaw vector"
          "\treference_frame  Reference frame in which the angular velocity is expressed."
          " Notice LOCAL_WORLD_ALIGNED is equivalent to WORLD");
      }
    }

  } // namespace python
} // namespace pinocchio
