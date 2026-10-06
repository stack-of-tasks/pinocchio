// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/math.hpp"

#include <nanobind/eigen/dense.h>
#include <stdexcept>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

using Vector3 = Eigen::Matrix<Scalar, 3, 1, Options>;
using Matrix3 = Eigen::Matrix<Scalar, 3, 3, Options>;

static Matrix3 rotate(const char axis, const Scalar ang)
{
  Vector3 u;
  u.setZero();
  switch (axis)
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
    throw std::invalid_argument(std::string("Invalid axis: ") + axis);
  }
  return Eigen::AngleAxis<Scalar>(ang, u).matrix();
}

void exposeRpy(nb::module_ m_)
{
  using namespace pinocchio::rpy;

  nb::module_ m = m_.def_submodule(
    "rpy",
    "Submodule for utilities related to roll-pitch-yaw (rpy) spatial geometry representation.");

  m.def(
    "rpyToMatrix",
    [](const Scalar & r, const Scalar & p, const Scalar & y) -> Matrix3 {
      return rpyToMatrix(r, p, y);
    },
    "roll"_a, "pitch"_a, "yaw"_a,
    "Given (r, p, y), the rotation is given as R = R_z(y)R_y(p)R_x(r),"
    " where R_a(theta) denotes the rotation of theta radians axis a");

  m.def(
    "rpyToMatrix", [](const Vector3 & rpy) -> Matrix3 { return rpyToMatrix(rpy); }, "rpy"_a,
    "Given (r, p, y), the rotation is given as R = R_z(y)R_y(p)R_x(r),"
    " where R_a(theta) denotes the rotation of theta radians axis a");

  m.def(
    "matrixToRpy", [](const Matrix3 & R) -> Vector3 { return matrixToRpy(R); }, "R"_a,
    "Given a rotation matrix R, the angles (r, p, y) are given so that R = R_z(y)R_y(p)R_x(r),"
    " where R_a(theta) denotes the rotation of theta radians axis a."
    " The angles are guaranteed to be in the ranges: r in [-pi,pi],"
    " p in[-pi/2,pi/2], y in [-pi,pi]");

  m.def(
    "rotate", &rotate, "axis"_a, "angle"_a,
    "Rotation matrix corresponding to a rotation about x, y or z"
    " e.g. R = rot('x', pi / 4): rotate pi/4 rad about x axis");

  m.def(
    "computeRpyJacobian",
    [](const Vector3 & rpy, const pinocchio::ReferenceFrame rf) -> Matrix3 {
      return computeRpyJacobian(rpy, rf);
    },
    "rpy"_a, "reference_frame"_a = pinocchio::LOCAL,
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

  m.def(
    "computeRpyJacobianInverse",
    [](const Vector3 & rpy, const pinocchio::ReferenceFrame rf) -> Matrix3 {
      return computeRpyJacobianInverse(rpy, rf);
    },
    "rpy"_a, "reference_frame"_a = pinocchio::LOCAL,
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

  m.def(
    "computeRpyJacobianTimeDerivative",
    [](const Vector3 & rpy, const Vector3 & rpydot, const pinocchio::ReferenceFrame rf) -> Matrix3 {
      return computeRpyJacobianTimeDerivative(rpy, rpydot, rf);
    },
    "rpy"_a, "rpydot"_a, "reference_frame"_a = pinocchio::LOCAL,
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
PINOCCHIO_PYTHON_NAMESPACE_END
