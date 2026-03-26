// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/spatial.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

using Vector3 = Eigen::Matrix<Scalar, 3, 1, Options>;
using Vector4 = Eigen::Matrix<Scalar, 4, 1, Options>;
using Vector6 = Eigen::Matrix<Scalar, 6, 1, Options>;
using Vector7 = Eigen::Matrix<Scalar, 7, 1, Options>;
using Matrix3 = Eigen::Matrix<Scalar, 3, 3, Options>;
using Matrix4 = Eigen::Matrix<Scalar, 4, 4, Options>;
using Matrix6 = Eigen::Matrix<Scalar, 6, 6, Options>;
using Quaternion = Eigen::Quaternion<Scalar, Options>;
using ConstQuaternionMap = Eigen::Map<const Quaternion>;

void exposeExplog(nb::module_ m)
{
  // --- SO3 exponential map ---

  m.def(
    "exp3", [](const Vector3 & v) -> Matrix3 { return pinocchio::exp3(v); }, "w"_a,
    "Exp: so3 -> SO3. Return the integral of the input"
    " vector w during time 1. This is also known as the Rodrigues formula.");

  m.def(
    "exp3_quat",
    [](const Vector3 & v) -> Vector4 { return pinocchio::quaternion::exp3(v).coeffs(); }, "w"_a,
    "Exp: so3 -> S3. Returns the integral of the input vector w during time 1, represented "
    "as a unit Quaternion.");

  m.def(
    "Jexp3",
    [](const Vector3 & v) -> Matrix3 {
      Matrix3 res;
      pinocchio::Jexp3(v, res);
      return res;
    },
    "w"_a,
    "Jacobian of exp(v) which maps from the tangent of SO(3) at R = exp(v) to"
    " the tangent of SO(3) at Identity.");

  // --- SO3 logarithmic map ---

  m.def(
    "log3", [](const Matrix3 & R) -> Vector3 { return pinocchio::log3(R); }, "R"_a,
    "Log: SO3 -> so3 is the pseudo-inverse of Exp: so3 -> SO3. Log maps from SO3"
    " -> { v in so3, ||v|| < 2pi }.");

  m.def(
    "log3",
    [](const Vector4 & quat_vec) -> Vector3 {
      ConstQuaternionMap q(quat_vec.data());
      return pinocchio::quaternion::log3(q);
    },
    "quat"_a,
    "Log: S^3 -> so3 is the pseudo-inverse of Exp: so3 -> S^3, the exponential map from so3 to "
    "the unit quaternions. It maps from S^3 -> { v in so3, ||v|| < 2pi }.");

  m.def(
    "Jlog3",
    [](const Matrix3 & R) -> Matrix3 {
      Matrix3 res;
      pinocchio::Jlog3(R, res);
      return res;
    },
    "R"_a,
    "Jacobian of log(R) which maps from the tangent of SO(3) at R to"
    " the tangent of SO(3) at Identity.");

  m.def(
    "Hlog3",
    [](const Matrix3 & R, const Vector3 & v) -> Matrix3 {
      Matrix3 res;
      pinocchio::Hlog3(R, v, res);
      return res;
    },
    "R"_a, "v"_a, "v^T * H where H is the Hessian of log(R)");

  // --- SE3 exponential map ---

  m.def(
    "exp6", [](const pinocchio::Motion & v) -> pinocchio::SE3 { return pinocchio::exp6(v); },
    "motion"_a,
    "Exp: se3 -> SE3. Return the integral of the input"
    " spatial velocity during time 1.");

  m.def(
    "exp6", [](const Vector6 & v) -> pinocchio::SE3 { return pinocchio::exp6(v); }, "v"_a,
    "Exp: se3 -> SE3. Return the integral of the input"
    " spatial velocity during time 1.");

  m.def(
    "exp6_quat", [](const Vector6 & v) -> Vector7 { return pinocchio::quaternion::exp6(v); }, "v"_a,
    "Exp: se3 -> R3 * S3. Return the integral of the input 6D spatial velocity over unit time,"
    " using quaternion to represent rotation as in the standard configuration layout"
    " for the Lie group SE3.");

  m.def(
    "Jexp6",
    [](const pinocchio::Motion & v) -> Matrix6 {
      Matrix6 res;
      pinocchio::Jexp6(v, res);
      return res;
    },
    "motion"_a,
    "Jacobian of exp(v) which maps from the tangent of SE(3) at exp(v) to"
    " the tangent of SE(3) at Identity.");

  m.def(
    "Jexp6",
    [](const Vector6 & vec6) -> Matrix6 {
      pinocchio::MotionRef<const Vector6> v(vec6);
      Matrix6 res;
      pinocchio::Jexp6(v, res);
      return res;
    },
    "v"_a,
    "Jacobian of exp(v) which maps from the tangent of SE(3) at exp(v) to"
    " the tangent of SE(3) at Identity.");

  // --- SE3 logarithmic map ---

  m.def(
    "log6", [](const pinocchio::SE3 & M) -> pinocchio::Motion { return pinocchio::log6(M); }, "M"_a,
    "Log: SE3 -> se3. Pseudo-inverse of exp from SE3"
    " -> { v,w in se3, ||w|| < 2pi }.");

  m.def(
    "log6", [](const Matrix4 & H) -> pinocchio::Motion { return pinocchio::log6(H); },
    "homegeneous_matrix"_a,
    "Log: SE3 -> se3. Pseudo-inverse of Exp: so3 -> SO3. Log maps from SE3"
    " -> { v,w in se3, ||w|| < 2pi }.");

  m.def(
    "log6_quat",
    [](const Vector7 & q) -> pinocchio::Motion {
      ConstQuaternionMap quat(q.template tail<4>().data());
      const Vector3 v(q.template head<3>());
      return pinocchio::log6(quat, v);
    },
    "q"_a,
    "Log: R^3 * S^3 -> se3. Pseudo-inverse of Exp: se3 -> R^3 * S^3,"
    " the variant of the SE3 Exp using quaternions for the rotations.");

  m.def(
    "Jlog6", [](const pinocchio::SE3 & M) -> Matrix6 { return pinocchio::Jlog6(M); }, "M"_a,
    "Jacobian of log(M) which maps from the tangent of SE(3) at M to"
    " the tangent of SE(3) at Identity.");
}
PINOCCHIO_PYTHON_NAMESPACE_END
