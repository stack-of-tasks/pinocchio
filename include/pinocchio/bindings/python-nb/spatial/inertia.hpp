// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "../utils/comparable.hpp"
#include "../utils/copyable.hpp"
#include "../utils/printable.hpp"

#include "pinocchio/spatial.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/bind_vector.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
template<class Inertia>
void exposeInertia(nb::module_ m)
{
  using namespace nb::literals;
  using Self = Inertia;
  using Scalar = typename Self::Scalar;
  using Vector3 = typename Self::Vector3;
  using Matrix3 = typename Self::Matrix3;
  using Matrix6 = typename Self::Matrix6;
  using VectorXs = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  using Motion = MotionTpl<Scalar, Self::Options>;
  using Force = ForceTpl<Scalar, Self::Options>;
  using SE3 = SE3Tpl<Scalar, Self::Options>;

  static const Scalar dummy_precision = Eigen::NumTraits<Scalar>::dummy_precision();

  nb::class_<Inertia>(
    m, "Inertia",
    "Sparse representation of a Spatial Inertia, defined by its mass, center of mass location, "
    "and rotational inertia expressed around this center of mass.")
    // Constructors
    .def(nb::init<>(), "Default constructor.")
    .def(nb::init<const Inertia &>(), "other"_a, "Copy constructor.")
    .def(
      "__init__",
      [](Inertia * self, const Scalar & mass, const Vector3 & lever, const Matrix3 & inertia) {
        if (!inertia.isApprox(inertia.transpose()))
          throw std::invalid_argument("The 3d inertia should be symmetric.");
        if (
          (Vector3::UnitX().transpose() * inertia * Vector3::UnitX() < 0)
          || (Vector3::UnitY().transpose() * inertia * Vector3::UnitY() < 0)
          || (Vector3::UnitZ().transpose() * inertia * Vector3::UnitZ() < 0))
          throw std::invalid_argument("The 3d inertia should be positive.");
        new (self) Inertia(mass, lever, inertia);
      },
      "mass"_a, "lever"_a, "inertia"_a, "Initialize from mass, lever and 3d inertia.")
    // Properties
    .def_prop_rw(
      "mass", [](const Self & self) { return self.mass(); },
      [](Self & self, Scalar mass) { self.mass() = mass; }, "Mass of the Spatial Inertia.")
    .def_prop_rw(
      "lever", [](Self & self) { return make_ref(self.lever()); },
      [](Self & self, const Vector3 & lever) { self.lever() = lever; },
      "Center of mass location of the Spatial Inertia. It corresponds to the location of the "
      "center of mass with respect to the frame where the Spatial Inertia is expressed.")
    // TODO: surely should return a Symmetric3 object no?
    .def_prop_rw(
      "inertia", [](const Self & self) { return Matrix3(self.inertia().matrix()); },
      [](Self & self, const Matrix3 & symmetric_inertia) {
        if (!symmetric_inertia.isApprox(symmetric_inertia.transpose()))
          throw std::invalid_argument("The 3d inertia should be symmetric.");
        self.inertia().data() << symmetric_inertia(0, 0), symmetric_inertia(1, 0),
          symmetric_inertia(1, 1), symmetric_inertia(0, 2), symmetric_inertia(1, 2),
          symmetric_inertia(2, 2);
      },
      "Rotational part of the Spatial Inertia, i.e. a symmetric matrix representing the "
      "rotational inertia around the center of mass.")
    // 6x6 matrix representations
    .def(
      "matrix", (Matrix6 (Inertia::*)() const) & Inertia::matrix, "Returns the 6x6 inertia matrix.")
    .def(
      "inverse", (Matrix6 (Inertia::*)() const) & Inertia::inverse,
      "Returns the inverse of the 6x6 inertia matrix.")
    .def_prop_ro(
      "np", (Matrix6 (Inertia::*)() const) & Inertia::matrix,
      "Returns the 6x6 inertia matrix (alias for matrix()).")
    // SE3 action
    .def(
      "se3Action", [](const Self & self, const SE3 & M) { return self.se3Action(M); }, "M"_a,
      "Returns the result of the action of M on this Inertia object.")
    .def(
      "se3ActionInverse", [](const Self & self, const SE3 & M) { return self.se3ActionInverse(M); },
      "M"_a, "Returns the result of the action of the inverse of M on this Inertia object.")
    // Mutating methods
    .def("setIdentity", &Inertia::setIdentity, "Set this Inertia object to the identity inertia.")
    .def("setZero", &Inertia::setZero, "Set all the components of this Inertia object to zero.")
    .def(
      "setRandom", &Inertia::setRandom,
      "Set all the components of this Inertia object to random values.")
    // Arithmetic operators
    .def(
      "__add__", [](const Inertia & a, const Inertia & b) { return Inertia(a + b); },
      nb::is_operator())
    .def(
      "__iadd__", [](Inertia & a, const Inertia & b) { return Inertia(a += b); }, nb::is_operator())
    .def(
      "__sub__", [](const Inertia & a, const Inertia & b) { return Inertia(a - b); },
      nb::is_operator())
    .def(
      "__isub__", [](Inertia & a, const Inertia & b) { return Inertia(a -= b); }, nb::is_operator())
    .def(
      "__mul__", [](const Inertia & I, const Motion & v) { return Force(I * v); },
      nb::is_operator())
    // Velocity-related methods
    .def(
      "vxiv", [](const Self & self, const Motion & v) { return self.template vxiv<Motion>(v); },
      "v"_a, "Returns the result of v x Iv.")
    .def(
      "vtiv", [](const Self & self, const Motion & v) { return self.template vtiv<Motion>(v); },
      "v"_a, "Returns the result of v.T * Iv.")
    .def(
      "vxi", [](const Self & self, const Motion & v) { return self.template vxi<Motion>(v); },
      "v"_a, "Returns the result of v x* I, a 6x6 matrix.")
    .def(
      "ivx", [](const Self & self, const Motion & v) { return self.template ivx<Motion>(v); },
      "v"_a, "Returns the result of I vx, a 6x6 matrix.")
    .def(
      "variation",
      [](const Self & self, const Motion & v) { return self.template variation<Motion>(v); }, "v"_a,
      "Returns the time derivative of the inertia.")
    // Comparison
    .def(
      "isApprox",
      [](const Inertia & self, const Inertia & other, Scalar prec) {
        return self.isApprox(other, prec);
      },
      "other"_a, nb::arg("prec") = dummy_precision,
      "Returns true if this Inertia object is approximately equal to other, within the precision "
      "given by prec.")
    .def(
      "isZero", [](const Inertia & self, Scalar prec) { return self.isZero(prec); },
      nb::arg("prec") = dummy_precision,
      "Returns true if this Inertia object is approximately equal to the zero Inertia, within "
      "the precision given by prec.")
    .def(ComparableVisitor<Inertia>())
    .def(CopyableVisitor<Inertia>())
    // Static factory methods
    .def_static("Identity", &Inertia::Identity, "Returns the identity Inertia.")
    .def_static("Zero", &Inertia::Zero, "Returns the zero Inertia.")
    .def_static("Random", &Inertia::Random, "Returns a random Inertia.")
    // Dynamic parameters
    .def(
      "toDynamicParameters",
      [](const Self & self) -> VectorXs { return self.toDynamicParameters(); },
      "Returns the representation of this Inertia object as a vector of dynamic parameters.\n"
      "The parameters are given as v = [m, mc_x, mc_y, mc_z, I_{xx}, I_{xy}, I_{yy}, "
      "I_{xz}, I_{yz}, I_{zz}]^T "
      "where I = I_C + mS^T(c)S(c) and I_C has its origin at the barycenter.")
    .def_static(
      "FromDynamicParameters",
      [](const VectorXs & params) {
        if (params.rows() != 10 || params.cols() != 1)
          throw std::invalid_argument(
            "Wrong size: expected shape (10, 1), got (" + std::to_string(params.rows()) + ", "
            + std::to_string(params.cols()) + ")");
        return Inertia::FromDynamicParameters(params);
      },
      "dynamic_parameters"_a,
      "Builds an Inertia from a vector of dynamic parameters.\n"
      "The parameters are given as dynamic_parameters = [m, mc_x, mc_y, mc_z, I_{xx}, "
      "I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T "
      "where I = I_C + mS^T(c)S(c) and I_C has its origin at the barycenter.")
    // Shape factories
    .def_static(
      "FromSphere", &Inertia::FromSphere, "mass"_a, "radius"_a,
      "Returns the Inertia of a sphere defined by a given mass and radius.")
    .def_static(
      "FromEllipsoid", &Inertia::FromEllipsoid, "mass"_a, "length_x"_a, "length_y"_a, "length_z"_a,
      "Returns the Inertia of an ellipsoid defined by its mass and semi-axis lengths "
      "length_{x,y,z}.")
    .def_static(
      "FromCylinder", &Inertia::FromCylinder, "mass"_a, "radius"_a, "length"_a,
      "Returns the Inertia of a cylinder defined by its mass, radius and length along the Z axis.")
    .def_static(
      "FromBox", &Inertia::FromBox, "mass"_a, "length_x"_a, "length_y"_a, "length_z"_a,
      "Returns the Inertia of a box defined by its mass and semi-axis dimensions length_{x,y,z}.")
    .def_static(
      "FromCapsule", &Inertia::FromCapsule, "mass"_a, "radius"_a, "height"_a,
      "Returns the Inertia of a capsule defined by its mass, radius and length along the Z axis. "
      "Assumes a uniform density.")
    // Pseudo inertia
    .def(
      "toPseudoInertia", &Inertia::toPseudoInertia,
      "Returns the pseudo inertia representation of this Inertia.")
    .def_static(
      "FromPseudoInertia", &Inertia::FromPseudoInertia, "pseudo_inertia"_a,
      "Returns the Inertia created from a pseudo inertia object.")
    .def_static(
      "FromLogCholeskyParameters", &Inertia::FromLogCholeskyParameters, "log_cholesky_parameters"_a,
      "Returns the Inertia created from log Cholesky parameters.")
    // Array interface
    .def(
      "__array__", [](const Self & self, nb::object, nb::bool_) { return Matrix6(self.matrix()); },
      "dtype"_a = nb::none(), nb::kw_only(), "copy"_a = nb::none())
    // String representation
    .def(PrintableVisitor<Inertia>());

  nb::bind_vector<std::vector<Inertia>, nb::rv_policy::reference_internal>(m, "StdVec_Inertia");
}

template<class PseudoInertia>
void exposePseudoInertia(nb::module_ m)
{
  using namespace nb::literals;
  using Self = PseudoInertia;
  using Scalar = typename Self::Scalar;
  using Vector3 = typename Self::Vector3;
  using Matrix3 = typename Self::Matrix3;
  using Matrix4 = typename Self::Matrix4;
  using VectorXs = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  nb::class_<PseudoInertia>(
    m, "PseudoInertia",
    "Pseudo inertia matrix defined by its mass, vector part h, and 3x3 matrix part sigma.")
    .def(
      nb::init<const Scalar &, const Vector3 &, const Matrix3 &>(), "mass"_a, "h"_a, "sigma"_a,
      "Initialize from mass, vector part and matrix part of the pseudo inertia.")
    .def(nb::init<const PseudoInertia &>(), "other"_a, "Copy constructor.")
    .def_prop_rw(
      "mass", [](const Self & self) { return self.mass; },
      [](Self & self, Scalar mass) { self.mass = mass; }, "Mass of the PseudoInertia.")
    .def_prop_rw(
      "h", [](Self & self) { return make_ref(self.h); },
      [](Self & self, const Vector3 & h) { self.h = h; }, "Vector part of the PseudoInertia.")
    .def_prop_rw(
      "sigma", [](Self & self) { return make_ref(self.sigma); },
      [](Self & self, const Matrix3 & sigma) { self.sigma = sigma; },
      "3x3 matrix part of the PseudoInertia.")
    .def("toMatrix", &PseudoInertia::toMatrix, "Returns the pseudo inertia as a 4x4 matrix.")
    .def(
      "toDynamicParameters", &PseudoInertia::toDynamicParameters,
      "Returns the dynamic parameters representation.")
    .def("toInertia", &PseudoInertia::toInertia, "Returns the Inertia representation.")
    .def_static(
      "FromDynamicParameters",
      [](const VectorXs & params) {
        if (params.rows() != 10 || params.cols() != 1)
          throw std::invalid_argument(
            "Wrong size: expected shape (10, 1), got (" + std::to_string(params.rows()) + ", "
            + std::to_string(params.cols()) + ")");
        return PseudoInertia::FromDynamicParameters(params);
      },
      "dynamic_parameters"_a,
      "Builds a PseudoInertia from a vector of dynamic parameters.\n"
      "The parameters are given as dynamic_parameters = [m, h_x, h_y, h_z, I_{xx}, "
      "I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T.")
    .def_static(
      "FromMatrix", &PseudoInertia::FromMatrix, "pseudo_inertia_matrix"_a,
      "Returns the PseudoInertia from a 4x4 pseudo inertia matrix.")
    .def_static(
      "FromInertia", &PseudoInertia::FromInertia, "inertia"_a,
      "Returns the PseudoInertia from an Inertia object.")
    .def(
      "__array__", [](const Self & self, nb::object, nb::bool_) { return self.toMatrix(); },
      "dtype"_a = nb::none(), nb::kw_only(), "copy"_a = nb::none())
    .def(CopyableVisitor<PseudoInertia>())
    .def(PrintableVisitor<PseudoInertia>());
}

template<class LogCholeskyParameters>
void exposeLogCholeskyParameters(nb::module_ m)
{
  using namespace nb::literals;
  using Self = LogCholeskyParameters;
  using Scalar = typename Self::Scalar;
  using Vector10 = typename Self::Vector10;
  using VectorXs = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  nb::class_<LogCholeskyParameters>(
    m, "LogCholeskyParameters",
    "Log Cholesky parameterization of a rigid-body inertia.\n\n"
    "Reference: Rucker, Caleb, and Patrick M. Wensing. \"Smooth parameterization of rigid-body "
    "inertia.\" IEEE Robotics and Automation Letters 7.2 (2022): 2771-2778.")
    .def(
      "__init__",
      [](LogCholeskyParameters * self, const VectorXs & params) {
        if (params.rows() != 10 || params.cols() != 1)
          throw std::invalid_argument(
            "Wrong size: expected shape (10, 1), got (" + std::to_string(params.rows()) + ", "
            + std::to_string(params.cols()) + ")");
        new (self) LogCholeskyParameters(params);
      },
      "log_cholesky_parameters"_a,
      "Initialize from a 10-dimensional vector of log Cholesky parameters.")
    .def(nb::init<const LogCholeskyParameters &>(), "other"_a, "Copy constructor.")
    .def_prop_rw(
      "parameters", [](Self & self) { return make_ref(self.parameters); },
      [](Self & self, const Vector10 & params) { self.parameters = params; },
      "Log Cholesky parameters as a 10-dimensional vector.")
    .def(
      "toDynamicParameters", &LogCholeskyParameters::toDynamicParameters,
      "Returns the dynamic parameters representation.")
    .def(
      "toPseudoInertia", &LogCholeskyParameters::toPseudoInertia,
      "Returns the PseudoInertia representation.")
    .def("toInertia", &LogCholeskyParameters::toInertia, "Returns the Inertia representation.")
    .def(
      "calculateJacobian", &LogCholeskyParameters::calculateJacobian,
      "Calculates the Jacobian of the log Cholesky parameters.")
    .def(
      "__array__",
      [](const Self & self, nb::object, nb::bool_ copy) {
        if (copy)
          return nb::cast(self.parameters);
        else
          return nb::cast(make_ref(self.parameters));
      },
      "dtype"_a = nb::none(), nb::kw_only(), "copy"_a = nb::none())
    .def(CopyableVisitor<LogCholeskyParameters>())
    .def(PrintableVisitor<LogCholeskyParameters>());
}
PINOCCHIO_PYTHON_NAMESPACE_END
