// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/utils/comparable.hpp"
#include "pinocchio/bindings/python-nb/utils/printable.hpp"
#include "pinocchio/bindings/python-nb/utils/boost-variant.hpp"
#include "pinocchio/bindings/python-nb/constraints/constraint-crtp-base.hpp"

#include <nanobind/stl/string.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

// constraints/cones.cpp
void exposeCones(nb::module_ m);
// constraints/constraint-collection.cpp
void exposeConstraintCollection(nb::module_ m);

static void exposeBaumgarteParameters(nb::module_ m)
{
  nb::class_<BaumgarteCorrectorParameters>(
    m, "BaumgarteCorrectorParameters", "Parameters of the Baumgarte corrector.")
    .def(nb::init<>())
    .def(nb::init<Scalar, Scalar>(), "Kp"_a, "Kd"_a)
    .def_rw("Kp", &BaumgarteCorrectorParameters::Kp, "Proportional corrector gain value.")
    .def_rw("Kd", &BaumgarteCorrectorParameters::Kd, "Damping corrector gain value.")
    .def(ComparableVisitor<BaumgarteCorrectorParameters>());
}

static void exposeConstraintGeneric(nb::module_ m)
{
  nb::class_<ConstraintModel>(m, "ConstraintModel", "Generic constraint model.")
    .def(nb::init<>(), "Default constructor.")
    .def(nb::init<const ConstraintModel &>(), "other"_a, "Copy constructor.")
    .def(nb::init_implicit<const ConstraintModelVariant &>(), "constraint_model"_a)
    .def(ConstraintModelBaseVisitor<ConstraintModel>())
    .def("extract", [](ConstraintModel & self) -> ConstraintModelVariant & { return self; })
    .def(PrintableVisitor<ConstraintModel>());

  nb::bind_vector<std::vector<ConstraintModel>, nb::rv_policy::reference_internal>(
    m, "StdVec_ConstraintModel");

  nb::class_<ConstraintData>(m, "ConstraintData", "Generic constraint data.")
    .def(nb::init<>(), "Default constructor.")
    .def(nb::init<const ConstraintData &>(), "other"_a, "Copy constructor.")
    .def(nb::init_implicit<const ConstraintDataVariant &>(), "constraint_data"_a)
    .def(ConstraintDataBaseVisitor<ConstraintData>())
    .def("extract", [](ConstraintData & self) -> ConstraintDataVariant & { return self; })
    .def(PrintableVisitor<ConstraintData>());

  nb::bind_vector<std::vector<ConstraintData>, nb::rv_policy::reference_internal>(
    m, "StdVec_ConstraintData");
}

void exposeConstraints(nb::module_ m)
{
  nb::enum_<pinocchio::ConstraintSelectionType>(m, "ConstraintSelectionType")
    .value("CURRENT", pinocchio::ConstraintSelectionType::CURRENT)
    .value("MAXIMAL", pinocchio::ConstraintSelectionType::MAXIMAL);

  exposeBaumgarteParameters(m);

  // Expose constraint sets
  exposeCones(m);

  // Expose constraint collection
  exposeConstraintCollection(m);

  // Expose the variant wrapper (generic) types
  exposeConstraintGeneric(m);
}
PINOCCHIO_PYTHON_NAMESPACE_END
