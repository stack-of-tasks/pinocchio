// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/comparable.hpp"
#include "pinocchio/bindings/python-nb/constraints/constraint-crtp-base.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

// constraints/cones.cpp
void exposeCones(nb::module_ m);
// constraints/constraint-solvers.cpp
void exposeConstraintSolvers(nb::module_ m);

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

void exposeConstraints(nb::module_ m)
{
  exposeBaumgarteParameters(m);

  using model_types = ConstraintModelVariant::types;

  using data_types = ConstraintDataVariant::types;
}
PINOCCHIO_PYTHON_NAMESPACE_END
