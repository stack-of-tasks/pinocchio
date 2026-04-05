// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/constraints/constraint-crtp-base.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
void exposeConstraints(nb::module_ m)
{
  using model_types = ConstraintModelVariant::types;

  using data_types = ConstraintDataVariant::types;
}
PINOCCHIO_PYTHON_NAMESPACE_END
