// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/bind_vector.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
template<class JointModel>
void exposeJointModel(nb::module_ m)
{
  nb::class_<JointModel>(m, "JointModel", "Generic Joint Model")
    .def(nb::init<>(), "Default constructor")
    .def(nb::init<const JointModel &>(), nb::arg("other"), "Copy constructor.");

  nb::bind_vector<std::vector<JointModel>>(m, "JointModelStdVec");
}
PINOCCHIO_PYTHON_NAMESPACE_END
