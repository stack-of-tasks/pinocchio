// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "../utils/printable.hpp"
#include "../utils/boost-variant.hpp"

#include "pinocchio/multibody/joint.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/bind_vector.h>

#include <type_traits>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

/// Expose generic JointModel.
template<class JointModel>
void exposeJointModel(nb::module_ m)
{
  using JointModelVariant = typename JointModel::JointModelVariant;
  nb::class_<JointModel>(m, "JointModel", "Generic Joint Model")
    .def(nb::init<>(), "Default constructor")
    .def(nb::init<const JointModel &>(), nb::arg("other"), "Copy constructor.")
    .def(nb::init<const JointModelVariant &>(), nb::arg("joint_model"))
    .def("extract", [](JointModel & self) -> JointModelVariant & { return self; })
    .def(PrintableVisitor<JointModel>());

  nb::bind_vector<std::vector<JointModel>, nb::rv_policy::reference_internal>(
    m, "JointModelStdVec");
}

template<template<class> class BaseTpl, typename T>
static constexpr bool is_tpl_base_of_v = std::is_base_of_v<BaseTpl<T>, T>;

/// Expose generic JointData.
template<class JointData>
void exposeJointData(nb::module_ m)
{
  using namespace nb::literals;
  using JointDataVariant = typename JointData::JointDataVariant;
  nb::class_<JointData>(m, "JointData", "Generic Joint Data")
    .def(nb::init<const JointDataVariant &>(), "joint_data"_a)
    .def("extract", [](JointData & self) -> JointDataVariant & { return self; })
    .def(PrintableVisitor<JointData>());

  nb::bind_vector<std::vector<JointData>, nb::rv_policy::reference_internal>(m, "JointDataStdVec");
}
PINOCCHIO_PYTHON_NAMESPACE_END
