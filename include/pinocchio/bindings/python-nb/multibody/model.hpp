// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/multibody.hpp"
#include "pinocchio/algorithm/check-data.hpp"

#include "../fwd.hpp"

#include <nanobind/stl/string.h>
#include <nanobind/stl/bind_vector.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
template<class Model>
void exposeModel(nb::module_ m)
{
  using Scalar = typename Model::Scalar;
  using Index = typename Model::Index;
  using IndexVector = typename Model::IndexVector;
  using SE3 = typename Model::SE3;
  using Data = typename Model::Data;

  nb::class_<Model>(m, "Model", "Articulated rigid-body model.")
    .def(nb::init<>(), "Default constructor")
    .def(nb::init<const Model &>(), nb::arg("other"))
    .def_ro("nq", &Model::nq, "Size of the configuration vector representation.")
    .def_ro("nv", &Model::nv, "Dimension of the velocity vector space.")
    .def_ro("nvExtended", &Model::nvExtended, "Dimension of the Jacobian matrix input space.")
    .def_ro("njoints", &Model::njoints, "Number of joints.")
    .def_ro("nbodies", &Model::nbodies, "Number of bodies.")
    .def_ro("nframes", &Model::nframes, "Number of frames.")
    .def_ro("inertias", &Model::inertias, "Vector of spatial inertias supported by each joint.")
    .def_rw(
      "jointPlacements", &Model::jointPlacements,
      "Vector of joint placements: placement of a joint i with respect to its parent joint frame.")
    .def_ro("joints", &Model::joints)
    //
    .def("createData", &Model::createData, "Create a Data object for the given model.")
    .def(
      "check", [](const Model & self, const Data & data) { return self.check(data); },
      nb::arg("data"), "Check consistency of data wrt the model.")
    .def(
      "hasConfigurationLimit", &Model::hasConfigurationLimit,
      "Returns list of booleans of whether joints have a configuration limit.")
    //
    .def_ro("children", &Model::children)
    .def_rw("name", &Model::name)
    .def_rw("names", &Model::names)
    .def_rw("armature", &Model::armature);

  nb::bind_vector<std::vector<Index>>(m, "IndexStdVec");
  nb::bind_vector<std::vector<IndexVector>>(m, "IndexVecVec");
  nb::bind_vector<std::vector<std::string>>(m, "StringStdVec");
}
PINOCCHIO_PYTHON_NAMESPACE_END
