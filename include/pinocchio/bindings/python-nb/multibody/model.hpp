// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/multibody.hpp"

#include "../fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
template<class Model>
void exposeModel(nb::module_ m)
{
  using Scalar = typename Model::Scalar;
  using Index = typename Model::Index;
  using SE3 = typename Model::SE3;

  nb::class_<Model>(m, "Model", "Articulated rigid-body model.")
    .def(nb::init<>(), "Default constructor")
    .def(nb::init<const Model &>())
    .def_ro("nq", &Model::nq, "Size of the configuration vector representation.")
    .def_ro("nv", &Model::nv, "Dimension of the velocity vector space.")
    .def_ro("nvExtended", &Model::nvExtended, "Dimension of the Jacobian matrix input space.")
    .def_ro("njoints", &Model::njoints, "Number of joints.")
    .def_ro("nbodies", &Model::nbodies, "Number of bodies.")
    .def_ro("nframes", &Model::nframes, "Number of frames.");
}
PINOCCHIO_PYTHON_NAMESPACE_END
