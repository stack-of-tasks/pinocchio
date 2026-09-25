// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/model-checker.hpp"

#include "pinocchio/algorithm/aba.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeABA(nb::module_ m)
{

  m.def(
    "computeMinverse",
    [](const Model & model, Data & data, ConstVectorRef q) -> const Data::RowMatrixXs & {
      computeMinverse(model, data, q);
      make_symmetric(data.Minv);
      return data.Minv;
    },
    "model"_a, "data"_a, "q"_a, nb::call_policy<mimic_not_supported_policy<0>>(),
    "Computes the inverse of the joint space inertia matrix using an extension of the "
    "Articulated Body algorithm.\n"
    "The result is stored in data.Minv.\n"
    "Parameters:\n"
    "\t model: Model of the kinematic tree\n"
    "\t data: Data related to the kinematic tree\n"
    "\t q: joint configuration (size model.nq)");

  m.def(
    "computeMinverse",
    [](const Model & model, Data & data) -> const Data::RowMatrixXs & {
      computeMinverse(model, data);
      make_symmetric(data.Minv);
      return data.Minv;
    },
    "model"_a, "data"_a, nb::call_policy<mimic_not_supported_policy<0>>(),
    "Computes the inverse of the joint space inertia matrix using an extension of the "
    "Articulated Body algorithm.\n"
    "The result is stored in data.Minv.\n"
    "Remarks: pinocchio.aba should have been called first.\n"
    "Parameters:\n"
    "\t model: Model of the kinematic tree\n"
    "\t data: Data related to the kinematic tree");

  m.def(
    "aba",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef tau,
      Convention convention) -> const Data::TangentVectorType & {
      return aba(model, data, q, v, tau, convention);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "tau"_a, "convention"_a = Convention::LOCAL,
    nb::call_policy<mimic_not_supported_policy<0>>(),
    "Compute ABA, store the result in data.ddq and return it.\n"
    "Parameters:\n"
    "\t model: Model of the kinematic tree\n"
    "\t data: Data related to the kinematic tree\n"
    "\t q: joint configuration (size model.nq)\n"
    "\t tau: joint velocity (size model.nv)\n"
    "\t v: joint torque (size model.nv)\n"
    "\t convention: Convention to use");

  m.def(
    "aba",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef tau,
      const std::vector<Force> & fext, Convention convention) -> const Data::TangentVectorType & {
      return aba(model, data, q, v, tau, fext, convention);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "tau"_a, "fext"_a, "convention"_a = Convention::LOCAL,
    nb::call_policy<mimic_not_supported_policy<0>>(),
    "Compute ABA with external forces, store the result in data.ddq and return it.\n"
    "Parameters:\n"
    "\t model: Model of the kinematic tree\n"
    "\t data: Data related to the kinematic tree\n"
    "\t q: joint configuration (size model.nq)\n"
    "\t v: joint velocity (size model.nv)\n"
    "\t tau: joint torque (size model.nv)\n"
    "\t fext: vector of external forces expressed in the local frame of the joint (size "
    "model.njoints)"
    "\t convention: Convention to use");
}
PINOCCHIO_PYTHON_NAMESPACE_END;
