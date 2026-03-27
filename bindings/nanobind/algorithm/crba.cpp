// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/crba.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

void exposeCRBA(nb::module_ m)
{
  using namespace nb::literals;
  m.def(
    "crba",
    [](const Model & model, Data & data, const VectorXs & q, const Convention convention) {
      data.M.fill(Scalar(0));
      crba(model, data, q, convention);
      make_symmetric(data.M);
      return data.M;
    },
    "model"_a, "data"_a, "q"_a, "convention"_a = Convention::LOCAL,
    "Computes CRBA, store the result in Data and return it.\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)\n"
    "\t convention: Convention to use");
}

PINOCCHIO_PYTHON_NAMESPACE_END
