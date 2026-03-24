#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/aba.hpp"

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
    "model"_a, "data"_a, "q"_a,
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
    "model"_a, "data"_a,
    "Computes the inverse of the joint space inertia matrix using an extension of the "
    "Articulated Body algorithm.\n"
    "The result is stored in data.Minv.\n"
    "Remarks: pinocchio.aba should have been called first.\n"
    "Parameters:\n"
    "\t model: Model of the kinematic tree\n"
    "\t data: Data related to the kinematic tree");

  // m.def("aba", &aba<);
}
PINOCCHIO_PYTHON_NAMESPACE_END;
