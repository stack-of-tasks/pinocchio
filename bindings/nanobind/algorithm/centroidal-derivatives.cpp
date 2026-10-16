// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/centroidal-derivatives.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

using Matrix6x = Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options>;

void exposeCentroidalDerivatives(nb::module_ m)
{
  m.def(
    "computeCentroidalDynamicsDerivatives",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef a)
      -> nb::tuple {
      Matrix6x dh_dq = Matrix6x::Zero(6, model.nv);
      Matrix6x dhdot_dq = Matrix6x::Zero(6, model.nv);
      Matrix6x dhdot_dv = Matrix6x::Zero(6, model.nv);
      Matrix6x dhdot_da = Matrix6x::Zero(6, model.nv);
      pinocchio::computeCentroidalDynamicsDerivatives(
        model, data, q, v, a, dh_dq, dhdot_dq, dhdot_dv, dhdot_da);
      return nb::make_tuple(dh_dq, dhdot_dq, dhdot_dv, dhdot_da);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "a"_a,
    "Computes the analytical derivatives of the centroidal dynamics\n"
    "with respect to the joint configuration vector, velocity and acceleration.\n\n"
    "Returns: (dh_dq, dhdot_dq, dhdot_dv, dhdot_da)");

  m.def(
    "getCentroidalDynamicsDerivatives",
    [](const Model & model, Data & data) -> nb::tuple {
      Matrix6x dh_dq = Matrix6x::Zero(6, model.nv);
      Matrix6x dhdot_dq = Matrix6x::Zero(6, model.nv);
      Matrix6x dhdot_dv = Matrix6x::Zero(6, model.nv);
      Matrix6x dhdot_da = Matrix6x::Zero(6, model.nv);
      pinocchio::getCentroidalDynamicsDerivatives(model, data, dh_dq, dhdot_dq, dhdot_dv, dhdot_da);
      return nb::make_tuple(dh_dq, dhdot_dq, dhdot_dv, dhdot_da);
    },
    "model"_a, "data"_a,
    "Retrieve the analytical derivatives of the centroidal dynamics\n"
    "from the RNEA derivatives.\n"
    "pinocchio.computeRNEADerivatives should have been called first.\n\n"
    "Returns: (dh_dq, dhdot_dq, dhdot_dv, dhdot_da)");
}
PINOCCHIO_PYTHON_NAMESPACE_END
