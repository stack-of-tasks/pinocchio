// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeCAT(nb::module_ m)
{
  m.def(
    "computeAllTerms",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v) {
      data.M.fill(Scalar(0));
      computeAllTerms(model, data, q, v);
      data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose().triangularView<Eigen::StrictlyLower>();
    },
    "model"_a, "data"_a, "q"_a, "v"_a,
    "Compute all the terms M, non linear effects, center of mass quantities, centroidal "
    "quantities and Jacobians in the same loop and store the results in data.\n"
    "This algorithm is equivalent to calling:\n"
    "\t- forwardKinematics\n"
    "\t- crba\n"
    "\t- nonLinearEffects\n"
    "\t- computeJointJacobians\n"
    "\t- centerOfMass\n"
    "\t- jacobianCenterOfMass\n"
    "\t- ccrba\n"
    "\t- computeKineticEnergy\n"
    "\t- computePotentialEnergy\n"
    "\t- computeGeneralizedGravity\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t data: data related to the model\n"
    "\t q: the joint configuration vector (size model.nq)\n"
    "\t v: the joint velocity vector (size model.nv)");
}
PINOCCHIO_PYTHON_NAMESPACE_END;
