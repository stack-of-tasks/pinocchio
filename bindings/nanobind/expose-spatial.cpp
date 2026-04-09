// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/spatial/motion.hpp"
#include "pinocchio/bindings/python-nb/spatial/force.hpp"
#include "pinocchio/bindings/python-nb/spatial/se3.hpp"
#include "pinocchio/bindings/python-nb/spatial/inertia.hpp"
#include "pinocchio/bindings/python-nb/spatial/symmetric3.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
namespace nb = nanobind;

void exposeSkew(nb::module_ m);
void exposeExplog(nb::module_ m);

void exposeSpatial(nb::module_ m)
{
  // Axes. Originally in bindings/python/module.cpp
  m.attr("XAxis") = pinocchio::XAxis::vector<Scalar>();
  m.attr("YAxis") = pinocchio::YAxis::vector<Scalar>();
  m.attr("ZAxis") = pinocchio::ZAxis::vector<Scalar>();

  exposeMotion<Motion>(m);
  exposeForce<Force>(m);
  exposeSE3<SE3>(m);
  exposePseudoInertia<PseudoInertia>(m);
  exposeLogCholeskyParameters<LogCholeskyParameters>(m);
  exposeInertia<Inertia>(m);
  exposeSymmetric3<Symmetric3>(m);
  exposeSkew(m);
  exposeExplog(m);
}
PINOCCHIO_PYTHON_NAMESPACE_END
