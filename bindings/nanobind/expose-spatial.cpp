// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/spatial/motion.hpp"
#include "pinocchio/bindings/python-nb/spatial/force.hpp"
#include "pinocchio/bindings/python-nb/spatial/se3.hpp"
#include "pinocchio/bindings/python-nb/spatial/inertia.hpp"
#include "pinocchio/bindings/python-nb/spatial/symmetric3.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
namespace nb = nanobind;

void exposeSpatial(nb::module_ m)
{
  exposeMotion<pinocchio::Motion>(m);
  exposeForce<pinocchio::Force>(m);
  exposeSE3<pinocchio::SE3>(m);
  exposeInertia<pinocchio::Inertia>(m);
  exposeSymmetric3<pinocchio::Symmetric3>(m);
}
PINOCCHIO_PYTHON_NAMESPACE_END
