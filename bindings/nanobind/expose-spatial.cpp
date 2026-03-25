// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/spatial/motion.hpp"
#include "pinocchio/bindings/python-nb/spatial/se3.hpp"
#include "pinocchio/bindings/python-nb/spatial/inertia.hpp"

namespace nb = nanobind;

void exposeSpatial(nb::module_ m)
{
  using namespace pinocchio::python_nb;

  exposeMotion<pinocchio::Motion>(m);
  exposeSE3<pinocchio::SE3>(m);
  exposeInertia<pinocchio::Inertia>(m);
}
