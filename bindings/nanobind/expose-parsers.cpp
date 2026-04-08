// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

void exposeConsoleBridge(nb::module_ m);
void exposeURDFModel(nb::module_ m);
void exposeURDFGeometry(nb::module_ m);
void exposeSRDF(nb::module_ m);
void exposeMJCF(nb::module_ m);

void exposeParsers(nb::module_ m)
{
  // URDF
  {
    exposeConsoleBridge(m);
    exposeURDFModel(m);
    exposeURDFGeometry(m);
  }

  exposeSRDF(m);
  exposeMJCF(m);
}

PINOCCHIO_PYTHON_NAMESPACE_END
