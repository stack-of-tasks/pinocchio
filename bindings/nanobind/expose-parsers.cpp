// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

void exposeConsoleBridge(nb::module_ m);
void exposeURDFModel(nb::module_ m);
void exposeURDFGeometry(nb::module_ m);

void exposeParsers(nb::module_ m)
{
  exposeConsoleBridge(m);
  exposeURDFModel(m);
  exposeURDFGeometry(m);
}

PINOCCHIO_PYTHON_NAMESPACE_END
