// Copyright (c) 2026 INRIA

#include "pinocchio/utils/version.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

// PINOCCHIO_PYTHON_MODULE_NAME is defined by the build system as the target name
// (e.g. pinocchio_pywrap_default).
NB_MODULE(PINOCCHIO_PYTHON_MODULE_NAME, m)
{
  m.doc() = "Pinocchio Python bindings (nanobind)";
  m.attr("__bindings_framework__") = "Nanobind";
  m.attr("__version__") = pinocchio::printVersion();
  m.attr("__raw_version__") = PINOCCHIO_VERSION;

#if defined(PINOCCHIO_PYTHON_INTERFACE_WITH_COLLISION_PYTHON_BINDINGS)
  m.import_("coal");
#endif
}
