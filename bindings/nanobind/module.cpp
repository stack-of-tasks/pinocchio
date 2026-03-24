// Copyright (c) 2026 INRIA

#include "pinocchio/utils/version.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;
using namespace nanobind::literals;

void exposeSpatial(nb::module_ m);
void exposeMultibody(nb::module_ m);
void exposeAlgorithms(nb::module_ m);

// PINOCCHIO_PYTHON_MODULE_NAME is defined by the build system as the target name
// (e.g. pinocchio_pywrap_default).
NB_MODULE(PINOCCHIO_PYTHON_MODULE_NAME, m)
{
  m.doc() = "Pinocchio Python bindings (nanobind)";
  m.attr("__bindings_framework__") = "Nanobind";
  m.attr("__version__") = pinocchio::printVersion();
  m.attr("__raw_version__") = PINOCCHIO_VERSION;

  m.attr("PINOCCHIO_MAJOR_VERSION") = PINOCCHIO_MAJOR_VERSION;
  m.attr("PINOCCHIO_MINOR_VERSION") = PINOCCHIO_MINOR_VERSION;
  m.attr("PINOCCHIO_PATCH_VERSION") = PINOCCHIO_PATCH_VERSION;

  m.def(
    "printVersion", pinocchio::printVersion, "delimiter"_a = ".",
    "Returns the current version of Pinocchio as a string.\n"
    "The user may specify the delimiter between the different semantic numbers.");

  m.def(
    "checkVersionAtLeast", &pinocchio::checkVersionAtLeast, "major"_a, "minor"_a, "patch"_a,
    "Checks if the current version of Pinocchio is at least"
    " the version provided by the input arguments.");

#if defined(PINOCCHIO_PYTHON_INTERFACE_WITH_COLLISION_PYTHON_BINDINGS)
  m.import_("coal");
#endif

  // spatial
  exposeSpatial(m);

  // multibody
  exposeMultibody(m);

  // algorithm
  exposeAlgorithms(m);
}
