// Copyright (c) 2026 INRIA

#include "pinocchio/utils/version.hpp"
#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/constraints/fwd.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;
using namespace nanobind::literals;

namespace pinocchio::python_nb
{
  void exposeSpatial(nb::module_ m);
  void exposeMultibody(nb::module_ m);
  void exposeAlgorithms(nb::module_ m);
} // namespace pinocchio::python_nb

// PINOCCHIO_PYTHON_MODULE_NAME is defined by the build system as the target name
// (e.g. pinocchio_pywrap_default).
NB_MODULE(PINOCCHIO_PYTHON_MODULE_NAME, m)
{
  using namespace pinocchio::python_nb;
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

  m.import_("nanoeigenpy");
#if defined(PINOCCHIO_PYTHON_INTERFACE_WITH_COLLISION_PYTHON_BINDINGS)
  m.import_("coal");
#endif

  // enums
  nb::enum_<pinocchio::ReferenceFrame>(m, "ReferenceFrame")
    .value("WORLD", pinocchio::WORLD)
    .value("LOCAL", pinocchio::LOCAL)
    .value("LOCAL_WORLD_ALIGNED", pinocchio::LOCAL_WORLD_ALIGNED)
    .export_values();

  nb::enum_<pinocchio::KinematicLevel>(m, "KinematicLevel")
    .value("POSITION", pinocchio::POSITION)
    .value("VELOCITY", pinocchio::VELOCITY)
    .value("ACCELERATION", pinocchio::ACCELERATION)
    .export_values();

  nb::enum_<pinocchio::Convention>(m, "Convention")
    .value("WORLD", pinocchio::Convention::WORLD)
    .value("LOCAL", pinocchio::Convention::LOCAL);

  nb::enum_<pinocchio::ArgumentPosition>(m, "ArgumentPosition")
    .value("ARG0", pinocchio::ARG0)
    .value("ARG1", pinocchio::ARG1)
    .value("ARG2", pinocchio::ARG2)
    .value("ARG3", pinocchio::ARG3)
    .value("ARG4", pinocchio::ARG4)
    .export_values();

  nb::enum_<pinocchio::ConstraintSelectionType>(m, "ConstraintSelectionType")
    .value("CURRENT", pinocchio::ConstraintSelectionType::CURRENT)
    .value("MAXIMAL", pinocchio::ConstraintSelectionType::MAXIMAL);

  // spatial
  exposeSpatial(m);

  // multibody
  exposeMultibody(m);

  // algorithm
  exposeAlgorithms(m);
}
